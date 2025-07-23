#include "hybrid_astar_searcher/smooth.h"

namespace planning
{
    Smoother::Smoother(/* args */){}
    Smoother::~Smoother(){}

    //代价目标函数
    double Smoother::costFunction(void *ptr, 
                                  const Eigen::VectorXd &x, 
                                  Eigen::VectorXd &g)  {
        auto instance = reinterpret_cast<Smoother *>(ptr);
        std::vector<Vec3d> smooth_path = instance->smooth_path_;//待优化路径
        const int points_num = smooth_path.size() - 4;
        Eigen::Matrix2Xd opt_points;//二行x列的矩阵
        opt_points.resize(2, smooth_path.size());
        // 第一个路径点
        opt_points(0,0) = smooth_path[0](0);
        opt_points(1,0) = smooth_path[0](1);
        // 第二个路径点
        opt_points(0,1) = smooth_path[1](0);
        opt_points(1,1) = smooth_path[1](1);
        
        // 将x中的前points_num个元素转置并赋值给opt_points的第三列开始的部分
        // 对opt_points矩阵的一个子块进行操作。具体而言，它表示从opt_points矩阵的第0行、第2列开始，选取1行、points_num列的子矩阵
        opt_points.block(0,2,1,points_num) = x.head(points_num).transpose();
        // 将x中的后points_num个元素转置并赋值给opt_points的第四列开始的部分
        opt_points.block(1,2,1,points_num) = x.tail(points_num).transpose();
        // 将倒数第三个平滑路径点的x和y坐标分别赋值给opt_points的倒数第三列
        // opt_points.col(smooth_path.size()-3)(0) = smooth_path[smooth_path.size()-3](0);
        // opt_points.col(smooth_path.size()-3)(1) = smooth_path[smooth_path.size()-3](1);
        // 将倒数第二个平滑路径点的x和y坐标分别赋值给opt_points的倒数第二列
        opt_points.col(smooth_path.size()-2)(0) = smooth_path[smooth_path.size()-2](0);
        opt_points.col(smooth_path.size()-2)(1) = smooth_path[smooth_path.size()-2](1);
        // 将最后一个平滑路径点的x和y坐标分别赋值给opt_points的最后一列
        opt_points.col(smooth_path.size()-1)(0) = smooth_path[smooth_path.size()-1](0);
        opt_points.col(smooth_path.size()-1)(1) = smooth_path[smooth_path.size()-1](1);
        
        Eigen::Matrix2Xd grad;
        grad.resize(2, points_num);//梯度，排除前两个点和最后三个点
        grad.setZero();
        double cost = 0.0;//代价
        
        double max_clearance = instance->max_clearance_;//处于这个值之内将受到碰撞惩罚
        auto grid_resolution = instance->grid_map_->info.resolution;//地图分辨率
        auto xmin = instance->grid_map_->info.origin.position.x;//地图最小x坐标
        auto ymin = instance->grid_map_->info.origin.position.y;
        
        
        //碰撞项
        //假设我们有一个点x_i，到最近障碍物的距离为dist2obs，最大允许距离为max_clearance。
        //我们希望在避免碰撞的前提下，将点x_i移动到一个更合适的位置。为了实现这个目标，我们需要计算出点x_i对碰撞代价的影响程度，即梯度。
        //std::cout << "calculate collision cost" << std::endl;
        //collision cost
        double collision_cost = 0.0;
        Eigen::Matrix2Xd collision_grad;//存储碰撞项的梯度
        collision_grad.resize(2, points_num);
        collision_grad.setZero();

        //遍历opt_points矩阵的列，从第2列开始到倒数第4列结束
        for (int i = 2; i < opt_points.cols()-2; ++i) {
            Vec2i x_i;
            // 根据opt_points矩阵中的值计算出一个2维向量x_i
            x_i(0) = static_cast<int>((opt_points(0, i) - (xmin)) / grid_resolution);//栅格索引
            x_i(1) = static_cast<int>((opt_points(1, i) - (ymin)) / grid_resolution);

            // 计算与最近障碍物的距离dist2obs，并计算从障碍物到x_i的向量vec_o2x
            //dist to the closet obstacle    unit m  grid_resolution分辨率
            double dist2obs = grid_resolution * instance->voronoi_.getDistance(x_i(0), x_i(1));
            //std::cout << "dist2obs:" << dist2obs << std::endl;

            // 计算出从障碍物到点x_i的向量vec_o2x
            Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
                    x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
            //std::cout << "vec_o2x:" << vec_o2x << std::endl;
            
            // 如果dist2obs - max_clearance < 0，表示存在碰撞，则更新碰撞代价collision_cost和碰撞梯度collision_grad
            if (dist2obs - max_clearance < 0) {
                // 碰撞代价与距离的关系：碰撞代价通常与点到最近障碍物的距离成正比。即距离越近，碰撞代价越大
                // 梯度的方向：如果点距离障碍物更近，那么点沿着指向障碍物的方向移动会导致碰撞代价增加。因此，在避免碰撞的情况下，我们希望将点沿着远离障碍物的方向移动。
                collision_cost += instance->w_obs_ * pow((dist2obs - max_clearance), 2);//代价
                Vec2d gradient;
                // 计算梯度gradient，并将其赋值给collision_grad的对应位置
                // vec_o2x是从障碍物到点x_i的向量。由于我们希望将点移动远离障碍物，所以使用该向量的方向作为梯度的方向。
                // 通过计算得到的梯度，可以根据优化算法（如梯度下降）来更新点的位置，以最小化碰撞代价。
                gradient = instance->w_obs_ * 2 * (dist2obs - max_clearance) / dist2obs * vec_o2x;//梯度
                collision_grad(0, i-2) = gradient(0);
                collision_grad(1, i-2) = gradient(1);
            // 否则，如果dist2obs - max_clearance >= 0，表示没有碰撞，则将collision_cost和collision_grad置为0
            } else {
                collision_cost += 0;
                collision_grad(0, i-2) = 0;
                collision_grad(1, i-2) = 0;
            }
        }
        // 循环结束后，将碰撞代价collision_cost添加到总的代价cost中，同时将碰撞梯度collision_grad添加到总的梯度grad中
        cost += collision_cost;
        grad += collision_grad;


        //平滑项：平滑项的目的是使路径更加平滑，避免出现过于尖锐的拐角。
        //std::cout << "calculate smooth cost" << std::endl;
        //smooth cost
        double smooth_cost = 0.0;
        Eigen::Matrix2Xd smooth_grad;
        smooth_grad.resize(2, points_num);
        smooth_grad.setZero();
        //std::cout << opt_points.cols()-1 << std::endl;
        for (int i = 2; i < opt_points.cols()-2; ++i)  {
            Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));//相邻的前两个点
            Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
            Vec2d x_c(opt_points(0, i), opt_points(1, i));//当前点
            Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));
            Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));//相邻的后两个点

            //平滑项的误差err
            Vec2d err = x_p + x_n - 2* x_c;//当前点与相邻两个点的差值

            smooth_cost += instance->w_smo_ * err.transpose() * err;//平方
            //std::cout << smooth_cost << std::endl;
            
            // 计算平滑项的梯度
            //smooth_grad.col(i-1) = ((-4) * x_p + 8 * x_c - 4 * x_n);
            smooth_grad.col(i-2) = instance->w_smo_ * 2 * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);//五点差分公式，比较准确地估计二阶导数（即路径的弯曲程度）
        }
        //std::cout << "smooth_grad" << smooth_grad << std::endl;
        cost += smooth_cost;
        grad +=  smooth_grad;
        //std::cout << "grad" << grad << std::endl;


        // 曲率项
        // 曲率代价项是用来惩罚路径上车辆转弯过大或曲率变化过快的部分
        // curvature cost
        double curvature_cost = 0.0;
        Eigen::Matrix2Xd curvature_grad;
        curvature_grad.resize(2, points_num);
        curvature_grad.setZero();
        //std::cout << opt_points.cols()-1 << std::endl;
        // 循环遍历路径上的点：
        for (int i = 2; i < opt_points.cols()-2; ++i)  {
            // 对于每个点，我们需要获取其前两个点（x_p2和x_p），当前点（x_c），以及后两个点（x_n和x_n2）的位置。
            Vec2d x_p2(opt_points(0, i-2), opt_points(1, i-2));//前两个点
            Vec2d x_p(opt_points(0, i-1), opt_points(1, i-1));
            Vec2d x_c(opt_points(0, i), opt_points(1, i));//当前点
            Vec2d x_n(opt_points(0, i+1), opt_points(1, i+1));//后两个点
            Vec2d x_n2(opt_points(0, i+2), opt_points(1, i+2));

            // 四段线
            // 计算四段线段的向量差,这些向量表示路径的曲线方向
            Vec2d delta_x_p = x_p - x_p2;
            Vec2d delta_x_c = x_c - x_p;
            Vec2d delta_x_n = x_n - x_c;
            Vec2d delta_x_n2 = x_n2 - x_n;

            // 计算角度差
            // 使用向量的内积和范数计算相邻向量之间的夹角。通过将内积除以向量的范数乘积，我们获得了两个向量之间的夹角的余弦值。然后，使用反余弦函数将余弦值转换为实际角度（delta_phi_p，delta_phi_c和delta_phi_n）。
            // norm()是一种计算向量的范数（或长度）的函数，欧几里得范数，在二维空间中，向量(x,y)的范数等于sqrt(x^2 + y^2)
            if (delta_x_p.norm() > 0 && delta_x_c.norm() > 0 && delta_x_n.norm() > 0 && delta_x_n2.norm() > 0) {
                //取[-1,1]防止出现nan
                double delta_phi_p = std::acos(std::min(std::max(delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm(), -1.0), 1.0));
                double delta_phi_c = std::acos(std::min(std::max(delta_x_c.dot(delta_x_n) / delta_x_c.norm() / delta_x_n.norm(), -1.0), 1.0));
                double delta_phi_n = std::acos(std::min(std::max(delta_x_n.dot(delta_x_n2) / delta_x_n.norm() / delta_x_n2.norm(), -1.0), 1.0));
                //std::cout << delta_x_p.dot(delta_x_c) / delta_x_p.norm() / delta_x_c.norm() << std::endl;
                //std::cout << "delta_phi_p:" << delta_phi_p << std::endl;

                // 计算曲率（kappa）
                // 曲率是路径曲线在某一点处的曲率半径的倒数。我们通过将角度差除以对应向量的长度来计算曲率。得到了三个曲率值，即kappa_p，kappa_c和kappa_n
                // 通过将角度差除以向量长度，我们得到了路径在前一个点处的弧长与弧长对应的角度之间的比例。这可以被视为曲率的近似。
                double kappa_p = delta_phi_p / delta_x_p.norm();//角度差和向量长度的比值来近似曲率
                double kappa_c = delta_phi_c / delta_x_c.norm();
                double kappa_n = delta_phi_n / delta_x_n.norm();

                // 计算代价和梯度
                // 如果当前点的曲率（kappa_c）超过设定的最大曲率（instance->max_kappa_），则需要计算代价和梯度。否则，代价为0，梯度为零向量。
                if (kappa_c > instance->max_kappa_ && kappa_p > 0 && kappa_n > 0) {
                    // &函数：
                    // compute_d_delta_phi函数用于计算角度变化率
                    auto compute_d_delta_phi = [](const double delta_phi) {
                        // 我们首先使用delta_phi计算了曲线与切线之间的角度差。然后，我们使用std::cos()函数计算了该角度的余弦值。最后，我们使用余弦值来计算曲率，即将-1除以根号下（1- cos(delta_phi)的平方）。这将给出曲率半径的倒数，从而得到曲率。
                        return -1.0 / std::sqrt(1.0 - std::pow(std::cos(delta_phi),2));
                    };
                    // compute_orthogonal_complement函数用于计算两个向量的正交补
                    auto compute_orthogonal_complement = [](Vec2d x0, Vec2d x1) {
                        return x0 - x1 * x0.dot(x1) / std::pow(x1.norm(), 2);
                    };

                    // 利用向量运算，我们计算了曲率的导数（d_kappa_p，d_kappa_c和d_kappa_n）。这些导数将用于计算代价项的梯度。
                    // d_delta_phi_p是角度变化率delta_phi_p对时间t的导数
                    double d_delta_phi_p = compute_d_delta_phi(delta_phi_p);
                    Vec2d d_cos_delta_phi_p = compute_orthogonal_complement(delta_x_p, delta_x_c) 
                                            /  delta_x_p.norm() / delta_x_c.norm();
                    // d_kappa_p是曲率kappa_p对时间t的导数
                    Vec2d d_kappa_p = 1.0 / delta_x_p.norm() * d_delta_phi_p *  d_cos_delta_phi_p;
                    Vec2d k_p = 2.0 * (kappa_p - instance->max_kappa_) * d_kappa_p;
                    // std::cout <<  std::pow(std::cos(delta_phi_p),2) << std::endl;
                    //std::cout << "d_delta_phi_p:" << d_delta_phi_p << std::endl;
                    //std::cout << "d_cos_delta_phi_p:" << d_cos_delta_phi_p << std::endl;
                    //std::cout << "d_kappa_p:" << d_kappa_p << std::endl;
                
                    //std::cout << "kp:" << k_p << std::endl;


                    double d_delta_phi_c = compute_d_delta_phi(delta_phi_c);
                    Vec2d d_cos_delta_phi_c = compute_orthogonal_complement(delta_x_n, delta_x_c) 
                                            /  delta_x_c.norm() / delta_x_n.norm()
                                            -compute_orthogonal_complement(delta_x_c, delta_x_n)
                                            / delta_x_c.norm() / delta_x_n.norm();

                    Vec2d d_kappa_c = 1.0 / delta_x_c.norm() * d_delta_phi_c *  d_cos_delta_phi_c 
                                    -delta_phi_c / std::pow(delta_x_c.norm(), 3) * delta_x_c;
                    Vec2d k_c = 2.0 * (kappa_c - instance->max_kappa_) * d_kappa_c;

                    //std::cout << "d_cos_delta_phi_c:" << d_cos_delta_phi_c << std::endl;
                    //std::cout << "k_c:" << k_c << std::endl;

                    double d_delta_phi_n = compute_d_delta_phi(delta_phi_n);
                    Vec2d d_cos_delta_phi_n = -compute_orthogonal_complement(delta_x_n2, delta_x_n) 
                                            /  delta_x_n.norm() / delta_x_n2.norm();
                    Vec2d d_kappa_n = 1.0 / delta_x_n.norm() * d_delta_phi_n *  d_cos_delta_phi_n 
                                    +delta_phi_n / std::pow(delta_x_n.norm(), 3) * delta_x_n;
                    Vec2d k_n = 2.0 * (kappa_n - instance->max_kappa_) * d_kappa_n;
                    //std::cout << "d_cos_delta_phi_n:" << d_cos_delta_phi_n << std::endl;
                    //std::cout << "kn:" << k_n << std::endl;

                    // 最后，我们计算了曲率代价项（curvature_cost）。曲率代价项是超过最大曲率的点的曲率与最大曲率之间的差值的平方乘以权重（instance->w_cur_）的总和。
                    curvature_cost += instance->w_cur_ * std::pow(kappa_c - instance->max_kappa_, 2);//代价
                    // 曲率代价项的梯度（curvature_grad）是超过最大曲率的点的曲率导数的加权平均
                    curvature_grad.col(i-2) = instance->w_cur_ * (  0.25*k_p +  0.5*k_c + 0.25*k_n );//梯度

                } else {
                    curvature_cost += 0;
                    curvature_grad.col(i-2) = Vec2d(0, 0);
                }
            }
        }
        //std::cout << "curvature_grad" << curvature_grad << std::endl;
        cost += curvature_cost;
        grad += curvature_grad;


        //voronoi项，为了让路线尽可能在voronoi图上延伸
        //voronoi cost
        // double voronoi_cost = 0.0;
        // Eigen::Matrix2Xd voronoi_grad;
        // voronoi_grad.resize(2, points_num);
        // voronoi_grad.setZero();
        // //std::cout << opt_points.cols()-1 << std::endl;
        // for (int i = 2; i < opt_points.cols()-2; ++i)  {
        //     Vec2i x_i;
        //     x_i(0) = static_cast<int>((opt_points(0, i) - (xmin)) / grid_resolution);
        //     x_i(1) = static_cast<int>((opt_points(1, i) - (ymin)) / grid_resolution);

        //     // 机器人到最近障碍物的距离
        //     //dist to the closet obstacle    单位：unit m
        //     double dist2obs = grid_resolution * instance->voronoi_.getDistance(x_i(0), x_i(1));
        //     //std::cout << "dist2obs:" << dist2obs << std::endl;

        //     Vec2d vec_o2x(x_i(0) - instance->voronoi_.data[x_i(0)][x_i(1)].obstX,
        //                   x_i(1) - instance->voronoi_.data[x_i(0)][x_i(1)].obstY);
        
        //     int x_v = x_i(0), y_v = x_i(1);//路点的栅格索引
        //     std::vector<Vec2i> voronoi_points;
        //     for (int i = x_v - 30; i < (x_v + 30); ++i) {
        //         for (int j = y_v - 30; j < (y_v + 30); ++j) {
        //             // 获取其邻近区域内的Voronoi图上的点
        //             if (instance->voronoi_.isVoronoi(x_v, y_v)) {
        //                 voronoi_points.push_back({x_v, y_v});
        //             }
        //         }
        //     }
        //     // 计算当前点到最近边界的距离
        //     double dist2edge;
        //     Vec2d vec_e2x;
        //     // 表示当前点周围没有在Voronoi图上的点
        //     if (voronoi_points.empty()) {
        //         dist2edge = 3;//将dist2edge设置为一个较大的值3
        //         vec_e2x  = -vec_o2x;
        //     } else {
        //         int min_idx = 0;
        //         double min_dist = 10;
        //         for (int i = 0; i < voronoi_points.size(); ++i) {
        //             // 根据当前点与邻近Voronoi点的距离，找到距离最小的Voronoi点
        //             double dist = 0.1 * (x_i - voronoi_points[i]).norm();
        //             if (dist < min_dist) {
        //                 min_dist = dist;
        //                 min_idx = i;
        //             }
                    
        //         }
        //         dist2edge = min_dist;
        //         // 计算从该Voronoi点到当前点的向量
        //         vec_e2x(x_i(0) - voronoi_points[min_idx](0),
        //                 x_i(1) - voronoi_points[min_idx](1));
        //     }

        //     double alpha = instance->alpha_;

        //     if (dist2obs - max_clearance < 0) {
        //         // std::cout << "求gradient:" << std::endl;
        //         voronoi_cost += instance->w_vor_ * alpha /(alpha + dist2obs)
        //                         * dist2edge / (dist2edge + dist2obs)
        //                         * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2);
                
        //         // std::cout << "求gradient:" << std::endl;
        //         Vec2d gradient;
        //         gradient = instance->w_vor_ * 
        //                    (alpha /(alpha + dist2obs)
        //                    * dist2edge / (dist2edge + dist2obs)
        //                    * (dist2obs - max_clearance) / pow(max_clearance, 2)
        //                    * ((max_clearance - dist2obs)/(alpha + dist2obs)
        //                      -(dist2obs - max_clearance) / (dist2obs + dist2edge) + 2)
        //                    * vec_o2x / dist2obs
                        
        //                    + 
                        
        //                     alpha /(alpha + dist2obs) 
        //                    * dist2obs / pow(dist2edge + dist2obs, 2)
        //                    * pow(dist2obs - max_clearance, 2) / pow(max_clearance, 2)
        //                    * vec_e2x / dist2edge
        //                    );
                            
        //         voronoi_grad(0, i-2) = gradient(0);
        //         voronoi_grad(1, i-2) = gradient(1);
        //     } else {
        //         voronoi_cost += 0;
        //         voronoi_grad(0, i-2) = 0;
        //         voronoi_grad(1, i-2) = 0;
        //     }
        // }
        // //std::cout << "smooth_grad" << smooth_grad << std::endl;
        // cost += voronoi_cost;
        // grad +=  voronoi_grad;



        g.setZero();
        // 将grad矩阵的第一行转置后赋值给了g矩阵的前points_num行
        g.head(points_num) = grad.row(0).transpose();
        // 将grad矩阵的第二行转置后赋值给了g矩阵的后points_num行
        g.tail(points_num) = grad.row(1).transpose();



        // std::cout << std::setprecision(10)
        // std::cout << "------------------------" << "\n";
        // std::cout << "Function Value: " << cost << "\n";
        // std::cout << "Gradient Inf Norm: " << g.cwiseAbs().maxCoeff() << "\n";
        // std::cout << "------------------------" << "\n";

        return cost;

    }


    //路径优化：l(Limited-memory)-bfgs平滑
    double Smoother::optimize(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map) {
        smooth_path_ = path;//待优化路径
        voronoi_ = voronoi;//动态voronoi_图
        grid_map_ = grid_map;//栅格地图

        int points_num = smooth_path_.size() - 4;//去掉前两个点和最后三个点(我们把目标点加入了path)，不优化
        Eigen::VectorXd x(2 * points_num);//表示路径点的x坐标和y坐标
        for (int i = 2; i < smooth_path_.size()-2; ++i) {//从第三个点开始，到倒数第四个点结束
            // 前points_num个元素存储路径点的x坐标，后points_num个元素存储路径点的y坐标
            x(i-2) = smooth_path_[i](0);//填充x
            x(i-2 + points_num) = smooth_path_[i](1);//y
        }

        //std::cout << "即将优化" << std::endl;
        double minCost = 0.0;
        lbfgs_params.mem_size = 256;// 这可能是指LBFGS算法使用的历史信息的数量，也就是LBFGS名称中的"Limited-memory"部分。这个值被设置为256，表示算法将使用最近的256次迭代的信息
        // lbfgs_params.past = 3;//这个参数可能是用来检查收敛性的。如果最近的past次迭代改善都非常微小，算法可能会停止并声明已经达到收敛。这个值被设置为3，表示算法将查看最近的3次迭代
        lbfgs_params.past = 0;
        // lbfgs_params.min_step = 1.0e-32;//这是算法中线搜索步长的下限。如果步长变得太小，那么算法可能会提前停止。这个值被设置为1.0e-32，是一个非常小的正数，使得算法在进行非常微小的步长时仍能继续
        // lbfgs_params.max_linesearch =256;
        lbfgs_params.max_linesearch =64;
        // lbfgs_params.g_epsilon = 0.0;//在某些实现中，这是梯度的阈值。当梯度的范数低于此值时，算法可能会停止并声明已经达到收敛。这个值被设置为0.0，可能表示这个条件被禁用
        lbfgs_params.delta = 1.0e-5;//这可能是用于收敛测试的容差值。如果连续几次迭代的改善都小于这个值，那么算法可能会停止。这个值被设置为1.0e-5

        int ret = lbfgs::lbfgs_optimize(x,
                                        minCost,
                                        &Smoother::costFunction,//代价目标函数
                                        nullptr,
                                        nullptr,
                                        this,
                                        lbfgs_params);
        //std::cout << "ret:" << ret << std::endl;
        
        //std::cout << "minCost" << minCost << std::endl;
        // 计算路径上各点的朝向角度,索引0-1为原来的点
        for (int i = 2; i < smooth_path_.size() -2; ++i) {
                smooth_path_[i](0) = x(i-2);//x
                smooth_path_[i](1) = x(i-2 + points_num);//y
                smooth_path_[i-1](2) = std::atan2(smooth_path_[i](1) - smooth_path_[i-1](1),
                                            smooth_path_[i](0) - smooth_path_[i-1](0));//yaw
        }
        //最后一个优化点的yaw
        smooth_path_[smooth_path_.size() - 3](2) = std::atan2(smooth_path_[smooth_path_.size() -2](1) - smooth_path_[smooth_path_.size() -3](1),
                                            smooth_path_[smooth_path_.size() -2](0) - smooth_path_[smooth_path_.size() -3](0));
        if (ret >= 0) {
            
            //std::cout << "smooth_path_[i-1](2)" << smooth_path_[i-1](2) << std::endl;
            
            std::cout << "Optimization success" << std::endl;
        } else {
            minCost = INFINITY;
            std::cout << "Optimization Failed: "
                        << lbfgs::lbfgs_strerror(ret)
                        << std::endl;
        }
        return minCost;
    }


    //  梯度下降版本
    void Smoother::smoothPath(DynamicVoronoi &voronoi, std::vector<Vec3d> &path, const nav_msgs::OccupancyGridConstPtr& grid_map) {
        path_ = path;
        smooth_path_ = path;
        voronoi_ = voronoi;
        grid_map_ = grid_map;//栅格地图
        resolution_ = grid_map_->info.resolution;
        x_min_ = grid_map_->info.origin.position.x;//地图原点(第一个栅格点)在frame_id坐标系下("map")的值
        x_max_ = x_min_ + grid_map_->info.width*resolution_;//grid.info.width:宽度:在x方向的栅格数
        y_min_ = grid_map_->info.origin.position.y;
        y_max_ = y_min_ + grid_map_->info.height*resolution_;
        //voronoi_宽高
        width_ = voronoi_.getSizeX();
        height_ = voronoi_.getSizeY();
        
        int iter = 0;
        int max_iter = 1000;

        double weight_sum = w_obs_ + w_cur_ + w_smo_ + w_vor_;

        while (iter < max_iter) {
            
            for (int i = 2; i < path_.size() - 2; ++i) {
                Vec2d x_p2(smooth_path_[i-2](0), smooth_path_[i-2](1));
                Vec2d x_p(smooth_path_[i-1](0), smooth_path_[i-1](1));
                Vec2d x_c(smooth_path_[i](0), smooth_path_[i](1));
                Vec2d x_n(smooth_path_[i+1](0), smooth_path_[i+1](1));
                Vec2d x_n2(smooth_path_[i+2](0), smooth_path_[i+2](1));

                Vec2d correction = Vec2d::Zero();

                correction = correction + calObstacleTerm(x_c);
                
                if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;

                
                correction = correction + calSmoothTerm(x_p2, x_p, x_c, x_n, x_n2);
                
                
                if (!isInMap(x_c - alpha_ * correction / weight_sum)) continue;


                x_c = x_c - alpha_ * correction / weight_sum;
                
                smooth_path_[i](0) = x_c(0);
                smooth_path_[i](1) = x_c(1);

                Vec2d delta_x = x_c - x_p;
                if (i > 1) {
                    smooth_path_[i-1](2) = std::atan2(delta_x(1), delta_x(0));
                }
            }
            ++iter;
        }
        std::cout << iter << std::endl;
    }

    // 计算碰撞项
    Vec2d Smoother::calObstacleTerm(Vec2d x) {
        Vec2d gradient;

        Vec2i x_i;
        // x_i(0) = static_cast<int>((x(0) - (-25)) / 0.1);
        x_i(0) = static_cast<int>((x(0) - x_min_) / resolution_);
        // x_i(1) = static_cast<int>((x(1) - (-25)) / 0.1);
        x_i(1) = static_cast<int>((x(1) - y_min_) / resolution_);

        //dist to the closet obstacle    unit m
        double dist2obs = 0.1 * voronoi_.getDistance(x_i(0), x_i(1));

        Vec2d vec_o2x(x_i(0) - voronoi_.data[x_i(0)][x_i(1)].obstX,
                    x_i(1) - voronoi_.data[x_i(0)][x_i(1)].obstY);
        
        if (dist2obs  < max_clearance_) {
            gradient = w_obs_ * 2 * (dist2obs - max_clearance_) / dist2obs * vec_o2x;
        } else {
            gradient = Vec2d::Zero();
        }
        return gradient;
    }

    //计算平滑项
    Vec2d Smoother::calSmoothTerm(Vec2d x_p2, Vec2d x_p, Vec2d x_c, Vec2d x_n, Vec2d x_n2) {
        Vec2d gradient;//梯度
        gradient = w_smo_ * (x_p2 - 4 * x_p + 6 * x_c - 4 * x_n + x_n2);
        return gradient;
    }

    // 点是否在地图中
    inline bool Smoother::isInMap(Vec2d x) {
        // if (x(0) < -25 || x(1) < -25 || x(0) >= 25 || x(1) >= 25) {
        if (x(0) < x_min_ || x(1) < y_min_ || x(0) >= x_max_ || x(1) >= y_max_) {
            return false;
        }
        return true;
    }


} // namespace planning


