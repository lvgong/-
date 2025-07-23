#include "hybrid_astar_searcher/hybrid_astar.h"

using namespace std;

namespace planning
{
    // HybridAstar::HybridAstar(grid_map::GridMap map) {
    //     map_ = map;
    //     xy_resolution_ = map.getResolution();
    //     theta_resolution_ = 0.1;
    //     x_min_ = map.getStartIndex()(0) * xy_resolution_ - 25;
    //     x_max_ = x_min_ + map.getLength().x();
    //     y_min_ = map.getStartIndex()(1) * xy_resolution_ -25;
    //     y_max_ = y_min_ + map.getLength().y();
        
    //     map_size_x_ = map.getSize()(0); 
    //     map_size_y_ = map.getSize()(1);
        
    //     cout << "map_size_x_" << map_size_x_ << endl;
    //     cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;
    //     grid_search_ptr_ = make_unique<GridSearch>(map_);//智能指针
    //     // reed_shepp_generator_ =  std::make_shared<ReedShepp>(0.3, 0.7);//RS   （最大曲率，步长）
    //     dubins_generator_ =  std::make_shared<DubinsStateSpace>(min_turn_radius_, path_step_size);//db曲线
        
    // }

    HybridAstar::HybridAstar(const nav_msgs::OccupancyGridConstPtr& grid) :
        grid_(grid){
        // grid_ = grid;
        xy_resolution_ = grid_->info.resolution;
        theta_resolution_ = 0.1;//弧度，角度分辨率
        x_min_ = grid_->info.origin.position.x;//地图原点(第一个栅格点)在frame_id坐标系下("map")的值
        x_max_ = x_min_ + grid_->info.width*xy_resolution_;//grid.info.width:宽度:在x方向的栅格数
        y_min_ = grid_->info.origin.position.y;
        y_max_ = y_min_ + grid_->info.height*xy_resolution_;
        
        map_size_x_ = grid_->info.width; 
        map_size_y_ = grid_->info.height;
        
        //碰撞检测用覆盖栅格模板所用栅格
        configurationSpace.updateGrid(grid);

        grid_search_ptr_ = make_unique<GridSearch>(grid_, configurationSpace);//智能指针

        // reed_shepp_generator_ =  std::make_shared<ReedShepp>(0.3, 0.7);//RS   （最大曲率，步长）
        //这个2.2（曲率半径）要设置合理
        dubins_generator_ =  std::make_shared<DubinsStateSpace>(dubins_radius_, path_step_size);//db曲线：考虑到达目标点时的方向
        // dubins_generator_ =  std::make_shared<DubinsStateSpace>(0.8, path_step_size);//db曲线

        ROS_INFO("HybridAstar Init!");
    }

    HybridAstar::~HybridAstar() {}
    
    /**
     * @brief 混合A星搜索
     * 
     * @param start_pose 
     * @param goal_pose 
     * @param result 
     * @return true 
     * @return false 
     */
    bool HybridAstar::plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result) {
 
        //生成Heuristic启发表  预估成本H
        auto dp_map_start = chrono::high_resolution_clock::now();

        //使用动态规划DP来计算目标点到某点的启发代价（以目标点为DP的起点）
        //生成graph的同时获得了目标点到图中任一点的cost，后续只需要查表，空间换时间*/
        // grid_search_ptr_->generateDpMap({goal_pose(0), goal_pose(1)}, {start_pose(0), start_pose(1)});

        // 初始化近似A*表示的距离值，传入的是起终点的真实坐标
        grid_search_ptr_->InitLookUpTable(goal_pose, start_pose);
        auto dp_map_end = chrono::high_resolution_clock::now();
        chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
        cout << "距离查找表生成耗时:" << dp_map_use_time.count() * 1000 << "ms" << endl;

        //求栅格索引
        Vec3i start_pose_map = getIndexFromPose(start_pose);
        Vec3i goal_pose_map = getIndexFromPose(goal_pose);

        start_node_.reset(new Node3d({start_pose(0)}, {start_pose(1)}, {start_pose(2)}));
        goal_node_.reset(new Node3d({goal_pose(0)}, {goal_pose(1)}, {goal_pose(2)}));

        // 检查起点start_node_和终点end_node_是否发生碰撞
        if (!validityCheck(start_node_)) {
            cout << "起点不合理" << endl;
            return false;
        }

        if (!validityCheck(goal_node_)) {
            cout << "终点不合理" << endl;
            return false;
        }

      double path_length = std::numeric_limits<double>::max();
    //   auto hybrid_astar_start_ = chrono::high_resolution_clock::now();//初始化
    //   auto hybrid_astar_end_ = chrono::high_resolution_clock::now();
    //   HybridAstarResult result_={};
      //或者动态分配规划失败次数--------------
        // for(auto j=0; j<3; j++){
            //每次规划，清空之前的缓存数据
            open_set_.clear();
            close_set_.clear();
            open_pq_ = decltype(open_pq_)();//清空变量
            final_node_ = nullptr;

            // 计时开始
            auto hybrid_astar_start = chrono::high_resolution_clock::now();

            // ROS_INFO("1");

            // 将起始点start_node_加入open_pq_队列和open_set_集合中
            start_node_->setIndex(start_pose_map, map_size_x_, map_size_y_);
            open_set_.emplace(start_node_->getIndex(), start_node_);
            // open_pq_队列根据节点cost由小到达的顺序排列
            open_pq_.emplace(start_node_->getIndex(), start_node_->getCost());

            // ROS_INFO("2");

            int explored_node_num = 0;
            vis.vis_nodes_clear();
            while (!open_pq_.empty()) {
                //cout << "循环" << endl;
                int cur_index = open_pq_.top().first;
                // cout << "cur_index:" << cur_index << endl;
                open_pq_.pop();
                // 弹出队列第一个节点current_node
                shared_ptr<Node3d> cur_node = open_set_[cur_index];
                
                //可视化生长节点
                vis.publishExploredNodes({cur_node->getX(), cur_node->getY(), cur_node->getTheta()});
                
                
                // if (explored_node_num > 2000) {
                //     explored_node_num=0;
                //     cout << "搜索失败" << endl;
                //     return false;
                //     // continue;
                // }

                //看看能否用DB曲线直接连接到终点
        /*在混合A*执行过程中，可间歇性地触发Dubins曲线来生成一条衔接当前位姿和终点位姿的曲线，再判断是否符合碰撞条件，如果符合，那问题就解决了，如果不符合，那抛弃这次结果，继续执行正常的搜索逻辑。相当于时不时地尝试抄下小道，如果通就万事大吉，如果不通也没事，继续往前走。*/
        /*“先生成符合动力学的路径再校验这条路径是否满足碰撞条件“*/
        // 一旦ReedShepp曲线段连接了当前节点和终点，并且通过了安全检测，本轮次的规划结束。
        //但这样会在衔接处曲率突变，需要后续优化
                if (AnalyticExpansion(cur_node)) {
                    //最后都会直连成功
                    cout << "直接连接成功" << endl;
                    // grid_search_ptr_->clear();
                    break;
                }
                //cout << "没连接成功" << endl;
        /*        检查当前节点到终点是否能通过RS曲线连接生成无碰撞路径AnalyticExpansion，如果可以则找到完整路径，break
                否则在close_set_加入current_node*/
                close_set_.emplace(cur_index, cur_node);

                for (int i = 0; i < next_node_num_; ++i) {//next_node_num_=6，6个朝向
                    //cout << "expand " << i << "node" << endl;
                    // 产生从current_node 可达的下一个节点next_node
                    // 从current_node出发，依次以不同steering，前进arc(对角线长度)
                    //一个grid内的最后一个路径点叫node，该grid内可以有多个路径点，该node的next_node一定在相邻的其他grid内
                    shared_ptr<Node3d> next_node = nextNodeGenerator(cur_node, i);
                    //超出边界了
                    if (next_node == nullptr) {
                        continue;
                    }            
                    //cout << "node" << i << "is in map" << endl;

                    //判断是否在closelist
                    //  和closet_set_的节点比较索引，剔除已经遍历过的节点
                    Vec3i index111 = getIndexFromPose({next_node->getX(), next_node->getY(), next_node->getTheta()});
                    next_node->setIndex(index111, map_size_x_, map_size_y_);
                    // cout << "node" << i << "index :" << next_node->getIndex() << endl;
                    // 在close list中，则跳过
                    if (close_set_.find(next_node->getIndex()) != close_set_.end()) {
                        //cout << "node " << i << "is in close set" << endl;
                        continue;
                    }
                    // 对next_node碰撞检查，剔除路径上会发生碰撞的节点
                    // 存在碰撞，则跳过
                    if (!validityCheck(next_node)) {
                        continue;
                    }
                    //cout << "node " << i << "is valid" << endl;

        /*    // 从未被探索，进行初始化 
            // open_set_其实是close_set_和 open_pq_的合集
            // 每个栅格的index由 x_grid_、y_grid_、phi_grid_共同决定，而不只是x_grid_、y_grid，
            // 不过这里phi_grid_根据 phi_grid_resolution 做了离散化，所以重叠程度还是有的*/


            /*检查节点是否已经在待遍历集合open_set_，如果没有，则计算节点cost，并加入到open_set_和open_pq_队列*/
                    if (open_set_.find(next_node->getIndex()) == open_set_.end()) {
                        explored_node_num++;
                        // cout << "探索的节点个数：" << explored_node_num << endl;
                        
                        calculateNodeCost(cur_node, next_node);
                        
                        open_pq_.emplace(next_node->getIndex(), next_node->getCost());
                        open_set_.emplace(next_node->getIndex(), next_node);
                    }
                }
            }

            if (final_node_ == nullptr) {
                cout << "搜索失败" << endl;
                return false;
                // continue;//进入下一次循环
            }
            // 回溯法得到路径，从final_node_顺藤摸瓜
            if (!getHybridAstarResult(result)) {
                cout << "未得到解" << endl;
                // continue;//进入下一次循环
            }


            auto hybrid_astar_end = chrono::high_resolution_clock::now();
            cout << "搜索的节点个数：" << explored_node_num << endl;
            cout << "路径长度:" << result.total_length << "m" << endl;

            // 总规划时间（不带后端优化）
            chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
            cout << "规划时间:" << hybrid_astar_use_time.count() * 1000 << "ms" << endl;

            // if(path_length > result.total_length){
            //     path_length = result.total_length;
            //     //保存该次时间戳
            //     // hybrid_astar_start_ = hybrid_astar_start;
            //     // hybrid_astar_end_ = hybrid_astar_end;
            //     result_ = HybridAstarResult();
            //     result_ = result;
            //     // path2smooth.clear();
            //     // for (int i = 0; i < result.x.size(); ++i) {
            //     //     path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
            //     // }
            // }

        // }
        // result = HybridAstarResult();
        // result = result_;
        // 空值
        // if(result.x.empty()) return false;
        return true;

    }


    //折线A*规划
    // void HybridAstar::Astarplan(Vec2d start_pose, Vec2d goal_pose) {
    //     // 原始A*,仅取了各直线段的首末点,对可视化来说不影响
    //     std::vector<Point2D> ret = grid_search_ptr_->calculateHeuByAstar(start_pose, goal_pose);
    //     // 被拉直的A*折线路线
    //     std::vector<Point2D> ret_ = grid_search_ptr_->PathShorten(ret);
    //     //可视化
    //     //可视化前需要将行列索引转真实坐标
    //     std::vector<geometry_msgs::PoseStamped> mret;
    //     std::vector<geometry_msgs::PoseStamped> mret_;
    //     for(auto const& idx : ret){
    //         geometry_msgs::PoseStamped p;
    //         p.header.frame_id = "map";
    //         p.pose.position.x = (idx.x+0.5) * xy_resolution_ + x_min_;
    //         p.pose.position.y = (idx.y+0.5) * xy_resolution_ + y_min_;
    //         mret.push_back(p);
    //     }
    //     for(auto const& idx : ret_){
    //         geometry_msgs::PoseStamped p;
    //         p.header.frame_id = "map";
    //         p.pose.position.x = (idx.x+0.5) * xy_resolution_ + x_min_;
    //         p.pose.position.y = (idx.y+0.5) * xy_resolution_ + y_min_;
    //         mret_.push_back(p);
    //     }
    //     // vis.vis_clear();
    //     vis.publishPrimitiveAstar(mret);
    //     vis.publishSimpleAstar(mret_);
    // }


    // 回溯法得到路径，从final_node_顺藤摸瓜
    bool HybridAstar::getHybridAstarResult(HybridAstarResult &result) {
        shared_ptr<Node3d> cur_node = final_node_;/*最后直连的部分*/

        auto l1 = cur_node->getXs().size();

        vector<double> res_x;
        vector<double> res_y;
        vector<double> res_theta;
    // 核心的while循环，回溯过程
        while (cur_node->getParent() != nullptr) {
            vector<double> x = cur_node->getXs();
            vector<double> y = cur_node->getYs();
            vector<double> theta = cur_node->getThetas();
            // 将traversed_x_ y_ phi_ 反转下，里面的子序列
            reverse(x.begin(), x.end());
            reverse(y.begin(), y.end());
            reverse(theta.begin(), theta.end());

            x.pop_back();//移除 x 向量中的最后一个元素
            y.pop_back();
            theta.pop_back();

            res_x.insert(res_x.end(), x.begin(), x.end());
            res_y.insert(res_y.end(), y.begin(), y.end());
            res_theta.insert(res_theta.end(), theta.begin(), theta.end());
            cur_node = cur_node->getParent();
        }
        //此时的cur_node为起点
        res_x.push_back(cur_node->getX());
        res_y.push_back(cur_node->getY());
        res_theta.push_back(cur_node->getTheta());
        // 上面traversed_x_ y_ phi_被反转了，得反转回来
        reverse(res_x.begin(), res_x.end());
        reverse(res_y.begin(), res_y.end());
        reverse(res_theta.begin(), res_theta.end());

        // 目标点序列代入
        // res_x.push_back(goal_node_->getX());
        // res_y.push_back(goal_node_->getY());
        // res_theta.push_back(goal_node_->getTheta());     

        // 此时 result只包含位姿信息 x,y,phi
        result.x = res_x;
        result.y = res_y;
        result.theta = res_theta;

        //计算该路线长度：
        // auto l2 = (res_x.size()- l1 -1)*step_size_;
        auto l2 = (res_x.size() - l1)*step_size_;
        // ROS_INFO("4");
        //完整长度
        result.total_length = db_length + l2;

        // cout << "db_length:" << db_length << endl;
        // cout << "l1:" << l1 << endl;
        // cout << "res size:" << res_x.size() << endl;
        // cout << "l2:" << l2 << endl;
        //cout << "得到结果" << endl;
        return true;
    }


    // 从 current_node ，以某一steering，前进arc(长度为一个栅格的对角线长度)
    // 扩展节点的重点在于把车辆运动学模型的约束考虑进去，根据限定的steering angle 去搜索相邻的grid。
    // 扩展节点，扩展一个node就是扩展了一个grid，但是会产生多个在同一grid内的路径点
    shared_ptr<Node3d> HybridAstar::nextNodeGenerator(shared_ptr<Node3d> cur_node, int next_node_idx) {
        double steering = 0.0;
        double traveled_distance = 0.0;
        
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ / 2 - 1);
        // double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 2);
        double delta_steering = 2 * max_steer_angle_ / (next_node_num_ - 1);
        //cout << "扩展节点" << endl;
        //cout << "next_node_idx :" << next_node_idx << endl;

        //得到正向反向移动距离和打角
    /*  // 首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的（前进和倒车）
    // next_node_num_前一半是前进，后一半是后退
    // steering = 初始偏移量 + 单位间隔 × index*/
    /****************************************************************************
     *      转向定义为左打舵为 “+”，右打舵为负“-”，carsim里是这样定义的
     *      (-max_steer_angle_) 4   < \     / >   3  (max_steer_angle_)
     *                                 \   /
     *               (后退方向)  5 <----- O -----> 2  (前进方向)
     *                                 /   \
     *       (max_steer_angle_) 6   < /     \ >   1 (-max_steer_angle_)
     * **************************************************************************/

    /*  //steering angle为什么这么算？
    //首先，根据next_node_index与next_node_num_的对比是可以区分运动方向的
    //这里的if-else就是区分运动正反方向讨论的（前进和倒车）
    //其次，车辆在当前的姿态下，既可以向左转、又可以向右转，那么steering angle的
    //取值范围其实是[-max_steer_angle_, max_steer_angle_]，在正向或反向下，
    //能取next_node_num_/2个有效值。
    //即，把[-max_steer_angle_, max_steer_angle_]分为（next_node_num_/2-1）份
    //所以，steering = 初始偏移量 + 单位间隔 × index
    //steering angle的正负取决于车的转向，而非前进的正反*/
        // if (next_node_idx < next_node_num_ / 2) {
        //     // 前进steering计算
        //     steering = -max_steer_angle_ + delta_steering * next_node_idx;
        //     traveled_distance = step_size_;
        // } else {
        //     // 后退steering计算
        //     steering = -max_steer_angle_ + delta_steering * (next_node_idx - next_node_num_ / 2);
        //     traveled_distance = -step_size_;
        // }

        //唯一倒退
        // if (next_node_idx == next_node_num_-1) {
        //     steering = 0.;
        //     traveled_distance = -step_size_;
        // }
        //其余5个方向都是前进方向
        // else {
            steering = -max_steer_angle_ + delta_steering * next_node_idx;
            traveled_distance = step_size_;            
        // }
        //cout << "steer:" << steering << endl;
        double steering1 = steering / 180 * M_PI; //弧度，前轮转角
        //cout << "steer1:" << steering1 << endl;


    /*过程：以steering角度，通过自行车运动学模型行驶arc长度，顺带记录递推过程（即沿途点）traversed_x_ 、traversed_y_、traversed_phi_*/

        //存储节点的一堆中间点
        double arc = sqrt(2) * xy_resolution_; //可以设置想走多长，这个是栅格对角线长度(转化成以m为单位)
        arc = 2.0; //可以设置想走多长 m
        //cout << "111111" << endl;
        vector<double> traversed_x;
        vector<double> traversed_y;
        vector<double> traversed_theta;
        double cur_x = cur_node->getX();
        double cur_y = cur_node->getY();
        double cur_theta = cur_node->getTheta();
        //cout << "11222" << endl;
        //把上一个节点也加进去了，作为首成员
        traversed_x.push_back(cur_x);
        traversed_y.push_back(cur_y);
        traversed_theta.push_back(cur_theta);
        //cout << "arc / step_size_:" << arc / step_size_ << endl;
        //cout << "112223333" << endl;

        //从 current_node 前进 arc(即对角线长度的一段弧)，这只是一个朝向的
        // 从当前grid前进到下一个grid，一个grid内可能有多个路径点=arc / step_size_，这样一个朝向就有好几个小路点构成
        for (int i = 0; i < arc / step_size_; ++i) {
        // 自行车模型 递推 x,y,phi
        // traveled_distance = v * dt
            // 车辆行驶一段距离后的位置和朝向的计算，简化
            double next_x = cur_x + traveled_distance * cos(cur_theta);
            double next_y = cur_y + traveled_distance * sin(cur_theta);
            double next_theta = cur_theta + traveled_distance * tan(steering1) / wheel_base_;//公式没问题
            //cout << "turn radius:" << 1 / (tan(steering1) / wheel_base_) << endl;
            mod2Pi(next_theta);
            
            traversed_x.push_back(next_x);
            traversed_y.push_back(next_y);
            traversed_theta.push_back(next_theta);

            cur_x = next_x;
            cur_y = next_y;
            cur_theta = next_theta;
        }
        //cout << "xuanhuanwan" << endl;
        //cout << traversed_x.size() << endl;

        //末态节点：
        //超出边界就返回nullptr
        if (traversed_x.back() < x_min_ || traversed_x.back() > x_max_ ||
            traversed_y.back() < y_min_ || traversed_y.back() > y_max_) {
            //cout << "超出边界" << endl;
            return nullptr;
        }
        
        //未超出边界，作为next_node
        shared_ptr<Node3d> next_node = make_shared<Node3d>(traversed_x, traversed_y, traversed_theta);
        next_node->setParent(cur_node);
        next_node->setDirection(traveled_distance > 0.0);//前进还是后退
        next_node->setSteering(steering);//前轮转角
        //cout << "xyt:" << traversed_x.back() << " " << traversed_y.back() << " " << traversed_theta.back() << endl;
        return next_node;

    }


    // 计算 f = h + g 中的 "g":移动代价G
    //节点移动代价包括两个节点之间前进或后退距离的代价、前轮转向角度的代价，两个节点之间存在换挡还有换挡的代价：
    double HybridAstar::TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        double piecewise_cost = 0.0;

        //前进或后退距离的代价
        // 这段代码根据下一个节点的方向和距离计算路径的代价，用于评估路径的优劣。不同的方向和距离会根据惩罚系数对路径的代价进行调整，以便在路径搜索中选择更优的路径。
        if (next_node->getDirection()) {
            // 前进
            // next_node->getStepSize()表示从当前节点到下一个节点的步数，step_size_表示每一步的长度
            // (next_node->getStepSize() - 1) * step_size_表示从当前节点到下一个节点的距离
            //你要注意尺度要与g那边的匹配
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_forward_penalty_;
        } else {
            // 后退
            piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_back_penalty_;
        }

        //前进后退切换
        if (cur_node->getDirection() != next_node->getDirection()) {
            // 挡位切换的惩罚
            piecewise_cost += traj_gear_switch_penalty_;
        }
        // 从当前节点到下一个节点转向的代价
        // 这样做是为了鼓励路径保持平稳的转向，避免频繁改变方向，提高路径规划的稳定性和可靠性。
        piecewise_cost += traj_steer_change_penalty_ * std::abs(
            next_node->getSteering() - cur_node->getSteering()
        );
        // 转向切换的惩罚
        // 为了鼓励路径保持较小的转向角度，即减小转弯的幅度，提高路径规划的平稳性和舒适性
        piecewise_cost += traj_steer_penalty_ * std::abs(next_node->getSteering());

        return piecewise_cost;
        //各种惩罚待补充
    }


    // 分别计算路径代价和启发代价
    void HybridAstar::calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
        // 计算 f = h + g

        // 混合A*中走过的轨迹的代价G
        next_node->setTrajCost(cur_node->getTrajCost() + TrajCost(cur_node, next_node));

        //holo with obs
        double holo_heu_cost = 0.0;
        // 计算 f = h + g 中的 "h"，通过DP_Map得到，其实就是以终点为start的Dijkstra方法
        // A*中从当前点到目标点的启发式代价H，采用了动态规划DP来计算（以目标点为DP的起点）
        // holo_heu_cost = grid_search_ptr_->lookupInDpMap({next_node->getX(), next_node->getY()});
        // cout << "holo heu cost:" << holo_heu_cost << endl;

        // 查询近似A*表示的距离值
        holo_heu_cost = grid_search_ptr_->CheckLookUpTable({next_node->getX(), next_node->getY()});

        // 实时A*距
        // holo_heu_cost = grid_search_ptr_->calculateHeuByAstar({next_node->getX(), next_node->getY()}, {goal_node_->getX(), goal_node_->getY()});

        //nonholo without obs不考虑碰撞的
    /*这段代码是用于计算非完整约束转向路径规划中的启发式代价（heuristic cost）。以下是对代码的解释：
    首先，通过ob::ReedsSheppStateSpace(3)创建了一个三维的Reeds-Shepp状态空间，该空间用于模拟非完整约束转向路径规划。然后，通过space->allocState()分配了两个SE2状态空间的状态对象，分别为rsStart和rsEnd。
    接下来，使用rsStart->setXY(next_node->getX(), next_node->getY())和rsStart->setYaw(next_node->getTheta())设置起始节点的坐标和航向角。再使用rsEnd->setXY(goal_node_->getX(), goal_node_->getY())和rsEnd->setYaw(goal_node_->getTheta())设置目标节点的坐标和航向角。
    然后，通过space->distance(rsStart, rsEnd)计算起始节点到目标节点的Reeds-Shepp距离，将结果存储在RS_heu_cost变量中。
    最后，通过比较Reeds-Shepp距离和之前计算的holonomic转向启发式代价（holo_heu_cost）的最大值，选取较大值作为当前节点的启发式代价，并将其设置为next_node的启发式代价。
    在计算启发式代价后，代码还输出了非完整约束转向路径规划所用的时间以及计算得到的Reeds-Shepp启发式代价。*/
        // auto non_holo_heu_start = chrono::system_clock::now();

        // ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(3));
        // ob::SE2StateSpace::StateType * rsStart = (ob::SE2StateSpace::StateType*)space->allocState();
        // ob::SE2StateSpace::StateType* rsEnd = (ob::SE2StateSpace::StateType*)space->allocState();
        // rsStart->setXY(next_node->getX(), next_node->getY());
        // rsStart->setYaw(next_node->getTheta());
        // rsEnd->setXY(goal_node_->getX(), goal_node_->getY());
        // rsEnd->setYaw(goal_node_->getTheta());
        // double RS_heu_cost = space->distance(rsStart, rsEnd);
        // auto non_holo_heu_end = chrono::system_clock::now();
        // chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
        // cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;
        // cout << "RS heu cost: " << RS_heu_cost << endl;
        // next_node->setHeuristicCost(max(RS_heu_cost, holo_heu_cost));
double DB_heu_cost;
try {
        // auto non_holo_heu_start = chrono::system_clock::now();
        ob::StateSpacePtr space(std::make_shared<ob::DubinsStateSpace>(dubins_radius_));//曲率半径
        ob::SE2StateSpace::StateType * dbStart = (ob::SE2StateSpace::StateType*)space->allocState();
        ob::SE2StateSpace::StateType* dbEnd = (ob::SE2StateSpace::StateType*)space->allocState();
        dbStart->setXY(next_node->getX(), next_node->getY());
        dbStart->setYaw(next_node->getTheta());
        dbEnd->setXY(goal_node_->getX(), goal_node_->getY());
        dbEnd->setYaw(goal_node_->getTheta());
        DB_heu_cost = space->distance(dbStart, dbEnd);
        // auto non_holo_heu_end = chrono::system_clock::now();
        // chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
        // cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;
        // cout << "DB heu cost: " << DB_heu_cost << endl;

} catch (const std::exception& e) {
    // 处理异常的代码
    std::cerr << "An exception occurred: " << e.what() << std::endl;
    ROS_ERROR("An exception occurred.");
}

        // std::shared_ptr<DubinsPath_> dubins_to_check = std::make_shared<DubinsPath_>();
        // dubins_generator_->ShortestDBP_(cur_node, goal_node_, *dubins_to_check);
        // double DB_heu_cost = dubins_to_check->total_length;

        // 设置H值
        next_node->setHeuristicCost(max(DB_heu_cost, holo_heu_cost));

    }


    //在当前路点生成车的四个顶点
    vector<Vec2d> HybridAstar::calculateCarBoundingBox(Vec3d pose) {
        vector<Vec2d> vertices;
        double shift_distance = length_ / 2 - back_edge_to_center_;//后轴中心对车中心的偏移
        //小车中心
        Vec2d center = {pose(0) + shift_distance * std::cos(pose(2)),//pose位姿点在后轴中心
                        pose(1) + shift_distance * std::sin(pose(2))};
        //设置一个膨胀值inflation_value，放大轮廓（保守）
        const double dx1 = std::cos(pose(2)) * (length_ / 2 + inflation_value );
        const double dy1 = std::sin(pose(2)) * (length_ / 2 + inflation_value);
        const double dx2 = std::sin(pose(2)) * (width_ / 2 + inflation_value);
        const double dy2 = -std::cos(pose(2)) * (width_ / 2 + inflation_value);

        // const double dx1 = std::cos(pose(2)) * length_ / 2 ;
        // const double dy1 = std::sin(pose(2)) * length_ / 2 ;
        // const double dx2 = std::sin(pose(2)) * width_ / 2 ;
        // const double dy2 = -std::cos(pose(2)) * width_ / 2 ;

        //四个顶点
        vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
        vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
        vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); //顺时针
        return vertices;
    }


    /**
     * @brief 查询一条线上的点是否在障碍物里（插值）
     * 
     * @param x0 
     * @param y0 
     * @param x1 
     * @param y1 
     * @return true 
     * @return false 
     */
    bool HybridAstar::isLinecollision(double x0, double y0, double x1, double y1) {
        // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / xy_resolution_) + 1;
        // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / 1) + 1;
        
        auto check_point_num = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / xy_resolution_);

        double delta_x = (x1 - x0) / check_point_num;
        double delta_y = (y1 - y0) / check_point_num;

        double cur_x = x0;
        double cur_y = y0;

        // 判断是否是障碍物
        // auto idx = i + j * map_.info.width;：这一行计算出给定坐标在地图 data 数组中的下标 idx。由于地图数据存储方式通常是一维数组，所以需要将二维坐标转换为一维下标。具体计算方法是将 j 乘以地图宽度，再加上 i。
        // return map_.data[idx] == 100;：最后，函数返回一个布尔值，表示给定坐标是否为障碍物。如果地图 data 数组中 idx 对应的值为 100，就认为该坐标是障碍物，返回 true；否则返回 false。
        auto mapObstacle = [&](double x, double y) {
            auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
            auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
            auto idx = i + j * grid_->info.width;
            return grid_->data[idx] == 100 || grid_->data[idx] == -1;//100为障碍物像素值,-1为未知
        };

        for (int i = 0; i < check_point_num; ++i) {
            if (!isInMap(cur_x, cur_y)) {
                return true;
            }

            // Vec3i idx = getIndexFromPose({cur_x, cur_y, 0});
            //障碍区和未知区都不能通过
            if(mapObstacle(cur_x, cur_y))  return true;
            // if (map_.atPosition("elevation", {cur_x, cur_y}) > 0) {
            //     return true;
            // }

            cur_x += delta_x;
            cur_y += delta_y;
        }
        return false;
    }


    /**
     * @brief 验证节点是否可行，需要得到车的轮廓，然后用上边的判断四条边是否穿过障碍物
     * 
     * @param node 
     * @return true 
     * @return false 
     */
    bool HybridAstar::validityCheck(std::shared_ptr<Node3d> node) {
        
        int node_step_size = node->getStepSize();//代表一个grid中有多少个路点
        //cout << "节点里有多少个点：" << node_step_size << endl;
        //子路点
        const auto traversed_x = node->getXs();
        const auto traversed_y = node->getYs();
        const auto traversed_theta = node->getThetas();

        int start_index = 0;
        if (node_step_size > 1) {  //不止一个节点，有中间节点
            start_index = 1;   //第一个是上一个节点的x y theta
        }

        //遍历节点集合里的点
        for (int i = start_index; i < node_step_size; ++i) {
            //有超出地图边界的，节点抛弃
            if (!isInMap(traversed_x[i], traversed_y[i])) {
                return false;
            }

            //法一：在每个路径点使用线碰撞检测
            //生成车的四个顶点
            // auto vertices = calculateCarBoundingBox({traversed_x[i], traversed_y[i], traversed_theta[i]});
            // //遍历四条边是否碰撞
            // for (int i = 0; i < vertices.size(); ++i) {
            //     if (isLinecollision(vertices[i].x(), vertices[i].y(), 
            //                         vertices[(i+1) % 4].x(), vertices[(i+1) % 4].y())){
            //         return false;                    
            //     }
            // }

            //法二：覆盖栅格模板
            if(!configurationSpace.isTraversable({traversed_x[i], traversed_y[i], traversed_theta[i]}))
                return false;//表示碰撞

        }
        return true;
    }

    Vec3i HybridAstar::getIndexFromPose(Vec3d pose) {
        Vec3i index;
        index(0) = std::floor((pose(0) - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
        index(1) = std::floor((pose(1) - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
        //使角度维持在[-pi,+pi]之间
        mod2Pi(pose(2));
        index(2) = static_cast<int>((pose(2)) / theta_resolution_);
        return index;
    }



    /**
     * @brief 直接用DB曲线连接到终点,尝试用DB曲线连接当前点与目标点
     * 
     * @param current_node 
     * @return true 
     * @return false 
     */
    bool HybridAstar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
        // std::shared_ptr<ReedSheppPath> reeds_shepp_to_check = std::make_shared<ReedSheppPath>();
        //     //ReedShepp曲线都是从当前点到终点的
        // if (!reed_shepp_generator_->ShortestRSP(current_node, goal_node_,
        //                                         *reeds_shepp_to_check)) {
        //     return false;
        // }
        // std::cout << "ShortestDBP found";
        
        std::shared_ptr<DubinsPath_> dubins_to_check = std::make_shared<DubinsPath_>();
        // 生成最短Dubins曲线
        dubins_generator_->ShortestDBP(current_node, goal_node_, *dubins_to_check);

        // std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        //   reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));

        std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
        dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));

        //将DB生成的节点带入碰撞检测，如果碰撞则这条路不合适
        if (!validityCheck(node)) {
            //碰撞则摒弃
            return false;
        }
        //将连接到目标点的一段ReedShepp曲线路点封装成node，放入Hybrid A*的集合中
        // std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
        // reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
        std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
            dubins_to_check->x, dubins_to_check->y, dubins_to_check->phi));//这三个应该是double类型的
        db_length = dubins_to_check->total_length;
        goal_node->setParent(current_node);

        // Vec3i index111 = getIndexFromPose({goal_node->getX(), goal_node->getY(), goal_node->getTheta()});
        // goal_node->setIndex(index111, map_size_x_, map_size_y_);

        close_set_.emplace(goal_node->getIndex(), goal_node);
        final_node_ = goal_node;
        return true;
    }


    inline void HybridAstar::mod2Pi(double &angle) {
        if (angle >  M_PI)  angle -= 2 *M_PI;
        if (angle < - M_PI)  angle += 2 * M_PI;
    }


    //判断所给pose是否超出地图范围
    inline bool HybridAstar::isInMap(double x, double y) {

      auto i = std::floor((x - grid_->info.origin.position.x) / grid_->info.resolution + 0.5);
      auto j = std::floor((y - grid_->info.origin.position.y) / grid_->info.resolution + 0.5);
      return (i >= 0 && j >= 0 && i < grid_->info.width && j < grid_->info.height);

        // if (x < x_min_ || x > x_max_ ||
        //     y < y_min_ || y > y_max_) {
        //     return false;
        // }
        
        // return true;
    }

} // namespace planning

