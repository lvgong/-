#include "gen_esdfmap.h"

namespace chibot::slam {

    // #define current_img_ md_.depth_image_[image_cnt_ & 1]
    // #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

    void SDFMap::initMap(ros::NodeHandle& nh) {
    node_ = nh;

    /* get parameter */
    double x_size, y_size, z_size;
    node_.param("sdf_map/resolution", mp_.resolution_, -1.0);//地图分辨率
    //地图尺寸
    node_.param("sdf_map/map_size_x", x_size, -1.0);
    node_.param("sdf_map/map_size_y", y_size, -1.0);
    node_.param("sdf_map/map_size_z", z_size, -1.0);
    // local_update_range_(0):的第一个元素
    //局部地图更新尺寸
    node_.param("sdf_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
    node_.param("sdf_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
    node_.param("sdf_map/local_update_range_z", mp_.local_update_range_(2), -1.0);
    //障碍物膨胀
    node_.param("sdf_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

    node_.param("sdf_map/fx", mp_.fx_, -1.0);
    node_.param("sdf_map/fy", mp_.fy_, -1.0);
    node_.param("sdf_map/cx", mp_.cx_, -1.0);
    node_.param("sdf_map/cy", mp_.cy_, -1.0);

    node_.param("sdf_map/use_depth_filter", mp_.use_depth_filter_, true);
    node_.param("sdf_map/depth_filter_tolerance", mp_.depth_filter_tolerance_, -1.0);
    node_.param("sdf_map/depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
    node_.param("sdf_map/depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
    node_.param("sdf_map/depth_filter_margin", mp_.depth_filter_margin_, -1);
    node_.param("sdf_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
    node_.param("sdf_map/skip_pixel", mp_.skip_pixel_, -1);

    node_.param("sdf_map/p_hit", mp_.p_hit_, 0.70);
    node_.param("sdf_map/p_miss", mp_.p_miss_, 0.35);
    node_.param("sdf_map/p_min", mp_.p_min_, 0.12);
    node_.param("sdf_map/p_max", mp_.p_max_, 0.97);
    node_.param("sdf_map/p_occ", mp_.p_occ_, 0.80);
    node_.param("sdf_map/min_ray_length", mp_.min_ray_length_, -0.1);
    node_.param("sdf_map/max_ray_length", mp_.max_ray_length_, -0.1);

    node_.param("sdf_map/esdf_slice_height", mp_.esdf_slice_height_, -0.1);
    node_.param("sdf_map/visualization_truncate_height", mp_.visualization_truncate_height_, -0.1);
    node_.param("sdf_map/virtual_ceil_height", mp_.virtual_ceil_height_, -0.1);

    node_.param("sdf_map/show_occ_time", mp_.show_occ_time_, false);
    node_.param("sdf_map/show_esdf_time", mp_.show_esdf_time_, false);
    node_.param("sdf_map/pose_type", mp_.pose_type_, 1);

    node_.param("sdf_map/frame_id", mp_.frame_id_, string("world"));
    node_.param("sdf_map/local_bound_inflate", mp_.local_bound_inflate_, 1.0);
    node_.param("sdf_map/local_map_margin", mp_.local_map_margin_, 1);
    node_.param("sdf_map/ground_height", mp_.ground_height_, 1.0);

    mp_.local_bound_inflate_ = max(mp_.resolution_, mp_.local_bound_inflate_);
    mp_.resolution_inv_ = 1 / mp_.resolution_;
    //表示地图的原点在地图的左下角，mp_.ground_height_默认值为-0.01
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
    //这俩个值要选好
    //这里地图的原点在地图的左下角
    mp_.map_origin_1 = Eigen::Vector2d(-x_size / 2.0, -y_size / 2.0);
    mp_.map_size_1 = Eigen::Vector2d(x_size, y_size);

    mp_.prob_hit_log_ = logit(mp_.p_hit_);//每判定一次占据,自增mp_.prob_hit_log_ 
    mp_.prob_miss_log_ = logit(mp_.p_miss_);//每判定一次空闲,自增mp_.prob_miss_log_ 
    mp_.clamp_min_log_ = logit(mp_.p_min_);//最终能够判定空闲的阈值
    mp_.clamp_max_log_ = logit(mp_.p_max_);//最终能够判定占据的阈值
    mp_.min_occupancy_log_ = logit(mp_.p_occ_);
    mp_.unknown_flag_ = 0.01;

    //终端输出结果
    cout << "hit: " << mp_.prob_hit_log_ << endl;
    cout << "miss: " << mp_.prob_miss_log_ << endl;
    cout << "min log: " << mp_.clamp_min_log_ << endl;
    cout << "max: " << mp_.clamp_max_log_ << endl;
    cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

    //表示栅格地图的索引（index）范围,mp_.resolution_表示地图分表率，默认值为0.1;ceil向上取整
    for (int i = 0; i < 3; ++i) mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);
    for (int i = 0; i < 2; ++i) mp_.map_grid_num_(i) = ceil(mp_.map_size_1(i) / mp_.resolution_);

    //表示地图的位置（pos）范围
    mp_.map_min_boundary_ = mp_.map_origin_; // 右下角作为地图原点(-20, -20, -2)
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;// (20, 20, 3)

    //表示地图的位置（pos）范围
    mp_.map_min_range_ = mp_.map_origin_1; // 右下角作为地图原点(-20, -20, -2)
    mp_.map_max_range_ = mp_.map_origin_1 + mp_.map_size_1;// (20, 20, 3)
    

    mp_.map_min_idx_ = Eigen::Vector3i::Zero();// 最小索引
    mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones(); // 最大索引

    // initialize data buffers
    //缓冲区大小
    int buffer_size = mp_.map_grid_num_(0) * mp_.map_grid_num_(1);

    //具体占据概率,初始化为-1.99243-0.01=-2.00243(空闲)
    md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
    //负
    md_.occupancy_buffer_neg = vector<char>(buffer_size, 0);
    //栅格占据与否,0表示空闲,1表示占据，只有这俩个状态，未知状态应该转为障碍区？
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

    md_.distance_buffer_ = vector<double>(buffer_size, 10000);
    md_.distance_buffer_neg_ = vector<double>(buffer_size, 10000);
    md_.distance_buffer_all_ = vector<double>(buffer_size, 10000);

    //统计涉及到的体素的次数,占据和空闲均会+1
    md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
    //统计涉及到的占据体素的次数,占据时会+1
    md_.count_hit_ = vector<short>(buffer_size, 0);
    md_.flag_rayend_ = vector<char>(buffer_size, -1);
    md_.flag_traverse_ = vector<char>(buffer_size, -1);

    md_.tmp_buffer1_ = vector<double>(buffer_size, 0);
    md_.tmp_buffer2_ = vector<double>(buffer_size, 0);
    md_.raycast_num_ = 0;

    md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
    md_.proj_points_cnt = 0;

    /* init callback */
    // 深度点云（过滤器）
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "/sdf_map/depth", 50));

    // 如果位置模式为POSE_STAMPED 则用ApproximateTime 策略（用自适应算法来匹配基于其时间戳的消息）处理grid_map/depth和grid_map/pose， 回调depthPoseCallback 。
    if (mp_.pose_type_ == POSE_STAMPED) {
        // 相机位姿，回调odomCallback 保存相机位置
        pose_sub_.reset(
            new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "/sdf_map/pose", 25));

        sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
            SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));

        sync_image_pose_->registerCallback(boost::bind(&SDFMap::depthPoseCallback, this, _1, _2));
    // 如果位置模式为ODOMETRY则用ApproximateTime策略处理grid_map/depth和grid_map/odom ，回调depthOdomCallback 。
    // 上面两个的区别就在于grid_map/pose 是相机在世界系下的坐标，而grid_map/odom 是机体在世界系下的坐标，需要用变换矩阵得到相机在世界系下的坐标。
    // 原程序中用订阅grid_map/cloud和grid_map/odom 实现比较简单快速。
    } else if (mp_.pose_type_ == ODOMETRY) {
        // 机体位姿（过滤器）
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/sdf_map/odom", 100));

        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
        sync_image_odom_->registerCallback(boost::bind(&SDFMap::depthOdomCallback, this, _1, _2));
    }

    // use odometry and point cloud
    // 激光点云，回调cloudCallback 更新膨胀地图
    indep_cloud_sub_ =
        node_.subscribe<sensor_msgs::PointCloud2>("/sdf_map/cloud", 10, &SDFMap::cloudCallback, this);
    indep_odom_sub_ =
        node_.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 10, &SDFMap::odomCallback, this);

    occ_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateOccupancyCallback, this);//更新栅格地图
    esdf_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::updateESDFCallback, this);//更新ESDF地图
    vis_timer_ = node_.createTimer(ros::Duration(0.05), &SDFMap::visCallback, this);//发布（膨胀）栅格地图

    // 栅格地图
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy", 10);
    // 膨胀栅格地图
    map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/occupancy_inflate", 10);
    esdf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 10);
    update_range_pub_ = node_.advertise<visualization_msgs::Marker>("/sdf_map/update_range", 10);

    unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown", 10);
    depth_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/depth_cloud", 10);

    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    md_.esdf_need_update_ = false;
    md_.has_first_depth_ = false;
    md_.has_odom_ = false;
    md_.has_cloud_ = false;
    md_.image_cnt_ = 0;

    md_.esdf_time_ = 0.0;
    md_.fuse_time_ = 0.0;
    md_.update_num_ = 0;
    md_.max_esdf_time_ = 0.0;
    md_.max_fuse_time_ = 0.0;

    rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
    rand_noise2_ = normal_distribution<double>(0, 0.2);
    random_device rd;
    eng_ = default_random_engine(rd());
    }


    // 将膨胀地图范围内的信息全部清空
    void SDFMap::resetBuffer() {
    Eigen::Vector3d min_pos = mp_.map_min_boundary_;
    Eigen::Vector3d max_pos = mp_.map_max_boundary_;

    resetBuffer(min_pos, max_pos);

    md_.local_bound_min_ = Eigen::Vector3i::Zero();//清零
    md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();//有三个整型元素，分别为1、1和1
    }


    // 将指定膨胀地图范围内的信息清空
    void SDFMap::resetBuffer(Eigen::Vector2d min_pos, Eigen::Vector2d max_pos) {

    Eigen::Vector2i min_id, max_id;
    //将二维位置转换为索引值
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);
    //确保最小索引min_id和最大索引max_id均在地图内
    boundIndex(min_id);
    boundIndex(max_id);

    /* reset occ and dist buffer */
    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y) {
            //空闲
            md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
            md_.distance_buffer_[toAddress(x, y)] = 10000;
        }
    }

    /*这是一个模板函数的声明。它定义了两个模板参数F_get_val和F_set_val，它们分别表示函数对象的类型。
    这段代码表明该函数可以接受任意类型的函数对象作为参数，并在函数的实现中使用这些函数对象来完成一些特定的操作。通过使用模板参数，可以使函数更加通用，适用于不同类型的函数对象。
    在这个函数声明中，F_get_val表示用于获取值的函数对象的类型，F_set_val表示用于设置值的函数对象的类型。具体的函数签名和实现细节需要根据函数的具体定义来确定。*/
    //为什么用模块参数，因为fillESDF前两个参数由lambda表达式的返回值构成，灵活通用
    template <typename F_get_val, typename F_set_val>
    /*该函数仅对多维空间的一维进行了计算
    在Fast Planner中，通过将该算法分别在3个维度中计算一次，并将上一个维度计算出的数值Df(p)赋值给下一个维度计算时的f(q)，
    经过3次遍历以后就实现了3D空间中空闲体素到最近障碍物物栅格距离的计算。该计算过程详见函数SDFMap::updateESDF3d()，其3次调用函数fillESDF()对每个维度单独计算距离（
    维度计算顺序：z → y → x）*/
    // 与论文中的伪代码一致
    //dim表示第几维度，0是X维，2是Z维
    void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {
        int v[mp_.map_grid_num_(dim)];//mp_.map_grid_num_(dim)：在dim维上的栅格数量  存储索引值
        double z[mp_.map_grid_num_(dim) + 1];//存储距离值

        int k = start;//start是在dim维上的更新范围起点
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();//-无穷大
        z[start + 1] = std::numeric_limits<double>::max();

        /*从start到end的范围内进行迭代。在每次迭代中，函数通过一系列计算来确定索引k和距离s的值。
        具体来说，函数通过计算当前位置的函数值和索引值与之前位置的函数值和索引值之间的差异，来确定距离值。
        如果计算得到的s小于或等于z[k]，则说明下包络不包含这一段抛物线，需要继续计算。
        最终得到的k和s值被存储到v和z数组中。*/
        for (int q = start + 1; q <= end; q++) {
            k++;
            double s;

            do {
            k--;
            // f_get_val传入的参数就是上层lambda表达式的输入参数，返回值是ambda表达式return的返回值
            s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);// s <= z[k], 说明下包络不包含第k段抛物线

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        // 从start到end的范围内进行迭代。在每次迭代中，函数根据之前计算得到的k和z数组，来确定索引k的位置，并计算出相应的距离值。
        // 最后，通过调用f_set_val函数对象，将计算得到的距离值存储到相应的位置。
        for (int q = start; q <= end; q++) {
            while (z[k + 1] < q) k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            // f_set_val输入的参数也是上层lambda表达式的输入参数
            f_set_val(q, val);//val值将被buffer保存
        }
    }


    //全局地图更新二维环境中的距离场（Distance Field）信息
    void SDFMap::updateESDF2d() {

        //全局地图更新,传入的应该是地图的尺寸

        Eigen::Vector2i min_esdf = md_.local_range_min_;//赋XYZ三个维度上最小更新范围
        Eigen::Vector2i max_esdf = md_.local_range_max_;

        /* ========== compute positive DT 正向距离 ========== */
        // 在计算正向距离时，首先使用fillESDF函数进行填充，然后逐层计算三维距离值，最终得到正向距离场信息md_.distance_buffer_

        //先更新Y维度上的距离
        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
        // 这里实际上就是传递了个函数进去, fillESDF内部调用的时候给定一个y, 返回障碍物占用信息: 占用 -> 0, 未占用 -> max(), 和论文中1(q)函数对应
            fillESDF(
        //lambda表达式接受一个y值作为输入，根据md_.occupancy_buffer_inflate_数组中相应位置的值来确定障碍物的占用信息。如果该位置未被占用，返回一个较大的值（std::numeric_limits<double>::max()），否则返回0。
                [&](int y) {
                    //返回0或max
                    return md_.occupancy_buffer_inflate_[toAddress(x, y)] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
        // 根据传入的lambda表达式计算出的值，将结果存储到md_.tmp_buffer1_数组的相应位置。这样，经过这段嵌套循环的计算，md_.tmp_buffer1_数组中存储了一部分正向距离场的信息。
                [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1], // 更新tmp_buffer1_
                max_esdf[1], 1);
                // 通过Z维度上的min_esdf和max_esdf来限定了循环遍历的范围，确保只更新指定区域的距离场信息
            
        }
        //再更新X维度上的距离
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            //md_.tmp_buffer1_是上一维度计算的结果，作为本维度计算的基础赋给 f(q) 
            fillESDF([&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
                    [&](int x, double val) {
                        md_.distance_buffer_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);//更新distance_buffer_，正向距离值
                        //  min(mp_.resolution_ * std::sqrt(val),
                        //      md_.distance_buffer_[toAddress(x, y, z)]);
                    },
                    min_esdf[0], max_esdf[0], 0);
            
        }

        /* ========== compute negative distance 负向距离 ========== */
        // 在计算负向距离时，首先根据障碍物占用情况对md_.occupancy_buffer_neg进行赋值，然后同样使用fillESDF函数进行填充，逐层计算三维负向距离值，最终得到负向距离场信息md_.distance_buffer_neg_
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y) {

                int idx = toAddress(x, y);
                //反转，看来只有两种状态：空闲/障碍
                if (md_.occupancy_buffer_inflate_[idx] == 0) {//空闲区
                md_.occupancy_buffer_neg[idx] = 1;//变障碍区

                } else if (md_.occupancy_buffer_inflate_[idx] == 1) {
                md_.occupancy_buffer_neg[idx] = 0;
                } else {
                ROS_ERROR("what?");
                }
            }

        ros::Time t1, t2;

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++) {
            fillESDF(
                [&](int y) {
                    return md_.occupancy_buffer_neg[x * mp_.map_voxel_num_(1) + y ] == 1 ?
                        0 :
                        std::numeric_limits<double>::max();
                },
                [&](int y, double val) { md_.tmp_buffer1_[toAddress(x, y)] = val; }, min_esdf[1],
                max_esdf[1], 1);
            
        }

        for (int y = min_esdf[1]; y <= max_esdf[1]; y++) {
            fillESDF([&](int x) { return md_.tmp_buffer1_[toAddress(x, y)]; },
                    [&](int x, double val) {
                        md_.distance_buffer_neg_[toAddress(x, y)] = mp_.resolution_ * std::sqrt(val);
                    },
                    min_esdf[0], max_esdf[0], 0);
            
        }

        /* ========== combine pos and neg DT ========== */
        //在合并正向距离和负向距离时，将两者结合起来得到整体的距离场信息md_.distance_buffer_all_。
        for (int x = min_esdf(0); x <= max_esdf(0); ++x)
            for (int y = min_esdf(1); y <= max_esdf(1); ++y) {

                int idx = toAddress(x, y);
                //赋距离值
                md_.distance_buffer_all_[idx] = md_.distance_buffer_[idx];

                if (md_.distance_buffer_neg_[idx] > 0.0)
                md_.distance_buffer_all_[idx] += (-md_.distance_buffer_neg_[idx] + mp_.resolution_);
            }
    }


    // 如果occ不为1或0则返回INVALID_IDX ，否则返回pos在地图中的地址。并且若occ==1 ，将该点为障碍物的几率上升 。
    int SDFMap::setCacheOccupancy(Eigen::Vector3d pos, int occ) {
    if (occ != 1 && occ != 0) return INVALID_IDX;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    int idx_ctns = toAddress(id);
    //体素占据和空闲,md_.count_hit_and_miss_均会自增1
    md_.count_hit_and_miss_[idx_ctns] += 1;
    //md_.count_hit_and_miss_[idx_ctns] == 1表示这个体素首次被发现,idx_ctns表示这个体素的一维索引
    if (md_.count_hit_and_miss_[idx_ctns] == 1) {
        // md_.cache_voxel_存储体素的三维索引
        md_.cache_voxel_.push(id);
    }

    if (occ == 1) md_.count_hit_[idx_ctns] += 1;

    return idx_ctns;
    }


    /*这一函数比较直观，就是通过相机投影模型，将图像坐标系上的点投影至相机坐标系，再通过相机的位姿将相机坐标系上点投影至世界坐标系，最后将所有点存入md.proj_points_这一vector容器中。*/
    //将点云转换到世界坐标系下，将接收到的深度点云转化到世界坐标系下保存到md_.proj_points_ 中。
    void SDFMap::projectDepthImage() {
    // md_.proj_points_.clear();
    md_.proj_points_cnt = 0;

    uint16_t* row_ptr;
    // int cols = current_img_.cols, rows = current_img_.rows;
    int cols = md_.depth_image_.cols;
    int rows = md_.depth_image_.rows;

    double depth;

    Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

    // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
    // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;

    if (!mp_.use_depth_filter_) {
        for (int v = 0; v < rows; v++) {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v);

        for (int u = 0; u < cols; u++) {

            Eigen::Vector3d proj_pt;
            depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
            proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
            proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
            proj_pt(2) = depth;

            proj_pt = camera_r * proj_pt + md_.camera_pos_;

            if (u == 320 && v == 240) std::cout << "depth: " << depth << std::endl;
            md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
        }
        }
    }
    /* use depth filter */
    else {

        if (!md_.has_first_depth_)
        md_.has_first_depth_ = true;
        else {
        Eigen::Vector3d pt_cur, pt_world, pt_reproj;

        Eigen::Matrix3d last_camera_r_inv;
        last_camera_r_inv = md_.last_camera_q_.inverse();
        const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

        for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_) {
            row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

            for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
                u += mp_.skip_pixel_) {

            depth = (*row_ptr) * inv_factor;
            row_ptr = row_ptr + mp_.skip_pixel_;

            // filter depth
            // depth += rand_noise_(eng_);
            // if (depth > 0.01) depth += rand_noise2_(eng_);

            if (*row_ptr == 0) {
                depth = mp_.max_ray_length_ + 0.1;
            } else if (depth < mp_.depth_filter_mindist_) {
                continue;
            } else if (depth > mp_.depth_filter_maxdist_) {
                depth = mp_.max_ray_length_ + 0.1;
            }

            // project to world frame
            pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
            pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
            pt_cur(2) = depth;

            pt_world = camera_r * pt_cur + md_.camera_pos_;
            // if (!isInMap(pt_world)) {
            //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
            // }

            md_.proj_points_[md_.proj_points_cnt++] = pt_world;

            // check consistency with last image, disabled...
            if (false) {
                pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
                double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
                double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

                if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
                if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                        pt_reproj.z()) < mp_.depth_filter_tolerance_) {
                    md_.proj_points_[md_.proj_points_cnt++] = pt_world;
                }
                } else {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
                }
            }
            }
        }
        }
    }

    /* maintain camera pose for consistency check */

    md_.last_camera_pos_ = md_.camera_pos_;
    md_.last_camera_q_ = md_.camera_q_;
    md_.last_depth_image_ = md_.depth_image_;
    }


    /*这一函数会对md.proj_points中的每一个点进行raycast流程。首先判断每一个点是否超出地图范围，是否超出ray_length，如果超出，就将此点重新赋值为一个允许的射线上最远的点。
    如果重新赋值，则利用setCacheOccupancy（）这一函数将md_.count_hit_and_miss_这一容器对应序列上的计数+1次。表示这一free空间被经过了一次。如果这一点是第一次被遍历，则应该把它加入到md_.cache_voxel这一容器中区
    如果不需要重新赋值，说明当前点是障碍物，则利用setCacheOccupancy将这一点在md_.count_hit容器中对应序列的位置计数+1。需要说明的是，不管当前点是不是障碍物，md_.count_hit_and_miss_容器对应位置处的计数都会被+1.
    当终点被setCache过后，就进入raycast环节，通过raycast.step函数从射线终点开始向相机点步进。并且将每一个中途点都利用setCacheOccupancy函数置一次free。需要注意的是，每一个中途点还利用md_.flag_traverse_容器进行了判断，如果对应序列处的值不是本轮raycast的num,则将其置为b本轮的racastnum.否则说明这一点及之后的点都已经被raycast过了，因此跳出当前射线终点的raycast循环。
    当完成md.proj_points容器中所有点的raycast循环后，开始对md_.cache_voxel中的点进行循环判断。首先根据md_.count_hit及md_.count_hit_and_miss中对应位置的值判断当前voxel为障碍物的概率。并且如果当前点的log_odds_update是prob_hit_log，且md_.occupancy_buffer_中对应位置的概率值还没有超过最大值或当前点的log_odds_update是prob_miss_log，且md_.occupancy_buffer_中对应位置的概率值还没有低于最小值。且当前点是在局部地图范围内，则更新md_.occupancy_buffer_中的概率值。*/
    //根据概率更新栅格地图
    void SDFMap::raycastProcess() {
    // if (md_.proj_points_.size() == 0)
    if (md_.proj_points_cnt == 0) return;

    ros::Time t1, t2;
    //md_.raycast_num_初始化为0
    md_.raycast_num_ += 1;

    int vox_idx;
    double length;

    // bounding box of updated region
    //map_max_boundary,map_min_boundary表示地图的位置（pos）范围
    double min_x = mp_.map_max_boundary_(0);
    double min_y = mp_.map_max_boundary_(1);
    double min_z = mp_.map_max_boundary_(2);

    double max_x = mp_.map_min_boundary_(0);
    double max_y = mp_.map_min_boundary_(1);
    double max_z = mp_.map_min_boundary_(2);

    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    for (int i = 0; i < md_.proj_points_cnt; ++i) {
        pt_w = md_.proj_points_[i];

        // set flag for projected point
    /*判断该点是否在地图内*/
    /*若该点不在地图内,则在地图范围内找一个与该点靠近的点；又若新找到的点不在射线追踪范围,
    依旧在找一个在射线追踪范围内的新的点.最终设为**空闲***/
        if (!isInMap(pt_w)) {
        pt_w = closetPointInMap(pt_w, md_.camera_pos_);

        length = (pt_w - md_.camera_pos_).norm();
        if (length > mp_.max_ray_length_) {//mp_.max_ray_length_ = 4.5
            pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        }
        vox_idx = setCacheOccupancy(pt_w, 0);
    /* 若该点在地图内:1.不在射线追踪范围,依据该点找一个在射线追踪范围内的新的点,设为**空闲** 
    2.在射线追踪范围,设为**占据** */
        } else {
        length = (pt_w - md_.camera_pos_).norm();

        if (length > mp_.max_ray_length_) {
            pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
            vox_idx = setCacheOccupancy(pt_w, 0);
        } else {
            vox_idx = setCacheOccupancy(pt_w, 1);
        }
        }
    //第一次进入for循环,相当于max_x = pt_w(0),pt_w是点云的位置坐标.之后是取待追踪点云的最小位置坐标
        max_x = max(max_x, pt_w(0));
        max_y = max(max_y, pt_w(1));
        max_z = max(max_z, pt_w(2));
    //第一次进入for循环,相当于min_x = pt_w(0).之后是取待追踪点云的最大位置坐标
        min_x = min(min_x, pt_w(0));
        min_y = min(min_y, pt_w(1));
        min_z = min(min_z, pt_w(2));

        // raycasting between camera center and point
    //vox_idx是个一维向量,即是一个数
        if (vox_idx != INVALID_IDX) {
            // md_.flag_rayend_ = vector<char>(buffer_size, -1);初始化为-1
        //确保一个vox_idx只塞进md_.flag_rayend_一次,用于提高效率
        if (md_.flag_rayend_[vox_idx] == md_.raycast_num_) {
            continue;
        } else {
            md_.flag_rayend_[vox_idx] = md_.raycast_num_;
        }
        }
    /*pt_w / mp_.resolution_表示点云的栅格坐标, 
    md_.camera_pos_ / mp_.resolution_表示相机的栅格坐标*/
    //raycaster.setInput的具体实现见raycast.cpp文件
        raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);
    /*   raycaster.step(ray_pt)=false表示射线追踪到了最后一个点,
    即md_.camera_pos_ / mp_.resolution_ .ray_pt表示当前射线追踪点的栅格坐标  */
        while (raycaster.step(ray_pt)) {
        //tmp表示当前射线追踪点的位置坐标 
        Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
        length = (tmp - md_.camera_pos_).norm();

        // if (length < mp_.min_ray_length_) break;

        vox_idx = setCacheOccupancy(tmp, 0);

        if (vox_idx != INVALID_IDX) {
            if (md_.flag_traverse_[vox_idx] == md_.raycast_num_) {
            break;
            } else {
            md_.flag_traverse_[vox_idx] = md_.raycast_num_;
            }
        }
        }
    }

    // determine the local bounding box for updating ESDF
    //tmp表示当前射线追踪点的位置坐标 
    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));
    //比较点云的最大位置 和 相机的位置, min_x取二者的最大值
    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));
    max_z = max(max_z, mp_.ground_height_);
    //将位置坐标转换为栅格坐标,重置更新栅格的范围
    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

    int esdf_inf = ceil(mp_.local_bound_inflate_ / mp_.resolution_);
    md_.local_bound_max_ += esdf_inf * Eigen::Vector3i(1, 1, 0);
    md_.local_bound_min_ -= esdf_inf * Eigen::Vector3i(1, 1, 0);
    //确保待更新栅格均在地图内.若不在,取地图边界值予以替代
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.local_updated_ = true;

    // update occupancy cached in queue
    // update occupancy cached(已缓存) in queue
    Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
    Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

    while (!md_.cache_voxel_.empty()) {

        Eigen::Vector3i idx = md_.cache_voxel_.front();
        int idx_ctns = toAddress(idx);
        md_.cache_voxel_.pop();
    //若判定占据,log_odds_update =mp_.prob_hit_log_;若判定空闲,log_odds_update =mp_.prob_miss_log_
        double log_odds_update =
            md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ?
            mp_.prob_hit_log_ :
            mp_.prob_miss_log_;

        md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;
    //判定占据
        if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_) {
        continue;
        //判定空闲
        } else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_) {
        md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
        continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
            idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
        if (!in_local) {
        //不在局部更新范围,将体素设定为空闲
        md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
        }
    //在局部更新范围,求得正确的占据概率
        md_.occupancy_buffer_[idx_ctns] =
            std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                    mp_.clamp_max_log_);
    }
    }


    // 把地图范围外的点pt拉到地图边缘
    Eigen::Vector3d SDFMap::closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) {
    Eigen::Vector3d diff = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
    Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

    double min_t = 1000000;

    for (int i = 0; i < 3; ++i) {
        if (fabs(diff[i]) > 0) {

        double t1 = max_tc[i] / diff[i];
        if (t1 > 0 && t1 < min_t) min_t = t1;

        double t2 = min_tc[i] / diff[i];
        if (t2 > 0 && t2 < min_t) min_t = t2;
        }
    }

    return camera_pt + (min_t - 1e-3) * diff;
    }


    /*这一函数首先将局部范围外一圈的点的occupancy_buffer对应值置为：mp_.clamp_min_log_ - mp_.unknown_flag_。
    然后将局部地图范围内的地图上一轮的occupancy_buffer_inflate值全部置为0；
    紧接着，对局部地图的occupancy_buffer中所有点的值进行一一判断，判断是否超过为障碍物的最低概率mp_.min_occupancy_log_，如若判断，就对该点进行膨胀，并将所有膨胀点的occupancy_buffer_inflate值全部置为1；*/
    // 构造膨胀地图，去除局部规划范围外的地图，并将障碍物按无人机的大小膨胀（参数mp_.obstacles_inflation_），最后加入虚拟屋顶（参数mp_.virtual_ceil_height_），限制飞行高度。
    void SDFMap::clearAndInflateLocalMap() {
    /*clear outside local*/
    const int vec_margin = 5;//栅格级别
    // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
    // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
    // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    //local_bound_min_表示更新栅格的范围,local_map_margin_ = 10 表示局部栅格地图的更新和清除
    Eigen::Vector3i min_cut = md_.local_bound_min_ -
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    boundIndex(min_cut_m);
    boundIndex(max_cut_m);

    // clear data outside the local range(设为空闲)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

        for (int z = min_cut_m(2); z < min_cut(2); ++z) {
            int idx = toAddress(x, y, z);
            // md_.occupancy_buffer_[idx] =-1.99243-0.01=-2.00243 < mp_.clamp_min_log_=-1.99243.相当于是设为空闲
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }

        for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
            int idx = toAddress(x, y, z);
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }
        }

    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

        for (int y = min_cut_m(1); y < min_cut(1); ++y) {
            int idx = toAddress(x, y, z);
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }

        for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
            int idx = toAddress(x, y, z);
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }
        }

    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) { 

        for (int x = min_cut_m(0); x < min_cut(0); ++x) {
            int idx = toAddress(x, y, z);
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }

        for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
            int idx = toAddress(x, y, z);
            md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
            md_.distance_buffer_all_[idx] = 10000;
        }
        }

    // inflate occupied voxels to compensate robot size
    //obstacles_inflation_=0.099,resolution_=0.1,ceil向上取整，inf_step=1.0
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    // int inf_step_z = 1;
    vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));//inf_pts是27行1列的容
    // inf_pts.resize(4 * inf_step + 3);
    Eigen::Vector3i inf_pt;

    // clear outdated data
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
            md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
        }

    // inflate obstacles
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z) {
            //thresh log = mp_.min_occupancy_log_ = 1.38629
        /*  hit: 0.619039
            miss: -0.619039
            min log: -1.99243
            max: 2.19722
            thresh log: 1.38629  */
            //判定为占据
            if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_) {
            //Eigen::Vector3i(x, y, z)栅格级别,左右各膨胀1*0.1=0.1m,膨胀后的栅格点的坐标存在inf_pts中
            inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

            for (int k = 0; k < (int)inf_pts.size(); ++k) {
                inf_pt = inf_pts[k];
                int idx_inf = toAddress(inf_pt);
                if (idx_inf < 0 ||
                    idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) {
                continue;
                }
                md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
            }
        }

    // add virtual ceiling to limit flight height
    //mp_.virtual_ceil_height_默认值=3.0, mp_.map_origin_(2) = mp_.ground_height_=-0.01
    if (mp_.virtual_ceil_height_ > -0.5) {
        int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);// ceil_id = 30 是栅格级别,相当于30*0.1=3m
        for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
            md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
        }
    }
    }


    /*首先将地图范围外一圈的点的occupancy_buffer对应值置为：mp_.clamp_min_log_ - mp_.unknown_flag_；
    然后将地图范围内的地图上一轮的occupancy_buffer_inflate值全部置为0；
    紧接着，对局部地图的occupancy_buffer中所有点的值进行一一判断，判断是否超过为障碍物的最低概率mp_.min_occupancy_log_，如若判断，就对该点进行膨胀，并将所有膨胀点的occupancy_buffer_inflate值全部置为1；*/
    //构造全局膨胀地图，去除局部规划范围外的地图，并将障碍物按无人机的大小膨胀（参数mp_.obstacles_inflation_），最后加入虚拟屋顶（参数mp_.virtual_ceil_height_），限制飞行高度。
    void SDFMap::clearAndInflateGlobalMap() {
        /*clear outside local*/
        const int vec_margin = 5;//栅格级别
        // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
        // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
        // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
        //local_bound_min_表示更新栅格的范围,local_map_margin_ = 10 表示局部栅格地图的更新和清除
        Eigen::Vector2i min_cut = md_.local_bound_min_ -
            Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
        Eigen::Vector2i max_cut = md_.local_bound_max_ +
            Eigen::Vector2i(mp_.local_map_margin_, mp_.local_map_margin_);
        boundIndex(min_cut);
        boundIndex(max_cut);

        Eigen::Vector2i min_cut_m = min_cut - Eigen::Vector2i(vec_margin, vec_margin);
        Eigen::Vector2i max_cut_m = max_cut + Eigen::Vector2i(vec_margin, vec_margin);
        boundIndex(min_cut_m);
        boundIndex(max_cut_m);

        // clear data outside the local range(设为空闲)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
            for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {

            for (int z = min_cut_m(2); z < min_cut(2); ++z) {
                int idx = toAddress(x, y, z);
                // md_.occupancy_buffer_[idx] =-1.99243-0.01=-2.00243 < mp_.clamp_min_log_=-1.99243.相当于是设为空闲
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
            for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {

            for (int y = min_cut_m(1); y < min_cut(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
            for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) { 

            for (int x = min_cut_m(0); x < min_cut(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }

            for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
                int idx = toAddress(x, y, z);
                md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
                md_.distance_buffer_all_[idx] = 10000;
            }
        }

        //inflate occupied voxels to compensate robot size 膨胀占据栅格，使得小车可以视作质点
        //obstacles_inflation_=0.099,resolution_=0.1,ceil向上取整，inf_step=1.0
        int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
        //创建了一个名为inf_pts（膨胀点）的std::vector<Eigen::Vector2i>对象，其长度为(2 * inf_step + 1) ^ 2，即2 * inf_step + 1个元素的平方
        //每个元素都是一个Eigen::Vector2i类型的向量，表示二维空间中的一个整数坐标点
        vector<Eigen::Vector2i> inf_pts(pow(2 * inf_step + 1, 2));
        Eigen::Vector2i inf_pt;

        // clear outdated data 清除过时数据,全局更新只更新一次，故不需要该步骤
        // for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
        //     for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        //         md_.occupancy_buffer_inflate_[toAddress(x, y)] = 0;
        //     }

        // inflate obstacles 膨胀障碍
        for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
            for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
            //thresh log = mp_.min_occupancy_log_ = 1.38629
            /*  hit: 0.619039
                miss: -0.619039
                min log: -1.99243
                max: 2.19722
                thresh log: 1.38629  */
                //判定为占据
                if (md_.occupancy_buffer_[toAddress(x, y)] > mp_.min_occupancy_log_) {
                    //Eigen::Vector3i(x, y, z)栅格级别,左右各膨胀1*0.1=0.1m,膨胀后的栅格点的坐标存在inf_pts中
                    inflatePoint(Eigen::Vector2i(x, y), inf_step, inf_pts);

                    for (int k = 0; k < (int)inf_pts.size(); ++k) {
                        inf_pt = inf_pts[k];
                        int idx_inf = toAddress(inf_pt);
                        if (idx_inf < 0 ||
                            idx_inf >= mp_.map_grid_num_(0) * mp_.map_grid_num_(1)) {
                        continue;
                        }
                        md_.occupancy_buffer_inflate_[idx_inf] = 1;
                    }
                }
            }
    }


    // 发布（膨胀）栅格地图
    void SDFMap::visCallback(const ros::TimerEvent& /*event*/) {
    publishMap();
    publishMapInflate(false);
    // publishUpdateRange();
    // publishESDF();

    // publishUnknown();
    // publishDepth();
    }


    /*这是最终要的一个定时器回调函数，地图节点通过这一回调函数定时更新地图。
    其中有两个重要的flag：md.occ_need_update，只有接收到新图像且位于地图范围之内，才会进行接下来的projectDepthImage()及raycastProcess()这两个流程;
    另外一个flag是md.local_updated,这一flag只在raycastProcess中判断深度图投影点数量不为零时才会置为true，这时才会进入clearAndInflateLocalMap（）这一流程，对局部地图进行膨胀和更新。*/
    //更新栅格地图
    /*如果深度相机的位姿和图像更新了则需要更新栅格地图。调用projectDepthImage()将点云转换到世界坐标系下，raycastProcess()根据概率更新栅格地图和clearAndInflateLocalMap()构造膨胀地图 。*/
    void SDFMap::updateOccupancyCallback(const ros::TimerEvent& /*event*/) {
    if (!md_.occ_need_update_) return;

    /* update occupancy */
    ros::Time t1, t2;
    t1 = ros::Time::now();

    projectDepthImage();
    raycastProcess();

    if (md_.local_updated_) clearAndInflateLocalMap();

    t2 = ros::Time::now();

    md_.fuse_time_ += (t2 - t1).toSec();
    md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

    if (mp_.show_occ_time_)
        ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
                md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

    md_.occ_need_update_ = false;
    if (md_.local_updated_) md_.esdf_need_update_ = true;
    md_.local_updated_ = false;
    }


    //更新ESDF地图
    void SDFMap::updateESDFCallback(const ros::TimerEvent& /*event*/) {
    if (!md_.esdf_need_update_) return;

    /* esdf */
    ros::Time t1, t2;
    t1 = ros::Time::now();

    //核心函数：更新三维环境中的距离场（Distance Field）信息
    updateESDF3d();

    t2 = ros::Time::now();

    md_.esdf_time_ += (t2 - t1).toSec();
    md_.max_esdf_time_ = max(md_.max_esdf_time_, (t2 - t1).toSec());//最大更新时间

    if (mp_.show_esdf_time_)
        ROS_WARN("ESDF: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
                md_.esdf_time_ / md_.update_num_, md_.max_esdf_time_);//显示平均更新时间和最大更新时间

    md_.esdf_need_update_ = false;
    }


    /*这一函数通过message_filter类接收同步后的最新的相机Pose与深度图，同时，如果相机的位置处于全局地图Map_size之外，则就会将md.occ_need_update这一flag置false，反之置为true。*/
    /*保存深度相机信息到md_.depth_image_ ，保存相机位姿到md_.camera_pos_和md_.camera_r_m_ 。如果相机在地图范围内，则md_.occ_need_update_=true，表示需要更新栅格地图。最后令md_.flag_use_depth_fusion=true 。*/
    void SDFMap::depthPoseCallback(const sensor_msgs::ImageConstPtr& img,
                                const geometry_msgs::PoseStampedConstPtr& pose) {
    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(md_.depth_image_);

    // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

    /* get pose */
    md_.camera_pos_(0) = pose->pose.position.x;
    md_.camera_pos_(1) = pose->pose.position.y;
    md_.camera_pos_(2) = pose->pose.position.z;
    md_.camera_q_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                        pose->pose.orientation.y, pose->pose.orientation.z);
    if (isInMap(md_.camera_pos_)) {
        md_.has_odom_ = true;
        md_.update_num_ += 1;
        md_.occ_need_update_ = true;
    } else {
        md_.occ_need_update_ = false;
    }
    }

    // 如果还没有收到第一份深度信息，则把相机位置保存在md_.camera_pos_
    void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr& odom) {
    if (md_.has_first_depth_) return;

    md_.camera_pos_(0) = odom->pose.pose.position.x;
    md_.camera_pos_(1) = odom->pose.pose.position.y;
    md_.camera_pos_(2) = odom->pose.pose.position.z;

    md_.has_odom_ = true;
    }


    /*对于得到的点云数据，若该点在规划半径内并且有里程计信息和相机位置，那么直接认为他为障碍物，更新膨胀地图并设置虚拟屋顶。*/
    void SDFMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& img) {

    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*img, latest_cloud);//将ROS点云消息类型转换为PCL标准库中点云消息类型

    md_.has_cloud_ = true;

    if (!md_.has_odom_) {
        // std::cout << "no odom!" << std::endl;
        return;
    }

    if (latest_cloud.points.size() == 0) return;

    if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2))) return;

    //md_.occupancy_buffer_inflate_[pos] = 0;
    this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                        md_.camera_pos_ + mp_.local_update_range_);

    pcl::PointXYZ pt;
    Eigen::Vector3d p3d, p3d_inf;

    //obstacles_inflation_=0.099,resolution_=0.1,ceil向上取整，inf_step=1.0
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;

    double max_x, max_y, max_z, min_x, min_y, min_z;
    //map_max_boundary,map_min_boundary表示地图的位置（pos）范围
    min_x = mp_.map_max_boundary_(0);
    min_y = mp_.map_max_boundary_(1);
    min_z = mp_.map_max_boundary_(2);

    max_x = mp_.map_min_boundary_(0);
    max_y = mp_.map_min_boundary_(1);
    max_z = mp_.map_min_boundary_(2);

    for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

        /* point inside update range */
        Eigen::Vector3d devi = p3d - md_.camera_pos_;
        Eigen::Vector3i inf_pt;
        //mp_.local_update_range_x=5.5,y=5.5,z=4.5
        if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2)) {

        /* inflate the point */
        for (int x = -inf_step; x <= inf_step; ++x)
            for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step_z; z <= inf_step_z; ++z) {

                p3d_inf(0) = pt.x + x * mp_.resolution_;
                p3d_inf(1) = pt.y + y * mp_.resolution_;
                p3d_inf(2) = pt.z + z * mp_.resolution_;
    //第一次进入循环:相当于max_x = p3d_inf(0)
                max_x = max(max_x, p3d_inf(0));
                max_y = max(max_y, p3d_inf(1));
                max_z = max(max_z, p3d_inf(2));
    //第一次进入循环:膨胀后的点云的位置与地图最大位置边界---二者取最小(相当于min_x = p3d_inf(0))
                min_x = min(min_x, p3d_inf(0));
                min_y = min(min_y, p3d_inf(1));
                min_z = min(min_z, p3d_inf(2));

                posToIndex(p3d_inf, inf_pt);

                if (!isInMap(inf_pt)) continue;

                int idx_inf = toAddress(inf_pt);
    //膨胀之后的占据栅格，一个占据点云左右各膨胀0.1(一个栅格)
                md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
        }
        //三层for循环结束后,max_x的值代表膨胀后的点云的最大位置坐标,
        //min_x的值代表膨胀后的点云的最小位置坐标
    }
    //比较膨胀后的点云的最小位置 和 相机的位置, min_x取二者的最小值
    min_x = min(min_x, md_.camera_pos_(0));
    min_y = min(min_y, md_.camera_pos_(1));
    min_z = min(min_z, md_.camera_pos_(2));
    //比较膨胀后的点云的最大位置 和 相机的位置, max_x取二者的最大值
    max_x = max(max_x, md_.camera_pos_(0));
    max_y = max(max_y, md_.camera_pos_(1));
    max_z = max(max_z, md_.camera_pos_(2));

    max_z = max(max_z, mp_.ground_height_);
    //重置更新栅格的范围
    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
    //确保待更新栅格均在地图内.若不在,取地图边界值予以替代
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);

    md_.esdf_need_update_ = true;
    }


    /*这两函数分别对occupancy_buffer及occupancy_buffer_inflate容器中局部地图范围内的所有点进行判断，若值分别超过障碍物最小概率及为1，且不超过高度范围，则将其从voxel序列还原成三维位置点，推入cloud容器中，最后一并发布。*/
    // 发布栅格地图，将栅格地图md_.occupancy_buffer_转化为点云发布
    void SDFMap::publishMap() {
    // pcl::PointXYZ pt;
    // pcl::PointCloud<pcl::PointXYZ> cloud;

    // Eigen::Vector3i min_cut = md_.local_bound_min_ -
    //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    // Eigen::Vector3i max_cut = md_.local_bound_max_ +
    //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);

    // boundIndex(min_cut);
    // boundIndex(max_cut);

    // for (int x = min_cut(0); x <= max_cut(0); ++x)
    //   for (int y = min_cut(1); y <= max_cut(1); ++y)
    //     for (int z = min_cut(2); z <= max_cut(2); ++z) {

    //       if (md_.occupancy_buffer_[toAddress(x, y, z)] <= mp_.min_occupancy_log_) continue;

    //       Eigen::Vector3d pos;
    //       indexToPos(Eigen::Vector3i(x, y, z), pos);
    //       if (pos(2) > mp_.visualization_truncate_height_) continue;

    //       pt.x = pos(0);
    //       pt.y = pos(1);
    //       pt.z = pos(2);
    //       cloud.points.push_back(pt);
    //     }

    // cloud.width = cloud.points.size();
    // cloud.height = 1;
    // cloud.is_dense = true;
    // cloud.header.frame_id = mp_.frame_id_;

    // sensor_msgs::PointCloud2 cloud_msg;
    // pcl::toROSMsg(cloud, cloud_msg);
    // map_pub_.publish(cloud_msg);

    // ROS_INFO("pub map");

    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    int lmm = mp_.local_map_margin_ / 2;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {
            if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > mp_.visualization_truncate_height_) continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_pub_.publish(cloud_msg);
    }


    // 发布膨胀栅格地图，将膨胀栅格地图md_.occupancy_buffer_inflate_ 转化为点云发布
    void SDFMap::publishMapInflate(bool all_info) {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    if (all_info) {
        int lmm = mp_.local_map_margin_;//lmm=10,栅格级别
        min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
        max_cut += Eigen::Vector3i(lmm, lmm, lmm);
    }

    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {
            //判定为空闲
            if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0) continue;

            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > mp_.visualization_truncate_height_) continue;//visualization_truncate_height_ = 2.9,可视化的最大高度的障碍

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;

    pcl::toROSMsg(cloud, cloud_msg);
    map_inf_pub_.publish(cloud_msg);

    // ROS_INFO("pub map");
    }

    void SDFMap::publishUnknown() {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    Eigen::Vector3i min_cut = md_.local_bound_min_;
    Eigen::Vector3i max_cut = md_.local_bound_max_;

    boundIndex(max_cut);
    boundIndex(min_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y)
        for (int z = min_cut(2); z <= max_cut(2); ++z) {

            if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3) {
            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, z), pos);
            if (pos(2) > mp_.visualization_truncate_height_) continue;

            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud.push_back(pt);
            }
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;

    // auto sz = max_cut - min_cut;
    // std::cout << "unknown ratio: " << cloud.width << "/" << sz(0) * sz(1) * sz(2) << "="
    //           << double(cloud.width) / (sz(0) * sz(1) * sz(2)) << std::endl;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    unknown_pub_.publish(cloud_msg);
    }

    void SDFMap::publishDepth() {
    pcl::PointXYZ pt;
    pcl::PointCloud<pcl::PointXYZ> cloud;

    for (int i = 0; i < md_.proj_points_cnt; ++i) {
        pt.x = md_.proj_points_[i][0];
        pt.y = md_.proj_points_[i][1];
        pt.z = md_.proj_points_[i][2];
        cloud.push_back(pt);
    }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    depth_pub_.publish(cloud_msg);
    }

    void SDFMap::publishUpdateRange() {
    Eigen::Vector3d esdf_min_pos, esdf_max_pos, cube_pos, cube_scale;
    visualization_msgs::Marker mk;
    indexToPos(md_.local_bound_min_, esdf_min_pos);
    indexToPos(md_.local_bound_max_, esdf_max_pos);

    cube_pos = 0.5 * (esdf_min_pos + esdf_max_pos);
    cube_scale = esdf_max_pos - esdf_min_pos;
    mk.header.frame_id = mp_.frame_id_;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.id = 0;

    mk.pose.position.x = cube_pos(0);
    mk.pose.position.y = cube_pos(1);
    mk.pose.position.z = cube_pos(2);

    mk.scale.x = cube_scale(0);
    mk.scale.y = cube_scale(1);
    mk.scale.z = cube_scale(2);

    mk.color.a = 0.3;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    mk.pose.orientation.w = 1.0;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;

    update_range_pub_.publish(mk);
    }

    void SDFMap::publishESDF() {
    double dist;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointXYZI pt;

    const double min_dist = 0.0;
    const double max_dist = 3.0;

    Eigen::Vector3i min_cut = md_.local_bound_min_ -
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    Eigen::Vector3i max_cut = md_.local_bound_max_ +
        Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
    boundIndex(min_cut);
    boundIndex(max_cut);

    for (int x = min_cut(0); x <= max_cut(0); ++x)
        for (int y = min_cut(1); y <= max_cut(1); ++y) {

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, 1), pos);
        pos(2) = mp_.esdf_slice_height_;

        dist = getDistance(pos);
        dist = min(dist, max_dist);
        dist = max(dist, min_dist);

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = -0.2;
        pt.intensity = (dist - min_dist) / (max_dist - min_dist);
        cloud.push_back(pt);
        }

    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);

    esdf_pub_.publish(cloud_msg);

    // ROS_INFO("pub esdf");
    }

    void SDFMap::getSliceESDF(const double height, const double res, const Eigen::Vector4d& range,
                            vector<Eigen::Vector3d>& slice, vector<Eigen::Vector3d>& grad, int sign) {
    double dist;
    Eigen::Vector3d gd;
    for (double x = range(0); x <= range(1); x += res)
        for (double y = range(2); y <= range(3); y += res) {

        dist = this->getDistWithGradTrilinear(Eigen::Vector3d(x, y, height), gd);
        slice.push_back(Eigen::Vector3d(x, y, dist));
        grad.push_back(gd);
        }
    }

    void SDFMap::checkDist() {
    for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
        for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
        for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
            Eigen::Vector3d pos;
            indexToPos(Eigen::Vector3i(x, y, z), pos);

            Eigen::Vector3d grad;
            double dist = getDistWithGradTrilinear(pos, grad);

            if (fabs(dist) > 10.0) {
            }
        }
    }

    bool SDFMap::odomValid() { return md_.has_odom_; }

    bool SDFMap::hasDepthObservation() { return md_.has_first_depth_; }

    double SDFMap::getResolution() { return mp_.resolution_; }

    Eigen::Vector3d SDFMap::getOrigin() { return mp_.map_origin_; }

    int SDFMap::getVoxelNum() {
    return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
    }

    void SDFMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) {
    ori = mp_.map_origin_, size = mp_.map_size_;
    }

    void SDFMap::getSurroundPts(const Eigen::Vector3d& pos, Eigen::Vector3d pts[2][2][2],
                                Eigen::Vector3d& diff) {
    if (!isInMap(pos)) {
        // cout << "pos invalid for interpolation." << endl;
    }

    /* interpolation position */
    Eigen::Vector3d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector3d::Ones();
    Eigen::Vector3i idx;
    Eigen::Vector3d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos); // 求插值的下界点
    diff = (pos - idx_pos) * mp_.resolution_inv_;

    // 计算待求点(x, y, z)周围的8个点的pos(xi, yi, zi)
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
        for (int z = 0; z < 2; z++) {
            Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
            Eigen::Vector3d current_pos;
            indexToPos(current_idx, current_pos);
            pts[x][y][z] = current_pos;// C000, C001, C010, C011, C100, C101, C110, C111
        }
        }
    }
    }


    /*和depthPoseCallback() 差不多，只不过收到的是机体在世界系下的坐标，多了一步使用md_.cam2body_ 得到相机在世界系下的坐标。*/
    void SDFMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                                const nav_msgs::OdometryConstPtr& odom) {
    /* get pose */
    md_.camera_pos_(0) = odom->pose.pose.position.x;
    md_.camera_pos_(1) = odom->pose.pose.position.y;
    md_.camera_pos_(2) = odom->pose.pose.position.z;
    md_.camera_q_ = Eigen::Quaterniond(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                                        odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    /* get depth image */
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
    }
    cv_ptr->image.copyTo(md_.depth_image_);

    md_.occ_need_update_ = true;
    }

    void SDFMap::depthCallback(const sensor_msgs::ImageConstPtr& img) {
    std::cout << "depth: " << img->header.stamp << std::endl;
    }

    void SDFMap::poseCallback(const geometry_msgs::PoseStampedConstPtr& pose) {
    std::cout << "pose: " << pose->header.stamp << std::endl;

    md_.camera_pos_(0) = pose->pose.position.x;
    md_.camera_pos_(1) = pose->pose.position.y;
    md_.camera_pos_(2) = pose->pose.position.z;
    }

}
// SDFMap
