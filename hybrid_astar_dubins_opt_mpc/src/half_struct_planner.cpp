#include "half_struct_planner.h"


namespace chibot::topologyplanning {

  //成员初始化列表用于在构造函数开始执行之前对类的成员变量进行初始化。成员初始化列表位于构造函数的参数列表后面，使用冒号分隔，用于给成员变量赋初值
  
  /*如果HalfStructPlanner的构造函数需要一个nav_msgs::OccupancyGrid对象，那么最好的方式是让它接受一个nav_msgs::OccupancyGridConstPtr类型的参数，
    而不是nav_msgs::OccupancyGrid或nav_msgs::OccupancyGrid&。这样，你就可以避免在调用构造函数时复制整个nav_msgs::OccupancyGrid对象。
    这样，HalfStructPlanner就会持有一个指向nav_msgs::OccupancyGrid的指针，而不是整个对象。当你调用half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg);时，不会发生任何拷贝，只是将msg中的指针复制到HalfStructPlanner的内部。
    注意，这种方式假定msg中的nav_msgs::OccupancyGrid对象在HalfStructPlanner的生命周期中一直有效。如果原始对象被销毁或修改，HalfStructPlanner中的指针可能会变得无效。如果你不能保证这一点，那么你可能需要在HalfStructPlanner的构造函数中复制nav_msgs::OccupancyGrid对象。*/
  HalfStructPlanner::HalfStructPlanner(const nav_msgs::OccupancyGridConstPtr& grid):
  // nav_msgs::OccupancyGrid const& grid
  // const nav_msgs::OccupancyGridConstPtr& msg
  // const nav_msgs::OccupancyGrid::Ptr map
    grid_(grid), 
    node_num_(0),
    graph_(nullptr),
    ContactS_paths(EXTRA_POINTS_NUM),
    ContactG_paths(EXTRA_POINTS_NUM),
    ContactS_paths_(EXTRA_POINTS_NUM),
    ContactG_paths_(EXTRA_POINTS_NUM),
    ContactSG_paths(EXTRA_POINTS_NUM),
    graph_bk_(nullptr) {
    // {
    // grid_ = grid;
    hybrid_astar_ptr.reset(new HybridAstar(grid));

    //create the voronoi object and initialize it with the map
    //生成voronoi图
    int size_x = grid_->info.width;
    int size_y = grid_->info.height;
    //初始化一个二进制地图，建voronoi图用
    bool **bin_map = NULL;//创建了一个size_x * size_y的布尔类型二维数组bin_map
    bin_map = new bool *[size_x];
    for (int i = 0; i < size_x; ++i) {//列索引
        bin_map[i] = new bool[size_y];
    }
    for (int j=size_y-1; j>=0; j--) {
        for (int i=0; i<size_x; i++) {
            auto index_ = j * grid_->info.width +i;//由行列索引得栅格索引
            //障碍栅格和未知栅格都作为障碍栅格
            if (grid_->data[index_] == 100 || grid_->data[index_] == -1) {
                bin_map[i][j] = true;//表示该位置是障碍物
            } else {
                bin_map[i][j] = false;
            }
        }
    }
    //最终，该二维数组bin_map记录了地图上每个单元格是否可以通行
    voronoiDiagram.initializeMap(size_x, size_y, bin_map);//初始化voronoi图
    voronoiDiagram.update();// update distance map and Voronoi diagram 更新voronoi图
    voronoiDiagram.prune();//剪枝
   


    // voronoiDiagram.visualize("../result.pgm");   
    // cout << "voronoi图可视化" << std::endl;
    // ROS_INFO("voronoi");

    // voronoi.header.frame_id = "map";
    // voronoi.header.stamp = ros::Time();
    // voronoi.ns = "voronoi";
    // voronoi.id = 0;
    // voronoi.type = visualization_msgs::Marker::SPHERE_LIST;
    // voronoi.action = visualization_msgs::Marker::ADD;
    // voronoi.color.b = 1.0;
    // voronoi.color.a = 1.0;
    // voronoi.scale.x = 0.1;
    // voronoi.scale.y = 0.1;
    // voronoi.scale.z = 0.1;
    // voronoi.pose.orientation.w = 1.0;

    // geometry_msgs::Point p;
    // for (int j = size_y-1; j >= 0; --j) {
    //     for (int i = 0; i < size_x; ++i) {
    //         if (voronoiDiagram.isVoronoi(i, j)) {//voronoi图上的节点
    //             Vec2d pos;
    //             pos(0) = grid_.info.origin.position.x + (i + 0.5) * grid_.info.resolution;
    //             pos(1) = grid_.info.origin.position.y + (j + 0.5) * grid_.info.resolution;
    //             p.x = pos(0);
    //             p.y = pos(1);
    //             p.z = 0.05;
    //             voronoi.points.push_back(p);
    //         }
    //     }
    // }
  }

  //析构函数
  HalfStructPlanner::~HalfStructPlanner() {
    if (node_num_ == 0) return;
    for (auto i = 0; i < node_num_; ++i) {
  /*delete [] graph_[i];是一个用于释放动态分配的内存的代码行。
    根据代码上下文来看，graph_是一个二维数组，并且它的每一行都指向动态分配的内存块。通过delete [] graph_[i];代码对第i行所指向的内存块进行释放操作。
    在C++中，使用new运算符来进行内存分配，而使用delete运算符来进行内存释放。当我们使用new运算符动态分配内存时，需要使用对应的delete运算符来释放这些内存，以避免内存泄漏。
    在这个特定的代码行中，delete [] graph_[i];表示释放第i行所指向的内存块。[]表示释放的是一块连续的内存，而不仅仅是单个对象。这个语法适用于数组或指向动态分配的内存块的指针。
    需要注意的是，在执行delete之前，我们必须确保该内存块已经被动态分配，并且没有被重复释放。否则，可能会导致未定义的行为或程序崩溃。*/
      delete [] graph_[i];
      delete [] graph_bk_[i];
    }
  /*delete [] graph_;是用于释放一个动态分配的二维数组所占用的内存的代码行。
    根据代码上下文，graph_应该是一个指向动态分配的二维数组的指针。通过delete [] graph_;代码，释放了整个二维数组所占用的内存。
    在C++中，使用new运算符进行动态内存分配时，我们需要使用对应的delete运算符来释放这些内存以避免内存泄漏。对于二维数组，我们需要使用两次delete []来释放内存，一次释放每一行的内存，然后再释放指向行指针的内存。*/
    delete [] graph_;
    delete [] graph_bk_;
  };

  //初始化
  void HalfStructPlanner::init() {
    ros::NodeHandle nh("/");
    vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>("traffic_route", 1);//rviz可视化半结构道路路网
    res_pub_ = nh.advertise<nav_msgs::Path>("traffic_route_points", 1);//发布路网上的最短路径，一系列位姿点，可视化位姿点
   
    //可视化优化前的曲线，混合A*规划出来的路线
    path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("db_path_marker", 10);
    //可视化优化前的曲线路点
    path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("db_path_point_marker", 10);
    //可视化平滑后的路径
    optimized_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("/path", 10);

    //voronoiDiagram_ptr.reset(new DynamicVoronoi());
    smoother_ptr.reset(new Smoother());//初始化平滑器

    //可视化voronoi图
    // voronoi_pub = nh.advertise<visualization_msgs::Marker>("voronoi", 10);

    //可视化起终点的路线
    SG_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("SG_path_marker", 100);
    //可视化拓扑
    topology_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("topology_path_marker", 100);
    //完整的一条备选拓扑路线，选不选它还得看它和直连的长度对比
    full_path_vis_pub = nh.advertise<geometry_msgs::PoseArray>("full_path_marker", 100);
  }


  //设置交通规则（起终点段）
  void HalfStructPlanner::set_traffic_route(lines_type const& lines) {
    traffic_routes_ = lines;//包含起终点坐标、该条边可以连到哪些边等信息，一个traffic_route代表一条边（不包含拓扑）
    
    // voronoi_pub.publish(voronoi);//可视化维诺图
              
    //保存路线
    std::string root_path;
    //参数服务，值取自launch文件
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    //保存起终点间的路线
    auto dir = root_path + "/path/";
    auto file_path_ = dir + name;

    //对各line进行规划  
    //先把所有生成的路径保存下来，再一起可视化
    vector<vector<Vec3d>> astarpath;//未优化的路线集
    vector<vector<Vec3d>> astarpath_;//优化的路线集

    // Visualize vis;
    for (const auto& traffic_route_ : traffic_routes_) {
      //由起终点连接的路线
      Vec3d start_pose, goal_pose;
      start_pose[0] = traffic_route_.a.x;
      start_pose[1] = traffic_route_.a.y;
      start_pose[2] = traffic_route_.a.theta;
      goal_pose[0] = traffic_route_.b.x;
      goal_pose[1] = traffic_route_.b.y;
      goal_pose[2] = traffic_route_.b.theta;
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth;
          path2smooth.clear();
          for (int i = 0; i < result.x.size(); ++i) {
              path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
          }
          astarpath.push_back(path2smooth);//保存优化前的路径
          
          //l(Limited-memory)-bfgs平滑
          smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
          vector<Vec3d> smooth_path;
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);

          write_file(file_path_, smooth_path, result.total_length);//将优化后的路线写入文件

          astarpath_.push_back(smooth_path);//保存优化后的路径
          
          // for (size_t i = 0; i < result.x.size(); ++i)
          // {
          //  //以平滑后的路径点做的车轮廓
          //  // vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);
          //  //以平滑前的路径点做的车轮廓
              // vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i);
          // }
      } else {
          ROS_ERROR("search fail");
          return;
      }   
    }
    visPath(astarpath);//可视化优化前路线
    visOptPath(astarpath_);//可视化优化后路线
  }


  //设置交通规则（拓扑段）
  void HalfStructPlanner::set_traffic_route_topology(lines_type const& lines) {
    traffic_routes_ = lines;//包含起终点坐标、该条边可以连到哪些边等信息，一个traffic_route代表一条边（不包含拓扑）
    
    // voronoi_pub.publish(voronoi);//可视化维诺图
              
    //保存路线
    std::string root_path;
    //参数服务，值取自launch文件
    ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << "path_" << std::put_time(std::localtime(&t), "%F %T");
    auto name = ss.str();
    //保存拓扑路线
    auto dir = root_path + "/topology_path/";
    auto file_path_ = dir + name;

    //对各line进行规划  
    //先把所有生成的路径保存下来，再一起可视化
    vector<vector<Vec3d>> astarpath;//未优化的路线集
    vector<vector<Vec3d>> astarpath_;//优化的路线集

    // Visualize vis;
    for (const auto& traffic_route_ : traffic_routes_) {
      Vec3d start_pose, goal_pose;
      //拓扑起点
      goal_pose[0] = traffic_route_.b.x;
      goal_pose[1] = traffic_route_.b.y;
      goal_pose[2] = traffic_route_.b.theta;

      //对拓扑
      if (traffic_route_.go_list.empty()) continue;
      for (auto const& line : traffic_route_.go_list) {
        if (line >= traffic_routes_.size()) continue;
        //拓扑终点
        start_pose[0] = traffic_routes_[line].a.x;
        start_pose[1] = traffic_routes_[line].a.y;
        start_pose[2] = traffic_routes_[line].a.theta;
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        if (hybrid_astar_ptr->plan(goal_pose, start_pose , result)) {
            vector<Vec3d> path2smooth_;
            path2smooth_.clear();
            // 摒弃拓扑端点，它们被包含在直连段中
            for (int i = 1; i < result.x.size()-1; ++i) {
                path2smooth_.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
            }
            astarpath.push_back(path2smooth_);//保存优化前路线
            //l(Limited-memory)-bfgs平滑
            smoother_ptr->optimize(voronoiDiagram, path2smooth_, grid_);
            //smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
            
            vector<Vec3d> smooth_path_;
            smooth_path_.clear();
            smoother_ptr->getSmoothPath(smooth_path_);

            write_file(file_path_, smooth_path_, result.total_length);//将优化后的拓扑路线写入文件

            astarpath_.push_back(smooth_path_);//保存优化后路线
        } else {
            ROS_ERROR("search fail");
            return;
        }       
      }
    }
    visPath(astarpath);
    visOptPath(astarpath_);

  }


  //获取首尾点所走路径上的位姿点
  auto HalfStructPlanner::get_path(geometry_msgs::PoseStamped const& start,
                                  geometry_msgs::PoseStamped const& goal) -> std::vector<geometry_msgs::PoseStamped> {
    // std::vector<Eigen::Vector3d> result_{};//保存拓扑段或直连段中的其中一种
    // std::vector<Eigen::Vector3d>* result_ = new std::vector<Eigen::Vector3d>();//保存拓扑段或直连段中的其中一种
    // std::vector<Eigen::Vector3d> result_plo{};//永远保存拓扑部分
    // std::vector<Eigen::Vector3d>* result_plo = new std::vector<Eigen::Vector3d>();
    std::vector<geometry_msgs::PoseStamped> result_p{};//以位姿表示的拓扑路线

    std::unique_ptr<std::vector<Eigen::Vector3d>> result_ = std::make_unique<std::vector<Eigen::Vector3d>>();
    std::unique_ptr<std::vector<Eigen::Vector3d>> result_plo = std::make_unique<std::vector<Eigen::Vector3d>>();

    if (!is_traffic_plan()) return result_p;//没有进行交通规则设置

    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        graph_[i][j] = graph_bk_[i][j];//代价矩阵
      }
    }
    ROS_INFO("4");
    // 0对应起点; 1对应终点
    nodes_[0] = start;
    nodes_[1] = goal;
    calculate_graph();//图拓扑和代价值更新，加入小车起终点后的拓扑，小车起点连到其接点，终点连到其接点
    //node_list存储路点标号
    ROS_INFO("6");
    auto node_list = dijkstra();//使用dijkstra算法进行规划，得到一条路径，其思想是根据距离代价值，只有路网上的代价，所以路线都在路网上
    node_list.emplace(node_list.begin(), 0);//把起点插入
    // ROS_INFO("5");
    nav_msgs::Path path;
    path.header.frame_id = "map";//父坐标
    path.header.stamp = ros::Time::now();

    double l_s=0.,l_p;
    // 输出连续的两个元素
    for (int i = 0; i < node_list.size() - 1; ++i) {
      // 拓扑总长度
      // 使用映射表
      try {
        std::pair<int, int> inputKey = {node_list[i], node_list[i+1]};
        //根据节点序号取出所有位姿构成一条完整路径
        const std::vector<Eigen::Vector3d>* sgPathPtr = mappingTable[inputKey]; // 获取指针
        // 直接拿指针
        result_plo->insert(result_plo->end(), sgPathPtr->begin(), sgPathPtr->end()); // 将 SG_paths 添加到 result_plo 的末尾
        
        // const std::vector<Eigen::Vector3d>& sgPathRef = *sgPathPtr; // 获取指针所指向的向量数据引用
        // result_plo.insert(result_plo.end(), sgPathRef.begin(), sgPathRef.end()); // 将 SG_paths 添加到 result_plo 的末尾
      
      } catch (const std::bad_alloc& ba) {  
        // 捕获 std::bad_alloc 异常  
        std::cerr << "Caught a bad_alloc exception: " << ba.what() << '\n';  
        // 在这里你可以采取一些恢复措施，比如释放其他内存、通知用户等  
      }  

      l_p += graph_[node_list[i]][node_list[i+1]];//累计各段长度
    }

    ROS_INFO("1");
    // 创建一个副本以保留 result_plo 中的原始数据
    std::vector<Eigen::Vector3d> result_plo_copy = *result_plo;
   // 使用移动语义将 result_plo 的内容转移给 result_
    *result_ = std::move(*result_plo);
    //看始末点直线距离，如果小于一个阈值，就不单独进行HA*规划了
    if(distance(start, goal) < 15. && line_safe(start, goal)){
      //允许起终点直连，后续通过规划得到其长度，和以上通过djstra搜索出的最短路线比较，得出最短路线
      Vec3d start_pose = Eigen::Vector3d {nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)};//起点
      Vec3d goal_pose = Eigen::Vector3d {nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};//终点
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      vector<Vec3d> smooth_path;
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth;
          path2smooth.clear();
          for (int i = 0; i < result.x.size(); ++i) {
              path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
          }
          //l(Limited-memory)-bfgs平滑
          smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);
      } 
      l_s = result.total_length;//直连总长度
      //取直连
      if(l_p > 3*l_s) {
        result_->clear();
        for (const auto& vec : smooth_path) {
          result_->push_back(Eigen::Vector3d(vec.x(), vec.y(), vec.z()));
        }
      }
    }
    // ROS_INFO("3");
    for (auto const& node : *result_) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      // p.header.stamp
      p.pose.position.x = node[0];
      p.pose.position.y = node[1];
      auto o = node[2];
      p.pose.orientation.z = std::sin(o / 2);//朝向
      p.pose.orientation.w = std::cos(o / 2);
      result_p.emplace_back(p);
    }    
    path.poses = result_p;
    // ROS_INFO("2");
    //显示位姿标红，这主要区分取的是直连和拓扑连
    visfullPath(result_plo_copy);

    res_pub_.publish(path);//发布路径，一系列位姿点，其实每条路线就拿一个点代表
    //显示该路径上的路点标号
    show_graph_(node_list);

    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    return result_p;
  }


  //获取小车起始位置最接近的接点，以该接点为起终点更新图  &：引用的方式可以修改引用的参数
  // auto HalfStructPlanner::get_nearest_point_of_start_and_update(geometry_msgs::PoseStamped const& start, geometry_msgs::PoseStamped& np) -> std::vector<geometry_msgs::PoseStamped> {
  //   std::vector<geometry_msgs::PoseStamped> result {};
  //   if (!is_traffic_plan()) return result;//没有进行交通规则设置

  //   // 0对应起点,这里表示小车起始位置
  //   nodes_[0] = start;
  //   np = get_nearest_point_of_start();//拿到离小车起始位置最近的接点

  //   for (auto i = 0; i < node_num_; ++i) {
  //     for (auto j = 0; j < node_num_; ++j) {
  //       graph_[i][j] = graph_bk_[i][j];//代价矩阵
  //     }
  //   }
  //   // 0对应起点; 1对应终点，起终点是同一个
  //   nodes_[0] = np;
  //   nodes_[1] = np;
  //   calculate_graph();//图拓扑和代价值更新，加入小车起终点后的拓扑，小车起点连到其接点，终点连到其接点
  //   auto node_list = dijkstra();//使用dijkstra算法进行规划，得到一条路径，其思想是根据距离代价值，只有路网上的代价，所以路线都在路网上
  //   for (auto const& node : node_list) {
  //     result.emplace_back(nodes_[node]);//取出这条路径上的节点 node节点序号   构成一系列位姿点
  //   }
  //   nav_msgs::Path path;
  //   path.header.frame_id = "map";//父坐标
  //   path.header.stamp = ros::Time::now();
  //   path.poses = result;
  // /*这段代码将nodes_[0]起点插入到path.poses容器的起始位置。
  //   具体解释如下：
  //   path.poses是一个容器
  //   emplace()函数用于在容器的指定位置插入新元素，并在插入位置处构造新对象。它接受参数包，用于构造新对象所需的参数。
  //   path.poses.begin()返回容器path.poses的起始位置的迭代器，即指向第一个元素的迭代器。
  //   nodes_[0]是要插入的新元素。
  //   通过调用emplace()函数，将nodes_[0]插入到path.poses容器的起始位置。
  //   这样做的效果是，在path.poses的起始位置插入了一个新元素nodes_[0]。原先在起始位置之前的元素都会被向后移动一个位置，腾出空间给新元素的插入。*/
  //   path.poses.emplace(path.poses.begin(), nodes_[0]);//把起点插入
  //   res_pub_.publish(path);//发布路径，一系列位姿点，其实每条路线就拿一个点代表

  //   return result;
  // }


  //判断有没有进行交通规则设置
  auto HalfStructPlanner::is_traffic_plan() -> bool {
    return !traffic_routes_.empty();
  }


  //可视化交通规则（路网）
  void HalfStructPlanner::show_traffic_route() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;//标记id号
    int serial = 0;//路网文本序号
    geometry_msgs::Point p1, p2;//表示箭头标记的起点和终点坐标
    geometry_msgs::Pose p;//表示文本标记的位姿
    /*这段代码是一个C++函数，用于生成一个包含箭头标记的 MarkerArray。MarkerArray 是 ROS 中的一种消息类型，用于在可视化工具中显示多个标记。
      在这段代码中，首先定义了一个 MarkerArray 对象 marker_array，以及两个整数变量 id 和 serial。
      然后定义了两个 geometry_msgs::Point 类型的变量 p1 和 p2，它们分别表示箭头标记的起点和终点坐标。
      通过使用 C++11 中的 lambda 表达式，定义了一个名为 make_arrow_marker 的匿名函数。该函数接受四个参数：r、g、b 分别表示红、绿、蓝三个颜色通道的值，p1 和 p2 表示箭头标记的起点和终点坐标。
      在函数内部，创建了一个 visualization_msgs::Marker 对象 marker，并设置了其各个属性，如标记类型为箭头 (ARROW)、操作为添加 (ADD)、尺寸为0.08x0.2x0.4、颜色透明度为0.5、颜色值为输入的 r、g、b，以及姿态的朝向为默认值。
      然后，通过调整 marker 的 points 属性的大小为2，并将起点和终点的坐标设置为输入的 p1 和 p2。
      最后，给 marker 设置了命名空间 (ns) 为 "arrow"，并将 id 递增后赋值给 marker.id。将 marker 添加到 marker_array.markers 中。
      这段代码的作用是生成一个带有箭头标记的 Marker，并将其添加到 MarkerArray 中。在 ROS 中，可以使用 MarkerArray 将多个标记一起发布，以在可视化工具中同时显示多个标记。*/
    //显示箭头
    //[&]是一个捕获列表，表示以引用的方式捕获所有外部变量。也就是说，在lambda函数体内部可以直接访问上下文中的所有变量
    auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;//箭头
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.3;
      marker.scale.y = 0.2;
      marker.scale.z = 0.4;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose.orientation.w = 1.0;
      marker.points.resize(2);
      marker.ns = "arrow";
      marker.id = id++;
      //箭头起终点坐标
      marker.points[0] = p1;
      marker.points[1] = p2;
      marker_array.markers.emplace_back(marker);
    };
    /*这段代码是C++函数中的另一个 lambda 表达式，用于创建包含文本标记的 Marker。与前面的函数类似，该函数接受四个参数：红、绿、蓝色通道值和 geometry_msgs::Pose 类型的变量 p，表示文本标记的位姿。
      在函数内部，首先创建了一个名为 marker 的 visualization_msgs::Marker 对象，并设置了其各种属性。其中 type 值为 TEXT_VIEW_FACING，表示文本标记会一直朝向相机，action 值为 ADD，表示添加新标记。该标记的 scale.z 值为 1.0，表示文本标记的高度为1个单位。
      接下来，该 marker 的颜色透明度为0.5，颜色值为输入的 r、g、b。标记的位姿 pose 采用输入的 p。命名空间 ns 命名为 "text"，并将 id 值递增后赋给 marker.id。最后，文本内容通过将 serial++ 转换为字符串设置到 marker.text 中。
      最后，将该 marker 对象添加到 marker_array.markers 容器中。这样，该函数将生成一个新的 Marker，其中包含了文本标记，并将该标记对象添加到 MarkerArray 中。在使用 ROS 的可视化工具查看 MarkerArray 时，会同时显示多个标记，包括箭头标记和文本标记。*/
    //显示数字标号，表示路径点号啦
    auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.3;//设置线条的宽度：0.3m
      marker.scale.y = 0.3;
      marker.scale.z = 1.0;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose = p;
      marker.ns = "text";
      marker.id = id++;
      marker.text = std::to_string(serial++);
      marker_array.markers.emplace_back(marker);
    };
    //路网拓扑（从一条边到另一条边）的可视化


    /*这段代码是一个范围基于范围的循环，用于遍历名为 traffic_routes_ 的容器（traffic_routes_ = lines;）中的元素。
    在循环的每一次迭代中，line 是 traffic_routes_ 中的当前元素的一个引用，可以通过 line 来访问该元素的值或属性。
    循环会自动遍历 traffic_routes_ 中的所有元素，并依次执行循环体中的代码块。在每次循环迭代中，你可以使用 line 来操作当前元素。
    这种循环语法使得遍历容器或者可迭代对象的元素变得更加简洁和易读。*/
    for (auto const& line : traffic_routes_) {//traffic_routes_包含起终点坐标、路网拓扑信息
      //起终点坐标
      p1.x = line.a.x;
      p1.y = line.a.y;
      p2.x = line.b.x;
      p2.y = line.b.y;
      //文本标记的坐标
      p.position.x = (p1.x + p2.x) / 2;
      p.position.y = (p1.y + p2.y) / 2;
      auto o = std::atan2(p2.y - p1.y, p2.x - p2.x);
      //朝向，四元数表示
      p.orientation.z = std::sin(o / 2);
      p.orientation.w = std::cos(o / 2);
      make_arrow_marker(0.0, 1.0, 0.0, p1, p2);//匿名函数，在上面被定义
      make_text_marker(0.0, 0.0, 0.0, p);

      // 在 for 循环中使用 return 语句的话，会导致函数立即返回，同时也会终止整个循环。这意味着，即使 for 循环并未遍历完所有的元素，return 语句的执行也会导致提前结束循环
      if (line.go_list.empty()) continue;//路网拓扑列表为空，继续循环
      //有路网拓扑，画某一边终点到另一条路线起点
      for (auto const& node : line.go_list) {
        if (node >= traffic_routes_.size()) continue;//拓扑边的数量都超过了原有的边了，非法
        //node对应原规划路线中第几条边（从0开始的），是边的序号，这个序号在那个路网文件里由上到下从0开始
        p1.x = traffic_routes_[node].a.x;//这里a表示被连接边的起点
        p1.y = traffic_routes_[node].a.y;
          /*全是直线的标记*/
        make_arrow_marker(0.0, 0.0, 1.0, p2, p1);
      }
    }
    vis_pub_.publish(marker_array);//rviz可视化发布路网及其拓扑
  }


  //可视化交通规则（路网），不含标号
  void HalfStructPlanner::show_traffic_route_() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;//标记id号
    int serial = 0;//路网文本序号
    int seq = 0;//判断是不是最后一个点
    bool flag;//圆弧朝向
    geometry_msgs::Point p1, p2;//表示箭头标记的起点和终点坐标
    geometry_msgs::Pose p;//表示文本标记的位姿
    /*这段代码是一个C++函数，用于生成一个包含箭头标记的 MarkerArray。MarkerArray 是 ROS 中的一种消息类型，用于在可视化工具中显示多个标记。
      在这段代码中，首先定义了一个 MarkerArray 对象 marker_array，以及两个整数变量 id 和 serial。
      然后定义了两个 geometry_msgs::Point 类型的变量 p1 和 p2，它们分别表示箭头标记的起点和终点坐标。
      通过使用 C++11 中的 lambda 表达式，定义了一个名为 make_arrow_marker 的匿名函数。该函数接受四个参数：r、g、b 分别表示红、绿、蓝三个颜色通道的值，p1 和 p2 表示箭头标记的起点和终点坐标。
      在函数内部，创建了一个 visualization_msgs::Marker 对象 marker，并设置了其各个属性，如标记类型为箭头 (ARROW)、操作为添加 (ADD)、尺寸为0.08x0.2x0.4、颜色透明度为0.5、颜色值为输入的 r、g、b，以及姿态的朝向为默认值。
      然后，通过调整 marker 的 points 属性的大小为2，并将起点和终点的坐标设置为输入的 p1 和 p2。
      最后，给 marker 设置了命名空间 (ns) 为 "arrow"，并将 id 递增后赋值给 marker.id。将 marker 添加到 marker_array.markers 中。
      这段代码的作用是生成一个带有箭头标记的 Marker，并将其添加到 MarkerArray 中。在 ROS 中，可以使用 MarkerArray 将多个标记一起发布，以在可视化工具中同时显示多个标记。*/
    //显示箭头
    //[&]是一个捕获列表，表示以引用的方式捕获所有外部变量。也就是说，在lambda函数体内部可以直接访问上下文中的所有变量
    auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time::now();
      marker.type = visualization_msgs::Marker::ARROW;//箭头
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = 0.08;
      marker.scale.y = 0.45;
      marker.scale.z = 0.8;
      marker.color.a = 0.5;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.pose.orientation.w = 1.0;
      marker.points.resize(2);
      marker.ns = "arrow";
      marker.id = id++;
      //箭头起终点坐标
      marker.points[0] = p1;
      marker.points[1] = p2;
      marker_array.markers.emplace_back(marker);
    };
    //路网拓扑（从一条边到另一条边）的可视化——圆弧标记

    /*这段代码是一个范围基于范围的循环，用于遍历名为 traffic_routes_ 的容器（traffic_routes_ = lines;）中的元素。
    在循环的每一次迭代中，line 是 traffic_routes_ 中的当前元素的一个引用，可以通过 line 来访问该元素的值或属性。
    循环会自动遍历 traffic_routes_ 中的所有元素，并依次执行循环体中的代码块。在每次循环迭代中，你可以使用 line 来操作当前元素。
    这种循环语法使得遍历容器或者可迭代对象的元素变得更加简洁和易读。*/
    for (auto const& line : traffic_routes_) {//traffic_routes_包含起终点坐标、路网拓扑信息
      //起终点坐标
      p1.x = line.a.x;
      p1.y = line.a.y;
      p2.x = line.b.x;
      p2.y = line.b.y;
      make_arrow_marker(0.0, 1.0, 0.0, p1, p2);//匿名函数，在上面被定义

      // 在 for 循环中使用 return 语句的话，会导致函数立即返回，同时也会终止整个循环。这意味着，即使 for 循环并未遍历完所有的元素，return 语句的执行也会导致提前结束循环
      if (line.go_list.empty()) continue;//路网拓扑列表为空，继续循环
      //有路网拓扑，画某一边终点到另一条路线起点
      for (auto const& node : line.go_list) {
        if (node >= traffic_routes_.size()) continue;//拓扑边的数量都超过了原有的边了，非法
        //node对应原规划路线中第几条边（从0开始的），是边的序号，这个序号在那个路网文件里由上到下从0开始
        p1.x = traffic_routes_[node].a.x;//这里a表示被连接边的起点
        p1.y = traffic_routes_[node].a.y;
        if(std::abs(p1.x-p2.x)==0) continue;//重合点
        // ROS_INFO("node= %d",node);
        auto o = std::atan2(p2.y - p1.y, p2.x - p2.x);  
        //朝向，四元数表示
        p.orientation.z = std::sin(o / 2);
        p.orientation.w = std::cos(o / 2);      
      }
    }
    vis_pub_.publish(marker_array);//rviz可视化
  }


  //预处理图，代价计算
  void HalfStructPlanner::calculate_pre_graph(lines_type const& lines, vector<double> const& distance_table) {
    // traffic_routes_ = lines;//包含起终点坐标、该条边可以连到哪些边等信息，一个traffic_route代表一条边
    // 使用 std::move 将 lines 中的内容转移给 traffic_routes_
    traffic_routes_ = std::move(lines);
    auto num = traffic_routes_.size();//起终点边的数量

    //还没有规划交通规则
    if (traffic_routes_.empty()) {
      ROS_WARN("Receive empty traffic routes, planner not working!");
      return;
    }
    /*static_cast 是 C++ 中的一个类型转换运算符。它用于执行静态类型转换，将一个表达式转换为指定的类型。
      使用 static_cast 进行类型转换可以在编译时进行类型检查，并帮助提高代码的可读性和安全性。它可以用于各种标准类型之间的转换，包括整型、浮点型、指针类型、引用类型等。
      在给定的代码中，static_cast 被用来将表达式 traffic_routes_.size() * 2 + 2 + EXTRA_POINTS_NUM * 2 的结果从数值类型转换为整型。这是因为 node_num_ 是一个整型变量，需要将计算结果转换为整型后再进行赋值。
      要注意的是，static_cast 在执行类型转换时不会进行运行时检查，它假设开发者已经确保了转换的安全性。因此，在使用 static_cast 进行类型转换时，需要确保转换是合理且安全的，避免可能导致错误或未定义行为的类型转换。*/
    // 路网起终点数目(traffic_routes_.size())*2(每一个起点同时作为终点)+小车的起终点(2)+小车的起终点带来的新增节点(EXTRA_POINTS_NUM * 2)
    node_num_ = static_cast<int>(traffic_routes_.size()*2+ 2 + EXTRA_POINTS_NUM * 2);
    // 序号含义：
    // 0: 起点 1: 终点
    // 2 ～ 1+EXTRA_POINTS_NUM: 起点带来的新节点
    // 2+EXTRA_POINTS_NUM ～ 1+2*EXTRA_POINTS_NUM: 终点带来的新节点
    // 2+2*EXTRA_POINTS_NUM ～ end: traffic routes节点 (本函数处理)
    nodes_.resize(node_num_);//使路径途径点数=node_num，会在这些点之间进行路径规划
    /*这行代码创建了一个指向 double* 类型的指针 graph_，并分配了一个 node_num_ 大小的动态数组来存储 double* 类型的元素。
    具体而言，graph_ 是一个指向指针数组的指针。通过使用 new 运算符，我们在堆上分配了 node_num_ 个 double* 类型的内存空间，并将指向该内存空间的地址赋给了 graph_。
    这样，graph_ 就可以用于存储 node_num_ 个 double* 类型的指针，每个指针可以指向一个 double 类型的值或者一组 double 值的数组。这种动态数组的创建可以方便地存储和操作不确定大小的数据。
    需要注意的是，在使用完 graph_ 后，应该调用 delete[] graph_; 来释放之前分配的内存，以避免内存泄漏。*/
    graph_ = new double *[node_num_];
    graph_bk_ = new double *[node_num_];
    for (auto i = 0; i < node_num_; ++i) {
    /*这行代码创建了一个指向 double 类型的指针数组，并将每个指针指向一个大小为 node_num_ 的 double 动态数组。
      具体来说，graph_ 是一个指向指针的指针，通过 graph_[i] 可以访问到第 i 个指针。通过使用 new 运算符，我们在堆上为每一个 graph_[i] 分配了一个大小为 node_num_ 的 double 数组，并将指向该数组的地址赋给 graph_[i]。
      这样，graph_[i] 就可以被用来存储一个大小为 node_num_ 的 double 数组，每个元素可以存储一个 double 值。
      需要注意的是，在使用完 graph_ 中的每个动态数组后，应该分别调用 delete[] graph_[i]; 来释放之前分配的内存，以避免内存泄漏。最后，还需要调用 delete[] graph_; 来释放 graph_ 这个指针数组本身的内存。*/
      graph_[i] = new double[node_num_];
      graph_bk_[i] = new double[node_num_];
      for (auto j = 0; j < node_num_; ++j) graph_[i][j] = std::numeric_limits<double>::max();//暂设定代价为无穷大
      graph_[i][i] = 0.0;//点自己到自己的代价为0
    }

    ROS_INFO("graph_ Init!");
    // 处理traffic routes预设边，有traffic_routes_.size()条边，每条边有首尾节点
    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * i;//某路网（边）的起点
      auto end = 2 + 2 * EXTRA_POINTS_NUM + 2 * i + 1;//某路网（边）的终点
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";//父坐标
      auto o = traffic_routes_[i].a.theta;
      p.pose.orientation.z = std::sin(o / 2);//朝向
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].a.x;//边的起点坐标
      p.pose.position.y = traffic_routes_[i].a.y;
      nodes_[start] = p;//首节点（边的起点）
      o = traffic_routes_[i].b.theta;
      p.pose.orientation.z = std::sin(o / 2);//朝向
      p.pose.orientation.w = std::cos(o / 2);
      p.pose.position.x = traffic_routes_[i].b.x;//边的终点坐标
      p.pose.position.y = traffic_routes_[i].b.y;
      nodes_[end] = p;//尾节点
      // 所有traffic routes都是直线，如果支持任意曲线的话，需要计算曲线长度
      // graph_[][]一张二维数组列表，存储俩俩节点之间的代价
      // 这里存储了某路网（边）的长度作为代价
      // graph_[start][end] = distance(nodes_[end], nodes_[start]);
      graph_[start][end] = distance_table[i];

      // 填充映射表
      // 使用引用存储 SG_paths 的引用
      const std::vector<Eigen::Vector3d>& sgPathRef = SG_paths[i];
      mappingTable[{start, end}] = &sgPathRef;
    }
    
    // ROS_INFO("graph_ Init2!");

    auto j=0;
    // 处理traffic routes终点能连接的起点及其构成的边
    for (auto i = 0; i < traffic_routes_.size(); ++i) {
      auto end = 2 + 2 * EXTRA_POINTS_NUM +  2 * i + 1;//某路网的终点
      //最后一条边的终点连最开始一条边的起点，用直线代价
      for (auto const& node : traffic_routes_[i].go_list) {
        if (node >= traffic_routes_.size()) {//非法
          ROS_ERROR("line %d go list %d out of range", i, node);
          continue;
        }
        auto start = 2 + 2 * EXTRA_POINTS_NUM + 2 * node;//被连接的那条边的起点
        //某路网的终点能连接到的另一路网的起点及它们构成的边---长度代价
        // graph_[end][start] = distance(nodes_[end], nodes_[start]);
        graph_[end][start] = distance_table[num+j];
        
      // 填充映射表
      const std::vector<Eigen::Vector3d>& tpPathRef = topology_paths[j];
      mappingTable[{end, start}] = &tpPathRef;
      j++;

      }
    }
    //备份
    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        graph_bk_[i][j] = graph_[i][j];
      }
    }

    // show_graph();//显示路点标号

  }


  //图更新，加入新的起终点后的拓扑
  //图更新：起点（所有半结构化道路的起点）连到新的起点，终点连到新的终点，起终点如果是同一条线，判断有无直连的可能
  void HalfStructPlanner::calculate_graph() {
    ContactS_paths.clear();
    ContactG_paths.clear();
    ContactS_paths_.clear();
    ContactG_paths_.clear();
    ContactSG_paths.clear();
    // ROS_INFO("25");
    /*当使用std::set容器存储setnode对象时，容器会根据operator<函数的定义自动进行排序，使得容器中的元素按照nearest_dist属性的升序排列。*/
    std::set<setnode> start_set, goal_set;//起点到各接点信息的集合,终点到各接点信息的集合，一个有序集
    auto start = point_t {nodes_[0].pose.position.x, nodes_[0].pose.position.y};//起点
    auto goal = point_t {nodes_[1].pose.position.x, nodes_[1].pose.position.y};//终点
    setnode sn {}, gn {};//{}表示初始化
    //这个循环可以得到小车起（终）点到各路网接点的所有信息，包括接点坐标、距离代价
// ROS_INFO("35");
    //除了暴力全遍历，其实可以用到一些数据结构的方法
    //不过这种全遍历可以保证找到有效接点
    
    for (auto i = 0; i < traffic_routes_.size(); ++i) {//traffic_routes_：当前路网数量,不包含拓扑
      //找到小车起点到各路网的接点，我们看到这个接点只会出现在边上，不会在拓扑上
      // ROS_INFO("55");
      //这里还是用引用的方式输入
      std::vector<Eigen::Vector3d> const & my_paths = SG_paths[i];
      sn = nearest_point_of_segment(start, &my_paths, 1);//输入小车起点，边（路网）的起终点
      //接点是该边的终点，且起点到接点会穿越障碍
      // if (sn.nearest_point == traffic_routes_[i].b || !line_safe(start, sn))
      // ROS_INFO("55_");
      // if (sn.nearest_point == traffic_routes_[i].b )
        // sn.nearest_dist = std::numeric_limits<double>::max();//设置距离无穷大作为代价无穷大     
      /*这段代码将名为 sn 的 setnode 对象插入到名为 start_set 的 std::set 容器中。insert 函数会将给定的元素插入到容器中，并根据容器类型（这里是 std::set）的特性（这里是距离）进行自动的排序和去重操作。
      在这里，如果 start_set 中已经存在与 sn 相等的元素（根据 setnode 类型的比较方式），则插入操作将被忽略，不会有新的元素插入。如果 start_set 中没有与 sn 相等的元素，则会将 sn 添加到 start_set 中，并保持容器中元素的有序性。
      综上，这段代码的作用是将 sn 插入到 start_set 容器中，并确保容器中的元素是有序且唯一的。*/
      if(sn.nearest_dist < 100) { //平方值
        sn.line = i;//第i条边（路网）
        start_set.insert(sn);
      }

      // ROS_INFO("25");
      //求终点到边ab的接点
      gn = nearest_point_of_segment(goal, &my_paths, 0);
      // ROS_INFO("45_");
      // if (gn.nearest_point == traffic_routes_[i].a)
        // gn.nearest_dist = std::numeric_limits<double>::max();
      if(gn.nearest_dist < 100) {
        gn.line = i;
        goal_set.insert(gn);
      }
      //不需要接点不是边的起点，实际上在小车起终点是同一个点的设置下，小车终点的接点就是边的起点
      // if (gn.nearest_point == traffic_routes_[i].a || !line_safe(goal, gn))

      // ROS_INFO("45");
    }
    
    // ROS_INFO("15");
    /*接下来，代码根据EXTRA_POINTS_NUM的值进行循环。在每次循环中，代码将起点集合中的节点存储到nodes_中，并更新相应的图中的边权重。具体操作是将起点集合中的节点的坐标信息存储到nodes_中，然后将起点集合中的节点与对应的线段节点之间的距离存储到图的对应位置。
      同样地，代码将终点集合中的节点存储到nodes_中，并更新相应的图中的边权重。*/
    int line_s, line_g;//用于保存当前接点对应的线段的起点和终点在节点列表 nodes_ 中的索引值，这些索引值将被用于更新代价矩阵

    for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
      geometry_msgs::PoseStamped p;
      p.header.frame_id = "map";
      //将起点与其接点信息集合中的接点存储到nodes_中，我们只存放前几个节点(EXTRA_POINTS_NUM),抛弃剩下的，前几个都是边长度较小的（排序过了）
      if (i < start_set.size()) {
        // ROS_INFO("start_set.size: %lu",start_set.size());
        auto sen = 2 + i;//nodes_[0]放起点，nodes[1]放终点，这里跳过这两个点
        line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line;
        // ROS_INFO("line: %d",std::next(start_set.begin(), i)->line);
        line_g = line_s + 1;//起点的某个接点所在边（原路网）的终点的索引

        //起点到接点的距离满足<EXTRA_POINTS_RANGE(3)，必更新第一个接点（距起点距离最短的接点）信息，因为肯定至少要有这么一条由起点去往接点的边
        if (i < 1 || std::next(start_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {

      //还需要规划起终点到接点的路线
      Vec3d start_pose = Eigen::Vector3d {nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)};//起点
      auto contact = std::next(start_set.begin(), i)->nearest_point;
      Vec3d goal_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta};//接点
      double lb,lf;
      result.theta.clear();
      result.x.clear();
      result.y.clear();
      vector<Vec3d> smooth_path;
      if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
          vector<Vec3d> path2smooth_b;
          path2smooth_b.clear();
          // 起点的接点不要最后一个点
          for (int i = 0; i < result.x.size()-1; ++i) {
              path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
          }
          
          //l(Limited-memory)-bfgs平滑
          smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
          
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);

          lb = result.total_length;
      } 
// ROS_INFO("16");
    // contact = std::next(start_set.begin(), i)->nearest_point_f;
    // goal_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta};//接点
    //   result.theta.clear();
    //   result.x.clear();
    //   result.y.clear();
    //   vector<Vec3d> smooth_path_f;
    //   if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
    //       vector<Vec3d> path2smooth_f;
    //       path2smooth_f.clear();
    //       for (int i = 0; i < result.x.size(); ++i) {
    //           path2smooth_f.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
    //       }
          
    //       //l(Limited-memory)-bfgs平滑
    //       smoother_ptr->optimize(voronoiDiagram, path2smooth_f, grid_);
    //       // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
          
    //       smooth_path_f.clear();
    //       smoother_ptr->getSmoothPath(smooth_path_f);

    //       lf = result.total_length;
    //   } 

    graph_[0][sen] = lb;//起点[0]到接点[sen]的距离更新
    // auto smooth_path = ( lb < lf ? smooth_path_b : smooth_path_f);
    //取最近距离的接点
    // contact = ( lb < lf ? std::next(start_set.begin(), i)->nearest_point_b : std::next(start_set.begin(), i)->nearest_point_f);
    contact = std::next(start_set.begin(), i)->nearest_point;
    
auto it = std::next(start_set.begin(), i);
auto cn = *it;
// ROS_INFO("cn.line: %d",cn.line);
// 修改元素的 nearest_index
// cn.nearest_index = (lb < lf ? it->nearest_index_b : it->nearest_index_f);
// ROS_INFO("cn.nearest_index: %d",cn.nearest_index);
    // 先从集合中删除元素,反正按路线长度排的序，重新插入还是原处
start_set.erase(it);
// 将修改后的元素重新插入到集合中
start_set.insert(cn);

    std::vector<Eigen::Vector3d>::const_iterator startIt;
    std::vector<Eigen::Vector3d>::const_iterator endIt;   
    startIt = smooth_path.begin();
    endIt = smooth_path.end();
// ROS_INFO("17");
    ContactS_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt)) ;
        // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
        // std::copy(startIt, endIt, subVec.data());

      const std::vector<Eigen::Vector3d>& CSPathRef = ContactS_paths[i];
      //这个是真实曲线
      mappingTable[{0, sen}] = &CSPathRef;

          //取接点到该边终点的所有位姿，这个是直线  
// auto v1 = Eigen::Vector3d {nodes_[0].pose.position.x, nodes_[0].pose.position.y, tf2::getYaw(nodes_[0].pose.orientation)};    
// auto v2 = Eigen::Vector3d {std::next(start_set.begin(), i)->nearest_point.x, std::next(start_set.begin(), i)->nearest_point.y, std::next(start_set.begin(), i)->nearest_point.theta};    
//           mappingTable[{0, sen}] ={v1, v2};

        // ROS_INFO("18");
          p.pose.position.x = contact.x;//接点x坐标
          p.pose.position.y = contact.y;
          auto o = contact.theta;
          p.pose.orientation.z = std::sin(o / 2);
          p.pose.orientation.w = std::cos(o / 2);
          nodes_[sen] = p;//nodes_的2+i号索引保存接点位置和朝向
          // ROS_INFO("14");
          //代价矩阵计算
          // graph_[0][sen] = std::next(start_set.begin(), i)->nearest_dist;//起点[0]到接点[sen]的距离更新
          //这个值只能取个大概了
          //大部分都是直接由dubis曲线得到的，直接代入分段尺寸0.3----现在都是0.3了
          // ROS_INFO("graph_[line_s][line_g]: %f",graph_[line_s][line_g]);
          // ROS_INFO("18");
          
          graph_[sen][line_g] = graph_[line_s][line_g] - static_cast<double>(0.5 * std::next(start_set.begin(), i)->nearest_index);//接点到接点所在边的终点的距离更新
          // ROS_INFO("11");
          //表示那条边的哪个接点
          auto path_pose = SG_paths[std::next(start_set.begin(), i)->line];
          int startIndex = std::next(start_set.begin(), i)->nearest_index;
          // ROS_INFO("startIndex: %d",std::next(start_set.begin(), i)->nearest_index);
          startIt = path_pose.begin() + startIndex + 1;
          endIt = path_pose.end();
          // std::vector<Eigen::Vector3d> subVec_(endIt - startIt);
          // std::copy(startIt, endIt, subVec_.data());

          ContactS_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt)) ;
          const std::vector<Eigen::Vector3d>& CSPathRef_ = ContactS_paths_[i];
          //这个是真实曲线
          mappingTable[{sen, line_g}] = &CSPathRef_;

        }
      }
// ROS_INFO("12");
      //同理终点与其接点信息更新
      if (i < goal_set.size()) {
        auto gen = 2 + EXTRA_POINTS_NUM + i;
        line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(goal_set.begin(), i)->line;//起点的某个接点所在边的起点的索引
        // ROS_INFO("line_2: %d",std::next(goal_set.begin(), i)->line);
        
        if (i < 1 || std::next(goal_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE) {

        Vec3d goal_pose = Eigen::Vector3d {nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};//终点
        auto contact = std::next(goal_set.begin(), i)->nearest_point;
        Vec3d start_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta};//接点
        result.theta.clear();
        result.x.clear();
        result.y.clear();
        double lb,lf;
        vector<Vec3d> smooth_path;
        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
            vector<Vec3d> path2smooth_b;
            path2smooth_b.clear();
            // 终点的接点不包含第一个点
            for (int i = 1; i < result.x.size(); ++i) {
                path2smooth_b.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
            }
          
          //l(Limited-memory)-bfgs平滑
          smoother_ptr->optimize(voronoiDiagram, path2smooth_b, grid_);
          // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
          smooth_path.clear();
          smoother_ptr->getSmoothPath(smooth_path);

          lb = result.total_length;
      } 
    // ROS_INFO("22");
      // contact = std::next(goal_set.begin(), i)->nearest_point_f;
      // start_pose = Eigen::Vector3d {contact.x, contact.y, contact.theta};//接点
      // result.theta.clear();
      // result.x.clear();
      // result.y.clear();
      // vector<Vec3d> smooth_path_f;
      // if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
      //     vector<Vec3d> path2smooth_f;
      //     path2smooth_f.clear();
      //     for (int i = 0; i < result.x.size(); ++i) {
      //         path2smooth_f.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
      //     }
          
      //     //l(Limited-memory)-bfgs平滑
      //     smoother_ptr->optimize(voronoiDiagram, path2smooth_f, grid_);
      //     // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
          
      //     smooth_path_f.clear();
      //     smoother_ptr->getSmoothPath(smooth_path_f);

      //     lf = result.total_length;
      // } 

    graph_[gen][1] = lb;//起点[0]到接点[sen]的距离更新
    // auto smooth_path = ( lb < lf ? smooth_path_b : smooth_path_f);
    contact = std::next(goal_set.begin(), i)->nearest_point;
        
    //取值改值
    auto it = std::next(goal_set.begin(), i);
    auto cn = *it;
    // 修改元素的 nearest_index
    // ROS_INFO("cn.line: %d",cn.line);
    // 修改元素的 nearest_index
    // cn.nearest_index = (lb < lf ? it->nearest_index_b : it->nearest_index_f);
    // ROS_INFO("cn.nearest_index: %d",cn.nearest_index);
    // 先从集合中删除元素
    goal_set.erase(it);
    // 将修改后的元素重新插入到集合中
    goal_set.insert(cn);

    std::vector<Eigen::Vector3d>::const_iterator startIt;
    std::vector<Eigen::Vector3d>::const_iterator endIt;   
    startIt = smooth_path.begin();
    endIt = smooth_path.end();
    // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
    // std::copy(startIt, endIt, subVec.data());

            //取接点到该边终点的所有位姿，这个是直线       
// auto v1 = Eigen::Vector3d {std::next(goal_set.begin(), i)->nearest_point.x, std::next(goal_set.begin(), i)->nearest_point.y, std::next(goal_set.begin(), i)->nearest_point.theta};    
//   auto v2 = Eigen::Vector3d {nodes_[1].pose.position.x, nodes_[1].pose.position.y, tf2::getYaw(nodes_[1].pose.orientation)};    
//           mappingTable[{gen, 1}] ={v1, v2};
    ContactG_paths.push_back(std::vector<Eigen::Vector3d>(startIt, endIt)) ;
        // std::vector<Eigen::Vector3d> subVec(endIt - startIt);
        // std::copy(startIt, endIt, subVec.data());

      const std::vector<Eigen::Vector3d>& CSPathRef = ContactG_paths[i];
      //这个是真实曲线
      mappingTable[{gen, 1}] = &CSPathRef;
      // const std::vector<Eigen::Vector3d>& sPathRef = smooth_path;
      // mappingTable[{gen, 1}] = &sPathRef;

          // mappingTable[{gen, 1}] ={subVec};      
        
        p.pose.position.x = contact.x;
        p.pose.position.y = contact.y;
        auto o = contact.theta;
        p.pose.orientation.z = std::sin(o / 2);
        p.pose.orientation.w = std::cos(o / 2);
          nodes_[gen] = p;
          // graph_[gen][1] = std::next(goal_set.begin(), i)->nearest_dist;//更新接点到小车终点的代价
          graph_[line_s][gen] = 0.5 * std::next(goal_set.begin(), i)->nearest_index;//更新接点所在边起点到接点的代价
          
          // ROS_INFO("graph_[line_s][gen]: %f",graph_[line_s][gen]);         
          // ROS_INFO("nearest_index: %d",std::next(goal_set.begin(), i)->nearest_index);         
          //表示那条边的哪个接点
          auto path_pose = SG_paths[std::next(goal_set.begin(), i)->line];
          int startIndex = std::next(goal_set.begin(), i)->nearest_index;
          startIt = path_pose.begin();
          endIt = path_pose.begin() + startIndex;
          // std::vector<Eigen::Vector3d> subVec_(endIt - startIt);
          // std::copy(startIt, endIt, subVec_.data());
          //取接点到该边起点的所有位姿        
          ContactG_paths_.push_back(std::vector<Eigen::Vector3d>(startIt, endIt)) ;
          const std::vector<Eigen::Vector3d>& CGPathRef_ = ContactG_paths_[i];
          //这个是真实曲线
          mappingTable[{line_s, gen}] = &CGPathRef_;
// ROS_INFO("21");
        
        }
      }
    }

    //更新起点带来的新点到终点带来的新点的代价，解决起终点在同一条line的情况
    auto m=0;
    for (auto i = 0; i < EXTRA_POINTS_NUM; ++i) {
      if (i < start_set.size()) {
        auto sen = 2 + i;//起点的接点索引
        line_s = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), i)->line;//起点的某个接点所在边的起点的索引
        // ROS_INFO("line_3: %d",std::next(start_set.begin(), i)->line);
        line_g = line_s + 1;//起点的某个接点所在边的终点的索引
        
        if(std::next(start_set.begin(), i)->nearest_dist < EXTRA_POINTS_RANGE){

        for (auto j = 0; j < EXTRA_POINTS_NUM; ++j) {
          if (j < goal_set.size()) {
            auto gen = 2 + EXTRA_POINTS_NUM + j;//终点的接点索引
            if(std::next(goal_set.begin(), j)->nearest_dist < EXTRA_POINTS_RANGE){
              //起终点的某俩个接点所在边（原路网）为同一条边 且 终点的接点到它们所在边的终点的距离<起点的接点到它们所在边的终点距离
              if (std::next(goal_set.begin(), j)->line == std::next(start_set.begin(), i)->line
                  && graph_[line_s][line_g] - graph_[line_s][gen] <= graph_[sen][line_g]) {
                  //当起终点的接点重合时不会带入代价
                  // ROS_INFO("line_4: %d",std::next(goal_set.begin(), j)->line);
                  graph_[sen][gen] = graph_[sen][line_g] - (graph_[line_s][line_g] - graph_[line_s][gen]);//取起终点的俩接点作为距离代价，不满足上述条件则代价为无穷大
                  //  表示那条边的哪个接点
                  // ROS_INFO("graph_[line_s][line_g]: %f",graph_[line_s][line_g]);         
                  // ROS_INFO("graph_[line_s][gen]: %f",graph_[line_s][gen]);         
                  // ROS_INFO("graph_[line_s][line_g] - graph_[line_s][gen]: %f",graph_[line_s][line_g] - graph_[line_s][gen]);         
                  // ROS_INFO("graph_[sen][line_g]: %f",graph_[sen][line_g]);
                  // ROS_INFO("graph_[sen][gen]: %f",graph_[sen][gen]);

                  auto path_pose = SG_paths[std::next(goal_set.begin(), j)->line];
                  int startIndex = std::next(start_set.begin(), i)->nearest_index;
                  int endIndex = std::next(goal_set.begin(), j)->nearest_index;
                  // std::vector<Eigen::Vector3d> subVec(path_pose.begin() + startIndex, path_pose.begin() + endIndex -1 );
                  ContactSG_paths.push_back(std::vector<Eigen::Vector3d>(path_pose.begin() + startIndex, path_pose.begin() + endIndex -1));

                  // ROS_INFO("20");
                  //取接点到该边终点的所有位姿        
          const std::vector<Eigen::Vector3d>& SGPathRef_ = ContactSG_paths[m++];
          //这个是真实曲线
          mappingTable[{sen, gen}] = &SGPathRef_;               
              }
            }

          }
        }
       }
      }
    }
    

    // print_graph();//打印代价矩阵
    // show_graph();
    // show_graph();//显示路点标号
}


//显示路点的标号
void HalfStructPlanner::show_graph() {
  visualization_msgs::MarkerArray marker_array;
  int id = 0;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;
  auto make_arrow_marker = [&](double r, double g, double b, geometry_msgs::Point const& p1, geometry_msgs::Point const& p2) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.2;
    marker.scale.z = 0.4;
    marker.color.a = 0.5;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose.orientation.w = 1.0;
    marker.points.resize(2);
    marker.ns = "arrow";
    marker.id = id++;
    marker.points[0] = p1;
    marker.points[1] = p2;
    marker_array.markers.emplace_back(marker);
  };
  auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p, std::string const& t) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose = p;
    marker.ns = "text";
    marker.id = id++;
    marker.text = t;
    marker_array.markers.emplace_back(marker);
  };
  for (auto i = 0; i < node_num_; ++i) {
    p.position.x = nodes_[i].pose.position.x;
    p.position.y = nodes_[i].pose.position.y;
    make_text_marker(0.0, 0.0, 0.0, p, std::to_string(i));
    // for (auto j = 0; j < node_num_; ++j) {
    //   if (graph_[i][j] == std::numeric_limits<double>::max()) continue;
    //   p1.x = nodes_[i].pose.position.x;
    //   p1.y = nodes_[i].pose.position.y;
    //   p2.x = nodes_[j].pose.position.x;
    //   p2.y = nodes_[j].pose.position.y;
    //   make_arrow_marker(1.0, 0.0, 0.0, p1, p2);
    // }
  }
  vis_pub_.publish(marker_array);
}


//显示更新后的路点标号
void HalfStructPlanner::show_graph_(std::vector<int> node_list) {
  visualization_msgs::MarkerArray marker_array_;
  int id = 0;
  geometry_msgs::Point p1, p2;
  geometry_msgs::Pose p;
  p.orientation.w = 1.0;

  auto make_text_marker = [&](double r, double g, double b, geometry_msgs::Pose const& p, std::string const& t) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.z = 1.0;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.pose = p;
    marker.ns = "text";
    marker.id = id++;
    marker.text = t;
    marker_array_.markers.emplace_back(marker);
  };
  for (auto i = 0; i < node_list.size(); ++i) {
    p.position.x = nodes_[node_list[i]].pose.position.x - 0.5;//偏移
    p.position.y = nodes_[node_list[i]].pose.position.y;
    make_text_marker(1.0, 0.0, 0.0, p, std::to_string(node_list[i]));
  }
  vis_pub_.publish(marker_array_);
}


  //获取离小车初始位姿最近的接点
  // auto HalfStructPlanner::get_nearest_point_of_start() -> geometry_msgs::PoseStamped {
  //   std::set<setnode> start_set;//起点到各接点信息的集合,终点到各接点信息的集合，一个有序集
  //   auto start = point_t {nodes_[0].pose.position.x, nodes_[0].pose.position.y};//起点
  //   setnode sn {};//{}表示初始化
  //   //这个循环可以得到小车起（终）点到各路网接点的所有信息，包括接点坐标、距离代价
  //   for (auto i = 0; i < traffic_routes_.size(); ++i) {//traffic_routes_：当前路网数量
  //     sn.line = i;//第i条边（路网）
  //     //找到小车起点到各路网的接点，我们看到这个接点只会出现在边上，不会在拓扑上
  //     sn.nearest_point = nearest_point_of_segment(start, traffic_routes_[i].a, traffic_routes_[i].b);//输入小车起点，边（路网）的起终点
  //     //接点是该边的终点，且起点到接点会穿越障碍
  //     // if (sn.nearest_point == traffic_routes_[i].b || !line_safe(sn.nearest_point, start))
  //     if (sn.nearest_point == traffic_routes_[i].b)
  //       sn.nearest_dist = std::numeric_limits<double>::max();//设置距离无穷大作为代价无穷大
  //     //接点不是该边的终点，且起点到接点不会穿越障碍
  //     else
  //       sn.nearest_dist = distance(start, sn.nearest_point);//将小车起点到各路网接点的距离作为代价值
  //     /*这段代码将名为 sn 的 setnode 对象插入到名为 start_set 的 std::set 容器中。insert 函数会将给定的元素插入到容器中，并根据容器类型（这里是 std::set）的特性（这里是距离）进行自动的排序和去重操作。
  //     在这里，如果 start_set 中已经存在与 sn 相等的元素（根据 setnode 类型的比较方式），则插入操作将被忽略，不会有新的元素插入。如果 start_set 中没有与 sn 相等的元素，则会将 sn 添加到 start_set 中，并保持容器中元素的有序性。
  //     综上，这段代码的作用是将 sn 插入到 start_set 容器中，并确保容器中的元素是有序且唯一的。*/
  //     start_set.insert(sn);

  //     //不需要终点到边的接点，我们的鸡舍环境下，起终点始终为同一个点，这样才能保证巡检一圈；离小车初始位置（模拟起点）最近的接点，将作为真正的巡检起终点
  //     //而且这个接点只是临时用，不用放入代价矩阵中

  //   }
  //   //取start_set里的第一个，这个是距离小车初始位置最近的接点
  //   int line_g;//用于保存当前接点对应的线段的起点和终点在节点列表 nodes_ 中的索引值，这些索引值将被用于更新代价矩阵
  //   geometry_msgs::PoseStamped p;
  //   p.header.frame_id = "map";
  //   if (start_set.size()) {//表明有接点
  //     p.pose.position.x = std::next(start_set.begin(), 0)->nearest_point.x;//接点x坐标
  //     p.pose.position.y = std::next(start_set.begin(), 0)->nearest_point.y;
  //     line_g = 2 + 2 * EXTRA_POINTS_NUM + 2 * std::next(start_set.begin(), 0)->line + 1;//该接点所在边（原路网）的终点的索引
  //     p.pose.orientation = nodes_[line_g].pose.orientation;//接点的朝向跟随其所在边的终点
  //   }
  //   else{//肯定不会到这里
  //     ROS_ERROR("there exist no nearest point of start positon!");
  //   }

  //   return p;

  //   //起终点之间的代价为无穷，不希望他们直接能走到，这是我们鸡舍环境的要求

  //   // print_graph();//打印代价矩阵
  // }


  //求点p到ab边（ab起终点构成的路网）的接点：计算点到线段最近点
  // auto HalfStructPlanner::nearest_point_of_segment(point_t const& p, point_t const& a, point_t const& b) -> point_t {
  //   if (a == b) return a;//ab边是一个点

  //   auto k = -((a.x - p.x) * (b.x - a.x) + (a.y - p.y) * (b.y - a.y)) / ((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
  //   if (k <= 0) return a;//接点取为a点
  //   if (k >= 1) return b;//接点取为b点
  //   point_t result;
  //   result.x = k * (b.x - a.x) + a.x;//接点x坐标
  //   result.y = k * (b.y - a.y) + a.y;//接点y坐标
  //   return result;
  // }


  //计算起（终）点到某一路线上的最近点,flag标记是起点还是终点
  auto HalfStructPlanner::nearest_point_of_segment(point_t const& p, vector<Eigen::Vector3d> const * paths, bool flag) -> setnode {
    float dist = FLT_MAX;
    int ii = 0;
    float min_dist = FLT_MAX;//初始化一个极大值
    int min_index = -1;
    // ROS_INFO("47");
    // std::vector<Eigen::Vector3d> const & my_paths = paths;
    for (const auto& path : *paths) {
          // ROS_INFO("y=%f",path[1]);
   //计算俩点之间的距离作为代价
    dist = static_cast<float>(std::pow(p.x - path[0], 2)
                              + std::pow(p.y - path[1], 2));//平方值

      //  ROS_INFO("66_");
    if (dist < min_dist) {
      min_dist = dist;
      min_index = ii;
    }
      ++ii;//索引+1
    }

      // ROS_INFO("65");
    setnode result{};
    int min_index_;
    if(flag){//起点=1
      //为了能够平滑规划起终点与接点之间的路线，让接点取最近接点后面的某个点
      if(min_index==0){
        min_index_ = min_index;
      }
      else{
        min_index_ = min_index + 8;
        if(min_index_ >= paths->size()-1) min_index_ = paths->size()-1;
      }
        result.nearest_point.x = (*paths)[min_index_][0];
        result.nearest_point.y = (*paths)[min_index_][1];//接点y坐标
        result.nearest_point.theta = (*paths)[min_index_][2];
        result.nearest_index = min_index_;
      
    }
    
    else{//终点=0
      if(min_index==paths->size()-1){
        min_index_ = min_index;
      }
      else{
      min_index_ = min_index - 8;
      if(min_index_ <= 0) min_index_ = 0;
      }
      result.nearest_point.x = (*paths)[min_index_][0];
      result.nearest_point.y = (*paths)[min_index_][1];//接点y坐标
      result.nearest_point.theta = (*paths)[min_index_][2];
      result.nearest_index = min_index_;
    }
    
    result.nearest_dist = min_dist;//需要这个数值进行排序
    return result;
  }


  //计算俩节点之间的距离作为代价
  auto HalfStructPlanner::distance(point_t const& a, point_t const& b) -> double {
    return std::hypot(a.x - b.x, a.y - b.y);//欧氏距离
  }

  //打印代价矩阵
  void HalfStructPlanner::print_graph() {
    std::cout << "node num: " << node_num_ << std::endl;
    /*在每次循环中，使用 std::cout 对象输出一段带有颜色的文本，并通过 std::setw 控制字段宽度。
      每次循环中，std::cout << GREEN 将后续输出的文本颜色设置为绿色（前提是定义了相关的 ANSI 颜色码），std::setw(3) 用于控制输出宽度为3个字符，以对齐输出结果，最后 WHITE 则是将输出颜色恢复为默认值。整个输出的格式为：i （宽度为3，右对齐） + 空格。
      这行代码通常被用于在命令行界面上展示节点索引，方便用户查看和理解节点间的关系。*/
    std::cout << SET_FONT_ON; // 开始设置字体大小
    std::cout << SET_FONT_1;   // 设置字体大小为3 
    for (auto i = 0; i < node_num_; ++i) std::cout << GREEN << std::setw(3) << i << " " << WHITE;
    std::cout << "\n";
    for (auto i = 0; i < node_num_; ++i) {
      for (auto j = 0; j < node_num_; ++j) {
        if (graph_[i][j] != std::numeric_limits<double>::max()) {
          std::string str {"000 "};
          auto gij = std::to_string(graph_[i][j]);
          for (auto k = 0; k < 3; ++k) {
            if (gij.size() > k) str[k] = gij[k];//将 gij 的前三个字符依次替换 str 字符串的前三个字符
          }
          std::cout << str.c_str();
        } else {
          std::cout << "*** ";//代价无穷
        }
      }
      std::cout << GREEN << " " << i << "\n" << WHITE;
    }
    std::cout << SET_FONT_ON;     // 开始设置字体大小
    std::cout << SET_FONT_0;       // 将字体大小设置为默认值
  }


  //Dijkstra算法规划出一条距离代价最小的路径
  auto HalfStructPlanner::dijkstra() -> std::vector<int> {
    std::vector<int> result;//保存路径，最终返回的结果

    struct dijnode {
      int node;//节点号
      int prev_node;//与之关联的上一个节点
      double cost;//距离代价
      //重载函数为了排序，从小到大
      bool operator<(dijnode const& rhs) const {
      /*在operator<()函数中，比较了当前对象的cost成员变量和传入对象（即已加入的排列 rhs）的cost成员变量。
        如果当前对象的cost小于传入对象的cost，则返回true，表示当前对象应排在传入对象之前。
        根据C++的标准库实现，优先队列会使用operator<()来确定元素的优先级。因此，当元素被插入到优先队列中时，会按照从小到大（升序）的顺序对元素进行排序。*/
        return cost < rhs.cost;
      }
      bool operator==(dijnode const& rhs) const {//自定义了相等比较的方式，在后面std::find(open_list.begin(), open_list.end(), open_node)中用到
        return node == rhs.node;
      }
    };

    //从graph_代价矩阵中（所以只有那几条路网可以通行）找到0-1(起终点)的路径
    /*std::set 是一个有序的关联容器，其中的元素按照一定的顺序进行排列，通常是从小到大。在这里，open_list 是一个存储 dijnode 类型对象的 std::set 容器。由于 dijnode 可以直接比较大小（定义了 < 运算符），因此 std::set 可以自动维护元素的有序性，并且可以快速地进行搜索、插入和删除操作。
      std::vector 是一个动态数组，可以随时改变其大小。在这里，close_list 是一个存储 dijnode 类型对象的 std::vector 向量，用于存储已经处理过的节点，相当于是一个备选列表。遍历所有的节点后，已经被加入到 close_list 中的节点就不再需要被考虑了。*/
    std::set<dijnode> open_list;//
    std::vector<dijnode> close_list;//将处理过的节点放入close_list
    open_list.insert(dijnode{0, 0, 0.0});//将起点作为初始节点插入到open_list中
    //只要open_list非空就一直循环
    while (!open_list.empty()) {
    /*这段代码使用了 std::set 容器中的 extract() 函数和返回的 node 对象进行赋值。
      std::set::extract() 函数用于从集合中提取（移除）指定位置的元素，并返回一个包含该元素的 std::set::node_type 对象。在这里，通过调用 open_list.extract(open_list.begin())，我们从 open_list 中提取了第一个元素，并将其返回。这个操作会同时从 open_list 中删除这个元素。
      然后，通过 .value() 成员函数，我们获取被提取的元素的值，并将其赋给名为 node 的变量。这样，node 就保存了之前在 open_list 中的第一个元素的值。
      总而言之，这段代码的作用是从 open_list 中提取并删除第一个元素，并将其值赋给变量 node。*/
      auto node = open_list.extract(open_list.begin()).value();

      //这里开始构建路径（close_list列表）
      //最终拿到了节点1（终点），表明open_list已处理完备
      if (node.node == 1) {
        while (node.prev_node != 0) {//终点不是连着起点（第一轮），节点不是连着起点（以后轮）
          result.emplace_back(node.node);//先将终点放入result列表（第一轮），先将节点放入result列表（以后轮）
        /*这段代码使用了 STL 库中的 std::find_if() 函数和 lambda 表达式，用于在 close_list 中查找满足某个条件的元素。
          std::find_if() 函数用于在指定范围内查找满足特定条件的第一个元素，并返回该元素的迭代器。在这里，我们传递给 std::find_if() 函数一个起始迭代器 close_list.begin() 和一个终止迭代器 close_list.end()，用于在 close_list 容器中查找满足条件的元素。
          lambda 表达式 [](dijnode const& a) { return a.node == node.prev_node; } 用于定义查找条件。其中，[&] 表示捕获所有外部变量的引用，并将其传入 lambda 表达式中，以便在 lambda 表达式中使用。dijnode const& a 是 lambda 函数的参数列表，表示接受一个名为 a 的常量引用参数。{ return a.node == node.prev_node; } 是 lambda 函数的主体部分，用于判断节点 a 的 node 成员是否等于 node.prev_node。
          如果查找成功，即在 close_list 中找到了满足条件的元素，则 std::find_if() 函数返回匹配到的元素的迭代器；否则返回终止迭代器 close_list.end()。
          代码中的 prev 是一个迭代器，用于保存在 close_list 中满足条件的元素的位置（迭代器）。如果找到了满足条件的元素，则 prev 将指向该元素；否则，它将指向 close_list.end()。
          需要注意的是，在使用 std::find_if() 查找元素时，要根据具体情况自定义 lambda 表达式来定义查找条件，并确保元素类型支持相等比较运算符（==），或者自定义了相等比较的方式。*/
          auto prev = std::find_if(close_list.begin(), close_list.end(),
                                  [&](dijnode const& a) { return a.node == node.prev_node; });//node（1）.prev_node终点的前一个节点，一般来说肯定有，没有就是路径不完整（第一轮）
          if (prev == close_list.end()) {
            ROS_ERROR("No whole path while graph search success, that should not happen!");
            break;
          }
          node = *prev;//node更新到连终点的前一个节点（第一轮）
        }

        result.emplace_back(node.node);//node.node这个就是起点的下一个节点了
    //      result.emplace_back(node.prev_node); // 不需要特意走到起点
        //路径反转，使得路径以正确的顺序排列
        std::reverse(result.begin(), result.end());//变成一条由起点的下一个节点到终点的路径
        //打印
        std::cout << RED << "0";
        for (auto const& r : result) std::cout << RED << "->" << r;//打印路径
        std::cout << WHITE << std::endl;
        break;//退出大循环，结束
      }

      //这里找到最短路径

      close_list.emplace_back(node);//放入close_list的点将不再被处理
      //填充open_list
      for (auto i = 0; i < node_num_; ++i) {
        if (i == node.node) continue;//与open_list中当前的节点相同
      /*这段代码使用了 STL 库中的 std::find_if() 函数和 lambda 表达式，用于在 close_list 中查找是否已经处理过节点 i。
        std::find_if() 函数用于在指定范围内查找满足某个条件的第一个元素，并返回该元素的迭代器。在这里，我们传递给 std::find_if() 函数一个起始迭代器 close_list.begin() 和一个终止迭代器 close_list.end()，用于在 close_list 容器中查找是否已经处理过节点 i。
        lambda 表达式 [](dijnode const& a) { return a.node == i; } 用于定义查找条件。其中，[&] 表示捕获所有外部变量的引用，并将其传入 lambda 表达式中，以便在 lambda 表达式中使用。dijnode const& a 是 lambda 函数的参数列表，表示接受一个名为 a 的常量引用参数。{ return a.node == i; } 是 lambda 函数的主体部分，用于判断节点 a 是否等于节点 i。
        如果查找成功，即在 close_list 中找到了已经处理过的节点 i，则 std::find_if() 函数返回 close_list.end() 迭代器，否则返回匹配到的元素的迭代器。如果找到了已经处理过的节点 i，则通过 continue 语句跳过本次循环，继续处理下一个节点。如果没有找到已经处理过的节点 i，则继续执行下面的代码。*/
        if (std::find_if(close_list.begin(), close_list.end(),
                        [&](dijnode const& a) { return a.node == i; }) != close_list.end()) continue;//在 close_list 中查找是否已经处理过节点 i，已经处理了则进行下一轮循环
        
        //当前i节点还未被处理

        if (graph_[node.node][i] == std::numeric_limits<double>::max()) continue;//忽略相互之间代价无穷大的节点

        //当前节点i与从open_list取出的代价最小的节点   graph_[node.node][i]：从上一个节点到当前节点的代价
        dijnode open_node {i, node.node, node.cost + graph_[node.node][i]};//剩下的一个节点一个节点来
        /*这段代码使用了 STL 库中的 std::find() 函数，用于在 open_list 中寻找与 open_node 相等的元素。
        std::find() 函数接受一个起始迭代器 open_list.begin() 和一个终止迭代器 open_list.end()，并在指定范围内查找与 open_node 相等的元素。如果找到匹配的元素，则返回该元素的迭代器；如果未找到匹配的元素，则返回终止迭代器 open_list.end()。
        在这里，open_list 是一个 std::set 容器，而 open_node 是 dijnode 类型的对象。由于 std::set 是有序集合，查找操作的时间复杂度为 O(log n)。它会根据元素的大小（通过 < 运算符）进行二分查找。
        代码中的 iter2 是一个迭代器，用于保存 open_list 中与 open_node 相等的元素的位置（迭代器）。如果找到了匹配的元素，则 iter2 将指向该元素；否则，它将指向 open_list.end()。
        需要注意的是，在使用 std::find() 查找元素时，要确保元素类型支持相等比较运算符（==），或者自定义了相等比较的方式。*/
        auto iter2 = std::find(open_list.begin(), open_list.end(), open_node);
        if (iter2 != open_list.end()) {
          //open_list中有与open_node相等的
          //open_list中当前的node代价值过大
          if (iter2->cost > open_node.cost) {
            open_list.erase(iter2);//剔除
            open_list.insert(open_node);//填入
          }
        } else {
          //==open_list.end()，open_list中没有与open_node相等的
          open_list.insert(open_node);
        }
      }
    }

    return result;
  }


  //代价值小于66返回true，表示无障碍物
  // auto HalfStructPlanner::is_free_(const geometry_msgs::PoseStamped& pose) const -> bool {
  //   auto cost = pose_cost_(pose);//获取某一个位姿点的代价值
  //   // ROS_INFO("cost= %d",cost);
  //   return cost < 66;
  // }


  // auto HalfStructPlanner::line_safe(point_t const& a, point_t const& b) -> bool {
  //   // if (!costmap_) return true;//有无使用代价地图
  //   //使用了代价地图
  //   auto num = static_cast<int>(distance(a, b) / 0.1);//0.1是地图分辨率
  //   uint32_t mx, my;
  //   double x, y;
  //   for (auto i = 0; i < num; ++i) {
  //     x = a.x + i * (b.x - a.x) / num;
  //     y = a.y + i * (b.y - a.y) / num;
  //       /*这段代码首先调用了costmap_对象的worldToMap函数，传入了x、y两个参数和mx、my两个引用参数。worldToMap函数的作用是将给定的世界坐标(x, y)转换为代价地图中的栅格坐标(mx, my)。
  //         如果转换成功，即worldToMap函数返回true，则条件表达式(!costmap_->worldToMap(x, y, mx, my))的值为false，不满足条件，代码继续执行。*/
  //     // if (!costmap_->worldToMap(x, y, mx, my)) return false;
  //       /*这段代码首先调用了costmap_对象的getCost函数，传入了mx、my两个参数。getCost函数的作用是获取代价地图中坐标为(mx, my)的栅格单元格的代价值。
  //         然后，该代码判断获取到的代价值是否大于等于66，如果大于等于66，则条件表达式的值为true，满足条件，代码执行到return false语句，函数返回false，表示该栅格单元格不可达。
  //         因此，这段代码的作用是判断给定的栅格坐标(mx, my)是否可达，如果该栅格单元格的代价值大于等于66，则认为该栅格单元格不可达，返回false表示不可达；否则认为该栅格单元格可达，继续执行后续的代码。*/
  //     // if (costmap_->getCost(mx, my) >= 66) return false;
  //   }

  //   return true;
  // }


    /**
     * @brief 查询一条线上的点是否在障碍物里（插值）
    **/
    // bool HalfStructPlanner::line_safe(point_t const& a, setnode const& b) {
    //     // if (!grid_) return true;
        
    //     auto check_point_num = static_cast<int>(b.nearest_dist / grid_.info.resolution);//0.1是分辨率

    //     double delta_x = (b.nearest_point.x - a.x) / check_point_num;
    //     double delta_y = (b.nearest_point.y - a.y) / check_point_num;

    //     double cur_x = a.x;
    //     double cur_y = a.y;

    //     // 判断是否是障碍物
    //     // auto idx = i + j * map_.info.width;：这一行计算出给定坐标在地图 data 数组中的下标 idx。由于地图数据存储方式通常是一维数组，所以需要将二维坐标转换为一维下标。具体计算方法是将 j 乘以地图宽度，再加上 i。
    //     // return map_.data[idx] == 100;：最后，函数返回一个布尔值，表示给定坐标是否为障碍物。如果地图 data 数组中 idx 对应的值为 100，就认为该坐标是障碍物，返回 true；否则返回 false。
    //     auto mapObstacle = [&](double x, double y) {
    //         auto i = std::floor((x - grid_.info.origin.position.x) / grid_.info.resolution + 0.5);
    //         auto j = std::floor((y - grid_.info.origin.position.y) / grid_.info.resolution + 0.5);
    //         auto idx = i + j * grid_.info.width;
    //         return grid_.data[idx] == 100 || grid_.data[idx] == -1;//100为障碍物像素值,-1为未知
    //     };

    //     for (int i = 0; i < check_point_num; ++i) {
    //         //障碍区和未知区都不能通过
    //         if(mapObstacle(cur_x, cur_y))  return false;
    //         cur_x += delta_x;
    //         cur_y += delta_y;
    //     }
    //     return true;
    // }


    bool HalfStructPlanner::line_safe(point_t const& a, point_t const& b) {
        auto check_point_num = static_cast<int>(std::hypot(a.x - b.x, a.y - b.y) / (grid_->info.resolution*5));

        double delta_x = (b.x - a.x) / check_point_num;
        double delta_y = (b.y - a.y) / check_point_num;

        double cur_x = a.x;
        double cur_y = a.y;

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

            // Vec3i idx = getIndexFromPose({cur_x, cur_y, 0});
            //障碍区和未知区都不能通过
            if(mapObstacle(cur_x, cur_y))  return false;
            // if (map_.atPosition("elevation", {cur_x, cur_y}) > 0) {
            //     return true;
            // }

            cur_x += delta_x;
            cur_y += delta_y;
        }
        return true;
    }


  /**
   * @brief 可视化平滑后的路径
   * 
   * @param path 
   */
  void HalfStructPlanner::visOptPath(std::vector<vector<Eigen::Vector3d>> paths) {
      visualization_msgs::MarkerArray marker_array;
      int id = 0;//标记id号
      auto path_marker = [&](vector<Vec3d> const& paths) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "opt_db_path";
        marker.id = id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 0.0; 
        marker.color.g = 0.0;
        marker.color.b = 1.0; // Blue color
        for (const auto& path : paths)
        {
            geometry_msgs::Point point;
            point.x = path[0];
            point.y = path[1];
            point.z = 0.05;
            marker.points.push_back(point);
        }
        marker_array.markers.emplace_back(marker);
      };

      for (auto const& path : paths) {
          path_marker(path);//可视化路线
      }
      optimized_path_vis_pub.publish(marker_array);
  }


  /**
   * @brief 可视化平滑前的路径 
   * 
   * @param path 
   */
  void HalfStructPlanner::visPath(std::vector<vector<Eigen::Vector3d>> paths) {
      visualization_msgs::MarkerArray marker_array;
      visualization_msgs::Marker path_point;
      int id = 0;//标记id号
      //可视化路线
      auto path_marker = [&](vector<Vec3d> const& paths) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "db_path";
        marker.id = id++;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.1; // Line width
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 0.0; 
        marker.color.g = 1.0;
        marker.color.b = 0.0; // Blue color
        for (const auto& path : paths)
        {
            geometry_msgs::Point point;
            point.x = path[0];
            point.y = path[1];
            point.z = 0.05;
            marker.points.push_back(point);
        }
        marker_array.markers.emplace_back(marker);
      };
      //显示路点
      auto point_marker = [&](vector<Vec3d> const& paths) {
        path_point.id = id++;
        path_point.header.frame_id = "map";
        path_point.header.stamp = ros::Time::now();
        path_point.type = visualization_msgs::Marker::POINTS;
        path_point.scale.x = 0.1;
        path_point.scale.y = 0.1;
        path_point.scale.z = 0.1;
        path_point.color.a = 1;
        path_point.color.r = 1;
        path_point.color.g = 0;
        path_point.color.b = 0;
        path_point.pose.orientation.w = 1.0;
        for (size_t i = 0; i < paths.size(); i ++) {
          geometry_msgs::Point p;
          p.x = paths[i][0];
          p.y = paths[i][1];
          p.z = 0.05;
          path_point.points.push_back(p);
        }
      };

      for (auto const& path : paths) {
          //显示路线
          path_marker(path);//匿名函数，在上面被定义
          //显示路点
          // point_marker(path);
      }
      path_vis_pub.publish(marker_array);//可视化优化前的曲线，混合A*规划出来的路线
      // path_point_vis_pub.publish(path_point);//可视化优化前的曲线路点
  }


  //将优化后的路线路点保存
  auto HalfStructPlanner::write_file(std::string& file_path, std::vector<Eigen::Vector3d> const& path, double length) -> bool {
    std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);//允许追加内容
    if (!out.is_open()) {
      ROS_ERROR("Open file %s failed!", file_path.c_str());
      return false;
    }

    out << std::to_string(length) << "\n";

    for (auto const& p : path) {
      out << std::to_string(p[0]) << " "
          << std::to_string(p[1]) << " "
          << std::to_string(p[2]) << "\n";
    }
    out << "EOP" << "\n";

    out.close();
    return true;
  }


  //显示起终点的路线
  void HalfStructPlanner::visSGPath(const std::vector<vector<Eigen::Vector3d>>& paths) {
      //接收起终点路线的所有位姿点
      // SG_paths = paths;
      // 使用 std::move 将 paths 中的内容转移给 SG_paths
      SG_paths = std::move(paths);
    
      int id = 0;//标记id号
      // 创建包含多个PoseArray的消息
    geometry_msgs::PoseArray multi_pose_array;
    multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        // 创建一个包含多个位姿的PoseArray消息
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        for (const auto& path : paths)
        {
      geometry_msgs::Pose pose;
      pose.position.x = path[0];
      pose.position.y = path[1];
      pose.position.z = 0.05;
      pose.orientation.z = std::sin(path[2] / 2.0);
      pose.orientation.w = std::cos(path[2] / 2.0);
      pose_array.poses.push_back(pose);
        }
      multi_pose_array.poses.insert(multi_pose_array.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
      };

      for (auto const& path : SG_paths) {
          path_marker(path);//可视化路线
      }
      // // 发布包含多个PoseArray的消息
    SG_path_vis_pub.publish(multi_pose_array);
  }


  //显示拓扑路线
  void HalfStructPlanner::vistopologyPath(const std::vector<vector<Eigen::Vector3d>>& paths) {
      //接收拓扑路线的所有位姿点
      // topology_paths = paths; 
      topology_paths = std::move(paths);

      int id = 0;//标记id号
      // 创建包含多个PoseArray的消息
    geometry_msgs::PoseArray multi_pose_array;
    multi_pose_array.header.frame_id = "map";

      auto path_marker = [&](vector<Vec3d> const& paths) {
        // 创建一个包含多个位姿的PoseArray消息
        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        for (const auto& path : paths)
        {
      geometry_msgs::Pose pose;
      pose.position.x = path[0];
      pose.position.y = path[1];
      pose.position.z = 0.05;
      pose.orientation.z = std::sin(path[2] / 2.0);
      pose.orientation.w = std::cos(path[2] / 2.0);
      pose_array.poses.push_back(pose);
        }
    multi_pose_array.poses.insert(multi_pose_array.poses.end(), pose_array.poses.begin(), pose_array.poses.end());
      };

      for (auto const& path : topology_paths) {
          path_marker(path);//可视化路线
      }
       // 发布包含多个PoseArray的消息
    topology_path_vis_pub.publish(multi_pose_array);
  }


  //显示完整备选的拓扑路线
  void HalfStructPlanner::visfullPath(const std::vector<Eigen::Vector3d>& poses) {
        // 创建一个包含多个位姿的PoseArray消息
      geometry_msgs::PoseArray pose_array;
      pose_array.header.frame_id = "map";
      auto pose_marker = [&](Vec3d const& pose) {
        geometry_msgs::Pose pose_;
        pose_.position.x = pose[0];
        pose_.position.y = pose[1];
        pose_.position.z = 0.05;
        pose_.orientation.z = std::sin(pose[2] / 2.0);
        pose_.orientation.w = std::cos(pose[2] / 2.0);
        pose_array.poses.push_back(pose_);
      };
      for (auto const& pose : poses) {
          pose_marker(pose);//可视化路线
      }
      full_path_vis_pub.publish(pose_array);
  }


  // Hybrid A*规划
  void HalfStructPlanner::HybridAstarplan(Vec3d start_pose, Vec3d goal_pose) {
      HybridAstarResult result;
      vector<vector<Vec3d>> astarpath;//未优化的路线集
      vector<vector<Vec3d>> astarpath_;//优化的路线集

      // vector<Vec3d> path2smooth;

      // double path_length = std::numeric_limits<double>::max();
      //最短路径的规划开始时间（带优化）
      auto hybrid_astar_start_ = chrono::high_resolution_clock::now();//初始化
      // auto hybrid_astar_end_ = chrono::high_resolution_clock::now();

      //搜索3次取距离最短的一条---------
      // for(auto j=0; j<3; j++){
        // 计时开始
        // auto hybrid_astar_start = chrono::high_resolution_clock::now();

        if (hybrid_astar_ptr->plan(start_pose, goal_pose, result)) {
            vector<Vec3d> path2smooth;
            // path2smooth.clear();
            for (int i = 0; i < result.x.size(); ++i) {
                path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
            }

            astarpath.push_back(path2smooth);//保存优化前的路径
            // 可视化优化前的路径和路径点
            visPath(astarpath);//可视化优化前路线
            // 原始A*和折线A星的可视化放在plan所在文件中

            

            //l(Limited-memory)-bfgs平滑
            smoother_ptr->optimize(voronoiDiagram, path2smooth, grid_);
            // smoother_ptr->smoothPath(voronoiDiagram, path2smooth, grid_); //这个是梯度下降版的
            
            vector<Vec3d> smooth_path;
            // smooth_path.clear();
            smoother_ptr->getSmoothPath(smooth_path);
            astarpath_.push_back(smooth_path);//保存优化的路径
            visOptPath(astarpath_);

            // for (size_t i = 0; i < result.x.size(); ++i)
            // { 
            //   //以平滑后的路径点做的车轮廓
            //   // vis.publishVehicleBoxes({smooth_path[i](0), smooth_path[i](1), smooth_path[i](2)}, i);
            //   //以平滑前的路径点做的车轮廓
            //   vis.publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i);
            // }

        } else {
            ROS_ERROR("search fail");
            return;
        }   
          // //规划结束时间（带优化）
          auto hybrid_astar_end = chrono::high_resolution_clock::now();

        // if(path_length > result.total_length){
        //     path_length = result.total_length;
        //     //保存该次时间戳
        //     hybrid_astar_start_ = hybrid_astar_start;
        //     hybrid_astar_end_ = hybrid_astar_end;
        //     path2smooth.clear();
        //     for (int i = 0; i < result.x.size(); ++i) {
        //         path2smooth.push_back({result.x[i], result.y[i], result.theta[i]});//装载待平滑路径
        //     }
        // }
      // }

      // 可视化优化前的路径和路径点
      // astarpath.push_back(path2smooth);//保存优化前的路径
      // visPath(astarpath);//可视化优化前路线
      // 原始A*和折线A星的可视化放在plan所在文件中

      chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start_;
      cout << "最短路径长度:" << result.total_length << "m" << endl;
      cout << "最短路径规划时间（带优化）:" << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
  }




}

