#include "simple_astar_test.h"


namespace chibot::topologyplanning {

  SimpleAstarTest::SimpleAstarTest():
  // line_ready_(false),
  has_start(false),
  has_goal(false),
  cur_state_(StateValue::Run){}

  //初始化
  void SimpleAstarTest::init() {
      ros::NodeHandle nh;
      //订阅栅格地图，这个带有回调函数的必须放在主程序里，且快入快出
      map_sub_ = nh.subscribe("/map", 1, &SimpleAstarTest::mapReceived, this);
      //设置拓扑起点集
      start_sub = nh.subscribe("/initialpose", 1, &SimpleAstarTest::startCallback, this);
      //设置拓扑终点集
      goal_sub = nh.subscribe("/move_base_simple/goal", 1, &SimpleAstarTest::goalCallback, this);

      //各种服务调用，服务器建立，很重要的一块
      // task_srv_ = nh.advertiseService(TASK_SRV, &SimpleAstarTest::task_service, this);
      
  }

  //主循环函数，给初始点和目标点就生成一条路径
  void SimpleAstarTest::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      //任务状态机，状态的切换在各事件中
      switch (cur_state_) {
        case StateValue::Idle:
          // do nothing, cancel goal and reset value in event 取消导航和清零操作在事件中完成
          
          // if(line_ready_){
          //   // half_struct_planner_->set_traffic_route(lines);//规划起终点间的路线并保存
          //   half_struct_planner_->set_traffic_route_topology(lines);//规划拓扑路线
          //   lines.clear();
          //   ROS_INFO("Ready!");
          //   line_ready_ = false;
          // }

          // if(path_ready_){
          // //可视化起终点路线，绿色
          // half_struct_planner_->visSGPath(paths);
          // //可视化拓扑路线，蓝色
          // half_struct_planner_->vistopologyPath(paths_);
          // half_struct_planner_->calculate_pre_graph(lines, distance_table);//显示路点标号，设置距离矩阵
          // ROS_INFO("Finish!");
          // paths.clear();
          // paths_.clear();
          // lines.clear();
          // distance_table.clear();
          // cur_state_ = StateValue::Run;
          // path_ready_ = false;
          // }

          break;
        case StateValue::Pause:
          // pause_state();
          break;

        case StateValue::Record_Traffic_route://录制路网，就是拓扑关系
          break;
          
        case StateValue::Run://路径跟随/走到路径起点/避障等
          // running_state();
          if(has_start && has_goal){
            // ROS_INFO("Start!");
            //由起终点取交通规则中的路径点（一堆带位姿的点）
            // auto poses = half_struct_planner_->get_path(start, goal);//poses为一堆带姿态的点

            // Vec2d start_pose{start.pose.position.x, start.pose.position.y};
            // Vec2d goal_pose{goal.pose.position.x, goal.pose.position.y};
            // hybrid_astar_ptr->Astarplan(start_pose, goal_pose);

            Vec3d start_pose = Eigen::Vector3d {start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation)};//起点
            Vec3d goal_pose = Eigen::Vector3d {goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation)};//终点
            half_struct_planner_->HybridAstarplan(start_pose, goal_pose);
            // if (poses.empty()) {
            //   ROS_INFO("Traffic path not found!");
            //   has_start = false;
            //   has_goal = false;   
            //   return;
            // }

            has_start = false;
            has_goal = false;      
          }
          break;

        default:
          ROS_ERROR("Error StateValue!");
          break;
      }
      rate.sleep();
    }
  }


  /**
   * @brief 接收起始点位姿，设置拓扑起点集，设置起终点使用
   * 
   * @param msg 
   */
  // void SimpleAstarTest::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
  //     if (cur_state_ != StateValue::Record_Traffic_route) return;
  //     Vec3d start_pose;
  //     start_pose[0] = msg.pose.pose.position.x;
  //     start_pose[1] = msg.pose.pose.position.y;
  //     start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);//偏航角
  //     start_pose_set.emplace_back(start_pose);
  //     // has_start = true;
  //     // ROS_INFO("receive start position");
  //     //最好可视化
  //     vis.publishStartPoses(start_pose);
  // }


  //描述起始位姿
  void SimpleAstarTest::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
      // if (cur_state_ != StateValue::Run) return;
      
      Vec3d start_pose;
      
      start_pose[0] = msg.pose.pose.position.x;
      start_pose[1] = msg.pose.pose.position.y;
      start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);//偏航角
      
      start.header = msg.header;
      start.pose.position = msg.pose.pose.position;
      start.pose.orientation = msg.pose.pose.orientation;
      has_start = true;
      // ROS_INFO("receive start position");

      vis.publishStartPoses(start_pose);//起点可视化
  }


  /**
   * @brief 接收目标点位姿，一个集合，用于模拟示教路径终点位姿
   * 
   * @param msg 
   */
  // void SimpleAstarTest::goalCallback(const geometry_msgs::PoseStamped msg) {
  //     if (cur_state_ != StateValue::Record_Traffic_route) return;
  //     Vec3d goal_pose;
  //     goal_pose[0] = msg.pose.position.x;
  //     goal_pose[1] = msg.pose.position.y;
  //     goal_pose[2] = tf2::getYaw(msg.pose.orientation);//偏航角
  //     goal_pose_set.emplace_back(goal_pose);
  //     // has_goal = true;
  //     // ROS_INFO("receive goal position");
  //     vis.publishGoalPoses(goal_pose);
  // }


  //描述终点位姿
  void SimpleAstarTest::goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
      // if (cur_state_ != StateValue::Run) return;
      Vec3d goal_pose;
      goal_pose[0] = msg->pose.position.x;
      goal_pose[1] = msg->pose.position.y;
      goal_pose[2] = tf2::getYaw(msg->pose.orientation);//偏航角
      goal = *msg;

      has_goal = true;
      // ROS_INFO("receive goal position");
      vis.publishGoalPoses(goal_pose);
  }


  //接收地图信息
  // msg是一个const nav_msgs::OccupancyGridConstPtr&类型的参数，这是一个指向nav_msgs::OccupancyGrid类型对象的智能指针（boost::shared_ptr）的引用。这意味着msg本身并不拥有指向的对象，只是持有一个指向该对象的指针。
  void SimpleAstarTest::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
      //互斥锁
      //通过创建 scoped_lock 对象 lock，在函数中加锁以确保线程安全性。这意味着在执行接下来的代码时，其他试图访问 map_mutex_ 的线程将被阻塞
      boost::mutex::scoped_lock lock(map_mutex_);
      // grid_ = *msg;
      // scoped_lock 对象 lock 的生命周期已经结束，离开了这个代码块，lock 对象会自动析构，从而释放对 map_mutex_ 的占用，也就是自动解锁。

      //半结构化道路
      half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg);//创建HalfStructPlanner类
      half_struct_planner_->init();//初始化

      //A*及Hybrid *初始化
      // hybrid_astar_ptr.reset(new HybridAstar(grid_));
      ROS_INFO("Map received!");
  }


  //各任务切换用服务器，服务器服务端
    // auto SimpleAstarTest::task_service(hybrid_astar_dubins_opt_mpc::chibot_task::Request& req, hybrid_astar_dubins_opt_mpc::chibot_task::Response& resp) -> bool {
    //   std::string root_path;
    //   //参数服务，值取自launch文件
    //   ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
    //   //首先判断客户端输入的type
    //   switch (req.type) {
    //     //记录起终点
    //     case hybrid_astar_dubins_opt_mpc::chibot_task::Request::RECORD_TRAFFIC_ROUTE: {
    //       switch (req.command) {
    //         //开始录制
    //         //因为服务调用时不设置command参数，那么参数默认为0，为了复用0，这里设置了"START"
    //         case hybrid_astar_dubins_opt_mpc::chibot_task::Request::START: {
    //           if (cur_state_ != StateValue::Idle) {
    //             resp.message = "Not in IDLE status, ignore record command.";
    //             return true;
    //           }
    //           //小车处于空闲态：
    //           auto name = req.path_name;
    //           if (name == "") {//无路网文件名(指的是调用服务未设置该参数)，则赋予默认的路网文件名
    //             std::stringstream ss;
    //             ss << "inspection_traffic_route_" << std::to_string(++route_num);//命名规则
    //             name = ss.str();
    //           }
    //           auto dir = req.dir;
    //           if (dir == "") {//未设置文件夹名
    //             dir = root_path + "/topology_map/";//map文件夹
    //           }
    //           map_path_ = dir + name;//完整路径

    //           //直到traffic_route文件不存在，即是新的文件，退出循环
    //           // while (check_file_exist(map_path_)) {
    //           //   resp.message = map_path_ + " already exist, will generate new traffic route file.";
    //           //   std::stringstream ss;
    //           //   ss << "inspection_traffic_route_" << std::to_string(++route_num);//命名规则
    //           //   name = ss.str();
    //           //   map_path_ = dir + name;//新的文件名
    //           // }

    //           start_pose_set.clear();
    //           goal_pose_set.clear();
    //           cur_state_ = StateValue::Record_Traffic_route;//切换到录制状态
    //           resp.message = "Start recording traffic route, save to " + map_path_ + " .";
    //           return true;
    //         }

    //         case hybrid_astar_dubins_opt_mpc::chibot_task::Request::KEEP_TRAFFIC_ROUTE://保存路网
    //         case hybrid_astar_dubins_opt_mpc::chibot_task::Request::DISCARD: {//丢弃路网
    //           if (cur_state_ != StateValue::Record_Traffic_route) {
    //             resp.message = "Not in Record Traffic Route status, ignore command.";
    //             return true;
    //           }
    //           cur_state_ = StateValue::Idle;
    //           //丢弃当前录制
    //           if (req.command == hybrid_astar_dubins_opt_mpc::chibot_task::Request::DISCARD) {
    //             route_num--;//继续延用当前的文件名
    //             resp.message = "Do not save traffic route, discard recorded data.";
    //             //清空操作在最后
    //           } else {
    //             //保存路网
    //             //保存半结构化道路首尾点的，手动把拓扑关系写进文件：终点-->另一边的起点
    //             //将路点（路网起终点有序写入文件），允许追加数据
    //             if (!write_traffic_route(map_path_, start_pose_set, goal_pose_set)) {
    //               resp.message = "Write file failed.";
    //             } else {
    //               resp.message = "Save traffic route successful.";
    //             }
    //           }
    //           vis.vis_clear();//清除可视化
    //           start_pose_set.clear();
    //           goal_pose_set.clear();
    //           return true;
    //         }
    //         default:
    //           resp.message = "Illegal service command.";
    //           break;
    //       }
    //       return true;
    //     }

    //     //加载交通规则，而后开始规划路线
    //     case hybrid_astar_dubins_opt_mpc::chibot_task::Request::LOAD_TRAFFIC_ROUTE: {
    //       //这个type不用管command参数是什么
    //       if (cur_state_ != StateValue::Idle) {
    //         resp.message = "Not in IDLE status, ignore load_traffic_route command.";
    //         return true;
    //       }
    //       //小车处于空闲态：
    //       auto name = req.path_name;
    //       if (name == "") {//未指定路网文件名(指的是调用服务未设置该参数)
    //         resp.message = "Not set traffic route file.";
    //         return true;
    //       }
    //       auto dir = req.dir;
    //       if (dir == "") {//未设置文件夹名
    //         dir = root_path + "/topology_map/";//map文件夹
    //       }
    //       map_path_ = dir + name;//完整路径

    //       //需要检查文件名重复性
    //       if (!check_file_exist(map_path_)) {//不存在文件
    //         resp.message = "File does not exist, need record new traffic route.";
    //         return true;
    //       }

    //       if (!read_traffic_route(map_path_, lines)) {//读取文件中的路点信息至lines中，一个line是一条边（不包含拓扑）
    //         resp.message = "read file failed.";
    //       } else {
    //         resp.message = "read file successful, got " + std::to_string(lines.size()) + " lines";
    //       }

    //       line_ready_ = true;

    //       return true;
    //     }

    //     //加载路线
    //     case hybrid_astar_dubins_opt_mpc::chibot_task::Request::LOAD_PATH: {
    //       //这个type不用管command参数是什么
    //       if (cur_state_ != StateValue::Idle) {
    //         resp.message = "Not in IDLE status, ignore load_traffic_route command.";
    //         return true;
    //       }
    //       //空闲态：
    //       auto name = req.path_name;
    //       if (name == "") {//未指定路网文件名(指的是调用服务未设置该参数)
    //         resp.message = "Not set path file.";
    //         return true;
    //       }
    //       //借用dir放topology_path里的topology_path路径
    //       auto dir = req.dir;
    //       if (dir == "") {//未设置文件夹名
    //         resp.message = "Not set topology path file.";
    //         return true;
    //       }
    //       file_path_ = root_path + "/path/" + name;//完整路径

    //       //需要检查文件名重复性
    //       if (!check_file_exist(file_path_)) {//不存在文件
    //         resp.message = "File does not exist, need record new traffic route.";
    //         return true;
    //       }

    //       if (!read_file(file_path_, paths)) {//读取文件中的路点信息至lines中，一个line是一条边（不包含拓扑）
    //         resp.message = "read path file failed.";
    //         return true;
    //       } else {
    //         file_path_ = root_path + "/topology_path/" + dir;//topology_path完整路径
    //         if (!read_file(file_path_, paths_)) {//读取文件中的路点信息至lines中，拓扑
    //           resp.message = "read topology path file failed.";
    //           return true;
    //         } 
    //         resp.message = "read path file successful.";

    //         map_path_ = root_path + "/topology_map/" + "inspection_traffic_route_2";//完整路径及拓扑关系
    //         read_traffic_route(map_path_, lines);
        
    //         //在主循环中可视化
    //         path_ready_ = true;
    //       }



    //       return true;
    //     }

    //     default: {
    //       ROS_ERROR("Illegal service type %d", req.type);
    //       break;
    //     }
    //   }

    //   return true;
    // }



  //将存储的路点保存到文件里
  // auto SimpleAstarTest::write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool {
  //   if ( (line_start.size()+line_goal.size()) % 2 != 0) {
  //     ROS_ERROR("line points is record, currently not support.");
  //     return false;
  //   }
  //   std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
  //   if (!out.is_open()) {
  //     ROS_ERROR("Open file %s failed!", file_path.c_str());
  //     return false;
  //   }
  //   for (auto i = 0; i < line_start.size(); i ++) {
  //     out << std::to_string(line_start[i][0]) << " " << std::to_string(line_start[i][1]) << " " <<  std::to_string(line_start[i][2])
  //         << " ===> "
  //         << std::to_string(line_goal[i][0]) << " " << std::to_string(line_goal[i][1]) << " " <<  std::to_string(line_goal[i][2])
  //         << "\n";
  //   }

  //   out.close();
  //   return true;
  // }

  //检查文件是否存在，接受一个文件路径作为参数
  // auto SimpleAstarTest::check_file_exist(std::string& file_path) -> bool {
  // std::ifstream exist(file_path.c_str());
  // return !!exist; //!!exist将exist对象转换为布尔值
  // }


  //读取文件中的路点信息
  // auto SimpleAstarTest::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
  //   std::ifstream in(file_path.c_str());
  //   if (!in.is_open()) {
  //     ROS_ERROR("Open file %s failed!", file_path.c_str());
  //     return false;
  //   }
  //   line_t line;
  //   std::string contend, temp;
  //   std::vector<std::string> temps;
  //   while (getline(in, contend)) {
  //     temps.clear();
  //     temp.clear();
  //     for (auto const& c : contend) {
  //       if (c != ' ' && c != '=' && c != '>') {
  //         temp += c;
  //       } else if (!temp.empty()) {
  //         temps.emplace_back(temp);
  //         temp.clear();
  //       }
  //     }
  //     if (!temp.empty()) temps.emplace_back(temp);
  //     if (temps.size() < 6) continue;
  //     line.a.x = std::stod(temps[0]);
  //     line.a.y = std::stod(temps[1]);
  //     line.a.theta = std::stod(temps[2]);
  //     line.b.x = std::stod(temps[3]);
  //     line.b.y = std::stod(temps[4]);
  //     line.b.theta = std::stod(temps[5]);
  //     line.go_list.clear();
  //     for (auto i = 6; i < temps.size(); ++i) {
  //       line.go_list.emplace_back(std::stoi(temps[i]));
  //     }
  //     lines.emplace_back(line);
  //   }

  //   return !lines.empty();
  // }


  //读取路线
  // auto SimpleAstarTest::read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool {
  //   std::ifstream in(file_path.c_str());
  //   if (!in.is_open()) {
  //     ROS_ERROR("Open file %s failed!", file_path.c_str());
  //     return false;
  //   }
    
  //   std::vector<Eigen::Vector3d> route;
    
  //   std::string contend, temp;
  //   std::vector<std::string> temps;
  //   while (getline(in, contend)) {
  //     if (contend == "EOP" && !route.empty()) {
  //       routes.emplace_back(route);
  //       route.clear();
  //       continue;
  //     }

  //     temps.clear();
  //     temp.clear();
  //     for (auto const& c : contend) {
  //       if (c != ' ') {
  //         temp += c;
  //       } else {
  //         temps.emplace_back(temp);
  //         temp.clear();
  //       }
  //     }
  //     if (!temp.empty()) temps.emplace_back(temp);
  //     if (temps.size() != 3 && temps.size()!= 1) continue;

  //     //拿个表格装所有的距离值
  //     if(temps.size() == 1) distance_table.emplace_back(std::stod(temps[0]));
  //     else{
  //     Eigen::Vector3d p;
  //     p[0] = std::stod(temps[0]);
  //     p[1] = std::stod(temps[1]);
  //     p[2] = std::stod(temps[2]);
  //     route.emplace_back(p);
  //     }
  //   }

  //   return !routes.empty();
  // }



  
}