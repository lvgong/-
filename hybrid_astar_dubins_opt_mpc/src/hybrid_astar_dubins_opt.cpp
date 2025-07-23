#include "hybrid_astar_dubins_opt.h"


namespace chibot::topologyplanning {

  //增大ts，在同等路径下，要求完成路径所需速度更小，表现为“MPC/v_max”可以更小，tmp=traj_duration_越大
  //ts=0.5m（路径点间隔）/0.3m/s（平均速度）=1.67s
  //适当提高T：t_step，配合N可以延长预测路线长度
  double ts=1.67, t_step = 0.25;
  #define N 25 //15步  影响耗时
  #define PI 3.1415926

  HybridAstarDubinsOpt::HybridAstarDubinsOpt():
  line_ready_(false),
  // has_start(false),
  has_goal(false),
  route_num(3),
  tf_(new tf2_ros::Buffer()),
  tfl_(new tf2_ros::TransformListener(*tf_)),
  receive_traj_(false){}

  //初始化
  void HybridAstarDubinsOpt::init() {
      ros::NodeHandle nh;
      //订阅栅格地图，这个带有回调函数的必须放在主程序里，且快入快出
      map_sub_ = nh.subscribe("/map", 1, &HybridAstarDubinsOpt::mapReceived, this);
      //设置拓扑起点集
      // start_sub = nh.subscribe("/initialpose", 1, &HybridAstarDubinsOpt::startCallback, this);
      //设置拓扑终点集
      goal_sub = nh.subscribe("/move_base_simple/goal", 1, &HybridAstarDubinsOpt::goalCallback, this);

      //各种服务调用，服务器建立，很重要的一块
      task_srv_ = nh.advertiseService(TASK_SRV, &HybridAstarDubinsOpt::task_service, this);
      
      vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布速度指令
      //可视化B样条轨迹
      trajectory_pub = nh.advertise<nav_msgs::Path>("/Bspline_trajectory",10);
      //MPC预测轨迹
      trajectory_predict_pub = nh.advertise<nav_msgs::Path>("/trajectory_predict",10);
    
      mpc_controller.MPC_init(nh);

      //等获得了B样条轨迹后再开启定时器
      //MPC周期控制
      cmd_timer = nh.createTimer(ros::Duration(0.05), &HybridAstarDubinsOpt::cmdCallback, this);
      
      //mpc有关参数 
      mpc_arg_pub_ = nh.advertise<mpc_arg>("/mpc_arg", 10);
  }

  //主循环函数，给初始点和目标点就生成一条路径
  void HybridAstarDubinsOpt::run() {
    ros::Rate rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      //任务状态机，状态的切换在各事件中
      switch (cur_state_) {
        case StateValue::Idle:
          // do nothing, cancel goal and reset value in event 取消导航和清零操作在事件中完成
          
          if(line_ready_){
            // half_struct_planner_->set_traffic_route(lines);//规划起终点间的路线并保存
            half_struct_planner_->set_traffic_route_topology(lines);//规划拓扑路线
            lines.clear();
            lines.shrink_to_fit();
            ROS_INFO("Ready!");
            line_ready_ = false;
          }

          if(path_ready_){
          //可视化起终点路线，绿色
          half_struct_planner_->visSGPath(paths);
          ROS_INFO("visSGPath!");
          paths.clear();
          paths.shrink_to_fit();// 尽可能地减小vector的容量
          //可视化拓扑路线，蓝色
          half_struct_planner_->vistopologyPath(paths_);
          ROS_INFO("vistopologyPath!");
          paths_.clear();
          paths_.shrink_to_fit();
          half_struct_planner_->calculate_pre_graph(lines, distance_table);//显示路点标号，设置距离矩阵
          ROS_INFO("Finish!");
          lines.clear();
          lines.shrink_to_fit();
          distance_table.clear();
          distance_table.shrink_to_fit();
          cur_state_ = StateValue::Run;
          path_ready_ = false;
          }

          break;
        case StateValue::Pause:
          // pause_state();
          break;

        case StateValue::Record_Traffic_route://录制路网，就是拓扑关系
          break;
          
        case StateValue::Run://路径跟随/走到路径起点/避障等
          // running_state();
          // if(has_start && has_goal){
          if( has_goal){
            ROS_INFO("Start!");
            //由起终点取交通规则中的路径点（一堆带位姿的点）
            // auto hybrid_astar_start_ = chrono::high_resolution_clock::now();//初始化

            start.header.frame_id = "map";
            start.header.stamp = ros::Time::now();
            start.pose = robot_pose().value();//获取小车当前位姿作为起点
            //可视化起点位姿
            Vec3d start_pose;
            start_pose[0] = start.pose.position.x;
            start_pose[1] = start.pose.position.y;
            start_pose[2] = tf2::getYaw(start.pose.orientation);//偏航角
            vis.publishStartPoses(start_pose);

            auto poses = half_struct_planner_->get_path(start, goal);//poses为一堆带姿态的点     
            if (poses.empty()) {
              ROS_INFO("Traffic path not found!");
              // has_start = false;
              has_goal = false;   
              return;
            }

            //初始化控制点矩阵(2D)
            MatrixXd control_points = MatrixXd::Zero(2, poses.size());
            for(int i=0;i<poses.size();i++)
            {
                //把得到的路径点作为控制点
                Vector2d current_point(poses[i].pose.position.x, poses[i].pose.position.y);
                control_points.col(i) = current_point;
            }        
            // 创建新的均匀B样条曲线pos_traj
            UniformBspline pos_traj(control_points, 3, ts);//3次4阶
            // 这个没必要暂时，x，y速度不好定，差速车没有横向速度
            // pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置x、y速度上限、加速度上限、容忍值

            // double ratio;
            // if (!pos_traj.checkFeasibility(ratio, false))
            // {
            //     cout << "Need to reallocate time." << endl;
            //     pos_traj.lengthenTime(ratio);
            // }

            // B_spline_trajectory.setUniformBspline(control_points,3,ts);//次4阶
            // 清空轨迹列表traj_，然后将pos_traj、它的一阶导数和二阶导数依次添加到traj_中。
            traj_.clear();
            traj_.push_back(pos_traj);//位置
            traj_.push_back(traj_[0].getDerivative());//速度
            // traj_.push_back(traj_[1].getDerivative());//加速度

            double t,tmp;
            //节点向量定义区间
            traj_[0].getTimeSpan(t,tmp);
            
            // B_spline_trajectory.getTimeSpan(t,tmp);
              
            ROS_INFO("t:%f",t);  //t=0.0s
            ROS_INFO("tmp:%f",tmp);  //tmp=traj_duration_
          // 计算轨迹的总时长，并将结果赋值给traj_duration_变量。
            traj_duration_ = traj_[0].getTimeSum();/////总时间
            ROS_INFO("traj_duration_:%f",traj_duration_); 

              nav_msgs::Path b_spline_trajectory;
              b_spline_trajectory.header.frame_id = "map";
              for(double t_c=t;t_c<tmp;t_c+=t_step)
              {
                  Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
                  // Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
                  geometry_msgs::PoseStamped current_pose;
                  current_pose.pose.position.x = point_temp(0);//轨迹点
                  current_pose.pose.position.y = point_temp(1);
                  current_pose.header.frame_id = "/map";
                  b_spline_trajectory.poses.push_back(current_pose);//样条曲线
              }
              trajectory_pub.publish(b_spline_trajectory);//可视化完整的B样条曲线

            start_time_ = ros::Time::now();
            //start_time_取t，将double装ros::Time格式
            // start_time_ = ros::Time::  t

          // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
            receive_traj_ = true;

            // has_start = false;
            has_goal = false;    

            // auto hybrid_astar_end = chrono::high_resolution_clock::now();
            // chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start_;
            // cout << "路网规划时间:" << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
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
  // void HybridAstarDubinsOpt::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
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
  void HybridAstarDubinsOpt::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
      if (cur_state_ != StateValue::Run) return;
      
      Vec3d start_pose;
      
      start_pose[0] = msg.pose.pose.position.x;
      start_pose[1] = msg.pose.pose.position.y;
      start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);//偏航角
      
      start.header = msg.header;
      start.pose.position = msg.pose.pose.position;
      start.pose.orientation = msg.pose.pose.orientation;
      has_start = true;
      // ROS_INFO("receive start position");

      //最好可视化
      vis.publishStartPoses(start_pose);
  }


  /**
   * @brief 接收目标点位姿，一个集合，用于模拟示教路径终点位姿
   * 
   * @param msg 
   */
  // void HybridAstarDubinsOpt::goalCallback(const geometry_msgs::PoseStamped msg) {
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
  void HybridAstarDubinsOpt::goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
      if (cur_state_ != StateValue::Run) return;
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
  void HybridAstarDubinsOpt::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
      //互斥锁
      //通过创建 scoped_lock 对象 lock，在函数中加锁以确保线程安全性。这意味着在执行接下来的代码时，其他试图访问 map_mutex_ 的线程将被阻塞
      boost::mutex::scoped_lock lock(map_mutex_);
      // grid_ = *msg;
      // scoped_lock 对象 lock 的生命周期已经结束，离开了这个代码块，lock 对象会自动析构，从而释放对 map_mutex_ 的占用，也就是自动解锁。

      //半结构化道路
      half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg);//创建HalfStructPlanner类
      half_struct_planner_->init();//初始化
  }


  //各任务切换用服务器，服务器服务端
    auto HybridAstarDubinsOpt::task_service(hybrid_astar_dubins_opt_mpc::chibot_task::Request& req, hybrid_astar_dubins_opt_mpc::chibot_task::Response& resp) -> bool {
      std::string root_path;
      //参数服务，值取自launch文件
      ROS_ERROR_COND(!ros::param::get("/root_path", root_path), "Get root path failed!");
      //首先判断客户端输入的type
      switch (req.type) {
        //记录起终点
        case hybrid_astar_dubins_opt_mpc::chibot_task::Request::RECORD_TRAFFIC_ROUTE: {
          switch (req.command) {
            //开始录制
            //因为服务调用时不设置command参数，那么参数默认为0，为了复用0，这里设置了"START"
            case hybrid_astar_dubins_opt_mpc::chibot_task::Request::START: {
              if (cur_state_ != StateValue::Idle) {
                resp.message = "Not in IDLE status, ignore record command.";
                return true;
              }
              //小车处于空闲态：
              auto name = req.path_name;
              if (name == "") {//无路网文件名(指的是调用服务未设置该参数)，则赋予默认的路网文件名
                std::stringstream ss;
                ss << "inspection_traffic_route_" << std::to_string(++route_num);//命名规则
                name = ss.str();
              }
              auto dir = req.dir;
              if (dir == "") {//未设置文件夹名
                dir = root_path + "/topology_map/";//map文件夹
              }
              map_path_ = dir + name;//完整路径

              //直到traffic_route文件不存在，即是新的文件，退出循环
              // while (check_file_exist(map_path_)) {
              //   resp.message = map_path_ + " already exist, will generate new traffic route file.";
              //   std::stringstream ss;
              //   ss << "inspection_traffic_route_" << std::to_string(++route_num);//命名规则
              //   name = ss.str();
              //   map_path_ = dir + name;//新的文件名
              // }

              start_pose_set.clear();
              goal_pose_set.clear();
              cur_state_ = StateValue::Record_Traffic_route;//切换到录制状态
              resp.message = "Start recording traffic route, save to " + map_path_ + " .";
              return true;
            }

            case hybrid_astar_dubins_opt_mpc::chibot_task::Request::KEEP_TRAFFIC_ROUTE://保存路网
            case hybrid_astar_dubins_opt_mpc::chibot_task::Request::DISCARD: {//丢弃路网
              if (cur_state_ != StateValue::Record_Traffic_route) {
                resp.message = "Not in Record Traffic Route status, ignore command.";
                return true;
              }
              cur_state_ = StateValue::Idle;
              //丢弃当前录制
              if (req.command == hybrid_astar_dubins_opt_mpc::chibot_task::Request::DISCARD) {
                route_num--;//继续延用当前的文件名
                resp.message = "Do not save traffic route, discard recorded data.";
                //清空操作在最后
              } else {
                //保存路网
                //保存半结构化道路首尾点的，手动把拓扑关系写进文件：终点-->另一边的起点
                //将路点（路网起终点有序写入文件），允许追加数据
                if (!write_traffic_route(map_path_, start_pose_set, goal_pose_set)) {
                  resp.message = "Write file failed.";
                } else {
                  resp.message = "Save traffic route successful.";
                }
              }
              vis.vis_clear();//清除可视化
              start_pose_set.clear();
              goal_pose_set.clear();
              return true;
            }
            default:
              resp.message = "Illegal service command.";
              break;
          }
          return true;
        }

        //加载交通规则，而后开始规划路线
        case hybrid_astar_dubins_opt_mpc::chibot_task::Request::LOAD_TRAFFIC_ROUTE: {
          //这个type不用管command参数是什么
          if (cur_state_ != StateValue::Idle) {
            resp.message = "Not in IDLE status, ignore load_traffic_route command.";
            return true;
          }
          //小车处于空闲态：
          auto name = req.path_name;
          if (name == "") {//未指定路网文件名(指的是调用服务未设置该参数)
            resp.message = "Not set traffic route file.";
            return true;
          }
          auto dir = req.dir;
          if (dir == "") {//未设置文件夹名
            dir = root_path + "/topology_map/";//map文件夹
          }
          map_path_ = dir + name;//完整路径

          //需要检查文件名重复性
          if (!check_file_exist(map_path_)) {//不存在文件
            resp.message = "File does not exist, need record new traffic route.";
            return true;
          }

          if (!read_traffic_route(map_path_, lines)) {//读取文件中的路点信息至lines中，一个line是一条边（不包含拓扑）
            resp.message = "read file failed.";
          } else {
            resp.message = "read file successful, got " + std::to_string(lines.size()) + " lines";
          }

          line_ready_ = true;

          return true;
        }

        //加载路线
        case hybrid_astar_dubins_opt_mpc::chibot_task::Request::LOAD_PATH: {
          //这个type不用管command参数是什么
          if (cur_state_ != StateValue::Idle) {
            resp.message = "Not in IDLE status, ignore load_traffic_route command.";
            return true;
          }
          //空闲态：
          auto name = req.path_name;
          if (name == "") {//未指定路网文件名(指的是调用服务未设置该参数)
            resp.message = "Not set path file.";
            return true;
          }
          //借用dir放topology_path里的topology_path路径
          auto dir = req.dir;
          if (dir == "") {//未设置文件夹名
            resp.message = "Not set topology path file.";
            return true;
          }
          file_path_ = root_path + "/path/" + name;//完整路径

          //需要检查文件名重复性
          if (!check_file_exist(file_path_)) {//不存在文件
            resp.message = "File does not exist, need record new traffic route.";
            return true;
          }

          if (!read_file(file_path_, paths)) {//读取文件中的路点信息至lines中，一个line是一条边（不包含拓扑）
            resp.message = "read path file failed.";
            return true;
          } else {
            file_path_ = root_path + "/topology_path/" + dir;//topology_path完整路径
            if (!read_file(file_path_, paths_)) {//读取文件中的路点信息至lines中，拓扑
              resp.message = "read topology path file failed.";
              return true;
            } 
            resp.message = "read path file successful.";


            // 这个需要手动改
            map_path_ = root_path + "/topology_map/" + "inspection_traffic_route_3";//完整路径及拓扑关系
            read_traffic_route(map_path_, lines);
        
            //在主循环中可视化
            path_ready_ = true;
          }



          return true;
        }

        default: {
          ROS_ERROR("Illegal service type %d", req.type);
          break;
        }
      }

      return true;
    }



  //将存储的路点保存到文件里
  auto HybridAstarDubinsOpt::write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool {
    if ( (line_start.size()+line_goal.size()) % 2 != 0) {
      ROS_ERROR("line points is record, currently not support.");
      return false;
    }
    std::ofstream out(file_path.c_str(), std::ios_base::out | std::ios_base::app);
    if (!out.is_open()) {
      ROS_ERROR("Open file %s failed!", file_path.c_str());
      return false;
    }
    for (auto i = 0; i < line_start.size(); i ++) {
      out << std::to_string(line_start[i][0]) << " " << std::to_string(line_start[i][1]) << " " <<  std::to_string(line_start[i][2])
          << " ===> "
          << std::to_string(line_goal[i][0]) << " " << std::to_string(line_goal[i][1]) << " " <<  std::to_string(line_goal[i][2])
          << "\n";
    }

    out.close();
    return true;
  }


  //检查文件是否存在，接受一个文件路径作为参数
  auto HybridAstarDubinsOpt::check_file_exist(std::string& file_path) -> bool {
  std::ifstream exist(file_path.c_str());
  return !!exist; //!!exist将exist对象转换为布尔值
  }


  //读取文件中的路点信息
  auto HybridAstarDubinsOpt::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
    std::ifstream in(file_path.c_str());
    if (!in.is_open()) {
      ROS_ERROR("Open file %s failed!", file_path.c_str());
      return false;
    }
    line_t line;
    std::string contend, temp;
    std::vector<std::string> temps;
    while (getline(in, contend)) {
      temps.clear();
      temp.clear();
      for (auto const& c : contend) {
        if (c != ' ' && c != '=' && c != '>') {
          temp += c;
        } else if (!temp.empty()) {
          temps.emplace_back(temp);
          temp.clear();
        }
      }
      if (!temp.empty()) temps.emplace_back(temp);
      if (temps.size() < 6) continue;
      line.a.x = std::stod(temps[0]);
      line.a.y = std::stod(temps[1]);
      line.a.theta = std::stod(temps[2]);
      line.b.x = std::stod(temps[3]);
      line.b.y = std::stod(temps[4]);
      line.b.theta = std::stod(temps[5]);
      line.go_list.clear();
      for (auto i = 6; i < temps.size(); ++i) {
        line.go_list.emplace_back(std::stoi(temps[i]));
      }
      lines.emplace_back(line);
    }

    return !lines.empty();
  }


  //读取路线
  auto HybridAstarDubinsOpt::read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool {
    std::ifstream in(file_path.c_str());
    if (!in.is_open()) {
      ROS_ERROR("Open file %s failed!", file_path.c_str());
      return false;
    }
    
    std::vector<Eigen::Vector3d> route;
    
    std::string contend, temp;
    std::vector<std::string> temps;
    while (getline(in, contend)) {
      if (contend == "EOP" && !route.empty()) {
        routes.emplace_back(route);
        route.clear();
        continue;
      }

      temps.clear();
      temp.clear();
      for (auto const& c : contend) {
        if (c != ' ') {
          temp += c;
        } else {
          temps.emplace_back(temp);
          temp.clear();
        }
      }
      if (!temp.empty()) temps.emplace_back(temp);
      if (temps.size() != 3 && temps.size()!= 1) continue;

      //拿个表格装所有的距离值
      if(temps.size() == 1) distance_table.emplace_back(std::stod(temps[0]));
      else{
      Eigen::Vector3d p;
      p[0] = std::stod(temps[0]);
      p[1] = std::stod(temps[1]);
      p[2] = std::stod(temps[2]);
      route.emplace_back(p);
      }
    }

    return !routes.empty();
  }

  //求机器人在地图中的位姿
  auto HybridAstarDubinsOpt::robot_pose() -> std::optional<geometry_msgs::Pose> {
    /*这段代码使用tf2_ros::Buffer对象的lookupTransform()方法来获取从map坐标系到base_link坐标系的变换信息。
      使用try-catch语句块来捕获可能抛出的异常。在try块中，调用tf_->lookupTransform("map", "base_link", ros::Time(0))来获取变换信息。这个函数的参数分别是目标坐标系（"map"）和源坐标系（"base_link"），以及时间戳（ros::Time(0)表示最新的变换）。函数将返回一个geometry_msgs::TransformStamped类型的变量，包含了变换的详细信息。
      如果在执行lookupTransform()时发生了异常，会被catch块捕获。捕获到异常后，会打印警告消息，并返回一个std::nullopt值，表示获取变换失败。
      总结起来，这段代码的作用是通过tf2_ros::Buffer对象的lookupTransform()方法获取从map坐标系到base_link坐标系的变换信息，并在出现异常时打印警告消息并返回失败的标志。*/
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tf_->lookupTransform("map", "base_link", ros::Time(0));
    } catch (tf2::TransformException &ex) {
      /*这段代码是一个try-catch语句块的catch块，用于捕获tf2::TransformException类型的异常。
      当try块中的代码抛出tf2::TransformException类型的异常时，程序会跳转到这个catch块中。&ex表示将捕获的异常对象以引用的方式传递给catch块。
      在catch块中，调用了ROS_WARN()函数来打印一个警告消息，消息内容为字符串格式化后的"tf error: %s"与异常对象的错误信息ex.what()。其中%s表示将在后面填充一个字符串，这里就是异常对象的错误信息。
      总的来说，这段代码的作用是在try块中执行特定的代码，如果出现了tf2::TransformException类型的异常，则跳转到catch块中处理。在这个catch块中，通过ROS的日志功能打印了一个警告消息，包含了异常对象的错误信息。*/
      ROS_WARN("tf error: %s", ex.what());
      return std::nullopt;
    }
    geometry_msgs::Pose result;
    result.position.x = transformStamped.transform.translation.x;
    result.position.y = transformStamped.transform.translation.y;
    result.orientation = transformStamped.transform.rotation;
    /*std::make_optional(result)是一个C++17中的函数模板，用于创建一个std::optional对象，并通过拷贝或移动构造函数将给定的result值包装在std::optional中。
      std::optional是一个可选值的容器类模板，它可以表示某个值的存在或不存在。当你想要返回一个可能为空的结果时，可以使用std::optional来封装这个结果。
      在这里，std::make_optional(result)将result的值包装在一个std::optional对象中，并返回这个对象。如果result是一个左值（例如一个变量），则会使用拷贝构造函数进行包装；如果result是一个右值（例如一个临时对象），则会使用移动构造函数进行包装。最终，你将得到一个包含了result值的std::optional对象。
      需要注意的是，要使用std::make_optional()函数，你的编译器需要支持C++17标准或更高版本。否则，你可以手动构造std::optional对象并进行赋值操作。*/
    return std::make_optional(result);
  }


void HybridAstarDubinsOpt::cmdCallback(const ros::TimerEvent &e)
{
    if (!receive_traj_){
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        vel_cmd_pub.publish(cmd);
        // is_orientation_init=false;
        return;
    }

    //ROS_WARN("Run here !");
    // static ros::Time time_last = ros::Time::now();
    ros::Time time_s = ros::Time::now();
    // 计算当前时间与轨迹起始时间之间的差值（以秒为单位）
    // 计算现在的时间和起始时间的间隔
    double t_cur = (time_s - start_time_).toSec();//时间差值

// 如果t_cur小于轨迹总时长traj_duration_且大于等于0，则调用MPC_calculate函数进行控制计算。同时记录控制时间

    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        ////time record
        //ROS_INFO("MPC_Calculate!");
        // clock_t start_clock = clock();
        MPC_calculate(t_cur);
        // clock_t end_clock = clock();
        // double duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        // ROS_INFO("Control times : %f ms",duration);
    }
    // 如果t_cur大于等于轨迹总时长，则停止机器人，并将is_orientation_init设置为false
    else if (t_cur >= traj_duration_)
    {
        cmd.angular.z = 0;
        cmd.linear.x = 0;
        // vel_cmd_pub.publish(cmd);
        // is_orientation_init=false;
        receive_traj_=false;
    }
    else cout << "[Traj server]: invalid time." << endl;
    
    // time_last = time_s;
     vel_cmd_pub.publish(cmd);
}

void HybridAstarDubinsOpt::MPC_calculate(double &t_cur)
{
    clock_t start_clock = clock();

    std::vector<Eigen::Vector3d> X_r;//参考状态
    std::vector<Eigen::Vector2d> U_r;//参考输入
    Eigen::MatrixXd u_k;//计算得到的控制输入
    Eigen::Vector2d pos_r,pos_r_1,pos_final,v_r_1,v_r_2;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r,x_r_1,x_r_2,X_k;
    double v_linear_1,w;
    double t_k,t_k_1;

    // ROS_INFO("Run to here!");

    //Eigen::Vector3d pos_first = traj_[0].evaluateDeBoor(t_cur);
    //Eigen::Vector3d pos_second = traj_[0].evaluateDeBoor(t_cur+t_step);
    Eigen::Vector2d vel_start = traj_[1].evaluateDeBoor(t_cur);//初始速度
    double yaw_start = atan2(vel_start(1),vel_start(0));//vel_start(1)：y向速度   vel_start(0)：x向速度
    bool is_orientation_adjust=false;
    double orientation_adjust=0;
    // pos_final = traj_[0].evaluateDeBoor(traj_duration_ - t_step);//目标位置，往前一个时刻，能够算出最后一个yaw角

    // ROS_INFO("Run to here-1!");
//    if(abs(yaw-yaw_start)>yaw_error_max&&is_orientation_init==false)
//    {
//        ROS_INFO("current yaw : %5.3f , start yaw : %5.3f",yaw,yaw_start);
//        if(abs(yaw-yaw_start)>PI)
//        {
//            cmd.twist.linear.x=0;
//            cmd.twist.angular.z = -(yaw_start-yaw)/abs(yaw-yaw_start)*PI/10;
//        }
//        else
//        {
//            cmd.twist.linear.x=0;
//            cmd.twist.angular.z = (yaw_start-yaw)/abs(yaw-yaw_start)*PI/10;
//        }
//        vel_cmd_pub.publish(cmd);
//        start_time_ = ros::Time::now();
//        //cout<<"current yaw : "<<yaw*180/PI<<endl;
//        //cout<<"target yaw : "<<yaw_start*180/PI<<endl;
//        //cout<<"current w : "<<web_cmd_vel.angular.z<<endl;
//    }
//    else
//    {
        // is_orientation_init=true;
        for(int i=0;i<N;i++) //N：预测步数
        {
            t_k = t_cur+i*t_step;//i时刻
            t_k_1 = t_cur+(i+1)*t_step;//下一时刻i+1

            //函数内部自带超限判定,所以时间超限也没有关系
            pos_r = traj_[0].evaluateDeBoor(t_k);//i时刻参考位置
            pos_r_1 = traj_[0].evaluateDeBoor(t_k_1);//i+1时刻参考位置

            x_r(0) = pos_r(0);//i时刻参考x坐标
            x_r(1) = pos_r(1);//y坐标

            v_r_1 = traj_[1].evaluateDeBoor(t_k);//i时刻参考速度向量
            v_r_2 = traj_[1].evaluateDeBoor(t_k_1);//i+1时刻参考速度
            // v_r_1(2)=0;
            // v_r_2(2)=0;
            v_linear_1 = v_r_1.norm();//范数，就是i时刻参考线速度
            if((t_k-traj_duration_)>=0)//>轨迹总时间
            {
                // x_r(2) = atan2((pos_r-pos_final)(1),(pos_r-pos_final)(0));//参考yaw
                //最后一个yaw改为我们目标点的yaw——在准均匀B样条曲线下：
                x_r(2) = tf2::getYaw(goal.pose.orientation);//在-3.14-3.14之间
                // auto yaw = tf2::getYaw(goal.pose.orientation);
                // if(abs(yaw)>(PI*5/6))//当前yaw
                // {
                //     if(yaw<0)
                //     {
                //         x_r(2) = yaw + 2*PI;
                //     }
                //     else
                //     {
                //         x_r(2) = yaw - 2*PI;
                //     }
                // }
                // else
                // {
                //     x_r(2) = yaw;
                // }

            }
            else
            {
                //x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
                x_r(2) = atan2(v_r_1(1),v_r_1(0));//参考yaw为速度分量夹角
            }

            double yaw1 = atan2(v_r_1(1),v_r_1(0));//i时刻yaw
            double yaw2 = atan2(v_r_2(1),v_r_2(0));//i+1时刻yaw

// ROS_INFO("Run to here-2!");
            if(abs(yaw2-yaw1)>PI)
            {
                //ROS_WARN("orientation suddenly change !");
                //cout<<"current index : "<<i+1<<endl;
                //cout<<"yaw 1 : "<<yaw1<<endl;
                //cout<<"yaw 2 : "<<yaw2<<endl;
                is_orientation_adjust = true;
                if((yaw2-yaw1)<0)
                {
                    orientation_adjust = 2*PI;
                    w = (2*PI+(yaw2-yaw1))/t_step;
                }
                else
                {
                    w = ((yaw2-yaw1)-2*PI)/t_step;
                    orientation_adjust = -2*PI;
                }
            }
            else{  
              //对于最后一个点就是0
                w = (yaw2-yaw1)/t_step;
            }

            if(is_orientation_adjust==true){
                //cout<<"orientation before adjust : "<< x_r(2)<<endl;
                x_r(2) +=orientation_adjust;
                //cout<<"orientation after adjust : "<< x_r(2)<<endl;
            }

            u_r(0) = v_linear_1;//参考线速度
            u_r(1) = w;//参考角速度
//                if(t_c>(tp-5*t_step))
//                {
//                    cout<<"Ur "<<i+1<<" : "<<endl<<u_r<<endl;
//                    cout<<"Xr "<<i+1<<" : "<<endl<<x_r<<endl;
//                }
            X_r.push_back(x_r);//参考x集合
            U_r.push_back(u_r);//参考u集合
        }
// ROS_INFO("Run to here-3!");
        geometry_msgs::PoseStamped x_current;
        x_current.header.frame_id = "map";
        x_current.header.stamp = ros::Time::now();
        x_current.pose = robot_pose().value();//获取小车当前位姿作为起点       
        X_k(0) = x_current.pose.position.x;//当前状态x
        X_k(1) = x_current.pose.position.y;//当前y
        auto yaw = tf2::getYaw(x_current.pose.orientation);

        // 实际位置量
        mpc_par.x_k = X_k(0);
        mpc_par.y_k = X_k(1);
        mpc_par.theta_k = yaw;

        if(yaw/X_r[0](2)<0&&abs(yaw)>(PI*5/6))//当前yaw
        {
            if(yaw<0) X_k(2) = yaw + 2*PI;
            else X_k(2) = yaw - 2*PI;
        }
        else X_k(2) = yaw;
        


        // cout<<"xr  : "<<X_r[0]<<endl;
        // cout<<"xk  : "<<X_k<<endl;
// ROS_INFO("Run to here-4!");
        //ROS_INFO("Run to here!");
        // 优化控制输入：调用 MPC 控制器的函数 MPC_Solve_qp，基于当前状态和轨迹信息，计算出优化后的控制输入 u_k
        u_k = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);//当前状态，参考状态，参考输入，预测步


//            cout<<"Xk "<<" : "<<endl<<X_k<<endl;
// 发布控制指令：根据优化后的控制输入，将线速度和角速度转换为机器人的控制命令，并通过发布器 vel_cmd_pub 发布给机器人执行器
        // if(dir.data == NEGATIVE)
        // {
        //     cmd.linear.x = -u_k.col(0)(0);
        // }
        // else
        // {
        cmd.linear.x = u_k.col(0)(0);
        // }

        cmd.angular.z = u_k.col(0)(1);

    //可视化预测轨迹
    //离散差速运动学模型的系数矩阵
    MatrixXd A_k = MatrixXd::Zero(3,3);
    MatrixXd I = MatrixXd::Identity(3,3);
    MatrixXd B_k = MatrixXd::Zero(3,2);
    Vector3d X_k_1;
    geometry_msgs::PoseStamped current_pose;
    current_pose.pose.position.x = X_k(0);//当前状态
    current_pose.pose.position.y = X_k(1);
    current_pose.header.frame_id = "map";
    nav_msgs::Path trajectory_predict;
    trajectory_predict.header.frame_id = "map";
    trajectory_predict.poses.push_back(current_pose);
    for(int i=0;i<N;i++)
    {
        A_k(0,2) = -U_r[i](0)*sin(X_r[i](2));//系数矩阵
        A_k(1,2) = U_r[i](0)*cos(X_r[i](2));
        B_k(0,0) = cos(X_r[i](2));
        B_k(1,0) = sin(X_r[i](2));
        B_k(2,1) = 1;
        //u_k.col(i)(1)*=-1;
        //离散模型,由当前状态量推下一个状态量
        X_k_1 = (I+t_step*A_k)*X_k+t_step*B_k*u_k.col(i)-t_step*A_k*X_r[i];
        X_k = X_k_1;//将此下一时刻状态量赋给当前状态

        current_pose.pose.position.x = X_k_1(0);//x
        current_pose.pose.position.y = X_k_1(1);//y
        trajectory_predict.poses.push_back(current_pose);//预测点
    }

    // t_c = t + N*t_step;
    //预测终点
    // predict_end_point.point.x = B_spline_trajectory.evaluateDeBoor(t_c)(0);
    // predict_end_point.point.y = B_spline_trajectory.evaluateDeBoor(t_c)(1);

    trajectory_predict_pub.publish(trajectory_predict);//MPC预测轨迹
    // start_point_pub.publish(start_point);//起始点
    // predict_end_point_pub.publish(predict_end_point);//预测终点

        // 调试信息输出：定期输出参考速度、角速度以及优化后的速度、角速度等信息，以便监控控制效果和调试程序。
      //  static int conut1 = 0;
      //  conut1+=1;
      //  if(conut1%20==0)//调试
      //  {
      //      ROS_WARN("Ur:");
      //      for(int i=0;i<U_r.size();i++)
      //      {
      //          cout<<"vel ref:"<<U_r[i](0)<<","<<"w ref: "<<U_r[i](1);
      //          cout<<endl;
      //      }
      //      cout<<endl;
      //      ROS_WARN("Uk:");
      //     //  for(int i=0;i<u_k.cols();i++)
      //     //  {
      //         //  cout<<"vel optimal:"<<u_k.col(i)(0)<<","<<"w optimal: "<<u_k.col(i)(1);
      //          cout<<"vel optimal:"<<u_k.col(0)(0)<<","<<"w optimal: "<<u_k.col(0)(1);
      //          cout<<endl;
      //     //  }
      //     //  cout<<endl;
      //     //  cout<<"current vel: "<<u_k.col(0)(0) <<"m/s"<<endl;
      //     //  cout<<"current w: "<<u_k.col(0)(1)<<"rad/s"<<endl;
      //      conut1=0;
      //  }

        clock_t end_clock = clock();
        double duration = (double)(end_clock - start_clock) / CLOCKS_PER_SEC *1000;
        ROS_INFO("Control times : %f ms",duration);

        // 计算耗时
        mpc_par.time = duration;
        // 参考量
        mpc_par.v_r = U_r[0](0);
        mpc_par.w_r = U_r[0](1);
        mpc_par.x_r = X_r[0](0);
        mpc_par.y_r = X_r[0](1);
        mpc_par.theta_r = X_r[0](2);
        // 位置偏差 = 参考位置-实际位置
        mpc_par.dx = X_r[0](0) - mpc_par.x_k;       
        mpc_par.dy = X_r[0](1) - mpc_par.y_k;       
        mpc_par.dtheta = X_r[0](2) - mpc_par.theta_k;  
             
        mpc_arg_pub_.publish(mpc_par);

        // ROS_INFO("Run to here-5!");
        // vel_cmd_pub.publish(cmd);

    // std::vector<Eigen::Vector3d> X_r;//参考状态
    // std::vector<Eigen::Vector2d> U_r;//参考输入
    // Eigen::Vector2d pos_r,pos_r_1,pos_r_2,u_r,v_r_1,v_r_2;
    // Eigen::Vector3d x_r,x_r_1,x_r_2,X_k;
    // MatrixXd u_k;//计算得到的控制输入
    // double v_linear_1,v_linear_2,w;
    // double t_c = t_cur;//节点区间起点
    // double t_k,t_k_1,t_k_2;

    // // ros::Time t_start = ros::Time::now();

    // Eigen::Vector2d pos_first = B_spline_trajectory.evaluateDeBoor(t);//第一个轨迹点
    // Eigen::Vector2d pos_second = B_spline_trajectory.evaluateDeBoor(t+t_step);//第二个轨迹点
    // double yaw_start = atan2((pos_second-pos_first)(1),(pos_second-pos_first)(0));//第一个yaw
    
    // for(int i=0;i<N;i++)//填充参考状态向量
    // {
    //     //连续的三个点
    //     t_k = t_c+i*t_step;
    //     t_k_1 = t_c+(i+1)*t_step;
    //     t_k_2 = t_c+(i+2)*t_step;

    //     //连续三个点的坐标
    //     pos_r = B_spline_trajectory.evaluateDeBoor(t_k);
    //     pos_r_1 = B_spline_trajectory.evaluateDeBoor(t_k_1);
    //     pos_r_2 = B_spline_trajectory.evaluateDeBoor(t_k_2);
    //     x_r(0) = pos_r(0);//第一个点的x坐标  参考状态
    //     x_r(1) = pos_r(1);//y坐标
    //     x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));//yaw
    //     v_r_1 = (pos_r_1-pos_r)/t_step;//参考线速度向量
    //     v_r_2 = (pos_r_2-pos_r_1)/t_step;
    //     v_linear_1 = v_r_1.norm();//参考线速度
    //     v_linear_2 = v_r_2.norm();

    //         if((t_k-traj_duration_)>=0)//>轨迹总时间
    //         {
    //             x_r(2) = atan2((pos_r-pos_final)(1),(pos_r-pos_final)(0));//参考yaw
    //         }
    //         else
    //         {
    //             //x_r(2) = atan2((pos_r_1-pos_r)(1),(pos_r_1-pos_r)(0));
    //             x_r(2) = atan2(v_r_1(1),v_r_1(0));//参考yaw为速度分量夹角
    //         }

    //     double yaw1 = atan2(v_r_1(1),v_r_1(0));//参考yaw,第二三个点
    //     double yaw2 = atan2(v_r_2(1),v_r_2(0));
    //     if(abs(yaw2-yaw1)>PI)
    //     {
    //         cout<<"current index : "<<i+1<<endl;
    //         cout<<"yaw 1 : "<<yaw1<<endl;
    //         cout<<"yaw 2 : "<<yaw2<<endl;
    //         if((yaw2-yaw1)<0)
    //         {
    //             w = (2*PI+(yaw2-yaw1))/t_step;
    //         }
    //         else
    //         {
    //             w = ((yaw2-yaw1)-2*PI)/t_step;
    //         }
    //     }
    //     else
    //     {
    //         w = (yaw2-yaw1)/t_step;//参考角速度
    //     }
    //     //cout<<"reference vel : "<<v_linear_1<<endl;
    //     //cout<<"reference w : "<<w<<endl;
    //     u_r(0) = v_linear_1;//参考线速度
    //     u_r(1) = w;//参考角速度
    //     X_r.push_back(x_r);
    //     U_r.push_back(u_r);
    // }

    // //当前状态-起点
    // X_k(0) = start.pose.position.x;//x0
    // X_k(1) = start.pose.position.y;//y0
    // X_k(2) = X_r[0](2);//yaw0
    // u_k = mpc_controller.MPC_Solve_qp(X_k,X_r,U_r,N);

}


}