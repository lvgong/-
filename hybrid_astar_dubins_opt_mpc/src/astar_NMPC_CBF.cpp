#include "astar_NMPC_CBF.h"


namespace chibot::topologyplanning {

  // uint8_t* AstarNMPCCBF::cost_translation_ = nullptr;
  //增大ts，在同等路径下，要求完成路径所需速度更小，表现为“MPC/v_max”可以更小，tmp=traj_duration_越大
  //ts=0.5m（路径点间隔）/0.3m/s（平均速度）=1.67s
  //适当提高T：t_step，配合N可以延长预测路线长度
  double ts=1.67, dt = 0.25;//同定时周期     ts = ctrl_pt_dist / max_vel_=0.5/0.5=1s
  #define N 25 //25步
  #define PI 3.1415926

  /*
    AstarNMPCCBF::AstarNMPCCBF() 
    : 成员1(初始值), 
      成员2(初始值), 
      ... 
    {
        // 构造函数函数体（可选）
    }
  */
  AstarNMPCCBF::AstarNMPCCBF()
     :line_ready_(false),
      // has_start(false),
      has_goal(false),
      tf_(new tf2_ros::Buffer()),                // 创建 TF 缓冲区
      tfl_(new tf2_ros::TransformListener(*tf_)),// 创建 TF 监听器
      // goto_ctrl_(new GotoCtrl("move_base_flex/move_base")),
      // exe_ctrl_(new ExeCtrl("move_base_flex/exe_path")),
      cur_state_(StateValue::Idle),
      /*
        类的成员初始化列表（在构造函数中初始化成员变量）
        将 cur_state_ 的初始值设为 StateValue::Idle（即 0）
      */
      running_state_(RunStateValue::Follow),
      // cur_index_(0),
      // obs_index_(0),
      route_num(3),//记录文件名
      receive_traj_(false),//B样条轨迹就绪
      is_target_rcv_(false),
      start_time_(0),
      rate(ros::Duration(0.1)){//100ms，10Hz

      // if (!cost_translation_) {
      //   //统一代价值0-254，代价分100份
      //   cost_translation_ = new uint8_t[101];
      //   for (int i = 0; i < 101; ++i) {
      //     cost_translation_[i] = static_cast<uint8_t>(i * 254 / 100);
      //   }
      // }

    }
  

  
  /**
   * @brief 初始化
   * 
   */
  void AstarNMPCCBF::init() {
      ros::NodeHandle nh;//啥也不表示就是以节点命名的相对路径

      ros::NodeHandle action_nh("move_base_flex/move_base");
      /*
        创建了一个名为action_nh的NodeHandle
        这个NodeHandle的命名空间被设置为/move_base_flex/move_base
      */
      action_goal_pub_ = action_nh.advertise<mbf_msgs::MoveBaseActionGoal>("goal", 1);
      ros::NodeHandle simple_nh("move_base_simple");
      //rviz中用于设置路点的话题，move_base_simple有命名空间之意
      goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&AstarNMPCCBF::goalCB, this, _1));
      goal_sub = nh.subscribe("/move_base_simple/goal", 1, &AstarNMPCCBF::goalCB, this);
      //订阅栅格地图，这个带有回调函数的必须放在主程序里，且快入快出
      map_sub_ = nh.subscribe("/map", 1, &AstarNMPCCBF::mapReceived, this);
      //设置拓扑起点集
      start_sub = nh.subscribe("/initialpose", 1, &AstarNMPCCBF::startCallback, this);
      //设置拓扑终点集
      goal_sub = nh.subscribe("/move_base_simple/goal", 1, &AstarNMPCCBF::goalCallback, this);

      //各种服务调用，服务器建立，很重要的一块
      task_srv_ = nh.advertiseService(TASK_SRV, &AstarNMPCCBF::task_service, this);

      vel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布速度指令
      // 控制信息供controller控制
      local_plan_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/local_plan", 10);

      //订阅代价地图，初始化或更新成本地图的信息
      costmap_sub_ = nh.subscribe(COSTMAP, 5, &AstarNMPCCBF::costmap_cb, this);
      //处理地图更新数据的回调函数，用于更新已存在的成本地图信息
      costmap_update_sub_ = nh.subscribe(COSTMAP_UPDATE, 5, &AstarNMPCCBF::costmap_update_cb, this);

      //发布当前位姿，可视化
      cur_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);

      //等待mbf服务器建立
      ROS_ERROR_COND(!goto_ctrl_->waitForServer(ros::Duration(5.0)), "move base action not online!");
      ROS_ERROR_COND(!exe_ctrl_->waitForServer(ros::Duration(5.0)), "exe path action not online!");

      // 发布预测头位姿，是跟踪/避障的切换时机（以后可以把点检测头改成面检测头）
      mpc_head_pub = nh.advertise<geometry_msgs::PoseStamped >("/head_pose", 1);
      // 订阅是否切换工作状态指令，由MPC->MPC-D-CBF
      change2dcbf_sub = nh.subscribe("/change2dcbf", 1, &AstarNMPCCBF::change2dcbf_cb, this);
      // 订阅是否切换工作状态指令，由MPC-D-CBF->MPC
      change2mpc_sub = nh.subscribe("/change2mpc", 1, &AstarNMPCCBF::change2mpc_cb, this);
      // 发布切换工作态，由MPC-D-CBF->MPC
      change2mpc_pub = nh.advertise<std_msgs::Bool>("/change2mpc", 1);

      //可视化B样条轨迹
      trajectory_pub = nh.advertise<nav_msgs::Path>("/Bspline_trajectory",10);
      predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 10);//预测路线
      
      //NMPC跟踪的构造函数
      mpc_ptr.reset(new Mpc(N, dt));

      //MPC周期控制
      control_cmd_pub = nh.createTimer(ros::Duration(0.1), &AstarNMPCCBF::publish_control_cmd, this);//控制小车
      //先关闭定时器，等获得了B样条轨迹后再开启定时器
      control_cmd_pub.stop();

      //更新局部的全局路径
      //供NMPC纯跟踪使用
      global_path_timer = nh.createTimer(ros::Duration(0.3), &AstarNMPCCBF::globalPathPub_Callback, this);
      global_path_timer.stop();

      // 供MPC-CBF使用的参考路径,同时作为rviz可视化
      global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);

      // 接收动态障碍物的轨迹函数信息
      obs_trajs_sub = nh.subscribe("/Dyn_Obs_Set", 100, &AstarNMPCCBF::obs_trajs_cb, this);
      //发布添加了障碍信息后的局部栅格地图，供A*搜路
      local_grid_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_grid_map", 10);

  }


  /**
   * @brief 主循环函数，给目标点就生成一条路径并控制小车运动
   * 
   */
  void AstarNMPCCBF::run() {
    // using goalState = actionlib::SimpleClientGoalState;//导航状态信息
    // ros::Rate rate(10); //10Hz
    while (ros::ok()) {
      ros::spinOnce();
      //任务状态机，状态的切换在各事件中
      switch (cur_state_) {
        case StateValue::Idle:

          //规划拓扑路线
          if(line_ready_){
            half_struct_planner_->set_traffic_route_topology(lines);//规划拓扑路线
            lines.clear();
            lines.shrink_to_fit();
            ROS_INFO("Ready!");
            line_ready_ = false;
          }

          //加载路线
          if(path_ready_){
            //可视化起终点路线，绿色
            half_struct_planner_->visSGPath(paths);
            paths.clear();
            paths.shrink_to_fit();// 尽可能地减小vector的容量
            //可视化拓扑路线，蓝色
            half_struct_planner_->vistopologyPath(paths_);
            paths_.clear();
            paths_.shrink_to_fit();
            half_struct_planner_->calculate_pre_graph(lines, distance_table);
            //显示路点标号，设置距离矩阵
            lines.clear();
            lines.shrink_to_fit();
            distance_table.clear();
            distance_table.shrink_to_fit();
            cur_state_ = StateValue::Pause;
            // running_state_ = RunStateValue::Goto;//使用点到点导航
            path_ready_ = false;
            ROS_INFO("Finish!");
          }
          break;

        case StateValue::Pause:
          // pause_state();
          //给一个目标点，同小车当前位姿点一起规划出一条路线并控制小车执行
          // if(has_start && has_goal){
          if(has_goal){
            receive_traj_ = false;
            // ROS_INFO("Start!");
            //由起终点取交通规则中的路径点（一堆带位姿的点）
            start.header.frame_id = "map";
            start.header.stamp = ros::Time::now();
            start.pose = robot_pose().value();//获取小车当前位姿作为起点

            //可视化起点位姿
            Vec3d start_pose;
            start_pose[0] = start.pose.position.x;
            start_pose[1] = start.pose.position.y;
            start_pose[2] = tf2::getYaw(start.pose.orientation);//偏航角
            vis.publishStartPoses(start_pose);

            //控制点路径
            control_path.header.frame_id = "map";
            control_path.header.stamp = ros::Time::now();
            control_path.poses  = half_struct_planner_->get_path(start, goal); 
            if (control_path.poses.empty()) {
              ROS_INFO("Traffic path not found!");
              has_goal = false;   
              return;
            }

            // parameterize the path to bspline
            // vector<Eigen::Vector3d> point_set, start_end_derivatives;
            // for(int i=0;i<control_path.poses.size();i++)
            // {
            //     //把得到的路径点作为型值点经过
            //     Vector3d current_point(control_path.poses[i].pose.position.x, control_path.poses[i].pose.position.y, 0);
            //     point_set.push_back(current_point);
            // }        
            // Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
            // Eigen::Vector3d end_pt_, end_vel_, end_acc_;                              // target state
            // start_vel_ = {0.0, 0.0, 0.0};
            // start_acc_.setZero();
            // end_vel_.setZero();
            // end_acc_= {0.0, 0.0, 0.0};

            // start_end_derivatives.push_back(start_vel_);
            // start_end_derivatives.push_back(end_vel_);
            // start_end_derivatives.push_back(start_acc_);
            // start_end_derivatives.push_back(end_acc_);

            // Eigen::MatrixXd ctrl_pts;
            // NonUniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
            
            /**
             * 矩阵初始化
             * MatrixXd::Zero(rows, cols)创建一个全零矩阵
             * 行数：control_path.poses.size()（路径点数量）  size() 是容器类的成员函数，用于返回容器中元素的数量。
             * 列数：2（每个点的 x,y 坐标）
             */
            //初始化控制点矩阵(2D)
            MatrixXd control_points = MatrixXd::Zero(control_path.poses.size(), 2);
            for(int i=0;i<control_path.poses.size();i++)
            {
                //把得到的路径点作为控制点
                Vector2d current_point(control_path.poses[i].pose.position.x, control_path.poses[i].pose.position.y);
                control_points.row(i) = current_point;
            }        

            // ros::Time t1  = ros::Time::now();
            // double  t_adjust = 0.0;
            // 创建新的均匀B样条曲线pos_traj
            // NonUniformBspline pos_traj(ctrl_pts, 3, ts);//3阶  ts与要求车速有关
            NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关

            double to = pos_traj.getTimeSum();
            pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置合速度上限、合加速度上限、容忍值
            bool feasible = pos_traj.checkFeasibility(false);

            int iter_num = 0;
            while (!feasible && ros::ok()) {
              feasible = pos_traj.reallocateTime();
              if (++iter_num >= 3) break;
            }

            // pos.checkFeasibility(true);
            // cout << "[Main]: iter num: " << iter_num << endl;

            double tn = pos_traj.getTimeSum();

            cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
            if (tn / to > 3.0) ROS_ERROR("reallocate error.");

            // t_adjust = (ros::Time::now() - t1).toSec();
            // cout << "[kino replan]: time: " << ", adjust time:" << t_adjust << endl;


            // B_spline_trajectory.setUniformBspline(control_points,3,ts);//3阶
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
            // ROS_INFO("traj_duration_:%f",traj_duration_); 

            // nav_msgs::Path b_spline_trajectory;
            b_spline_trajectory.header.frame_id = "map";
            for(double t_c=t;t_c<tmp;t_c+=dt)
            {
                Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
                // Vector3d point_temp = traj_[0].evaluateDeBoor(t_c);
                geometry_msgs::PoseStamped current_pose;
                current_pose.pose.position.x = point_temp(0);//轨迹点
                current_pose.pose.position.y = point_temp(1);
                // current_pose.pose.position.z = point_temp(2);
                current_pose.header.frame_id = "map";
                b_spline_trajectory.poses.push_back(current_pose);//样条曲线
            }
            trajectory_pub.publish(b_spline_trajectory);//可视化完整的B样条曲线
            b_spline_trajectory.poses.clear();


            // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
            receive_traj_ = true;
            has_goal = false;   

            //切换状态，变为跟随态
            cur_state_ = StateValue::Run;
            running_state_ = RunStateValue::Follow;

          }
          break;

        case StateValue::Record_Traffic_route://录制路网，就是拓扑关系
          break;
          
        case StateValue::Run://路径跟随/避障等
          running_state();
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
  // void AstarNMPCCBF::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
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
  // void AstarNMPCCBF::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg) {
  //     if (cur_state_ != StateValue::Run) return;
    
  //     Vec3d start_pose;
    
  //     start_pose[0] = msg.pose.pose.position.x;
  //     start_pose[1] = msg.pose.pose.position.y;
  //     start_pose[2] = tf2::getYaw(msg.pose.pose.orientation);//偏航角
    
  //     start.header = msg.header;
  //     start.pose.position = msg.pose.pose.position;
  //     start.pose.orientation = msg.pose.pose.orientation;
  //     has_start = true;
  //     // ROS_INFO("receive start position");

  //     //最好可视化
  //     vis.publishStartPoses(start_pose);
  // }


  /**
   * @brief 接收目标点位姿，一个集合，用于模拟示教路径终点位姿
   * 
   * @param msg 
   */
  // void AstarNMPCCBF::goalCallback(const geometry_msgs::PoseStamped msg) {
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
  // void AstarNMPCCBF::goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg) {
  //     if (cur_state_ != StateValue::Run) return;
  //     Vec3d goal_pose;
  //     goal_pose[0] = msg->pose.position.x;
  //     goal_pose[1] = msg->pose.position.y;
  //     goal_pose[2] = tf2::getYaw(msg->pose.orientation);//偏航角
  //     goal = *msg;
  //     has_goal = true;
  //     // ROS_INFO("receive goal position");
  //     vis.publishGoalPoses(goal_pose);
  // }


  //转化为mbf的action_goal
  void AstarNMPCCBF::goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (cur_state_ != StateValue::Pause) return;

    //暂时不用直接的点到点导航
    // ROS_DEBUG_NAMED("move_base_flex","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    // mbf_msgs::MoveBaseActionGoal action_goal;
    // action_goal.header.stamp = ros::Time::now();
    // action_goal.goal.target_pose = *msg;
    // action_goal_pub_.publish(action_goal);

    Vec3d goal_pose;
    goal_pose[0] = msg->pose.position.x;
    goal_pose[1] = msg->pose.position.y;
    goal_pose[2] = tf2::getYaw(msg->pose.orientation);//偏航角
    goal = *msg;

    has_goal = true;
    // ROS_INFO("receive goal position");
    vis.publishGoalPoses(goal_pose);

  }


  /**
   * @brief 接收地图消息
   * 
   * @param msg 
   */
  void AstarNMPCCBF::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
    //互斥锁，通过创建 scoped_lock 对象 lock，在函数中加锁以确保线程安全性。这意味着在执行接下来的代码时，其他试图访问 map_mutex_ 的线程将被阻塞
    boost::mutex::scoped_lock lock(map_mutex_);
    grid_ = *msg;//这个grid_是一个全局变量
    // scoped_lock 对象 lock 的生命周期已经结束，离开了这个代码块，lock 对象会自动析构，从而释放对 map_mutex_ 的占用，也就是自动解锁。

    //半结构化道路
    half_struct_planner_ = std::make_shared<HalfStructPlanner>(msg);//创建HalfStructPlanner类
    half_struct_planner_->init();//初始化

    //A*构造函数
    astar_path_finder = std::make_shared<AstarPathFinder>();

    //Hybrid A*构造函数
    // hybrid_astar_path_finder = std::make_shared<DynamicHybridAstar>();


    //后续需要改造grid_添加上动态障碍物等效的占据栅格（已实现）

  }

  /**
  * @brief 各任务切换用服务器，服务器服务端
  * 
  * @param  
  */
  auto AstarNMPCCBF::task_service(hybrid_astar_dubins_opt_mpc::chibot_task::Request& req, hybrid_astar_dubins_opt_mpc::chibot_task::Response& resp) -> bool {
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

          ////////////////////////////////////////这个特别重要
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



  /**
  * @brief 将存储的路点保存到文件里
  * 
  * @param 
  */
  auto AstarNMPCCBF::write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool {
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



  /**
  * @brief 检查文件是否存在，接受一个文件路径作为参数
  * 
  * @param 
  */
  auto AstarNMPCCBF::check_file_exist(std::string& file_path) -> bool {
  std::ifstream exist(file_path.c_str());
  return !!exist; // (!!exist) 将exist对象转换为布尔值
  }



  /**
  * @brief 读取文件中的路点信息
  * 
  * @param 
  */
  auto AstarNMPCCBF::read_traffic_route(std::string& file_path, lines_type& lines) -> bool {
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



  /**
  * @brief 读取路线
  * 
  * @param 
  */
  auto AstarNMPCCBF::read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool {
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


  //订阅代价地图，初始化或更新成本地图的信息
  /*在ROS中，nav_msgs::OccupancyGrid消息类型表示一个二维栅格地图，其中data字段是一个一维数组，代表了地图中每个栅格的占据情况。在这个数组中，每个元素的取值范围通常是0到100，分别代表着不同的含义：
  -1: 未知的状态
  0-100: 表示该栅格的占据概率，0表示空闲，100表示完全占据
  因此，nav_msgs::OccupancyGrid::data中的值范围应该是-1到100之间。*/

  //   void AstarNMPCCBF::costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const& msg) {
  //     // 使用std::lock_guard对map_update_mutex_进行加锁，以确保在处理地图数据时不会被其他线程打断；退出函数自动析构
  //     std::lock_guard<std::mutex> lock(map_update_mutex_);
  //     if (!costmap_) {
  // // 如果costmap_为空，表示需要初始化新的成本地图，使用std::make_shared创建了一个名为costmap_的costmap_2d::Costmap2D类型的共享指针，并传入了地图的宽度、高度、分辨率和原点位置等信息。
  //       ROS_INFO("Initiate new costmap");
  //       costmap_ = std::make_shared<costmap_2d::Costmap2D>(msg->info.width, msg->info.height, msg->info.resolution,
  //                                                         msg->info.origin.position.x, msg->info.origin.position.y);
  //     } else {
  //       // 更新现有的成本地图
  //       ROS_INFO("Update costmap!");
  //       // 重新调整地图的大小和分辨率
  //       costmap_->resizeMap(msg->info.width, msg->info.height, msg->info.resolution,
  //                           msg->info.origin.position.x, msg->info.origin.position.y);
  //     }

  // // 双重循环遍历收到的地图数据，根据其中的值来设置成本地图中对应位置的代价值。如果地图数据中的值小于0，表示该位置未知，将其设置为costmap_2d::NO_INFORMATION；否则根据cost_translation_映射将其设置为对应的代价值
  //     uint32_t x;
  //     for (uint32_t y = 0; y < msg->info.height; y++) {
  //       for (x = 0; x < msg->info.width; x++) {
  //         if (msg->data[y * msg->info.width + x] < 0) {//-1：表示未知 0：表示空闲
  //           costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
  //           continue;
  //         }
  //         //占用率转代价值
  //         costmap_->setCost(x, y, cost_translation_[msg->data[y * msg->info.width + x]]);
  //       }
  //     }
  //   }


  // 处理地图更新数据的回调函数，用于更新已存在的成本地图信息

  //   void AstarNMPCCBF::costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const& msg) {
  //     // 使用std::lock_guard对map_update_mutex_进行加锁，以确保线程安全性
  //     std::lock_guard<std::mutex> lock(map_update_mutex_);
  //     if (!costmap_) {
  //       ROS_WARN("Costmap not initiate yet");
  //       return;
  //     }
  // // 验证收到的地图更新数据的有效性，包括数据大小是否匹配、更新范围是否超出了成本地图的边界等。如果数据不合法，则记录错误信息并直接返回
  //     if (msg->width * msg->height != msg->data.size()
  //         || msg->x + msg->width > costmap_->getSizeInCellsX()
  //         || msg->y + msg->height > costmap_->getSizeInCellsY()) {
  //       ROS_ERROR("Costmap update got invalid data set");
  //       return;
  //     }
  // // 通过双重循环遍历接收到的地图更新数据，并根据数据内容更新成本地图中对应位置的代价值。根据地图更新数据中的值来设置成本地图中对应位置的代价值
  //     size_t index = 0;
  //     int x;
  //     for (auto y = msg->y; y < msg->y + msg->height; y++) {
  //       for (x = msg->x; x < msg->x + msg->width; x++) {
  //         if (msg->data[index] < 0) {
  //           costmap_->setCost(x, y, costmap_2d::NO_INFORMATION);
  //           index++;
  //           continue;
  //         }
  //         //遍历所有网格，重新赋予代价值0-254  占用率转代价
  //         costmap_->setCost(x, y, cost_translation_[msg->data[index++]]);
  //       }
  //     }
  //   }


  //求机器人在地图中的位姿
  auto AstarNMPCCBF::robot_pose() -> std::optional<geometry_msgs::Pose> {
    /*这段代码使用tf2_ros::Buffer对象的lookupTransform()方法来获取从map坐标系到base_link坐标系的变换信息。
      使用try-catch语句块来捕获可能抛出的异常。在try块中，调用tf_->lookupTransform("map", "base_link", ros::Time(0))来获取变换信息。这个函数的参数分别是目标坐标系（"map"）和源坐标系（"base_link"），
      以及时间戳（ros::Time(0)表示最新的变换）。函数将返回一个geometry_msgs::TransformStamped类型的变量，包含了变换的详细信息。
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


 //执行示教路径，这里的index是示教路径中的，如果车中途暂停可以恢复到这个点（其他函数中修正index）
  // auto AstarNMPCCBF::send_exe(size_t const& index) -> bool {
  //   //合法性校验
  //   if (index == std::numeric_limits<size_t>::max() || index > cur_route_.poses.size()) {
  //     ROS_ERROR_STREAM("send_exe index error " << index << " / " << cur_route_.poses.size());
  //     return false;
  //   }

  //   mbf_msgs::ExePathGoal goal{};
  //   nav_msgs::Path route;
  //   route.header = cur_route_.header;
  //   //截断原路径，从index开始复制一条新的路径执行
  //   route.poses.assign(cur_route_.poses.begin() + index, cur_route_.poses.end());
  //   goal.path = route;
  //   // ROS_INFO("send_exe goal");
  //   exe_ctrl_->sendGoal(goal,
  //                       boost::bind(&AstarNMPCCBF::exe_done, this, _1, _2),
  //                       ExeCtrl::SimpleActiveCallback(),//默认的回调
  //                       ExeCtrl::SimpleFeedbackCallback());
  //   return true;
  // }

  // void AstarNMPCCBF::exe_done(const actionlib::SimpleClientGoalState& state,
  //                             const mbf_msgs::ExePathResultConstPtr& result) {
  //   ROS_INFO("ExePath got state [%s]", state.toString().c_str());
  //   if (!result) return;
  //   ROS_INFO("ExePath got result [%d]", result->outcome);
  // }


  //运行态Run分支状态机
  void AstarNMPCCBF::running_state() {
    // using goalState = actionlib::SimpleClientGoalState;
    // ros::Rate rate(ros::Duration(0.1));
    switch (running_state_) {
      // 1 轨迹跟随状态
      case RunStateValue::Follow: {

        // 修改发布频率为0.1秒
        // rate = ros::Rate(10.0);  // 10Hz
        if (start_time_.toSec() == 0) {// start_time_ 未被赋值
          start_time_ = ros::Time::now();//在这里赋初值，记得MPC控制完要清零
        } 
        //MPC周期性控制
        publish_control_cmd();
        //初始化设定目标点开关，该目标点用于指定避障路径的终点
        if(is_target_rcv_) is_target_rcv_ =false;

        break;
      }

    // 提供局部路径给MPC-CBF
    case RunStateValue::Goto: {

      // 修改发布频率为0.25秒
      // rate = ros::Rate(5.0);  // 4Hz
      // if(circle_R_.empty() || center_.empty()) break;
      local_gird_map();
      //不断发送MPC-CBF用的全局路径
      // globalPathPub_Callback();//先有这个才有target_pose

      //判断邻近目标点时切换回跟随状态------------
      // if(distance(robot_pose().value(), target_pose) < 0.5) {   //还是想在py那边判断
      //   std_msgs::Bool msg;
      //   msg.data = true;
      //   change2mpc_pub.publish(msg);//发送到MPC-CBF控制器，停止避障工作
        
      //   change2mpc();
      //   break;
      // }

      
      //局部路径用的NMPC
      // local_control_cmd();

      break;
    }

      // 错误状态（不会进来）
      default:
        ROS_INFO("Error RunStateValue!");
        break;
    }
  }


  //代价值小于66返回true，表示无障碍物
  // auto AstarNMPCCBF::is_free(const geometry_msgs::PoseStamped& pose) const -> bool {
  //   auto cost = pose_cost(pose);//获取某一个位姿点的代价值
  //   return cost < 66;
  // }


  //代价值大于253返回true，表示有障碍物
  // auto AstarNMPCCBF::is_danger(const geometry_msgs::PoseStamped& pose) const -> bool {
  //   auto cost = pose_cost(pose);
  //   return cost >= 253;
  // }


  //获取某一个位姿点的代价值
  // auto AstarNMPCCBF::pose_cost(const geometry_msgs::PoseStamped& pose) const -> uint8_t {
  //   if (!costmap_) {//还没有代价地图
  //     ROS_WARN("No costmap yet");
  //     return true;
  //   }
  //   uint32_t mx, my;
  //   //转地图坐标系下的座标点
  //   if (!costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
  //     ROS_WARN("Can't find point mx, my in cost map");
  //     return 0;
  //   }
  //   return costmap_->getCost(mx, my);//获取该点代价值
  // }


  //计算两点之间的距离和角度差
  // inline auto AstarNMPCCBF::distance(geometry_msgs::Pose const& a,
  //                             geometry_msgs::Pose const& b) -> std::pair<double, double> {
  inline auto AstarNMPCCBF::distance(geometry_msgs::Pose const& a,
                              geometry_msgs::Pose const& b) -> double {
    // std::pair<double, double> result;
    //距离
    return std::hypot(a.position.x - b.position.x, a.position.y - b.position.y);
    //角度差
    // result.second = std::abs(tf2::getYaw(a.orientation) - tf2::getYaw(b.orientation));
    // return result;
  }


  //MPC跟踪控制周期性执行
  // void AstarNMPCCBF::publish_control_cmd(const ros::TimerEvent &e)
  void AstarNMPCCBF::publish_control_cmd()
  {
      // geometry_msgs::Twist cmd;
      // 设置 local_plan 的大小
      std_msgs::Float32MultiArray local_plan; //控制量 存储一维浮点数数组
      local_plan.layout.dim.push_back(std_msgs::MultiArrayDimension());
      local_plan.layout.dim[0].size = 2;//size表示数组的长度，即数组中元素的个数
      local_plan.layout.dim[0].stride = 1;//stride表示数组的步长，即在多维数组中，每一维的步长大小。这里是一个一维数组，因此步长设置为1
      local_plan.layout.dim[0].label = "local_plan";//描述这个维度的标签
      local_plan.data.clear();

      if (!receive_traj_){
          local_plan.data.push_back(0.0);
          local_plan.data.push_back(0.0);
          local_plan_pub_.publish(local_plan);//发送控制指令
          // cmd.angular.z = 0;
          // cmd.linear.x = 0;
          // vel_cmd_pub.publish(cmd);
          // is_orientation_init=false;
          return;
      }

      ros::Time time_s = ros::Time::now();
      // 计算当前时间与轨迹起始时间之间的差值（以秒为单位）
      // 计算现在的时间和起始时间的间隔
      double t_cur = (time_s - start_time_).toSec();//时间差值
      // ROS_INFO("start_time_ = %f", start_time_.toSec());
      // ROS_INFO("t_cur = %f", t_cur);
      // Eigen::Vector3d pos, vel, acc, pos_f;
      Eigen::VectorXd pos, vel, vel_1;
      // double yaw, yawdot;
      double yaw;
      bool is_orientation_adjust=false;
      double orientation_adjust=0.;

      Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N+1, 3);//N+1 * 3

      //预测部分都在时间段内
      //  if (t_cur + (N-1) * dt <= traj_duration_ && t_cur > 0) {
      //   for (int i = 0; i <= N; ++i) {
      //     pos = traj_[0].evaluateDeBoorT(t_cur + i * dt);//参考位置
      //     vel = traj_[1].evaluateDeBoorT(t_cur + i * dt);//参考速度
      //     vel_1 = traj_[1].evaluateDeBoorT(t_cur + (i+1) * dt);//参考速度
      //     // acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);//参考加速度

      //     yaw = atan2(vel(1),vel(0));//参考yaw为速度分量夹角
      //     // yaw = traj_[3].evaluateDeBoorT(t_cur + i * dt)[0];//参考yaw
      //     // yawdot = traj_[4].evaluateDeBoorT(t_cur + i * dt)[0];

      //    double yaw1 = atan2(vel(1),vel(0));//i时刻yaw
      //     double yaw2 = atan2(vel_1(1),vel_1(0));//i+1时刻yaw

      //     if(abs(yaw2-yaw1)>PI)
      //     {
      //         is_orientation_adjust = true;
      //         if((yaw2-yaw1)<0)
      //         {
      //             orientation_adjust = 2*PI;
      //             // w = (2*PI+(yaw2-yaw1))/t_step;
      //         }
      //         else
      //         {
      //             // w = ((yaw2-yaw1)-2*PI)/t_step;
      //             orientation_adjust = -2*PI;
      //         }
      //     }
          
      //     if(is_orientation_adjust==true)
      //     {
      //         yaw += orientation_adjust;
      //     }

      //     desired_state(i, 0) = pos[0];//参考x
      //     desired_state(i, 1) = pos[1];//参考y
      //     desired_state(i, 2) = yaw;//参考yaw
      //   }
      // //预测段超出最后时间段的部分
      // } else if (t_cur + (N-1) * dt > traj_duration_ && t_cur < traj_duration_) {
      //     int more_num = (t_cur + (N-1) * dt - traj_duration_) / dt;
      //     //这部分在traj_duration_内
      //     for (int i = 0; i < N - more_num; ++i) {
      //       pos = traj_[0].evaluateDeBoorT(t_cur + i * dt);
      //       vel = traj_[1].evaluateDeBoorT(t_cur + i * dt);
      //       // acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);
      //       // yaw = traj_[3].evaluateDeBoorT(t_cur + i * dt)[0];
      //       yaw = atan2(vel(1),vel(0));
      //       // yawdot = traj_[4].evaluateDeBoorT(t_cur + i * dt)[0];

      //    double yaw1 = atan2(vel(1),vel(0));//i时刻yaw
      //     double yaw2 = atan2(vel_1(1),vel_1(0));//i+1时刻yaw

      //     if(abs(yaw2-yaw1)>PI)
      //     {
      //         is_orientation_adjust = true;
      //         if((yaw2-yaw1)<0)
      //         {
      //             orientation_adjust = 2*PI;
      //             // w = (2*PI+(yaw2-yaw1))/t_step;
      //         }
      //         else
      //         {
      //             // w = ((yaw2-yaw1)-2*PI)/t_step;
      //             orientation_adjust = -2*PI;
      //         }
      //     }
          
      //     if(is_orientation_adjust==true)
      //     {
      //         yaw += orientation_adjust;
      //     }

      //       desired_state(i, 0) = pos(0);
      //       desired_state(i, 1) = pos(1);
      //       desired_state(i, 2) = yaw;          
      //     }
      //     //填充剩下的，在traj_duration_外
      //     for (int i = N - more_num; i < N; ++i) {
      //       pos = traj_[0].evaluateDeBoorT(traj_duration_);
      //       // vel.setZero();
      //       // acc.setZero();
      //       yaw = tf2::getYaw(goal.pose.orientation);//在-3.14-3.14之间
      //       // yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
      //       // yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];


      //       desired_state(i, 0) = pos(0);
      //       desired_state(i, 1) = pos(1);
      //       desired_state(i, 2) = yaw;
      //     }
      // } 

      double t_k,t_k_1;
      if (t_cur < traj_duration_ && t_cur >= 0.0) {
        for (int i = 0; i <= N; ++i) {
          t_k = t_cur+i*dt;//i时刻
          t_k_1 = t_cur+(i+1)*dt;//下一时刻i+1
          pos = traj_[0].evaluateDeBoor(t_k);//参考位置
          vel = traj_[1].evaluateDeBoor(t_k);//参考速度
          vel_1 = traj_[1].evaluateDeBoor(t_k_1);//参考速度
          // acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);//参考加速度
          
          if((t_k-traj_duration_)>=0)//>轨迹总时间
          {
            yaw = tf2::getYaw(goal.pose.orientation);//在-3.14-3.14之间
          }
          else
          {
            yaw = atan2(vel(1),vel(0));//参考yaw为速度分量夹角

            double yaw1 = atan2(vel(1),vel(0));//i时刻yaw
            double yaw2 = atan2(vel_1(1),vel_1(0));//i+1时刻yaw

          if(abs(yaw2-yaw1)>PI)
          {
              is_orientation_adjust = true;
              if((yaw2-yaw1)<0)
              {
                  orientation_adjust = 2*PI;
                  // w = (2*PI+(yaw2-yaw1))/t_step;
              }
              else
              {
                  // w = ((yaw2-yaw1)-2*PI)/t_step;
                  orientation_adjust = -2*PI;
              }
          }
          if(is_orientation_adjust==true)
          {
              yaw += orientation_adjust;
          }
          }
          // yaw = traj_[3].evaluateDeBoorT(t_cur + i * dt)[0];//参考yaw
          // yawdot = traj_[4].evaluateDeBoorT(t_cur + i * dt)[0];

          desired_state(i, 0) = pos[0];//参考x
          desired_state(i, 1) = pos[1];//参考y
          desired_state(i, 2) = yaw;//参考yaw
        }
        //预测段超出最后时间段的部分
      } 
          
      //全部预测部分都超限
      else if (t_cur >= traj_duration_)  {
        // pos = traj_[0].evaluateDeBoorT(traj_duration_);
        // // vel.setZero();
        // // acc.setZero();
        // yaw = tf2::getYaw(goal.pose.orientation);//在-3.14-3.14之间
        // // yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
        // // yawdot = traj_[4].evaluateDeBoorT(traj_duration_)[0];
        // for (int i = 0; i <= N; ++i) {
        //     desired_state(i, 0) = pos(0);
        //     desired_state(i, 1) = pos(1);
        //     desired_state(i, 2) = yaw;

        // cmd.angular.z = 0;
        // cmd.linear.x = 0;
        // vel_cmd_pub.publish(cmd);
        // is_orientation_init=false;
        // local_plan.data.clear();
        local_plan.data.push_back(0.0);
        local_plan.data.push_back(0.0);
        local_plan_pub_.publish(local_plan);
        receive_traj_=false;
        // 清零
        start_time_ = ros::Time(0);
        //便于再从设置目标点
        cur_state_ = StateValue::Pause;
        return;
        
      } else {
        cout << "[Traj server]: invalid time." << endl;
        return;
      }

      Eigen::MatrixXd desired_state1 = desired_state.transpose();

      Eigen::Vector3d current_state;
      geometry_msgs::PoseStamped x_current;
      x_current.header.frame_id = "map";
      x_current.header.stamp = ros::Time::now();
      x_current.pose = robot_pose().value();//获取小车当前位姿作为起点 
      current_state(0) = x_current.pose.position.x;
      current_state(1) = x_current.pose.position.y;
      current_state(2) = tf2::getYaw(x_current.pose.orientation);
        
      // auto yaw_ = tf2::getYaw(x_current.pose.orientation);
        // if(yaw_/desired_state(0,2)<0&&abs(yaw_)>(PI*5/6))//当前yaw
        // {
        //     if(yaw_<0)
        //     {
        //         current_state(2) = yaw_ + 2*PI;
        //     }
        //     else
        //     {
        //         current_state(2) = yaw_ - 2*PI;
        //     }
        // }
        // else
        // {
            // current_state(2) = yaw_;
        // }

      smooth_yaw(desired_state1, current_state);

      ros::Time solve_start = ros::Time::now();
      //MPC模型预测核心
      auto success = mpc_ptr->solve(current_state, desired_state1);//当前状态，参考状态
      // cout << "solve success" << endl;


      //获得首个控制量
      auto control_cmd = mpc_ptr->getFirstU();
      // cout << "got cmd" << endl;

      // cmd.linear.x = control_cmd[0];//v线速度
      // cmd.angular.z = control_cmd[1];//w角速度
      // vel_cmd_pub.publish(cmd);
      //cout << "u:" << result[0] << " " << "r:" << result[1] << endl;
      local_plan.data.push_back(control_cmd[0]);
      local_plan.data.push_back(control_cmd[1]);
      local_plan_pub_.publish(local_plan);

      
      static int conut1 = 0;
      conut1+=1;
      if(conut1%15==0)//调试
      {
          cout<<"v: "<<control_cmd[0] <<"m/s"<<endl;
          cout<<"w: "<<control_cmd[1]<<"rad/s"<<endl;
          ros::Time solve_end = ros::Time::now();
          double t_cur1 = (solve_end - solve_start).toSec();
          cout << "solve time: " << t_cur1 << "s" << endl;
          conut1=0;
      }
        
      nav_msgs::Path predict_path;
      predict_path.header.frame_id = "map";
      predict_path.header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped pose_msg;
      // geometry_msgs::Point pt;
      auto predict_states = mpc_ptr->getPredictX();//预测的N步状态
      // cout << "got predict x" << endl;
      for (int i = 0; i < predict_states.size(); i += 2) {
          pose_msg.pose.position.x = predict_states[i];
          pose_msg.pose.position.y = predict_states[i + 1];
          predict_path.poses.push_back(pose_msg);
      }
      predict_path_pub.publish(predict_path);//可视化N步状态轨迹
      predict_path.poses.clear();

      //假设小车全局感知到障碍物，我们要做的，是障碍物在小车前方多少范围里将其纳入CBF计算
      // 将当前小车位姿前方3m距离作为碰撞检测点

      geometry_msgs::PoseStamped pose_head;
      // vector<geometry_msgs::Point> pose_head_array;
      pose_head.header.frame_id = "map";
      pose_head.header.stamp = ros::Time::now();
      // pose_head.pose.position.x = predict_states[predict_states.size()-2];
      // pose_head.pose.position.y = predict_states[predict_states.size()-1];

      // 计算前方3m的位置坐标
      pose_head.pose.position.x = current_state(0) + 3 * cos(current_state(2));
      pose_head.pose.position.y = current_state(1) + 3 * sin(current_state(2));


        //检测头->检测面，没必要，我们只需要在检测障碍物那边处理好
        // const double radius = 3; //半径为3米
        // const double frontAngle = current_state(2);  // 正前方对应的角度
        // const double halfAngle = 15.0;  // 夹角的一半为30°的一半
        // const double angleIncrement = 5.0;  // 角度间隔为5度
        // const double startAngle = frontAngle - halfAngle;
        // const double endAngle = frontAngle + halfAngle;
        // for (double angle = startAngle; angle <= endAngle; angle += angleIncrement) {
        //   double theta = angle*DEG2RAD;
        //   geometry_msgs::Point point;
        //   point.x = current_state(0) + radius * std::cos(theta);// 根据角度计算x坐标
        //   point.y = current_state(1) + radius * std::sin(theta);
        //   point.z = 0.;
        //   pose_head_array.push_back(point);
        // }


      //检测MPC预测头是否碰触障碍，用于切换避障控制算法
      mpc_head_pub.publish(pose_head);

  }

  //给跟踪局部路径用的NMPC/LMPC？
  void AstarNMPCCBF::local_control_cmd()
  {
      // geometry_msgs::Twist cmd;
      // 设置 local_plan 的大小
      std_msgs::Float32MultiArray local_plan; //控制量 存储一维浮点数数组
      local_plan.layout.dim.push_back(std_msgs::MultiArrayDimension());
      local_plan.layout.dim[0].size = 2;//size表示数组的长度，即数组中元素的个数
      local_plan.layout.dim[0].stride = 1;//stride表示数组的步长，即在多维数组中，每一维的步长大小。这里是一个一维数组，因此步长设置为1
      local_plan.layout.dim[0].label = "local_plan";//描述这个维度的标签
      local_plan.data.clear();

      if (!receive_traj_){
          local_plan.data.push_back(0.0);
          local_plan.data.push_back(0.0);
          local_plan_pub_.publish(local_plan);
          return;
      }

      ros::Time time_s = ros::Time::now();
      // 计算当前时间与轨迹起始时间之间的差值（以秒为单位）
      // 计算现在的时间和起始时间的间隔
      double t_cur = (time_s - start_time_).toSec();//时间差值
      // ROS_INFO("start_time_ = %f", start_time_.toSec());
      // ROS_INFO("t_cur = %f", t_cur);
      // Eigen::Vector3d pos, vel, acc, pos_f;
      Eigen::VectorXd pos, vel, vel_1;
      // double yaw, yawdot;
      double yaw;
      bool is_orientation_adjust=false;
      double orientation_adjust=0.;

      Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N+1, 3);//N+1 * 3

      double t_k,t_k_1;
      if (t_cur < traj_duration_ && t_cur >= 0.0) {
        for (int i = 0; i <= N; ++i) {
          t_k = t_cur+i*dt;//i时刻
          t_k_1 = t_cur+(i+1)*dt;//下一时刻i+1
          pos = traj_[0].evaluateDeBoor(t_k);//参考位置
          vel = traj_[1].evaluateDeBoor(t_k);//参考速度
          vel_1 = traj_[1].evaluateDeBoor(t_k_1);//参考速度
          // acc = traj_[2].evaluateDeBoorT(t_cur + i * dt);//参考加速度
          
          if((t_k-traj_duration_)>=0)//>轨迹总时间
          {
            yaw = tf2::getYaw(goal.pose.orientation);//在-3.14-3.14之间
          }
          else
          {
            yaw = atan2(vel(1),vel(0));//参考yaw为速度分量夹角

                double yaw1 = atan2(vel(1),vel(0));//i时刻yaw
          double yaw2 = atan2(vel_1(1),vel_1(0));//i+1时刻yaw

          if(abs(yaw2-yaw1)>PI)
          {
              is_orientation_adjust = true;
              if((yaw2-yaw1)<0)
              {
                  orientation_adjust = 2*PI;
                  // w = (2*PI+(yaw2-yaw1))/t_step;
              }
              else
              {
                  // w = ((yaw2-yaw1)-2*PI)/t_step;
                  orientation_adjust = -2*PI;
              }
          }
          if(is_orientation_adjust==true)
          {
              yaw += orientation_adjust;
          }
          }

          desired_state(i, 0) = pos[0];//参考x
          desired_state(i, 1) = pos[1];//参考y
          desired_state(i, 2) = yaw;//参考yaw
        }
      //预测段超出最后时间段的部分
      } 
          
      //全部预测部分都超限
      else if (t_cur >= traj_duration_)  {

          local_plan.data.push_back(0.0);
          local_plan.data.push_back(0.0);
          local_plan_pub_.publish(local_plan);
          receive_traj_=false;
          // 清零
          start_time_ = ros::Time(0);
          //便于再从设置目标点
          cur_state_ = StateValue::Pause;
          return;
        
      } else {
        cout << "[Traj server]: invalid time." << endl;
        return;
    }

      Eigen::MatrixXd desired_state1 = desired_state.transpose();

      Eigen::Vector3d current_state;
      geometry_msgs::PoseStamped x_current;
      x_current.header.frame_id = "map";
      x_current.header.stamp = ros::Time::now();
      x_current.pose = robot_pose().value();//获取小车当前位姿作为起点 
      current_state(0) = x_current.pose.position.x;
      current_state(1) = x_current.pose.position.y;
      current_state(2) = tf2::getYaw(x_current.pose.orientation);
        
      // auto yaw_ = tf2::getYaw(x_current.pose.orientation);
        // if(yaw_/desired_state(0,2)<0&&abs(yaw_)>(PI*5/6))//当前yaw
        // {
        //     if(yaw_<0)
        //     {
        //         current_state(2) = yaw_ + 2*PI;
        //     }
        //     else
        //     {
        //         current_state(2) = yaw_ - 2*PI;
        //     }
        // }
        // else
        // {
            // current_state(2) = yaw_;
        // }

      smooth_yaw(desired_state1, current_state);

      ros::Time solve_start = ros::Time::now();
      //MPC模型预测核心
      auto success = mpc_ptr->solve(current_state, desired_state1);//当前状态，参考状态
      // cout << "solve success" << endl;


      //获得首个控制量
      auto control_cmd = mpc_ptr->getFirstU();
      // cout << "got cmd" << endl;

      // cmd.linear.x = control_cmd[0];//v线速度
      // cmd.angular.z = control_cmd[1];//w角速度
      // vel_cmd_pub.publish(cmd);
      //cout << "u:" << result[0] << " " << "r:" << result[1] << endl;
      local_plan.data.push_back(control_cmd[0]);
      local_plan.data.push_back(control_cmd[1]);
      local_plan_pub_.publish(local_plan);

      
      static int conut1 = 0;
      conut1+=1;
      if(conut1%15==0)//调试
      {
          cout<<"v: "<<control_cmd[0] <<"m/s"<<endl;
          cout<<"w: "<<control_cmd[1]<<"rad/s"<<endl;
          ros::Time solve_end = ros::Time::now();
          double t_cur1 = (solve_end - solve_start).toSec();
          cout << "solve time: " << t_cur1 << "s" << endl;
          conut1=0;
      }
        
      nav_msgs::Path predict_path;
      predict_path.header.frame_id = "map";
      predict_path.header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped pose_msg;
      // geometry_msgs::Point pt;
      auto predict_states = mpc_ptr->getPredictX();//预测的N步状态
      // cout << "got predict x" << endl;
      for (int i = 0; i < predict_states.size(); i += 2) {
          pose_msg.pose.position.x = predict_states[i];
          pose_msg.pose.position.y = predict_states[i + 1];
          predict_path.poses.push_back(pose_msg);
      }
      predict_path_pub.publish(predict_path);//可视化N步状态轨迹
      predict_path.poses.clear();


  }

  // 平滑yaw角 (Yaw 角通常被定义为物体在水平面上，绕着垂直于该平面的轴（通常称为 Z 轴）旋转的角度。)
  void AstarNMPCCBF::smooth_yaw(Eigen::MatrixXd& ref_traj, Eigen::Vector3d current_state) {
    double dyaw = ref_traj(2, 0) - current_state[2];

    while (dyaw >= M_PI / 2)
    {
      ref_traj(2, 0) -= M_PI * 2;
      dyaw = ref_traj(2, 0) - current_state[2];
    }
    while (dyaw <= -M_PI / 2)
    {
      ref_traj(2, 0) += M_PI * 2;
      dyaw = ref_traj(2, 0) - current_state[2];
    }

    for (int i = 0; i < N - 1; i++)
    {
      dyaw = ref_traj(2, i+1) - ref_traj(2, i);
      while (dyaw >= M_PI / 2)
      {
        ref_traj(2, i+1) -= M_PI * 2;
        dyaw = ref_traj(2, i+1) - ref_traj(2, i);
      }
      while (dyaw <= -M_PI / 2)
      {
        ref_traj(2, i+1) += M_PI * 2;
        dyaw = ref_traj(2, i+1) - ref_traj(2, i);
      }
    }
  }


  //切到MPC-CBF状态
  // void AstarNMPCCBF::change2dcbf_cb(const std_msgs::Bool::ConstPtr& msg) {
  //   if(msg->data){
  //     //状态切换，由于publish端可能会一直发msg来，这里不需要太多操作
  //       cur_state_ = StateValue::Run;
  //       running_state_ = RunStateValue::Goto;//使用MPC-CBF

  //       receive_traj_=false;
  //       // 清零
  //       start_time_ = ros::Time(0);
        
  //       ROS_INFO("Change2dcbf-------------------");

  //   }
  //   else{//保持状态
  //       cur_state_ = StateValue::Run;
  //       running_state_ = RunStateValue::Follow;//使用MPC

  //   }
  // }


  // NMPC->MPC-CBF
  void AstarNMPCCBF::change2dcbf_cb(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data){
        //状态切换，由于publish端可能会一直发msg来，这里不需要太多操作

        cur_state_ = StateValue::Run;
        running_state_ = RunStateValue::Goto;//在这个Goto中使用MPC-CBF算法

        receive_traj_=false;//清空当前跟踪的B-样条轨迹，NMPC跟踪停止，速度指令归零
        // 清零
        start_time_ = ros::Time(0);

        // 开启局部路径规划定时器
        global_path_timer.start();


        ROS_INFO("Change2dcbf-------------------");
    }
    else{//保持状态
        cur_state_ = StateValue::Run;
        running_state_ = RunStateValue::Follow;//继续维持NMPC纯跟踪

    }
  }


  // 回到普通mpc态
  void AstarNMPCCBF::change2mpc_cb(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data){

      //状态切换
    // 使用std::lock_guard对map_update_mutex_进行加锁，以确保在处理地图数据时不会被其他线程打断；退出函数自动析构
      std::lock_guard<std::mutex> lock(change_state_mutex_);

    // 关闭定时发布局部路径的定时器
    global_path_timer.stop();

      // 裁剪出从control_index_开始的新control_path
      auto control_path_ = control_path.poses;
      control_path_.erase(control_path_.begin(), control_path_.begin() + control_index_);
    //获取当前小车位姿
    auto pose = robot_pose().value();//获取小车当前位姿作为起点
    geometry_msgs::PoseStamped pose_;
    pose_.header.frame_id = "map";
    pose_.pose = pose;
      control_path_.insert(control_path_.begin(), pose_);
      control_path.poses.clear();
      control_path.header.frame_id = "map";
      control_path.poses = control_path_;

      //重新规划B样条
      //初始化控制点矩阵(2D)
      MatrixXd control_points = MatrixXd::Zero(control_path.poses.size(), 2);
      for(int i=0;i<control_path.poses.size();i++)
      {
          //把得到的路径点作为控制点
          Vector2d current_point(control_path.poses[i].pose.position.x, control_path.poses[i].pose.position.y);
          control_points.row(i) = current_point;
      }        

  // 创建新的均匀B样条曲线pos_traj
    NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关
    // B_spline_trajectory.setUniformBspline(control_points,3,ts);//3阶

    double to = pos_traj.getTimeSum();
    pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置合速度上限、合加速度上限、容忍值
    bool feasible = pos_traj.checkFeasibility(false);

    int iter_num = 0;
    while (!feasible && ros::ok()) {
      feasible = pos_traj.reallocateTime();
      if (++iter_num >= 3) break;
    }

    // pos.checkFeasibility(true);
    // cout << "[Main]: iter num: " << iter_num << endl;

    double tn = pos_traj.getTimeSum();

    cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
    if (tn / to > 3.0) ROS_ERROR("reallocate error.");

    // t_adjust = (ros::Time::now() - t1).toSec();
    // cout << "[kino replan]: time: " << ", adjust time:" << t_adjust << endl;


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
    // ROS_INFO("traj_duration_:%f",traj_duration_); 

    // nav_msgs::Path b_spline_trajectory;
    b_spline_trajectory.header.frame_id = "map";
    for(double t_c=t;t_c<tmp;t_c+=dt)
    {
        Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
        // Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = point_temp(0);//轨迹点
        current_pose.pose.position.y = point_temp(1);
        current_pose.header.frame_id = "map";
        b_spline_trajectory.poses.push_back(current_pose);//样条曲线
    }
    trajectory_pub.publish(b_spline_trajectory);//可视化完整的B样条曲线
    b_spline_trajectory.poses.clear();

    // start_time_ = ros::Time::now();

    // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
    receive_traj_ = true;

    //切换状态，变为跟随态
    cur_state_ = StateValue::Run;
    running_state_ = RunStateValue::Follow;
    ROS_INFO("Change2mpc--------------------------");
    
    }
    else{//维持状态
        cur_state_ = StateValue::Run;
        running_state_ = RunStateValue::Goto;//使用MPC-CBF
    }
  }


  // 判断邻近目标点时切换回跟踪状态
  // void AstarNMPCCBF::change2mpc(){

  //   // 关闭定时发布局部路径的定时器
  //   global_path_timer.stop();

  //   // 裁剪出从control_index_开始的新control_path
  //   auto control_path_ = control_path.poses;
  //   control_path_.erase(control_path_.begin(), control_path_.begin() + control_index_);
  //   //获取当前小车位姿
  //   auto pose = robot_pose().value();//获取小车当前位姿作为起点
  //   geometry_msgs::PoseStamped pose_;
  //   pose_.header.frame_id = "map";
  //   pose_.pose = pose;
  //   control_path_.insert(control_path_.begin(), pose_);
  //   control_path.poses.clear();
  //   control_path.header.frame_id = "map";
  //   control_path.poses = control_path_;

  //   //重新规划B样条
  //   //初始化控制点矩阵(2D)
  //   MatrixXd control_points = MatrixXd::Zero(control_path.poses.size(), 2);
  //   for(int i=0;i<control_path.poses.size();i++)
  //   {
  //       //把得到的路径点作为控制点
  //       Vector2d current_point(control_path.poses[i].pose.position.x, control_path.poses[i].pose.position.y);
  //       control_points.row(i) = current_point;
  //   }        

  //   // 创建新的均匀B样条曲线pos_traj
  //   NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关
  //   // B_spline_trajectory.setUniformBspline(control_points,3,ts);//3阶
  //   // 清空轨迹列表traj_，然后将pos_traj、它的一阶导数和二阶导数依次添加到traj_中。
  //   double to = pos_traj.getTimeSum();
  //   pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置合速度上限、合加速度上限、容忍值
  //   bool feasible = pos_traj.checkFeasibility(false);

  //   int iter_num = 0;
  //   while (!feasible && ros::ok()) {
  //     feasible = pos_traj.reallocateTime();
  //     if (++iter_num >= 3) break;
  //   }

  //   // pos.checkFeasibility(true);
  //   // cout << "[Main]: iter num: " << iter_num << endl;

  //   double tn = pos_traj.getTimeSum();

  //   cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  //   if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  //   // t_adjust = (ros::Time::now() - t1).toSec();
  //   // cout << "[kino replan]: time: " << ", adjust time:" << t_adjust << endl;

  //   traj_.clear();
  //   traj_.push_back(pos_traj);//位置
  //   traj_.push_back(traj_[0].getDerivative());//速度
  //   // traj_.push_back(traj_[1].getDerivative());//加速度

  //   double t,tmp;
  //   //节点向量定义区间
  //   traj_[0].getTimeSpan(t,tmp);
    
  //   // B_spline_trajectory.getTimeSpan(t,tmp);
      
  //   ROS_INFO("t:%f",t);  //t=0.0s
  //   ROS_INFO("tmp:%f",tmp);  //tmp=traj_duration_
  // // 计算轨迹的总时长，并将结果赋值给traj_duration_变量。
  //   traj_duration_ = traj_[0].getTimeSum();/////总时间
  //   // ROS_INFO("traj_duration_:%f",traj_duration_); 

  //   // nav_msgs::Path b_spline_trajectory;
  //   b_spline_trajectory.header.frame_id = "map";
  //   for(double t_c=t;t_c<tmp;t_c+=dt)
  //   {
  //       Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
  //       // Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
  //       geometry_msgs::PoseStamped current_pose;
  //       current_pose.pose.position.x = point_temp(0);//轨迹点
  //       current_pose.pose.position.y = point_temp(1);
  //       current_pose.header.frame_id = "map";
  //       b_spline_trajectory.poses.push_back(current_pose);//样条曲线
  //   }
  //   trajectory_pub.publish(b_spline_trajectory);//可视化完整的B样条曲线
  //   b_spline_trajectory.poses.clear();

  //   // start_time_ = ros::Time::now();

  //   // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
  //   receive_traj_ = true;

  //   //切换状态，变为跟随态
  //   cur_state_ = StateValue::Run;
  //   running_state_ = RunStateValue::Follow;
  //   ROS_INFO("Change2mpc--------------------------");  
  // }


  // //实时更新MPC-CBF的全局路径----------点到点直线版本
  // void AstarNMPCCBF::globalPathPub_Callback() {
  //   void AstarNMPCCBF::globalPathPub_Callback(const ros::TimerEvent &e){
  //       // 检查定时器是否已经停止
  //   if (!global_path_timer.isValid())  return;
  //   //获取当前小车位姿
  //   auto pose = robot_pose().value();//获取小车当前位姿作为起点
  //   start_pos[0] = pose.position.x;
  //   start_pos[1] = pose.position.y;
  //   //获取目标控制点，若该控制点被占用，则选择下一个控制点（未实现）
  //   //每次开启该状态后，目标点仅选择一次
  //   if(!is_target_rcv_){
  //     //control_path:控制点集合
  //     size_t idx_;
  //     auto dis = std::numeric_limits<double>::max();
  //     //索引一段路径点找出离小车位姿点最近的路径点
  //     for (auto i = 0; i < control_path.poses.size(); ++i) {
  //       auto err = distance(pose, control_path.poses[i].pose);
  //       if (err < dis) {
  //         dis = err;
  //         idx_ = static_cast<size_t>(i);
  //       }
  //     }
  //     //最近的路径点的索引
  //     if(idx_+13 > control_path.poses.size()-1) idx_=control_path.poses.size()-1;
  //     else idx_ += 13; //为的是目标点不要离小车当前位姿太近
  //     control_index_ = idx_;
  //     target_pos[0] = control_path.poses[idx_].pose.position.x;
  //     target_pos[1] = control_path.poses[idx_].pose.position.y;
  //     is_target_rcv_ = true;
  //   }

  //   double dist = (target_pos - start_pos).norm();
  //   Eigen::Vector3d diff = (target_pos - start_pos) / dist;//分度

  //   // const double step = 0.28;   // = v_max * Ts
  //   double v_max = 0.3;
  //   double step_time = 0.25;
  //   const double step = v_max*step_time;   // = v_max * Ts

  //   nav_msgs::Path global_path;
  //   global_path.header.stamp = ros::Time::now();
  //   global_path.header.frame_id = "map";
  //   geometry_msgs::PoseStamped pose_stamped;
  //   pose_stamped.header.stamp = ros::Time::now();
  //   pose_stamped.header.frame_id = "map";
  //   pose_stamped.pose.orientation.x = 0;
  //   pose_stamped.pose.orientation.y = 0;
  //   pose_stamped.pose.orientation.z = 0;
  //   pose_stamped.pose.orientation.w = 1;

  //   int idx = 0;
  //   for (double i = 0.0; i < dist; i += step) //小车离目标点太近的话，自然pose_stamped就少了
  //   {
  //     pose_stamped.header.seq = idx++;

  //     Eigen::Vector3d waypoint = start_pos + i * diff;
  //     pose_stamped.pose.position.x = waypoint.x();
  //     pose_stamped.pose.position.y = waypoint.y();
  //     pose_stamped.pose.position.z = 0;

  //     global_path.poses.push_back(pose_stamped);
  //   }

  //   global_path_pub.publish(global_path);
  // }


  // 实时更新MPC-CBF的全局路径---------------B样条曲线版本
  // void AstarNMPCCBF::globalPathPub_Callback() {
  //   //获取当前小车位姿
  //   auto pose = robot_pose().value();//获取小车当前位姿作为起点
  //   start_pos[0] = pose.position.x;
  //   start_pos[1] = pose.position.y;
  //   //获取目标控制点，若该控制点被占用，则选择下一个控制点（未实现）
  //   //每次开启该状态后，目标点仅选择一次
  //   if(!is_target_rcv_){
  //     //control_path:控制点集合
  //     size_t idx_;
  //     auto dis = std::numeric_limits<double>::max();
  //     //索引一段路径点找出离小车位姿点最近的路径点
  //     for (auto i = 0; i < control_path.poses.size(); ++i) {
  //       auto err = distance(pose, control_path.poses[i].pose);
  //       if (err < dis) {
  //         dis = err;
  //         idx_ = static_cast<size_t>(i);
  //       }
  //     }
  //     //最近的路径点的索引
  //     if(idx_+17 > control_path.poses.size()-1) control_index_=control_path.poses.size()-1;
  //     else control_index_ = idx_+17; //为的是目标点不要离小车当前位姿太近
  //     target_pos[0] = control_path.poses[control_index_].pose.position.x;
  //     target_pos[1] = control_path.poses[control_index_].pose.position.y;
  //     is_target_rcv_ = true;

  //     // 裁剪掉从0开始到inx_的控制点
  //     // 裁剪掉从control_index_ + 17 (不包括17所用)开始到最后(包括最后)的控制点
  //     // 定义要保留的中间一段数据的起始和结束索引
  //     int start_index = idx_;  // 起始索引（包含）
  //     int end_index = control_index_;    // 结束索引（不包含）
  //     // 使用切片操作裁剪向量
  //     std::vector<geometry_msgs::PoseStamped> control_path_global(control_path.poses.begin() + start_index, control_path.poses.begin() + end_index +1);
  //     control_path_global_.poses.clear();
  //     control_path_global_.header.frame_id = "map";    
  //     control_path_global_.poses = control_path_global;

  //   }

  //   //利用B样条曲线,生成路线
  //   //找到离当前点最近的控制点
  //     size_t idx_;
  //     auto dis = std::numeric_limits<double>::max();
  //     //索引一段路径点找出离小车位姿点最近的路径点
  //     for (auto i = 0; i < control_path_global_.poses.size(); ++i) {
  //       auto err = distance(pose, control_path_global_.poses[i].pose);
  //       if (err < dis) {
  //         dis = err;
  //         idx_ = static_cast<size_t>(i);
  //       }
  //     }
  //     // 裁剪出全局路径控制点
  //     // 为了防止指针越界,满足退出条件(mpc_dcbf_soft.py中)一定要严谨
  //     // 使用 std::distance 函数来计算两个迭代器之间的距离
  //     std::vector<geometry_msgs::PoseStamped> control_path_global_a;
  //     if (std::distance(control_path_global_.poses.begin() + idx_+1, control_path_global_.poses.end()) <= 0)
  //     {
  //         //取control_path_global_最后一个控制点
  //         control_path_global_a.push_back(control_path_global_.poses[control_path_global_.poses.size()-1]);
  //     } 
  //     else{
  //       //end()函数用于返回一个指向容器中最后一个元素之后位置的迭代器
  //       std::vector<geometry_msgs::PoseStamped> control_path_global(control_path_global_.poses.begin() + idx_+1, control_path_global_.poses.end());
  //       control_path_global_a = control_path_global;
  //     }


  //     ///当前位置不太对

  //     geometry_msgs::PoseStamped pose_;
  //     pose_.header.frame_id = "map";
  //     pose_.header.stamp = ros::Time::now();;
  //     pose_.pose = pose;
  //     control_path_global_a.insert(control_path_global_a.begin(), pose_);//当前小车位姿

  //    //找到离目标点最近的控制点(就是目标点本身)

  //   //将当前点-控制点集-目标点构造成B样条曲线
  //   //初始化控制点矩阵(2D)
  //   MatrixXd control_points = MatrixXd::Zero(control_path_global_a.size(), 2);
  //   for(int i=0;i<control_path_global_a.size();i++)
  //   {
  //       //把得到的路径点作为控制点
  //       Vector2d current_point(control_path_global_a[i].pose.position.x, control_path_global_a[i].pose.position.y);
  //       control_points.row(i) = current_point;
  //   }        
  //   // 创建新的均匀B样条曲线pos_traj
  //   NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关 ts=0.8s契合vmax=0.5
  //   // 清空轨迹列表traj_，然后将pos_traj、它的一阶导数和二阶导数依次添加到traj_中。
  //   traj_.clear();
  //   traj_.push_back(pos_traj);//位置

  //   //获取曲线上的位姿点
  //   double t,tmp;
  //   //节点向量定义区间
  //   traj_[0].getTimeSpan(t,tmp);
  //   // ROS_INFO("t:%f",t);  //t=0.0s
  //   ROS_INFO("tmp:%f",tmp);  //tmp=traj_duration_
  //   // 计算轨迹的总时长，并将结果赋值给traj_duration_变量。
  //   // traj_duration_ = traj_[0].getTimeSum();/////总时间

  //   nav_msgs::Path global_path;
  //   global_path.header.stamp = ros::Time::now();
  //   global_path.header.frame_id = "map";
  //   geometry_msgs::PoseStamped pose_stamped;
  //   pose_stamped.header.stamp = ros::Time::now();
  //   pose_stamped.header.frame_id = "map";
  //   pose_stamped.pose.orientation.x = 0;
  //   pose_stamped.pose.orientation.y = 0;
  //   pose_stamped.pose.orientation.z = 0;
  //   pose_stamped.pose.orientation.w = 1;
  //   int idx = 0;

  //   double step_time = 0.3;
  //   for(double t_c=t;t_c<tmp;t_c+=step_time)
  //   {
  //     pose_stamped.header.seq = idx++;
  //     Vector2d waypoint = traj_[0].evaluateDeBoor(t_c);
  //     pose_stamped.pose.position.x = waypoint.x();
  //     pose_stamped.pose.position.y = waypoint.y();
  //     pose_stamped.pose.position.z = 0;
  //     global_path.poses.push_back(pose_stamped);
  //   }
  //   global_path_pub.publish(global_path);
  // }


  //实时更新MPC-CBF的参考路径----------A*/Hybrid A*版本
  // void AstarNMPCCBF::globalPathPub_Callback() {
  void AstarNMPCCBF::globalPathPub_Callback(const ros::TimerEvent &e){
    // 检查定时器是否已经停止
    if (!global_path_timer.isValid())  return;
    
    //获取当前小车位姿
    auto pose = robot_pose().value();//获取小车当前位姿作为起点
    start_pos[0] = pose.position.x;
    start_pos[1] = pose.position.y;
    start_pos[2] = tf2::getYaw(pose.orientation);
    //获取目标控制点，若该控制点被占用，则选择下一个控制点（未实现）
    //每次开启该状态后，目标点仅选择一次
    if(!is_target_rcv_){
      //control_path:控制点集合
      size_t idx_;
      auto dis = std::numeric_limits<double>::max();
      //索引一段路径点找出离小车位姿点最近的路径点
      for (auto i = 0; i < control_path.poses.size(); ++i) {
        auto err = distance(pose, control_path.poses[i].pose);
        if (err < dis) {
          dis = err;
          idx_ = static_cast<size_t>(i);
        }
      }
      //最近的路径点的索引
      if(idx_+12 > control_path.poses.size()-1) idx_=control_path.poses.size()-1;
      else idx_ = idx_+12; //为的是目标点不要离小车当前位姿太近
      control_index_ = idx_;
      target_pos[0] = control_path.poses[idx_].pose.position.x;
      target_pos[1] = control_path.poses[idx_].pose.position.y;
      target_pos[2] = tf2::getYaw(control_path.poses[idx_].pose.orientation);
      target_pose.position.x = target_pos[0];//目标点，用于判断当前车位是否靠近目标点
      target_pose.position.y = target_pos[1];
      is_target_rcv_ = true;
    }

    auto distance_c = [&](Eigen::VectorXd c1, Eigen::Vector2d c2) {
      return std::hypot(c2.x() - c1.x(), c2.y() - c1.y());
    };

    auto min_dis = std::numeric_limits<double>::max();
    for (const auto& obs : obs_c)
    {
      //索引一段路径点找出离小车位姿点最近的路径点
        auto err = distance_c(start_pos, obs);
        if (err < min_dis) {
          min_dis = err;
        }
    }
    nav_msgs::Path gridPath;
    // A*规划
    if(min_dis < 0.6)   gridPath = astar_path_finder->makePlanandBezierOpt(start_pos, target_pos);
    else   gridPath = astar_path_finder->makePlanandBezierOpt_Collision(start_pos, target_pos);
    // Hybrid A*规划
    // auto gridPath = hybrid_astar_path_finder->HybridAstarSearch(start_pos, target_pos);
    if (gridPath.poses.empty()) {
      ROS_ERROR("Astar path not found!");
      return;
    }
    // while(gridPath.poses.empty()){
    //   ROS_ERROR("Astar path not found!");
    //   auto num=0;
      
    //   control_index_++;//往前推5个控制点
    //   if(num++ > 5) 
    //   // if(control_index_ > control_path.poses.size()-1) control_index_=control_path.poses.size()-1;
    //   target_pos[0] = control_path.poses[control_index_].pose.position.x;
    //   target_pos[1] = control_path.poses[control_index_].pose.position.y;
    //   target_pos[2] = tf2::getYaw(control_path.poses[control_index_].pose.orientation);
    //   target_pose.position.x = target_pos[0];//目标点，用于判断当前车位是否靠近目标点
    //   target_pose.position.y = target_pos[1];
    //   gridPath = astar_path_finder->makePlanandBezierOpt(start_pos, target_pos);
    // }

    //对这些控制点进行B样条化
      //初始化控制点矩阵(2D)
      MatrixXd control_points = MatrixXd::Zero(gridPath.poses.size(), 2);
      for(int i=0;i<gridPath.poses.size();i++)
      {
          //把得到的路径点作为控制点
          Vector2d current_point(gridPath.poses[i].pose.position.x, gridPath.poses[i].pose.position.y);
          control_points.row(i) = current_point;
      }        

  // 创建新的均匀B样条曲线pos_traj
    NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关
    // B_spline_trajectory.setUniformBspline(control_points,3,ts);//3阶

    // double to = pos_traj.getTimeSum();
    // pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置合速度上限、合加速度上限、容忍值
    // bool feasible = pos_traj.checkFeasibility(false);

    // int iter_num = 0;
    // while (!feasible && ros::ok()) {
    //   feasible = pos_traj.reallocateTime();
    //   if (++iter_num >= 3) break;
    // }

    // pos.checkFeasibility(true);
    // cout << "[Main]: iter num: " << iter_num << endl;

    // double tn = pos_traj.getTimeSum();

    // cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
    // if (tn / to > 3.0) ROS_ERROR("reallocate error.");

    // t_adjust = (ros::Time::now() - t1).toSec();
    // cout << "[kino replan]: time: " << ", adjust time:" << t_adjust << endl;
      
  // 清空轨迹列表traj_，然后将pos_traj、它的一阶导数和二阶导数依次添加到traj_中。
    traj_.clear();
    traj_.push_back(pos_traj);//位置
    // traj_.push_back(traj_[0].getDerivative());//速度
    // traj_.push_back(traj_[1].getDerivative());//加速度

    double t,tmp;
    //节点向量定义区间
    traj_[0].getTimeSpan(t,tmp);
    
    // B_spline_trajectory.getTimeSpan(t,tmp);
      
    // ROS_INFO("t:%f",t);  //t=0.0s
    // ROS_INFO("tmp:%f",tmp);  //tmp=traj_duration_
  // 计算轨迹的总时长，并将结果赋值给traj_duration_变量。
    // traj_duration_ = traj_[0].getTimeSum();/////总时间
    // ROS_INFO("traj_duration_:%f",traj_duration_); 

    nav_msgs::Path b_spline_global;
    b_spline_global.header.frame_id = "map";
    for(double t_c=t;t_c<tmp;t_c+=0.1)//同MPC-DCF的时间间隔
    {
        Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
        // Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
        geometry_msgs::PoseStamped current_pose;
        current_pose.pose.position.x = point_temp(0);//轨迹点
        current_pose.pose.position.y = point_temp(1);
        current_pose.header.frame_id = "map";
        b_spline_global.poses.push_back(current_pose);//样条曲线
    }
    global_path_pub.publish(b_spline_global);//可视化完整的B样条曲线

    // start_time_ = ros::Time::now();

    // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
    // receive_traj_ = true;


  /**************************************************************************
    // double dist = (target_pos - start_pos).norm();
    // Eigen::Vector2d diff = (target_pos - start_pos) / dist;//分度

    // const double step = 0.28;   // = v_max * Ts
    // double v_max = 0.5;
    // double step_time = 0.3;
    // const double step = v_max*step_time;   // = v_max * Ts

    // nav_msgs::Path global_path;
    // global_path.header.stamp = ros::Time::now();
    // global_path.header.frame_id = "map";
    // geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.stamp = ros::Time::now();
    // pose_stamped.header.frame_id = "map";
    // pose_stamped.pose.orientation.x = 0;
    // pose_stamped.pose.orientation.y = 0;
    // pose_stamped.pose.orientation.z = 0;
    // pose_stamped.pose.orientation.w = 1;

    // int idx = 0;
    // for (double i = 0; i < gridPath.size(); i++) //小车离目标点太近的话，自然pose_stamped就少了
    // {
    //   pose_stamped.header.seq = idx++;

    //   pose_stamped.pose.position.x = gridPath[i].x();
    //   pose_stamped.pose.position.y = gridPath[i].y();
    // change2mpc_cbed.pose.position.z = 0;

    //   global_path.poses.push_back(pose_stamped);
    // }

  ************************************************************ **/
    // global_path_pub.publish(gridPath);

  }


  // 实时更新NMPC-CBF要跟踪的局部路径---------------A*版本---定时器版本
  // void AstarNMPCCBF::globalPathPub_Callback() {
  // void AstarNMPCCBF::globalPathPub_Callback(const ros::TimerEvent &e){
  //   // 检查定时器是否已经停止
  //   if (!global_path_timer.isValid())
  //   {
  //       return;
  //   }
  //   //获取当前小车位姿
  //   auto pose = robot_pose().value();//获取小车当前位姿作为起点
  //   start_pos[0] = pose.position.x;
  //   start_pos[1] = pose.position.y;
  //   start_pos[2] = tf2::getYaw(pose.orientation);
  //   //获取目标控制点，若该控制点被占用，则选择下一个控制点（未实现）
  //   //每次开启该状态后，目标点仅选择一次
  //   if(!is_target_rcv_){
  //     //control_path:控制点集合
  //     size_t idx_;
  //     auto dis = std::numeric_limits<double>::max();
  //     //索引一段路径点找出离小车位姿点最近的路径点
  //     for (auto i = 0; i < control_path.poses.size(); ++i) {
  //       auto err = distance(pose, control_path.poses[i].pose);
  //       if (err < dis) {
  //         dis = err;
  //         idx_ = static_cast<size_t>(i);
  //       }
  //     }
  //     //最近的路径点的索引
  //     if(idx_+15 > control_path.poses.size()-1) idx_=control_path.poses.size()-1;
  //     else idx_ = idx_+15; //为的是目标点不要离小车当前位姿太近
  //     control_index_ = idx_;
  //     target_pos[0] = control_path.poses[idx_].pose.position.x;
  //     target_pos[1] = control_path.poses[idx_].pose.position.y;
  //     target_pos[2] = tf2::getYaw(control_path.poses[idx_].pose.orientation);
  //     target_pose.position.x = target_pos[0];
  //     target_pose.position.y = target_pos[1];
  //     is_target_rcv_ = true;
  //   }

  //   //确保调用这个函数时，起终点都在grid范围内（一般都能保证）
  //   // A*规划
  //   auto gridPath = astar_path_finder->makePlanandBezierOpt(start_pos, target_pos);
  //   // Hybrid A*规划------用来配合NMPC跟踪的话效果很差
  //   // auto gridPath = hybrid_astar_path_finder->HybridAstarSearch(start_pos, target_pos);
  //   if (gridPath.poses.empty()) {
  //     ROS_INFO("Astar path not found!");
  //     return;
  //   }

  //   //对这些控制点进行B样条化
  //     //初始化控制点矩阵(2D)
  //     MatrixXd control_points = MatrixXd::Zero(gridPath.poses.size(), 2);
  //     for(int i=0;i<gridPath.poses.size();i++)
  //     {
  //         //把得到的路径点作为控制点
  //         Vector2d current_point(gridPath.poses[i].pose.position.x, gridPath.poses[i].pose.position.y);
  //         control_points.row(i) = current_point;
  //     }        

  // // 创建新的均匀B样条曲线pos_traj
  //   NonUniformBspline pos_traj(control_points, 3, ts);//3阶  ts与要求车速有关
  //   // B_spline_trajectory.setUniformBspline(control_points,3,ts);//3阶

  //   double to = pos_traj.getTimeSum();
  //   pos_traj.setPhysicalLimits(0.5, 2.5, 0.05);//设置合速度上限、合加速度上限、容忍值
  //   bool feasible = pos_traj.checkFeasibility(false);

  //   int iter_num = 0;
  //   while (!feasible && ros::ok()) {
  //     feasible = pos_traj.reallocateTime();
  //     if (++iter_num >= 3) break;
  //   }

  //   // pos.checkFeasibility(true);
  //   // cout << "[Main]: iter num: " << iter_num << endl;

  //   double tn = pos_traj.getTimeSum();

  //   cout << "[kino replan]: Reallocate ratio: " << tn / to << endl;
  //   if (tn / to > 3.0) ROS_ERROR("reallocate error.");

  //   // t_adjust = (ros::Time::now() - t1).toSec();
  //   // cout << "[kino replan]: time: " << ", adjust time:" << t_adjust << endl;
      
  // // 清空轨迹列表traj_，然后将pos_traj、它的一阶导数和二阶导数依次添加到traj_中。
  //   traj_.clear();
  //   traj_.push_back(pos_traj);//位置
  //   traj_.push_back(traj_[0].getDerivative());//速度
  //   // traj_.push_back(traj_[1].getDerivative());//加速度

  //   double t,tmp;
  //   //节点向量定义区间
  //   traj_[0].getTimeSpan(t,tmp);
    
  //   // B_spline_trajectory.getTimeSpan(t,tmp);
      
  //   ROS_INFO("t:%f",t);  //t=0.0s
  //   ROS_INFO("tmp:%f",tmp);  //tmp=traj_duration_
  // // 计算轨迹的总时长，并将结果赋值给traj_duration_变量。
  //   traj_duration_ = traj_[0].getTimeSum();/////总时间
  //   // ROS_INFO("traj_duration_:%f",traj_duration_); 

  //   nav_msgs::Path b_spline_global;
  //   b_spline_global.header.frame_id = "map";
  //   for(double t_c=t;t_c<tmp;t_c+=dt)
  //   {
  //       Vector2d point_temp = traj_[0].evaluateDeBoor(t_c);
  //       // Vector2d point_temp = B_spline_trajectory.evaluateDeBoor(t_c);
  //       geometry_msgs::PoseStamped current_pose;
  //       current_pose.pose.position.x = point_temp(0);//轨迹点
  //       current_pose.pose.position.y = point_temp(1);
  //       current_pose.header.frame_id = "map";
  //       b_spline_global.poses.push_back(current_pose);//样条曲线
  //   }
  //   global_path_pub.publish(b_spline_global);//可视化完整的B样条曲线

  //   start_time_ = ros::Time::now();

  //   // 设置receive_traj_变量为true，表示已经成功接收到了轨迹。
  //   receive_traj_ = true;

  //   //切换状态，变为跟随态
  //   // cur_state_ = StateValue::Run;
  //   // running_state_ = RunStateValue::Follow;

  //   // global_path_pub.publish(gridPath);
  // }


  // 我们假定，如果能接收到这个回调函数，说明障碍物在小车可感知的范围内
  // 这样一直回调，说明一直能感知到障碍---------
  // 接收障碍物信息的回调函数
  void AstarNMPCCBF::obs_trajs_cb(const dynamic_simulator::DynTrajList& msg)     
  { 
    //使用MPC-CBF时再开启该回调
    if(running_state_ != RunStateValue::Goto)  return;
    //添加动态障碍物的实时变化的占据栅格，先添加对应点上一圈栅格，后续看效果再加上未来一段时间的占据栅格
    boost::mutex::scoped_lock lock(map_mutex_);

    // 把感知到的障碍物一次性添加过来，即一次回调一次性处理
    // 其实最好是将这些信息封装起来
    vector<double> circle_R_;  //外接圆半径
    vector<Eigen::Vector2d> center_;//障碍物外接圆圆心
    // vector<int32_t> id_;//障碍物id号
    
    for (const auto& obs : msg.obs_set)
    {
      Eigen::Vector2d bbox_;
      bbox_ << obs.bbox[0], obs.bbox[1];//尺寸
      Eigen::Vector2d bbox_half = bbox_ / 2.0;
      // 取bbox外接圆半径
      // 这里的膨胀交给了局部地图那边-----膨胀地图用
      circle_R_.push_back(bbox_half.norm());//膨胀50cm：小车宽度的一半，后续就交给CBF或者碰撞检测（如果不膨胀的话）
      // 碰撞检测算法用
      // circle_R_.push_back(bbox_half.norm());

      Eigen::Vector2d center;
      center << obs.pos.x, obs.pos.y;
      center_.push_back(center);
      // id_.push_back(obs.id);//障碍物id号
    }

    // 计算两个点之间的距离
    auto distance_c = [&](Eigen::VectorXd c1, Eigen::Vector2d c2) {
      return std::hypot(c2.x() - c1.x(), c2.y() - c1.y());
    };
    // 根据移动方向和距离移动圆的中心
    auto moveCircleCenter = [&](Eigen::Vector2d c1, double directionX, double directionY, double distance) {
        Eigen::Vector2d c_new;
        c_new.x() = c1.x() + directionX * distance;
        c_new.y() = c1.y() + directionY * distance;
        return c_new;
    };
    
    static bool once=true; 
    Eigen::Vector2d movedCenter;
    if(!once){

      auto pose = robot_pose().value();//获取小车当前位姿作为起点
      start_pos[0] = pose.position.x;
      start_pos[1] = pose.position.y;

      auto min_dis = std::numeric_limits<double>::max();
      for (const auto& obs : center_)
      {
        //索引一段路径点找出离小车位姿点最近的路径点
          auto err = distance_c(start_pos, obs);
          if (err < min_dis) {
            min_dis = err;
          }
      }
      if(min_dis > 0.6) {
        //其实应该根据id号
        auto num = circle_R_.size();
        for(auto i = 0; i < num; i++){
          // 计算移动方向和距离
          double d = distance_c(obs_c[i], center_[i]);
          if (d == 0) {
            movedCenter = center_[i];
          }
          else{
            double directionX = (center_[i].x() - obs_c[i].x()) / d;
            double directionY = (center_[i].y() - obs_c[i].y()) / d;

            // 实际上可以接收 /obs_Manager_node/obs_predict_pub 消息

            // 假设移动 x 米
            double d0 = circle_R_[i]*2.3;  // 移动直径距离，（不考虑障碍物往回走
            // 根据移动方向和距离移动圆的中心
            movedCenter = moveCircleCenter(center_[i], directionX, directionY, d0);//新圆的圆心
          }

          center_.push_back(movedCenter);
          circle_R_.push_back(circle_R_[i]/1.5);
        }

      }

    }

    // 保存，供局部栅格地图用
    obs_R_ = circle_R_;
    obs_c = center_;
    once = false;
    // obs_id = id_;
  }


  // 生成供局部的全局路径规划的局部的栅格地图，MPC-CBF使用
  void AstarNMPCCBF::local_gird_map()  
  {
    if(obs_R_.empty() || obs_c.empty()) return;
    nav_msgs::OccupancyGrid grid;//每次进入这个函数重新赋值
    grid.info.resolution = grid_.info.resolution;
    grid.info.map_load_time = ros::Time::now();
    grid.info.origin.orientation = grid_.info.origin.orientation;
  
    //获取当前小车位姿
    auto pose = robot_pose().value();//获取小车当前位姿作为起点
    //转化为栅格索引
    int cx = (int)((pose.position.x - grid_.info.origin.position.x) / grid_.info.resolution);
    int cy = (int)((pose.position.y - grid_.info.origin.position.y) / grid_.info.resolution);  
    //以小车当前位姿为中心，上下左右各拓宽10m（0.1分辨率下为100个栅格），作动态窗口
    VecXd limit_zone(4);
    //栅格索引：xmin,xmax,ymin,ymax
    limit_zone(0) = std::max(cx-70, 0);//区域左的x坐标索引
    limit_zone(1) = std::min(cx+70, (int)(grid_.info.width-1));//右
    limit_zone(2) = std::max(cy-70, 0);//区域下的y坐标索引
    limit_zone(3) = std::min(cy+70, (int)(grid_.info.height-1));//上
    // ROS_INFO("0= %d" ,limit_zone(0));
    // ROS_INFO("1= %d" ,limit_zone(1));
    // ROS_INFO("2= %d" ,limit_zone(2));
    // ROS_INFO("3= %d" ,limit_zone(3));
    grid.info.height = static_cast<uint32_t>(limit_zone(3) - limit_zone(2) + 1);  //行索引数
    // ROS_INFO("height= %d" ,grid.info.height);
    grid.info.width = static_cast<uint32_t>(limit_zone(1) - limit_zone(0) + 1);  //列索引数
    // ROS_INFO("width= %d" ,grid.info.width);
    //求左下角原点的坐标（以原map坐标系为参照）  索引->坐标
    grid.info.origin.position.x = (limit_zone(0)+0.5) * grid_.info.resolution + grid_.info.origin.position.x;
    grid.info.origin.position.y = (limit_zone(2)+0.5) * grid_.info.resolution + grid_.info.origin.position.y;
    grid.info.origin.position.z = 0.;

    grid.data.resize(grid.info.width * grid.info.height);//确定大小

    // ROS_INFO("22222222");

    //应该还要加上原地图的占据栅格才对，所以只能从原grid_上切片，这一步在初始化时做
    //从索引i->grid_中的索引
    int i=0;
    for (int y = limit_zone(2); y < limit_zone(2) + grid.info.height; y++) {
        for (int x = limit_zone(0); x < limit_zone(0) + grid.info.width; x++) {
            grid.data[i++] = grid_.data[y * grid_.info.width + x];//初始化
        }
    }
    //给原始地图膨胀
    //膨胀栅格---碰撞检测算法不需要
    const double chibot_rad = 0.3;
    const int inf_num = ceil(chibot_rad/grid.info.resolution);
    // ROS_INFO("inf_num= %d" ,inf_num);
    set<int> o={};
    find_gezi(grid.info.width, grid.info.width * grid.info.height, grid.data, inf_num, o);

    // std::vector<signed char> p(a, a+counts);
    // output.data=p;
    // copy_map(output,carto_map_inflation);

    // ROS_INFO("3333333333");

    //对grid操作
    //获取外接圆上的所有点的坐标，转化为索引，赋值为占据
    for (size_t j = 0; j < obs_R_.size(); j++)
    {
      const double rSquare = obs_R_[j] * obs_R_[j];
      const double resolution = grid_.info.resolution;
      const double resolution2 = 2*resolution;//反正那边会膨胀，这里可以间隔采样
      for (double x = obs_c[j][0] - obs_R_[j]; x <= obs_c[j][0] + obs_R_[j]; x += resolution2)
      {
          for (double y = obs_c[j][1] - obs_R_[j]; y <= obs_c[j][1] + obs_R_[j]; y += resolution2)
          {
              // 障碍物外接圆面上的点
              if (std::pow(x - obs_c[j][0], 2) + std::pow(y - obs_c[j][1], 2) - rSquare < resolution)
              {
                //圆面上点的x,y索引（相对于grid_）
                int xi = (int)((x - grid_.info.origin.position.x) / resolution);
                int yi = (int)((y - grid_.info.origin.position.y) / resolution);
                //筛选出小车动态窗口中的点，存入
                if (xi >= limit_zone(0) && xi <= limit_zone(1) && yi >= limit_zone(2) && yi <= limit_zone(3)) {
                    //计算（xi,yi）相对于grid的索引
                    xi = xi-limit_zone(0);
                    yi = yi-limit_zone(2);
                    int index = (int)(xi + yi * grid.info.width);
                    //这里面的索引应该是相对这个grid的
                    grid.data[index] = 100;//标记为障碍
                }
              }
          }
      }
    }
    
    // ROS_INFO("444444444");
    //发布小车周围变化的局部栅格地图作为A*的搜路地图
    //新的nav_msgs::OccupancyGrid构建
    local_grid_pub.publish(grid);
    obs_R_.clear();
    obs_c.clear();
  }


  // 膨胀栅格:行数,总栅格数,a为膨胀前/后,膨胀格数,s为标记膨胀的栅格
  // void AstarNMPCCBF::find_gezi(const int hang,const int counts,int8_t* a,int floor,set<int>& s){
  void AstarNMPCCBF::find_gezi(const int hang, const int counts, std::vector<int8_t> a, int floor, set<int>& s){
    if (floor > 0 )
    {
      int f[8]={ };
      for (int i = 0; i < counts; ++i)
      {
        //代价值为1（障碍物占用）
        if (a[i]==100)
        {
          //标记需要膨胀的栅格
          f[0]=i-hang-1;
          f[3]=i-1;
          f[6]=i+hang-1;
          f[1]=i-hang;
          f[7]=i+hang;
          f[2]=i-hang+1;
          f[5]=i+1;
          f[8]=i+hang+1;
          // f[4]=i;
        }
        for (int j = 0; j < 8; j++ )
        { 
          //将所有要膨胀的栅格赋给s
          s.insert(f[j]);
        }
      }
      //给标记为膨胀的栅格膨胀，赋100，为障碍物
      for( auto beg=s.begin(),end=s.end(); beg!=end ; ++beg)
      {
        if (-1 < *beg && *beg < counts)
          a[*beg]=100;
      }
    }else {
      return;
    }
    find_gezi(hang,counts,a,floor-1,s);//递归，直到floor=0结束膨胀
  }


}
