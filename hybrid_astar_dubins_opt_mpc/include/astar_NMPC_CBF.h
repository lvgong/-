#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>//nav_msgs/GetMap是map_server提供的服务的消息类型。

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>//提供boost::mutex类，用于多线程环境下的同步控制，防止数据竞争
#include <vector>//引入动态数组容器std::vector，提供动态大小的数组，支持随机访问、尾部高效插入/删除
#include <fstream>//用于读写文件
#include <chrono>//引入时间库，提供高精度计时和时间点操作
#include <iostream> //手动输出信息到终端 std::cout <<

// #include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <mutex>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include "hybrid_astar_searcher/visualize.h"//提供算法中间/最终结果的图形化调试工具
#include "hybrid_astar_dubins_opt_mpc/chibot_task.h"//集成规划（Hybrid A* + Dubins)和控制(MPC)，实现任务级逻辑
#include "half_struct_planner.h"//处理复杂或非完全结构化环境中的规划问题
// #include "mbf_msgs/MoveBaseAction.h"
// #include "mbf_msgs/ExePathAction.h"
// #include "costmap_2d/costmap_2d.h"
// #include "costmap_2d/costmap_2d_ros.h"
#include "math.h"
#include "time.h"

// 它们放在子头文件夹里，所以要加子文件夹名
#include "bspline/non_uniform_bspline.h"
#include "mpc_tracking/mpc.h"
// 还得加上对应的这个功能包
#include "dynamic_simulator/DynTrajList.h"   // 动态障碍物预测轨迹消息类型
#include "astarPlanner/astar_searcher.h"  // A*相关
#include "hybrid_astar_searcher/dynamic_hybrid_astar.h"   //hybrid A*相关

/*
  chibot 外层命名空间
  topologyplanning 内层命名空间,表示拓扑规划相关功能模块

  等价的传统的嵌套写法：
  namespace chibot {
    namespace topologyplanning {
        // 类、函数、变量等定义
    }
  }
*/
namespace chibot::topologyplanning {
  /*
  constexpr 表示该变量是编译时常量，值在编译阶段即可确定（而非运行是计算）
  优势：编译器会直接替换它的值，避免运行时开销，可能带来性能优化

  static 表示该变量是静态存储期的，且作用域限于当前文件或当前类

  const char* 定义一个指向常量字符（字符串）的指针，即字符串内容不可修改（只读）
  */
  constexpr static const char* NODE_NAME = "hybrid_astar_dubins_opt";//结合 混合A*算法和Dubins路径 的路径规划节点
  constexpr static const char* MPC_NODE_NAME = "hybrid_astar_mpc";//结合 混合A*和模型预测控制（MPC) 的轨迹优化节点
  constexpr static const char* LMPC_NODE_NAME = "hybrid_astar_Bspline_LMPC";//使用 线性模型预测控制（LMPC) 节点
  constexpr static const char* NMPC_NODE_NAME = "hybrid_astar_Bspline_NMPC";//使用 非线性模型预测控制(NMPC) 节点
  constexpr static const char* CBF_NODE_NAME = "astar_NMPC_CBF";//结合 A*算法、非线性MPC和弘治屏蔽函数(CBF) 的安全控制节点
  constexpr static const char* TASK_SRV = "inspection_task";//用于触发或管理机器人巡检人物的ROS服务名称
  // constexpr static const char* RVIZ_POINT = "/clicked_point";
  // constexpr static const char* COSTMAP = "/move_base_flex/global_costmap/costmap";
  // constexpr static const char* COSTMAP_UPDATE = "/move_base_flex/global_costmap/costmap_updates";
  constexpr static const double DEG2RAD = M_PI / 180;//角度转换成弧度的常量
  // constexpr static const double RECORD_PATH_LEN_DENS = 0.3;//0.3m，记录路径时两个记录点离得过近时的限度，每0.3m一个记录点
  // constexpr static const double PATH_SAFE_DIS_NUM = 2.7 / RECORD_PATH_LEN_DENS; // 2.7m 若障碍物未消失，前向路径索引的导航点位置
  // constexpr static const int WAIT_COUNT = 5 * 10; // 5s 等待前方障碍物消失时间，为什么5s？因为大循环0.1s一次心跳
  // namespace ob = ompl::base;
  // namespace og = ompl::geometric;
  // typedef ompl::base::SE2StateSpace::StateType State;
  using namespace planning;    // 引入 planning 命名空间
  using namespace Eigen;       // 引入 Eigen 命名空间（线性代数库）
  using namespace std;         // 引入标准库（如 vector, cout）
  using namespace fast_planner;// 引入 fast_planner 命名空间
  // 进行点到点的任务，规划加控制
  // using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
  // //砍掉规划，执行示教路径，让控制器沿着示教路径走
  // using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
  
  /*
    enum class用于定义一个强类型枚举
    StateValue 枚举类的名称，之后可使用该名称引用该枚举类型
    uint8_t 数据类型
  */
  
  //状态机设计：使用StateValue枚举管理系统的不同工作状态（空闲、录制、运行等）
  enum class StateValue : uint8_t {
      Idle = 0,//????
      Record,//记录示教路径
      
      Record_Traffic_route,//记录路网
      
      Run,
      Pause
  };

  //运行态Run再分为小状态
  enum class RunStateValue : uint8_t {
    Goto = 0,//点到点任务，到达示教路径起点或中途回到示教路径点
  
    Follow,//跟随示教路径

    Wait//等待障碍物能否自行消失

  };


  class AstarNMPCCBF {
    public:
      AstarNMPCCBF();             //构造函数声明
      ~AstarNMPCCBF() = default;  //显式默认化析构函数
      void init();
      void run();
      // void stop();
    
    //其后声明的所有成员只能在类的内部被访问
    private:
      // void Gridmap_Callback(grid_map_msgs::GridMap msg);
      // void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
      // void goalCallback(const geometry_msgs::PoseStamped msg);
      // void goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);
      void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
      /*
        nav_msgs::OccupancyGridConstPtr 一个类型名   Const表示常量 Ptr是指针的缩写
        & 引用符号，表明msg是一个引用类型
        msg 参数的名称 函数内部可使用这个名称来访问传入的OccupancyGrid消息
      */
      // void point_cb(geometry_msgs::PointStampedConstPtr const& msg);
      auto read_traffic_route(std::string& file_path, lines_type& lines) -> bool;
      /*
        auto 编译器根据return语句来明确函数的返回类型
        -> bool 表明函数的返回类型是bool（布尔类型）
      */
      auto check_file_exist(std::string& file_path) -> bool;
      auto write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool;
      auto task_service(hybrid_astar_dubins_opt_mpc::chibot_task::Request& req, hybrid_astar_dubins_opt_mpc::chibot_task::Response& resp) -> bool;
      auto read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool;
      // void costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const& msg);
      // void costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const& msg);
      //获取小车在地图坐标系中的位姿
      auto robot_pose() -> std::optional<geometry_msgs::Pose>;//返回机器人的三维位姿
      // auto send_exe(size_t const& index) -> bool;
      // void exe_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::ExePathResultConstPtr& result);
      void running_state();//Run态的状态机
      // void pub_zero_vel();
      // void cancel_goal();//取消下一个导航点
      // void reset();
      // auto is_free(const geometry_msgs::PoseStamped& pose) const -> bool;
      // auto is_danger(const geometry_msgs::PoseStamped& pose) const -> bool;
      // auto pose_cost(const geometry_msgs::PoseStamped& pose) const -> uint8_t;
      // auto nearest_info(nav_msgs::Path const& path,
      //                   size_t const& index,
      //                   int lb = std::numeric_limits<int>::max(),
      //                   int rb = std::numeric_limits<int>::max(),
      //                   bool following = false) -> std::pair<size_t, double>;
      // inline auto distance(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) -> std::pair<double, double>; 
      inline auto distance(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) -> double; 
      /*
        inline 是 C++ 中的一个关键字，用于向编译器提出建议，表明该函数可以被内联展开。
        当函数被内联展开时，编译器会在调用该函数的地方直接插入函数体的代码，而非进行常规的函数调用（如保存现场、跳转等操作）。
        这样做的好处是可以减少函数调用的开销，提高程序的执行效率，但可能会增加代码的体积。
        不过，inline 只是一个建议，编译器有权决定是否真正进行内联展开。
      */
      // auto send_goto(size_t const& index) -> bool;
      // void goto_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::MoveBaseResultConstPtr& result);
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
      // void publish_control_cmd(const ros::TimerEvent &e);
      void publish_control_cmd();
      void local_control_cmd();
      // void MPC_calculate(double &t_cur);
      // void globalPathPub_Callback();
      void globalPathPub_Callback(const ros::TimerEvent &e);

      void change2dcbf_cb(const std_msgs::Bool::ConstPtr& msg);
      void change2mpc_cb(const std_msgs::Bool::ConstPtr& msg);
      // void change2mpc();
      void smooth_yaw(Eigen::MatrixXd& ref_traj, Eigen::Vector3d current_state);
      void obs_trajs_cb(const dynamic_simulator::DynTrajList& msg);  
      void local_gird_map();   
      void find_gezi(const int hang, const int counts, std::vector<int8_t> a, int floor, set<int>& s);

    private:
      std::shared_ptr<HalfStructPlanner> half_struct_planner_;
      std::shared_ptr<AstarPathFinder> astar_path_finder;
      std::shared_ptr<DynamicHybridAstar> hybrid_astar_path_finder;
      
      ros::Subscriber map_sub_;
      ros::Subscriber start_sub;
      ros::Subscriber goal_sub;
      ros::Subscriber point_sub_;
      ros::ServiceServer task_srv_;
      // ros::Publisher vel_cmd_pub;
      ros::Publisher local_plan_pub_;
      // ros::Subscriber costmap_sub_;
      // ros::Subscriber costmap_update_sub_;
      ros::Publisher cur_pose_pub_;
      ros::Publisher action_goal_pub_;
      ros::Subscriber goal_sub_;

      ros::Publisher mpc_head_pub;
          
      /*---------optimize related-------------*/
      ros::Publisher  optimized_traj_pub;

      nav_msgs::Path path;
      geometry_msgs::PoseStamped pose;


      // Eigen::Vector3d start, goal;
      bool has_start, has_goal;

      // Vec3d start_pose, goal_pose;
      geometry_msgs::PoseStamped start, goal;

      std::vector<Vec3d> start_pose_set;
      std::vector<Vec3d> goal_pose_set;

      nav_msgs::OccupancyGrid grid_{};
      std::atomic_bool grid_ready_;

      boost::mutex map_mutex_;

      StateValue cur_state_;
      //std::vector<geometry_msgs::PointStamped> record_points_;//录制路点
      size_t route_num;//map文件的编号 inspection_traffic_route_x

      std::string map_path_;
      std::string file_path_;
      Visualize vis;

      // Vec3d goal_pose;
      // Vec3d start_pose;
        lines_type lines;
      std::atomic_bool line_ready_, path_ready_;
      vector<vector<Eigen::Vector3d>> paths;
      vector<vector<Eigen::Vector3d>> paths_;
      vector<double> distance_table;
      // vector<double> distance_table_topology;

      std::unique_ptr<tf2_ros::Buffer> tf_;
      std::unique_ptr<tf2_ros::TransformListener> tfl_;
      // std::unique_ptr<GotoCtrl> goto_ctrl_;
      // std::unique_ptr<ExeCtrl> exe_ctrl_;

      std::mutex map_update_mutex_;
      // std::shared_ptr<costmap_2d::Costmap2D> costmap_;
      // static uint8_t* cost_translation_;

      RunStateValue running_state_;

      // nav_msgs::Path cur_route_;
      // std::list<nav_msgs::Path> routes_;
      // size_t cur_index_;
      // size_t obs_index_;

      // UniformBspline B_spline_trajectory;
      unique_ptr<Mpc> mpc_ptr;

      ros::Publisher predict_path_pub;
      // ros::Publisher motion_path_pub;
      ros::Timer control_cmd_pub;
      ros::Timer global_path_timer;

      ros::Publisher trajectory_pub;

      bool receive_traj_;
      double traj_duration_;
      ros::Time start_time_;
      vector<NonUniformBspline> traj_;
      geometry_msgs::Twist cmd;

      bool is_target_rcv_;

      ros::Publisher global_path_pub;
      ros::Subscriber change2dcbf_sub;
      ros::Subscriber change2mpc_sub;
      // ros::Publisher change2mpc_pub;
      Eigen::Vector3d start_pos, target_pos;
      geometry_msgs::Pose target_pose;
      
      nav_msgs::Path b_spline_trajectory;
      std::mutex change_state_mutex_;

      nav_msgs::Path control_path_global_;
      ros::Subscriber obs_trajs_sub;
      ros::Publisher local_grid_pub;
      ros::Rate rate;

      vector<double> obs_R_;  //外接圆半径
      vector<Eigen::Vector2d> obs_c;//障碍物外接圆圆心
      // vector<int32_t> obs_id;//障碍物外接圆圆心

  protected:
      size_t control_index_;
      nav_msgs::Path control_path;

};
}


