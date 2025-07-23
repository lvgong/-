#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetMap.h>//nav_msgs/GetMap是map_server提供的服务的消息类型。
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <fstream>
#include <chrono>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <mutex>

#include "hybrid_astar_searcher/visualize.h"
#include "hybrid_astar_dubins_opt_mpc/chibot_task.h"
#include "half_struct_planner.h"
#include "mbf_msgs/MoveBaseAction.h"
#include "mbf_msgs/ExePathAction.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/costmap_2d_ros.h"


namespace chibot::topologyplanning {
    constexpr static const char* NODE_NAME = "hybrid_astar_dubins_opt";
    constexpr static const char* MPC_NODE_NAME = "hybrid_astar_mpc";
    constexpr static const char* TASK_SRV = "inspection_task";
    constexpr static const char* RVIZ_POINT = "/clicked_point";
    constexpr static const char* COSTMAP = "/move_base_flex/global_costmap/costmap";
    constexpr static const char* COSTMAP_UPDATE = "/move_base_flex/global_costmap/costmap_updates";
    constexpr static const double DEG2RAD = M_PI / 180;
    constexpr static const double RECORD_PATH_LEN_DENS = 0.3;//0.3m，记录路径时两个记录点离得过近时的限度，每0.3m一个记录点
    constexpr static const double PATH_SAFE_DIS_NUM = 2.7 / RECORD_PATH_LEN_DENS; // 2.7m 若障碍物未消失，前向路径索引的导航点位置
    constexpr static const int WAIT_COUNT = 5 * 10; // 5s 等待前方障碍物消失时间，为什么5s？因为大循环0.1s一次心跳
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    typedef ompl::base::SE2StateSpace::StateType State;
    using namespace planning;
    using namespace std;
    // 进行点到点的任务，规划加控制
    using GotoCtrl = actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>;
    //砍掉规划，执行示教路径，让控制器沿着示教路径走
    using ExeCtrl = actionlib::SimpleActionClient<mbf_msgs::ExePathAction>;
   
    enum class StateValue : uint8_t {
        Idle = 0,
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

class HybridAstarMPC {
    public:
        HybridAstarMPC();
        ~HybridAstarMPC() = default;
        void init();
        void run();
        void stop();

    private:
        // void Gridmap_Callback(grid_map_msgs::GridMap msg);
        void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg);
        void goalCallback(const geometry_msgs::PoseStamped msg);
        void goalCallback(geometry_msgs::PoseStamped::ConstPtr const& msg);
        void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
        // void point_cb(geometry_msgs::PointStampedConstPtr const& msg);
        auto read_traffic_route(std::string& file_path, lines_type& lines) -> bool;
        auto check_file_exist(std::string& file_path) -> bool;
        auto write_traffic_route(std::string& file_path, std::vector<Vec3d> const& line_start, std::vector<Vec3d> const& line_goal) -> bool;
        auto task_service(hybrid_astar_dubins_opt_mpc::chibot_task::Request& req, hybrid_astar_dubins_opt_mpc::chibot_task::Response& resp) -> bool;
        auto read_file(std::string& file_path, vector<vector<Eigen::Vector3d>>& routes) -> bool;
      void costmap_cb(nav_msgs::OccupancyGrid::ConstPtr const& msg);
      void costmap_update_cb(map_msgs::OccupancyGridUpdate::ConstPtr const& msg);
      //获取小车在地图坐标系中的位姿
      auto robot_pose() -> std::optional<geometry_msgs::Pose>;     
      auto send_exe(size_t const& index) -> bool;
      void exe_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::ExePathResultConstPtr& result);
      void running_state();//Run态的状态机
      void pub_zero_vel();
      void cancel_goal();//取消下一个导航点
      void reset();
      auto is_free(const geometry_msgs::PoseStamped& pose) const -> bool;
      auto is_danger(const geometry_msgs::PoseStamped& pose) const -> bool;
      auto pose_cost(const geometry_msgs::PoseStamped& pose) const -> uint8_t;
      auto nearest_info(nav_msgs::Path const& path,
                        size_t const& index,
                        int lb = std::numeric_limits<int>::max(),
                        int rb = std::numeric_limits<int>::max(),
                        bool following = false) -> std::pair<size_t, double>;
      inline auto distance(geometry_msgs::Pose const& a, geometry_msgs::Pose const& b) -> std::pair<double, double>; 
      auto send_goto(size_t const& index) -> bool;
      void goto_done(const actionlib::SimpleClientGoalState& state, const mbf_msgs::MoveBaseResultConstPtr& result);
      void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

    private:
        std::shared_ptr<HalfStructPlanner> half_struct_planner_;
        
        ros::Subscriber map_sub_;
        ros::Subscriber start_sub;
        ros::Subscriber goal_sub;
        ros::Subscriber point_sub_;
        ros::ServiceServer task_srv_;
        ros::Publisher vel_pub_;
        ros::Subscriber costmap_sub_;
        ros::Subscriber costmap_update_sub_;
        ros::Publisher cur_pose_pub_;
        ros::Publisher action_goal_pub_;
        ros::Subscriber goal_sub_;
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
        std::vector<geometry_msgs::PointStamped> record_points_;//录制路点
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
        std::unique_ptr<GotoCtrl> goto_ctrl_;
        std::unique_ptr<ExeCtrl> exe_ctrl_;

        std::mutex map_update_mutex_;
        std::shared_ptr<costmap_2d::Costmap2D> costmap_;
        static uint8_t* cost_translation_;

        RunStateValue running_state_;

        nav_msgs::Path cur_route_;
        std::list<nav_msgs::Path> routes_;
        size_t cur_index_;
        size_t obs_index_;
};
}


