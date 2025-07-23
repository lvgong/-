#ifndef _ASTART_SEARCHER_H
#define _ASTART_SEARCHER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "node.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include "AstarPlanner/cubic_spline_interpolator.h"
#include <tf/tf.h>
#include "astarPlanner/bezier_curve_path.h"
#include <mutex>
#include <map>
#include <cmath>


using namespace Eigen;
using namespace std;

// 存入8个朝向及其对应的角度（弧度制）
// static const std::map<std::pair<int, int>, double> directionAngles = {
//     {{-1,-1}, -3 * M_PI_4},
//     {{-1,0}, M_PI},
//     {{-1,1}, 3 * M_PI_4},
//     {{0,-1}, -M_PI_2},
//     {{0,1}, M_PI_2},
//     {{1,-1}, -M_PI_4},
//     {{1,0}, 0.0},
//     {{1,1}, M_PI_4}
// };

//车长
const double length_ = 0.72;
//车宽
const double width_ = 0.58;
//最小转弯半径 R，L固定，tan大，R最小
// const double min_turn_radius_ = 0.942;//设置了，但其实没用到
//最大前轮转角：tan=L/R
// const double max_steer_angle_ = 6.0; /////////////这个值选小一点可以使路径曲率小一点
// const double max_steer_angle_rate_ = 8.552;
// const int steer_ratio_ = 16;
// 前轮和后轮的距离L，我们没这个东西
// const double wheel_base_ = 0.48;
//中心是后轴中心，我们假设我们的差速车是阿克曼一样的后驱的
//中心距前边沿距离
const double front_edge_to_center_ = 0.6;  
//中心距后边沿距离
const double back_edge_to_center_ = 0.12; 
//中心距左边沿距离
const double left_edge_to_center_ = 0.29;
const double right_edge_to_center_ = 0.29;
//碰撞检测膨胀值
const double inflation_value = 0.1;

//Point2D结构体 double x,y
struct Point2D {
  double x, y;
};


class AstarPathFinder {
 private:
  ros::NodeHandle nh;
  Vector2d start_pt;
  Vector2d goal_pt;
  // Vector2d map_lower, map_upper;
  nav_msgs::OccupancyGrid map_;
  // ros related
  ros::Subscriber map_sub, start_sub, goal_sub;
  ros::Publisher path_pub, local_goal_pub, smoothedPath_pub;

  /// The start pose set through RViz
  geometry_msgs::Pose start;
  /// The goal pose set through RViz
  geometry_msgs::PoseStamped goal;
  nav_msgs::Path path;
  geometry_msgs::PoseStamped targetPoint_cur;
  geometry_msgs::Pose start_cur;
  /// Flags for allowing the planner to plan
  bool validStart = false;
  /// Flags for allowing the planner to plan
  bool validGoal = false;
  bool validMap = false;
  //bool is_path;
  std::vector<Vector2d> gridPath_;
  std::vector<Vector2d> visitedNotes_;
  //std::vector<Node2DPtr> expandedNodes;
  //visualization_msgs::MarkerArray path_in_rviz_;
  //visualization_msgs::MarkerArray visited_notes_in_rviz_;
  
  SubscribeAndPublishPath BezierOpt;
  double total_length;
  boost::mutex map_mutex_;

 protected:
  int8_t* data;            // this 1D array store binary grid information
  Node2DPtr** GridNodeMap;  // this 2D array store Node2D
  Eigen::Vector2i goalIdx;
  int X_SIZE, Y_SIZE, grid_number, max_x_id, max_y_id;
  // node2com，turn需要每次进入主处理程序时复位
  Node2DPtr node2com;  //记录上一次用于计算yaw的节点
  int turn; //每隔turn个点进行碰撞检测

  double resolution, inv_resolution;
  // double xl, yl;
  // double xu, yu;
  double originX,originY;
  Node2DPtr terminatePtr;
  std::multimap<double, Node2DPtr> openSet;
  
  double getHeu(Node2DPtr node1, Node2DPtr node2);
  void AstarGetSucc(Node2DPtr currentPtr,
                    std::vector<Node2DPtr>& neighborPtrSets,
                    std::vector<double>& edgeCostSets);

  bool isOccupied(const int& idx_x, const int& idx_y) const;
  bool isOccupied(const Eigen::Vector2i& index) const;
  bool isFree(const int& idx_x, const int& idx_y) const;
  bool isFree(const Eigen::Vector2i& index) const;

  Eigen::Vector2d gridIndex2coord(const Eigen::Vector2i& index);
  Eigen::Vector2i coord2gridIndex(const Eigen::Vector2d& pt);
  // auto deal_path(const std::vector<Eigen::Vector2d>& input, int distance) -> std::vector<Eigen::Vector2d>;
  void find_gezi(const int hang,const int counts,int8_t* a,int floor,set<int>& s);
  // void find_gezi(const int hang, const int counts, std::vector<int8_t> a, int floor, set<int>& s);
  // bool validityCheck(Node2DPtr node, Node2DPtr cunode);
  // bool validityCheck(Node2DPtr node, Node2DPtr curnode);
  bool validityCheck(Node2DPtr nenode, Node2DPtr curnode, int j, int num);
  vector<Vector2d> calculateCarBoundingBox(Eigen::Vector3d pose);
  bool isLinecollision(double x0, double y0, double x1, double y1);

 public:
  AstarPathFinder();
  ~AstarPathFinder(){};
  bool AstarGraphSearch_Collision();
  bool AstarGraphSearch();
  void resetGrid(Node2DPtr ptr);
  void resetUsedGrids();

  void initGridMap();
  //void setObs(const double coord_x, const double coord_y);
  Eigen::Vector2d coordRounding(const Eigen::Vector2d& coord);
  std::vector<Eigen::Vector2d> getPath();
  std::vector<Eigen::Vector2d> getVisitedNodes();

  void setMapCallBack(const nav_msgs::OccupancyGrid::Ptr map);
  void setStartCallback(
      const geometry_msgs::Pose::ConstPtr& start);
  void setGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& end);
  void makePlan();
  auto makePlanandBezierOpt(Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) -> nav_msgs::Path;
  auto makePlanandBezierOpt_Collision(Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) -> nav_msgs::Path;
  
  void visGridPath();
  //void visGridSmoothPath();
  void publocal();
  //void visVisitedNode();
  void deleteVariablesData();
};

#endif