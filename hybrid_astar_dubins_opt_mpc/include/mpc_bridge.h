
// #pragma once 是 C++ 中的一种预处理指令，用于确保头文件只被编译一次。当编译器遇到 #pragma once 时，
// 它会检查当前源文件是否已经包含了相同的头文件，如果是，则跳过该头文件的处理，以避免重复定义和编译时间的浪费。
#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include "hybrid_astar_dubins_opt_mpc/fusion_analysis.h"
#include "hybrid_astar_dubins_opt_mpc/mpc_arg.h"


// 组合消息，时间上的同步
namespace chibot::topologyplanning {
constexpr static const char* NODE_NAME = "mpc_bridge";
constexpr static const char* CommandTopic = "/cmd_vel";
constexpr static const char* FeedbackTopic = "/odom";
constexpr static const char* FusionTopic = "/fusion_analysis";
constexpr static const char* OdomTransform = "/odom_transform";
// constexpr static const double WheelSeparation = 0.39592;//轮距
// constexpr static const double WheelRadius = 0.1;//轮子半径
constexpr static const double TimerDuration = 0.2;

class MpcBridge {
  typedef hybrid_astar_dubins_opt_mpc::fusion_analysis fu_msg;
  typedef hybrid_astar_dubins_opt_mpc::mpc_arg mpc_arg;
public:
  MpcBridge() = default;

  ~MpcBridge();

  void init();

private:
  void control_callback(const geometry_msgs::Twist::ConstPtr& msg);
  void mpc_callback(const mpc_arg::ConstPtr& msg);

  void feedback_callback(const nav_msgs::Odometry::ConstPtr& msg);

  void timer_callback(const ros::TimerEvent& e);

private:
  ros::Publisher fusion_analysis_pub_;
  ros::Publisher odom_transform_pub_;

  ros::Subscriber control_sub_;
  ros::Subscriber feedback_sub_;

  ros::Timer fusion_analysis_timer_;

  fu_msg pub_msg_;
  geometry_msgs::TransformStamped ot_msg_;
  mpc_arg mpc_par;

  ros::Subscriber mpc_arg_sub_;

};
}
