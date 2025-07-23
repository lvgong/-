

#include "mpc_bridge.h"

namespace chibot::topologyplanning {
MpcBridge::~MpcBridge() {
  fusion_analysis_timer_.stop();
}

void MpcBridge::init() {
  ros::NodeHandle nh;
  fusion_analysis_pub_ = nh.advertise<fu_msg>(FusionTopic, 10);
  // odom_transform_pub_ = nh.advertise<geometry_msgs::TransformStamped>(OdomTransform, 10);
  control_sub_ = nh.subscribe(CommandTopic, 1, &MpcBridge::control_callback, this);
  // 用topic传送uk，ur
  //mpc有关参数 
  mpc_arg_sub_ = nh.subscribe("/mpc_arg", 1, &MpcBridge::mpc_callback, this);

  feedback_sub_ = nh.subscribe(FeedbackTopic, 1, &MpcBridge::feedback_callback, this, ros::TransportHints().tcpNoDelay());
  fusion_analysis_timer_ = nh.createTimer(ros::Duration(TimerDuration), &MpcBridge::timer_callback, this);
}

void MpcBridge::control_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  pub_msg_.v_k = msg->linear.x;//线速度控制量
  pub_msg_.w_k = msg->angular.z;//角速度
  // pub_msg_.lwheel_control = (msg->linear.x - 0.5 * msg->angular.z * WheelSeparation) / WheelRadius;
  // pub_msg_.rwheel_control = (msg->linear.x + 0.5 * msg->angular.z * WheelSeparation) / WheelRadius;
  
  // 控制量偏差2 = 参考控制量 - 当前控制量（不是反馈的，是控制指令）
  pub_msg_.mpc.dv_k = pub_msg_.mpc.v_r - pub_msg_.v_k;
  pub_msg_.mpc.dw_k = pub_msg_.mpc.w_r - pub_msg_.w_k;
}

void MpcBridge::mpc_callback(const mpc_arg::ConstPtr& msg) {
  pub_msg_.mpc.time = msg->time;//计算耗时
  pub_msg_.mpc.v_r = msg->v_r;//线速度参考
  pub_msg_.mpc.w_r = msg->w_r;//
  pub_msg_.mpc.x_r = msg->x_r;//
  pub_msg_.mpc.y_r = msg->y_r;//
  pub_msg_.mpc.dx = msg->dx;//偏差
  pub_msg_.mpc.dy = msg->dy;//
  pub_msg_.mpc.dtheta = msg->dtheta;//
  pub_msg_.mpc.x_k = msg->x_k;//当前状态量
  pub_msg_.mpc.y_k = msg->y_k;//
  pub_msg_.mpc.theta_k = msg->theta_k;//
  pub_msg_.mpc.theta_r = msg->theta_r;//参考航向角

}

void MpcBridge::feedback_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // static double last_lwheel_feedback = 0;
  // static double last_rwheel_feedback = 0;
  pub_msg_.v_c = msg->twist.twist.linear.x;
  pub_msg_.w_c = msg->twist.twist.angular.z;
  // pub_msg_.lwheel_feedback = (msg->twist.twist.linear.x - 0.5 * msg->twist.twist.angular.z * WheelSeparation) / WheelRadius;
  // pub_msg_.rwheel_feedback = (msg->twist.twist.linear.x + 0.5 * msg->twist.twist.angular.z * WheelSeparation) / WheelRadius;
  // pub_msg_.lwheel_acc = (pub_msg_.lwheel_feedback - last_lwheel_feedback) / TimerDuration;
  // pub_msg_.rwheel_acc = (pub_msg_.rwheel_feedback - last_rwheel_feedback) / TimerDuration;
  // last_lwheel_feedback = pub_msg_.lwheel_feedback;
  // last_rwheel_feedback = pub_msg_.rwheel_feedback;

  // 控制量偏差 = 参考控制量 - 当前控制量（反馈的，不是控制指令）
  pub_msg_.mpc.dv_c = pub_msg_.mpc.v_r - pub_msg_.v_c;
  pub_msg_.mpc.dw_c = pub_msg_.mpc.w_r - pub_msg_.w_c;



  // pub_msg_.odom_pose.x = msg->pose.pose.position.x;
  // pub_msg_.odom_pose.y = msg->pose.pose.position.y;
  // pub_msg_.odom_pose.yaw = tf2::getYaw(msg->pose.pose.orientation);
  // ot_msg_.header = msg->header;
  // ot_msg_.child_frame_id = "base_link";
  // ot_msg_.transform.translation.x = msg->pose.pose.position.x;
  // ot_msg_.transform.translation.y = msg->pose.pose.position.y;
  // ot_msg_.transform.translation.z = msg->pose.pose.position.z;
  // ot_msg_.transform.rotation = msg->pose.pose.orientation;
  // odom_transform_pub_.publish(ot_msg_);
}

void MpcBridge::timer_callback(const ros::TimerEvent& e) {
  fusion_analysis_pub_.publish(pub_msg_);
}
}
