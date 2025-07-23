 
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>


/// A listener that awaits transforms
//tf::TransformListener listener; //监听TF变换的变换树
/// A transform for moving start positions
//tf::StampedTransform transform;//用于改变始点的变换
int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "my_odom_publisher");

    // 创建节点句柄
    ros::NodeHandle n;

    tf::TransformListener listener;//把这行放到初始化函数的外面会报上面的错。

    //tf::StampedTransform transform;
    // 创建一个Publisher，发布名为/my_odom的topic，消息类型为geometry_msgs::PoseStamped，队列长度1
    ros::Publisher my_odom_pub = n.advertise<geometry_msgs::Pose>("/my_odom", 1);

    // 设置循环的频率
    ros::Rate loop_rate(10.0);
    while (ros::ok())
    {
        tf::StampedTransform transform;
        try 
        {
	      ros::Time now = ros::Time(0);
	      listener.waitForTransform("/map", "/base_footprint",now, ros::Duration(3.0));
              listener.lookupTransform("/map","/base_footprint", now, transform);
        }
        catch (tf::TransformException &ex) 
        {
             ROS_ERROR("%s",ex.what());
             ros::Duration(1.0).sleep();
             continue;
        }
        geometry_msgs::Pose current_pose_ros;
        // 初始化geometry_msgs::TransformStamped类型的消息
        geometry_msgs::TransformStamped transform_pose;
        tf::transformStampedTFToMsg(transform, transform_pose);
        current_pose_ros.position.x = transform.getOrigin().x();
        current_pose_ros.position.y = transform.getOrigin().y();
        current_pose_ros.position.z = 0.0;
        current_pose_ros.orientation=transform_pose.transform.rotation;
       
        // 发布消息
        my_odom_pub.publish(current_pose_ros);
	       	
        // 按照循环频率延时
        loop_rate.sleep();
    }

    return 0;
}
