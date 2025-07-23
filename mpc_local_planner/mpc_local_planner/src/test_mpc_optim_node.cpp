/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <ros/ros.h>

#include <Eigen/Core>

#include <mpc_local_planner/controller.h>
#include <mpc_local_planner/mpc_local_planner_ros.h>
#include <mpc_local_planner/utils/publisher.h>
#include <teb_local_planner/obstacles.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>

#include <memory>

namespace mpc = mpc_local_planner;

class TestMpcOptimNode
{
 public:
    TestMpcOptimNode() = default;

    void start(ros::NodeHandle& nh);

 protected:
    void CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                                 interactive_markers::InteractiveMarkerServer* marker_server);
    void CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback);
    void CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg);
    void CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg);
    void CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg);

    teb_local_planner::ObstContainer _obstacles;
    int _no_fixed_obstacles;
    std::vector<teb_local_planner::PoseSE2> _via_points;
};


void TestMpcOptimNode::start(ros::NodeHandle& nh)
{
    std::string map_frame = "map";

    // interactive marker server for simulated dynamic obstacles
    // 创建一个交互式标记服务器（InteractiveMarkerServer），用于在地图上模拟动态障碍物。参数 "marker_obstacles" 是该交互式标记服务器的名称，它用于在 ROS 中唯一标识这个服务器
    interactive_markers::InteractiveMarkerServer marker_server("marker_obstacles");

    // configure obstacles
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(-3, 1));//位置
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(6, 2));
    _obstacles.push_back(boost::make_shared<teb_local_planner::PointObstacle>(4, 0.1));

    // Add interactive markers
    for (int i = 0; i < (int)_obstacles.size(); ++i)
    {
        // Add interactive markers for all point obstacles
        // 将每个障碍物表示为 teb_local_planner::PointObstacle 类的实例
        boost::shared_ptr<teb_local_planner::PointObstacle> pobst = boost::dynamic_pointer_cast<teb_local_planner::PointObstacle>(_obstacles[i]);
        // 转换成功（即指针 pobst 不为空）
        if (pobst)
        {
            // 调用 CreateInteractiveMarker() 函数创建一个交互式标记，用于在地图上显示该障碍物的位置。这个函数会使用障碍物的 x 和 y 坐标、障碍物的索引 i、地图坐标系的名称 map_frame，以及交互式标记服务器 marker_server
            CreateInteractiveMarker(pobst->x(), pobst->y(), i, map_frame, &marker_server);
        }
    }
    // 通过为每个障碍物创建交互式标记，可以让用户在可视化工具（如RViz）中直观地看到障碍物的位置，并进行交互操作，比如移动或删除障碍物。
    marker_server.applyChanges();//确保所有的改变都被应用
    _no_fixed_obstacles = (int)_obstacles.size();

    // setup callback for custom obstacles
    ros::Subscriber custom_obst_sub = nh.subscribe("obstacles", 1, &TestMpcOptimNode::CB_customObstacle, this);

    // setup callback for clicked points (in rviz) that are considered as via-points
    ros::Subscriber clicked_points_sub = nh.subscribe("/clicked_point", 5, &TestMpcOptimNode::CB_clicked_points, this);

    // setup callback for via-points (callback overwrites previously set via-points)
    ros::Subscriber via_points_sub = nh.subscribe("via_points", 1, &TestMpcOptimNode::CB_via_points, this);

    // Setup robot shape model
    // 从参数服务器中获取机器人的轮廓模型
    teb_local_planner::RobotFootprintModelPtr robot_model = mpc_local_planner::MpcLocalPlannerROS::getRobotFootprintFromParamServer(nh);

    mpc_local_planner::Controller controller;
    _via_points.emplace_back(3, 1, 1.57);
    _via_points.emplace_back(2, 1, 1.57);
    if (!controller.configure(nh, _obstacles, robot_model, _via_points))
    {
        ROS_ERROR("Controller configuration failed.");
        return;
    }
// controller.getRobotDynamics() 是一个函数调用，用于获取控制器对象中的机器人动力学模型
    mpc::Publisher publisher(nh, controller.getRobotDynamics(), map_frame);

    teb_local_planner::PoseSE2 x0(0, 0, 1.5707);//起始点
    teb_local_planner::PoseSE2 xf(5, 2, 0);//目标点

    corbo::TimeSeries::Ptr x_seq = std::make_shared<corbo::TimeSeries>();//状态序列
    corbo::TimeSeries::Ptr u_seq = std::make_shared<corbo::TimeSeries>();//控制序列

    geometry_msgs::Twist vel;

    bool success = false;

    ros::Rate rate(20);
    while (ros::ok())
    {
        // 配置起始点和目标点传到controller.step()函数中，并且while (ros::ok())不停循环执行step()
        success = controller.step(x0, xf, vel, rate.expectedCycleTime().toSec(), ros::Time::now(), u_seq, x_seq);

        if (success)
            publisher.publishLocalPlan(*x_seq);//发布本地轨迹
        else
            ROS_ERROR("OCP solving failed.");

        publisher.publishObstacles(_obstacles);//发布障碍物信息
        publisher.publishRobotFootprintModel(x0, *robot_model);// 发布机器人足迹模型信息
        publisher.publishViaPoints(_via_points);//发布途经点信息
        ros::spinOnce();
        rate.sleep();
    }
}

void TestMpcOptimNode::CreateInteractiveMarker(const double& init_x, const double& init_y, unsigned int id, std::string frame,
                                               interactive_markers::InteractiveMarkerServer* marker_server)
{
    // create an interactive marker for our server
    visualization_msgs::InteractiveMarker i_marker;
    i_marker.header.frame_id = frame;
    i_marker.header.stamp    = ros::Time::now();
    std::ostringstream oss;
    // oss << "obstacle" << id;
    oss << id;
    i_marker.name               = oss.str();
    i_marker.description        = "Obstacle";
    i_marker.pose.position.x    = init_x;
    i_marker.pose.position.y    = init_y;
    i_marker.pose.orientation.w = 1.0f;  // make quaternion normalized

    // create a grey box marker
    visualization_msgs::Marker box_marker;
    box_marker.type               = visualization_msgs::Marker::CUBE;
    box_marker.id                 = id;
    box_marker.scale.x            = 0.2;
    box_marker.scale.y            = 0.2;
    box_marker.scale.z            = 0.2;
    box_marker.color.r            = 0.5;
    box_marker.color.g            = 0.5;
    box_marker.color.b            = 0.5;
    box_marker.color.a            = 1.0;
    box_marker.pose.orientation.w = 1.0f;  // make quaternion normalized

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl box_control;
    box_control.always_visible = true;
    box_control.markers.push_back(box_marker);

    // add the control to the interactive marker
    i_marker.controls.push_back(box_control);

    // create a control which will move the box, rviz will insert 2 arrows
    visualization_msgs::InteractiveMarkerControl move_control;
    move_control.name             = "move_x";
    move_control.orientation.w    = 0.707107f;
    move_control.orientation.x    = 0;
    move_control.orientation.y    = 0.707107f;
    move_control.orientation.z    = 0;
    move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

    // add the control to the interactive marker
    i_marker.controls.push_back(move_control);

    // add the interactive marker to our collection
    marker_server->insert(i_marker);
    marker_server->setCallback(i_marker.name, boost::bind(&TestMpcOptimNode::CB_obstacle_marker, this, boost::placeholders::_1));
}

void TestMpcOptimNode::CB_obstacle_marker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
    std::stringstream ss(feedback->marker_name);
    unsigned int index;
    ss >> index;

    if (index >= _no_fixed_obstacles) return;
    teb_local_planner::PointObstacle* pobst = static_cast<teb_local_planner::PointObstacle*>(_obstacles.at(index).get());
    pobst->position()                       = Eigen::Vector2d(feedback->pose.position.x, feedback->pose.position.y);
}


void TestMpcOptimNode::CB_customObstacle(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
    // resize such that the vector contains only the fixed obstacles specified inside the main function
    _obstacles.resize(_no_fixed_obstacles);//调整 _obstacles 向量的大小，使其仅包含在主函数中指定的固定障碍物数量

    // Add custom obstacles obtained via message (assume that all obstacles coordiantes are specified in the default planning frame)
    for (size_t i = 0; i < obst_msg->obstacles.size(); ++i)
    {
        if (obst_msg->obstacles.at(i).polygon.points.size() == 1)
        {
            if (obst_msg->obstacles.at(i).radius == 0)//teb_local_planner::PointObstacle
            {
                _obstacles.push_back(teb_local_planner::ObstaclePtr(new teb_local_planner::PointObstacle(
                    obst_msg->obstacles.at(i).polygon.points.front().x, obst_msg->obstacles.at(i).polygon.points.front().y)));
            }
            else//teb_local_planner::CircularObstacle
            {
                _obstacles.push_back(teb_local_planner::ObstaclePtr(
                    new teb_local_planner::CircularObstacle(obst_msg->obstacles.at(i).polygon.points.front().x,
                                                            obst_msg->obstacles.at(i).polygon.points.front().y, obst_msg->obstacles.at(i).radius)));
            }
        }
        else//teb_local_planner::PolygonObstacle
        {
            teb_local_planner::PolygonObstacle* polyobst = new teb_local_planner::PolygonObstacle;
            for (size_t j = 0; j < obst_msg->obstacles.at(i).polygon.points.size(); ++j)
            {
                polyobst->pushBackVertex(obst_msg->obstacles.at(i).polygon.points[j].x, obst_msg->obstacles.at(i).polygon.points[j].y);
            }
            polyobst->finalizePolygon();
            _obstacles.push_back(teb_local_planner::ObstaclePtr(polyobst));
        }
// 设置障碍物对象的速度和方向
        if (!_obstacles.empty()) _obstacles.back()->setCentroidVelocity(obst_msg->obstacles.at(i).velocities, obst_msg->obstacles.at(i).orientation);
    }
}

void TestMpcOptimNode::CB_clicked_points(const geometry_msgs::PointStampedConstPtr& point_msg)
{
    // we assume for simplicity that the fixed frame is already the map/planning frame
    // consider clicked points as via-points
    // 将该点击点的坐标（x、y 分量）以及高度（z 分量，默认为0.0）封装成一个 teb_local_planner::ViaPoint 对象，并添加到 _via_points 向量中
    _via_points.emplace_back(point_msg->point.x, point_msg->point.y, 0.0);
    ROS_INFO_STREAM("Via-point (" << point_msg->point.x << "," << point_msg->point.y << ") added.");
}

// 将接收到的途经点消息中的坐标信息提取出来
void TestMpcOptimNode::CB_via_points(const nav_msgs::Path::ConstPtr& via_points_msg)
{
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    _via_points.clear();
    for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
    {
        _via_points.emplace_back(pose.pose.position.x, pose.pose.position.y, 0);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_optim_node");
    ros::NodeHandle n("~");

    TestMpcOptimNode mpc_test;
    mpc_test.start(n);

    return 0;
}
