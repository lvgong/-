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

#include <mpc_local_planner/mpc_local_planner_ros.h>

#include <mpc_local_planner/utils/math_utils.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/algorithm/string.hpp>

// MBF return codes
#include <mbf_msgs/ExePathResult.h>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

// register this planner both as a BaseLocalPlanner and as a MBF's CostmapController plugin
PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlannerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(mpc_local_planner::MpcLocalPlannerROS, mbf_costmap_core::CostmapController)

namespace mpc_local_planner {

MpcLocalPlannerROS::MpcLocalPlannerROS()
    : _costmap_ros(nullptr),
      _tf(nullptr),
      _costmap_model(nullptr),
      _costmap_converter_loader("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
      /*dynamic_recfg_(NULL),*/
      _goal_reached(false),
      _no_infeasible_plans(0),
      /*last_preferred_rotdir_(RotType::none),*/
      _initialized(false)
{
}

MpcLocalPlannerROS::~MpcLocalPlannerROS() {}

/*
void MpcLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig& config, uint32_t level)
{
  cfg_.reconfigure(config);
}
*/

void MpcLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
    // check if the plugin is already initialized
    if (!_initialized)
    {
        // create Node Handle with name of plugin (as used in move_base for loading)
        ros::NodeHandle nh("~/" + name);

        // load plugin related main parameters
        nh.param("controller/xy_goal_tolerance", _params.xy_goal_tolerance, _params.xy_goal_tolerance);
        nh.param("controller/yaw_goal_tolerance", _params.yaw_goal_tolerance, _params.yaw_goal_tolerance);
        nh.param("controller/global_plan_overwrite_orientation", _params.global_plan_overwrite_orientation,
                 _params.global_plan_overwrite_orientation);
        nh.param("controller/global_plan_prune_distance", _params.global_plan_prune_distance, _params.global_plan_prune_distance);
        nh.param("controller/max_global_plan_lookahead_dist", _params.max_global_plan_lookahead_dist, _params.max_global_plan_lookahead_dist);
        nh.param("controller/global_plan_viapoint_sep", _params.global_plan_viapoint_sep, _params.global_plan_viapoint_sep);
        _controller.setInitialPlanEstimateOrientation(_params.global_plan_overwrite_orientation);

        // special parameters
        nh.param("odom_topic", _params.odom_topic, _params.odom_topic);
        nh.param("footprint_model/is_footprint_dynamic", _params.is_footprint_dynamic, _params.is_footprint_dynamic);
        nh.param("collision_avoidance/include_costmap_obstacles", _params.include_costmap_obstacles, _params.include_costmap_obstacles);
        nh.param("collision_avoidance/costmap_obstacles_behind_robot_dist", _params.costmap_obstacles_behind_robot_dist,
                 _params.costmap_obstacles_behind_robot_dist);

        nh.param("collision_avoidance/collision_check_no_poses", _params.collision_check_no_poses, _params.collision_check_no_poses);
        nh.param("collision_avoidance/collision_check_min_resolution_angular", _params.collision_check_min_resolution_angular,
                 _params.collision_check_min_resolution_angular);

        // costmap converter plugin related parameters
        nh.param("costmap_converter_plugin", _costmap_conv_params.costmap_converter_plugin, _costmap_conv_params.costmap_converter_plugin);
        nh.param("costmap_converter_rate", _costmap_conv_params.costmap_converter_rate, _costmap_conv_params.costmap_converter_rate);
        nh.param("costmap_converter_spin_thread", _costmap_conv_params.costmap_converter_spin_thread,
                 _costmap_conv_params.costmap_converter_spin_thread);

        // reserve some memory for obstacles
        _obstacles.reserve(700);

        // init other variables
        _tf          = tf;
        _costmap_ros = costmap_ros;
        _costmap     = _costmap_ros->getCostmap();  // locking should be done in MoveBase.

        _costmap_model = std::make_shared<base_local_planner::CostmapModel>(*_costmap);

        _global_frame     = _costmap_ros->getGlobalFrameID();
        _robot_base_frame = _costmap_ros->getBaseFrameID();

        // create robot footprint/contour model for optimization
        _robot_model = getRobotFootprintFromParamServer(nh, _costmap_ros);

        // create the planner instance
        if (!_controller.configure(nh, _obstacles, _robot_model, _via_points))///一些配置
        {
            ROS_ERROR("Controller configuration failed.");
            return;
        }

        // create visualization instance
        _publisher.initialize(nh, _controller.getRobotDynamics(), _global_frame);

        // Initialize a costmap to polygon converter
        if (!_costmap_conv_params.costmap_converter_plugin.empty())
        {
            try
            {
                _costmap_converter         = _costmap_converter_loader.createInstance(_costmap_conv_params.costmap_converter_plugin);
                std::string converter_name = _costmap_converter_loader.getName(_costmap_conv_params.costmap_converter_plugin);
                // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
                boost::replace_all(converter_name, "::", "/");
                _costmap_converter->setOdomTopic(_params.odom_topic);
                _costmap_converter->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
                _costmap_converter->setCostmap2D(_costmap);

                _costmap_converter->startWorker(ros::Rate(_costmap_conv_params.costmap_converter_rate), _costmap,
                                                _costmap_conv_params.costmap_converter_spin_thread);
                ROS_INFO_STREAM("Costmap conversion plugin " << _costmap_conv_params.costmap_converter_plugin << " loaded.");
            }
            catch (pluginlib::PluginlibException& ex)
            {
                ROS_WARN(
                    "The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error "
                    "message: %s",
                    ex.what());
                _costmap_converter.reset();
            }
        }
        else
            ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

        // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        _footprint_spec = _costmap_ros->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(_footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius);

        // init the odom helper to receive the robot's velocity from odom messages
        _odom_helper.setOdomTopic(_params.odom_topic);

        // setup dynamic reconfigure
        /*
        dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<MpcLocalPlannerReconfigureConfig>>(nh);
        dynamic_reconfigure::Server<MpcLocalPlannerReconfigureConfig>::CallbackType cb =
            boost::bind(&MpcLocalPlanner::reconfigureCB, this, _1, _2);
        dynamic_recfg_->setCallback(cb);
        */

        // validate optimization footprint and costmap footprint
        validateFootprints(_robot_model->getInscribedRadius(), _robot_inscribed_radius, _controller.getInequalityConstraint()->getMinimumDistance());

        // setup callback for custom obstacles
        _custom_obst_sub = nh.subscribe("obstacles", 1, &MpcLocalPlannerROS::customObstacleCB, this);

        // setup callback for custom via-points
        _via_points_sub = nh.subscribe("via_points", 1, &MpcLocalPlannerROS::customViaPointsCB, this);

        // additional move base params
        ros::NodeHandle nh_move_base("~");
        nh_move_base.param("controller_frequency", _params.controller_frequency, _params.controller_frequency);

        // set initialized flag
        _initialized = true;

        ROS_DEBUG("mpc_local_planner plugin initialized.");
    }
    else
    {
        ROS_WARN("mpc_local_planner has already been initialized, doing nothing.");
    }
}


bool MpcLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
    // check if plugin is initialized
    if (!_initialized)
    {
        ROS_ERROR("mpc_local_planner has not been initialized, please call initialize() before using this planner");
        return false;
    }

    // store the global plan
    _global_plan.clear();
    _global_plan = orig_global_plan;

    // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
    // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

    // reset goal_reached_ flag
    _goal_reached = false;

    return true;
}


// 调用局部路径规划器插件的接口函数
bool MpcLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    std::string dummy_message;
    geometry_msgs::PoseStamped dummy_pose;
    geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
    uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
    cmd_vel          = cmd_vel_stamped.twist;
    return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

uint32_t MpcLocalPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity,
                                                     geometry_msgs::TwistStamped& cmd_vel, std::string& message)
{
    // check if plugin initialized
    // 检查一下initialize函数是否已经执行，初始化速度和姿态变量，并获取当前的速度和姿态；
/*（1）该函数中首先检查了该路径规划器是否初始化、并定义了一些变量等，然后读取了机器人当前的位姿和速度信息。*/
    if (!_initialized)
    {
        ROS_ERROR("mpc_local_planner has not been initialized, please call initialize() before using this planner");
        message = "mpc_local_planner has not been initialized";
        return mbf_msgs::ExePathResult::NOT_INITIALIZED;
    }

    static uint32_t seq     = 0;
    cmd_vel.header.seq      = seq++;
    cmd_vel.header.stamp    = ros::Time::now();
    cmd_vel.header.frame_id = _robot_base_frame;
    cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;//归零
    _goal_reached                                                             = false;

    // Get robot pose
    // 读取了机器人当前的位姿和速度信息
    geometry_msgs::PoseStamped robot_pose;
    _costmap_ros->getRobotPose(robot_pose);
    _robot_pose = PoseSE2(robot_pose.pose);

    // Get robot velocity
    geometry_msgs::PoseStamped robot_vel_tf;
    _odom_helper.getRobotVel(robot_vel_tf);
    _robot_vel.linear.x  = robot_vel_tf.pose.position.x;
    _robot_vel.linear.y  = robot_vel_tf.pose.position.y;
    _robot_vel.angular.z = tf2::getYaw(robot_vel_tf.pose.orientation);

    // prune global plan to cut off parts of the past (spatially before the robot)
    // 截取全局路径：裁切掉机器人后方的一部分路径
    /*（2）调用pruneGlobalPlan函数，对全局路径进行修剪*/
    /*由于机器人是实时运动的，所以需要对已经走过的全局路径裁减处理，
    其实现思路是从全局路径规划容器的起点开始一个个遍历，计算其与机器人当前点的距离差，
    直到找到距离小于我们设定的阈值的点，将该点之前的全局路径点全部删除，
    这样就可以将位于机器人后方距离超过一定阈值的点删除掉。
    该过程涉及到参数配置文件中的参数 global_plan_prune_distance*/
    pruneGlobalPlan(*_tf, robot_pose, _global_plan, _params.global_plan_prune_distance);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    // TF转化：将初始的全局路径转换到局部坐标系下，便于后续进行局部规划，挺重要，主要是根据参数规划长度，输出了当前目标点索引。
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    geometry_msgs::TransformStamped tf_plan_to_global;
    /*（3）将全局路径转换到局部代价地图的坐标系下，并从中选择合适的点作为局部路径点*/
/* 调用transformGlobalPlan函数，通过tf树将全局路径信息转换到局部代价地图的坐标系下，
值得注意的是，在这个函数中并不仅仅进行了坐标系的变换，还对全局路径信息点进行了一些处理，从中选出合适的点作为局部路径规划点(一定范围内的点)。 
其具体过程是依次对全局路径进行坐标变换，同时计算其与当前位置的距离，并统计已转换的全局路径点之间的距离和（即已转换的全局路径的长度），直到超过设定的长度阈值或者与当前点的距离大于设定的距离阈值，则丢弃剩余的路径点。
 其中，长度阈值由配置文件中参数max_global_plan_lookahead_dist 决定，距离阈值由局部代价地图的大小width和height以及系数0.85有关*/
    if (!transformGlobalPlan(*_tf, _global_plan, robot_pose, *_costmap, _global_frame, _params.max_global_plan_lookahead_dist, transformed_plan,
                             &goal_idx, &tf_plan_to_global))
    {
        ROS_WARN("Could not transform the global plan to the frame of the controller");
        message = "Could not transform the global plan to the frame of the controller";
        return mbf_msgs::ExePathResult::INTERNAL_ERROR;
    }

    // update via-points container
    // 更新路径上的航迹点：主要是根据最小间隔的参数，从转化过来的规划上选取等间隔的控制点。
    /* （4）对得到的局部路径规划点进行进一步的筛选*/
/*调用updateViaPointsContainer函数，对上一步得到的局部路径规划点进行进一步的筛选，其实现思路是，循环遍历局部路径规划点，从中选出距离上一个被选中的点的距离大于设定的阈值的点（局部路径规划点中的第一个点为本步筛选中第一个被选中的点）*/
/*这样处理可以将上一步得到的局部路径点稀疏化，从而减少局部路径点的个数，阈值由配置文件中的参数global_plan_viapoint_sep 决定，若该参数设置为负数，则表示跳过本步处理，不进行筛选*/
    if (!_custom_via_points_active) updateViaPointsContainer(transformed_plan, _params.global_plan_viapoint_sep);

    // check if global goal is reached
    // 检查全局目标是否已经到达
/*（5）判断机器人是否到达目标点
   得到局部路径的状态点后，先检查是否已经到达了全局目标点，具体要检查以下内容：
   （1）当前点距离目标点的距离是否小于由参数配置文件设定的阈值xy_goal_tolerance
   （2）当前点的姿态角与目标姿态角是否小于由外参数配置文件设定的阈值yaw_goal_tolerance*/
    geometry_msgs::PoseStamped global_goal;
    tf2::doTransform(_global_plan.back(), global_goal, tf_plan_to_global);
    double dx           = global_goal.pose.position.x - _robot_pose.x();
    double dy           = global_goal.pose.position.y - _robot_pose.y();
    double delta_orient = g2o::normalize_theta(tf2::getYaw(global_goal.pose.orientation) - _robot_pose.theta());
    if (std::abs(std::sqrt(dx * dx + dy * dy)) < _params.xy_goal_tolerance && std::abs(delta_orient) < _params.yaw_goal_tolerance)
    {
        _goal_reached = true;
        return mbf_msgs::ExePathResult::SUCCESS;
    }

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
        ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
        message = "Transformed plan is empty";
        return mbf_msgs::ExePathResult::INVALID_PATH;
    }

    // Get current goal point (last point of the transformed plan)
    // 获取当前的目标点
    _robot_goal.x() = transformed_plan.back().pose.position.x;
    _robot_goal.y() = transformed_plan.back().pose.position.y;
    // Overwrite goal orientation if needed
    // 改变目标点的航向，根据参数global_plan_overwrite_orientation判断
/*（6）调整局部目标点的角度
   根据外部参数 global_plan_overwrite_orientation 决定是否调整局部目标点的角度（也就是经过上述处理后的路径的最后一个点的角度），调整是通过调用estimateLocalGoalOrientation函数来实现的*/
    if (_params.global_plan_overwrite_orientation)
    {
/*当需要修改时，会判断当前局部路径规划的终点与全局路径规划的终点是否接近，如果两个点比较接近(就是同一个点或者下一个点就是终点)
则直接使用当前局部路径规划中终点的方向作为规划的方向。如果当前局部路径规划的终点跟全局目标点之间还间隔很多个点的话，会使用当前局部路径的终点以及向后两个点的角度进行平滑处理，得到一个新的角度值。*/
        _robot_goal.theta() = estimateLocalGoalOrientation(_global_plan, transformed_plan.back(), goal_idx, tf_plan_to_global);
        // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
        tf2::Quaternion q;
        q.setRPY(0, 0, _robot_goal.theta());
        tf2::convert(q, transformed_plan.back().pose.orientation);
    }
    else
    {
        // 当不需要修改时，采用局部目标点的原有角度
        _robot_goal.theta() = tf2::getYaw(transformed_plan.back().pose.orientation);
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    // 用机器人的实际位置覆盖转换后的计划开始点
/* （7）更新局部路径的起点
   上一步完成了局部规划的目标点的确认，这一步要完成局部规划的起始点的确认，前面虽然根据全局路径规划中截取了局部路径规划的点，但是这个容器的起点不一定是是机器人当前时刻所处的点。*/
    if (transformed_plan.size() == 1)  // plan only contains the goal
    {
/*如果路径中只包含一个目标点，即没有中间的路径点，程序会在路径的起始点处插入机器人当前的姿态。这是为了将路径用作初始化轨迹，以确保路径规划从机器人当前位置开始。如果路径包含多个点，则使用机器人当前位姿点覆盖更新容器中的第一个路径点。*/
        transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped());  // insert start (not yet initialized)
    }
    transformed_plan.front() = robot_pose;  // update start

    // clear currently existing obstacles
    _obstacles.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    // 更新障碍物，选择是否使用costmap_converter插件转换障碍物信息
/*（8）更新障碍物信息
   根据配置文件中参数 costmap_converter决定如何更新障碍物信息，若该参数不为空，则调用costmap_converter插件进行地图信息更新，若为空直接使用costmap的信息进行更新。*/
    if (_costmap_converter)
/*若使用costmap_converter插件进行更新，则在updateObstacleContainerWithCostmapConverter函数中调用了getObstacles()函数从代价地图转换器获得障碍物，
并对障碍物根据其形状（圆、点、线、多边形）进行分类，将其转换成相应的障碍物类型，然后将障碍物坐标存储到obstacles_容器中，针对移动的障碍物还会设定其速度。*/
        updateObstacleContainerWithCostmapConverter();
    else
/*若使用costmap的信息进行更新，根据外部参数 include_costmap_obstacles 决定是否考虑costmap中的障碍物，若考虑则遍历整个代价地图，
对于每一个代价地图单元格，如果该单元格的代价为LETHAL_OBSTACLE（代价值为100），则将其视为障碍物，并检测该障碍物是否足够引人注意(例如，离机器人不远)，
如果障碍物在机器人后面且距离机器人的距离超过了外部参数决定的阈值 costmap_obstacles_behind_robot_dist，则忽略该障碍物。最后将得到的障碍物添加到障碍物容器中。*/
        updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    // 除此之外，还要从ROS消息中获取障碍物信息，然后将这些障碍物添加到当前的障碍物容器中。该过程通过函数updateObstacleContainerWithCustomObstacles完成
    // 还要考虑custom障碍(必须在其他更新之后调用，因为障碍容器没有被清除)
    /*消息来源于：
    custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);*/
    updateObstacleContainerWithCustomObstacles();

    // estimate current state vector and previous control
    // updateControlFromTwist()

    // Do not allow config changes during the following optimization step
    // TODO(roesmann): we need something similar in case we allow dynamic reconfiguration:
    // boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

    // Now perform the actual planning
    // bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
    ros::Time t = ros::Time::now();
    // controller_frequency might not be established in case planning takes longer:
    // value dt affects e.g. control deviation bounds (acceleration limits) and we want a goot start value
    // let's test how it works with the expected frequency instead of using the actual one
    double dt = 1.0 / _params.controller_frequency;
    // double dt = time_last_cmd_.isZero() ? 0.0 : (t - time_last_cmd_).toSec();

    // set previous control value for control deviation bounds
/* （9）使用 _u_seq 中的先前控制输入作为优化控制问题的输入
   若_u_seq存在且不为空，则通过_controller 对象获取了一个优化控制问题（Optimal Control Problem）的实例。
通过调用 setPreviousControlInput 方法，将控制输入设置为 _u_seq 中的第一个元素（索引为 0）并传递了一个时间步长 dt 作为参数。
这表示使用 _u_seq 中的先前控制输入作为优化控制问题的输入，以便在下一步的规划中考虑到之前的控制决策。*/
    if (_u_seq && !_u_seq->isEmpty()) _controller.getOptimalControlProblem()->setPreviousControlInput(_u_seq->getValuesMap(0), dt);

    bool success = false;

    {
        std::lock_guard<std::mutex> vp_lock(_via_point_mutex);
        std::lock_guard<std::mutex> obst_lock(_custom_obst_mutex);
        // 执行路径规划，调用 _controller.step() 函数来执行实际的路径规划计算。函数会接收转换后的路径、机器人的当前速度、时间间隔等作为输入，然后计算出机器人下一步的速度命令。
/*（10）进行局部路径规划
   执行路径规划，调用 _controller.step() 函数来执行实际的路径规划计算。函数会接收转换后的路径、机器人的当前速度、时间间隔等作为输入，然后计算出机器人下一步的速度命令。*/
        success = _controller.step(transformed_plan, _robot_vel, dt, t, _u_seq, _x_seq);
    }

/*（11）检查路径规划是否成功
   在路径规划完成后，接着会检查路径规划是否成功。如果路径规划失败，通常是因为没有找到可行的路径，程序会记录一个警告信息。然后，程序会重置 _controller，以便下一次的路径规划。同时，会增加计数器 _no_infeasible_plans，用于跟踪连续不可行规划的次数。如果路径规划成功，程序会继续执行后续的步骤。*/
    if (!success)
    {
        _controller.reset();  // force reinitialization for next time
        ROS_WARN("mpc_local_planner was not able to obtain a local plan for the current setting.");

        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = ros::Time::now();
        _last_cmd                  = cmd_vel.twist;
        message                    = "mpc_local_planner was not able to obtain a local plan";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }

    // Check feasibility (but within the first few states only)
/* （12）检查规划路径的可行性
   可行性检查通常包括考虑机器人的轮廓、碰撞检测以及动力学约束等方面。如果规划出的轨迹不可行，程序会记录一个警告信息，并返回相应的错误代码。同时，程序会重置 _controller，以准备下一次的规划。*/
    if (_params.is_footprint_dynamic)
    {
        // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
        _footprint_spec = _costmap_ros->getRobotFootprint();
        costmap_2d::calculateMinAndMaxDistances(_footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius);
    }
    // 检查轨迹可行性
    bool feasible = _controller.isPoseTrajectoryFeasible(_costmap_model.get(), _footprint_spec, _robot_inscribed_radius, _robot_circumscribed_radius,
                                                         _params.collision_check_min_resolution_angular, _params.collision_check_no_poses);
    if (!feasible)
    {
        cmd_vel.twist.linear.x = cmd_vel.twist.linear.y = cmd_vel.twist.angular.z = 0;

        // now we reset everything to start again with the initialization of new trajectories.
        _controller.reset();  // force reinitialization for next time
        ROS_WARN("MpcLocalPlannerROS: trajectory is not feasible. Resetting planner...");
        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = ros::Time::now();
        _last_cmd                  = cmd_vel.twist;
        message                    = "mpc_local_planner trajectory is not feasible";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }

    // Get the velocity command for this sampling interval
    // TODO(roesmann): we might also command more than just the imminent action, e.g. in a separate thread, until a new command is available
    // 从 _controller 获取机器人的速度命令，包括线性和角速度，它们将用于控制机器人的运动。速度命令的计算依赖于上面局部路径规划的结果和机器人的当前状态。
/*（13）计算控制机器人运动的速度指令
   从 _controller 获取机器人的速度命令，包括线性和角速度，它们将用于控制机器人的运动。速度命令的计算依赖于上面局部路径规划的结果和机器人的当前状态。*/
    if (!_u_seq || !_controller.getRobotDynamics()->getTwistFromControl(_u_seq->getValuesMap(0), cmd_vel.twist))
    {
        _controller.reset();
        ROS_WARN("MpcLocalPlannerROS: velocity command invalid. Resetting controller...");
        ++_no_infeasible_plans;  // increase number of infeasible solutions in a row
        _time_last_infeasible_plan = ros::Time::now();
        _last_cmd                  = cmd_vel.twist;
        message                    = "mpc_local_planner velocity command invalid";
        return mbf_msgs::ExePathResult::NO_VALID_CMD;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to early termination or soft cosntraints).
    // saturateVelocity(cmd_vel.twist.linear.x, cmd_vel.twist.linear.y, cmd_vel.twist.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
    //                 cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

    // a feasible solution should be found, reset counter
    _no_infeasible_plans = 0;

    // store last command (for recovery analysis etc.)
    // 将上一次有效的速度命令存储在 _last_cmd 中，以备后续参考。这对于分析路径规划的效果、机器人行为等非常有用。
/*（14）存储控制指令
   将上一次有效的速度命令存储在 _last_cmd 中，以备后续参考。这对于分析路径规划的效果、机器人行为等非常有用。*/
    _last_cmd      = cmd_vel.twist;
    _time_last_cmd = ros::Time::now();

    // Now visualize everything
    /*
    （15）发布可视化信息
     最后，程序会发布各种信息，用于可视化或监测路径规划的效果。这包括本地路径、障碍物信息、全局路径、途经点、机器人的轮廓模型等。可视化信息可以帮助我们了解机器人的行为和路径规划的效果，以及发现潜在的问题或改进点。*/
    _publisher.publishLocalPlan(*_x_seq);//局部路径
    _publisher.publishObstacles(_obstacles);//障碍
    _publisher.publishGlobalPlan(_global_plan);//全局路径
    _publisher.publishViaPoints(_via_points);//途径点
    _publisher.publishRobotFootprintModel(_robot_pose, *_robot_model);//小车轮廓
    return mbf_msgs::ExePathResult::SUCCESS;
}

bool MpcLocalPlannerROS::isGoalReached()
{
    if (_goal_reached)
    {
        ROS_INFO("GOAL Reached!");
        // planner_->clearPlanner();
        return true;
    }
    return false;
}

void MpcLocalPlannerROS::updateObstacleContainerWithCostmap()
{
    // Add costmap obstacles if desired
    if (_params.include_costmap_obstacles)
    {
        Eigen::Vector2d robot_orient = _robot_pose.orientationUnitVec();

        for (unsigned int i = 0; i < _costmap->getSizeInCellsX() - 1; ++i)
        {
            for (unsigned int j = 0; j < _costmap->getSizeInCellsY() - 1; ++j)
            {
                if (_costmap->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
                {
                    Eigen::Vector2d obs;
                    _costmap->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

                    // check if obstacle is interesting (e.g. not far behind the robot)
                    Eigen::Vector2d obs_dir = obs - _robot_pose.position();
                    if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > _params.costmap_obstacles_behind_robot_dist) continue;

                    _obstacles.push_back(ObstaclePtr(new PointObstacle(obs)));
                }
            }
        }
    }
}

void MpcLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
{
    if (!_costmap_converter) return;

    // Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = _costmap_converter->getObstacles();
    if (!obstacles) return;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
        const costmap_converter::ObstacleMsg* obstacle = &obstacles->obstacles.at(i);
        const geometry_msgs::Polygon* polygon          = &obstacle->polygon;

        if (polygon->points.size() == 1 && obstacle->radius > 0)  // Circle
        {
            _obstacles.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
        }
        else if (polygon->points.size() == 1)  // Point
        {
            _obstacles.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
        }
        else if (polygon->points.size() == 2)  // Line
        {
            _obstacles.push_back(
                ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y, polygon->points[1].x, polygon->points[1].y)));
        }
        else if (polygon->points.size() > 2)  // Real polygon
        {
            PolygonObstacle* polyobst = new PolygonObstacle;
            for (std::size_t j = 0; j < polygon->points.size(); ++j)
            {
                polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
            }
            polyobst->finalizePolygon();
            _obstacles.push_back(ObstaclePtr(polyobst));
        }

        // Set velocity, if obstacle is moving
        if (!_obstacles.empty()) _obstacles.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
    }
}

void MpcLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
{
    // Add custom obstacles obtained via message
    std::lock_guard<std::mutex> l(_custom_obst_mutex);

    if (!_custom_obstacle_msg.obstacles.empty())
    {
        // We only use the global header to specify the obstacle coordinate system instead of individual ones
        Eigen::Affine3d obstacle_to_map_eig;
        try
        {
            geometry_msgs::TransformStamped obstacle_to_map =
                _tf->lookupTransform(_global_frame, ros::Time(0), _custom_obstacle_msg.header.frame_id, ros::Time(0),
                                     _custom_obstacle_msg.header.frame_id, ros::Duration(0.5));
            obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            obstacle_to_map_eig.setIdentity();
        }

        for (size_t i = 0; i < _custom_obstacle_msg.obstacles.size(); ++i)
        {
            if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 1 && _custom_obstacle_msg.obstacles.at(i).radius > 0)  // circle
            {
                Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                _obstacles.push_back(
                    ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), _custom_obstacle_msg.obstacles.at(i).radius)));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 1)  // point
            {
                Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                    _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                _obstacles.push_back(ObstaclePtr(new PointObstacle((obstacle_to_map_eig * pos).head(2))));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.size() == 2)  // line
            {
                Eigen::Vector3d line_start(_custom_obstacle_msg.obstacles.at(i).polygon.points.front().x,
                                           _custom_obstacle_msg.obstacles.at(i).polygon.points.front().y,
                                           _custom_obstacle_msg.obstacles.at(i).polygon.points.front().z);
                Eigen::Vector3d line_end(_custom_obstacle_msg.obstacles.at(i).polygon.points.back().x,
                                         _custom_obstacle_msg.obstacles.at(i).polygon.points.back().y,
                                         _custom_obstacle_msg.obstacles.at(i).polygon.points.back().z);
                _obstacles.push_back(
                    ObstaclePtr(new LineObstacle((obstacle_to_map_eig * line_start).head(2), (obstacle_to_map_eig * line_end).head(2))));
            }
            else if (_custom_obstacle_msg.obstacles.at(i).polygon.points.empty())
            {
                ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
                continue;
            }
            else  // polygon
            {
                PolygonObstacle* polyobst = new PolygonObstacle;
                for (size_t j = 0; j < _custom_obstacle_msg.obstacles.at(i).polygon.points.size(); ++j)
                {
                    Eigen::Vector3d pos(_custom_obstacle_msg.obstacles.at(i).polygon.points[j].x,
                                        _custom_obstacle_msg.obstacles.at(i).polygon.points[j].y,
                                        _custom_obstacle_msg.obstacles.at(i).polygon.points[j].z);
                    polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
                }
                polyobst->finalizePolygon();
                _obstacles.push_back(ObstaclePtr(polyobst));
            }

            // Set velocity, if obstacle is moving
            if (!_obstacles.empty())
                _obstacles.back()->setCentroidVelocity(_custom_obstacle_msg.obstacles[i].velocities, _custom_obstacle_msg.obstacles[i].orientation);
        }
    }
}

void MpcLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped>& transformed_plan, double min_separation)
{
    _via_points.clear();

    if (min_separation <= 0) return;

    std::size_t prev_idx = 0;
    for (std::size_t i = 1; i < transformed_plan.size(); ++i)  // skip first one, since we do not need any point before the first min_separation [m]
    {
        // check separation to the previous via-point inserted
        if (distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation) continue;

        // add via-point
        _via_points.emplace_back(transformed_plan[i].pose);
        prev_idx = i;
    }
}

Eigen::Vector2d MpcLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose& tf_vel)
{
    Eigen::Vector2d vel;
    vel.coeffRef(0) = std::sqrt(tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY());
    vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
    return vel;
}

bool MpcLocalPlannerROS::pruneGlobalPlan(const tf2_ros::Buffer& tf, const geometry_msgs::PoseStamped& global_pose,
                                         std::vector<geometry_msgs::PoseStamped>& global_plan, double dist_behind_robot)
{
    if (global_plan.empty()) return true;

    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        geometry_msgs::TransformStamped global_to_plan_transform =
            tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
        geometry_msgs::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

        double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::PoseStamped>::iterator it        = global_plan.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
        while (it != global_plan.end())
        {
            double dx      = robot.pose.position.x - it->pose.position.x;
            double dy      = robot.pose.position.y - it->pose.position.y;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq < dist_thresh_sq)
            {
                erase_end = it;
                break;
            }
            ++it;
        }
        if (erase_end == global_plan.end()) return false;

        if (erase_end != global_plan.begin()) global_plan.erase(global_plan.begin(), erase_end);
    }
    catch (const tf::TransformException& ex)
    {
        ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
        return false;
    }
    return true;
}

bool MpcLocalPlannerROS::transformGlobalPlan(const tf2_ros::Buffer& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                             const geometry_msgs::PoseStamped& global_pose, const costmap_2d::Costmap2D& costmap,
                                             const std::string& global_frame, double max_plan_length,
                                             std::vector<geometry_msgs::PoseStamped>& transformed_plan, int* current_goal_idx,
                                             geometry_msgs::TransformStamped* tf_plan_to_global) const
{
    // this method is a slightly modified version of base_local_planner/goal_functions.h

    const geometry_msgs::PoseStamped& plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
        if (global_plan.empty())
        {
            ROS_ERROR("Received plan with zero length");
            *current_goal_idx = 0;
            return false;
        }

        // get plan_to_global_transform from plan frame to global_frame
        geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(
            global_frame, ros::Time(), plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

        // let's get the pose of the robot in the frame of the plan
        geometry_msgs::PoseStamped robot_pose;
        tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);

        // we'll discard points on the plan that are outside the local costmap
        double dist_threshold =
            std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0, costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
        dist_threshold *= 0.85;  // just consider 85% of the costmap size to better incorporate point obstacle that are
                                 // located on the border of the local costmap

        int i                    = 0;
        double sq_dist_threshold = dist_threshold * dist_threshold;
        double sq_dist           = 1e10;

        // we need to loop to a point on the plan that is within a certain distance of the robot
        for (int j = 0; j < (int)global_plan.size(); ++j)
        {
            double x_diff      = robot_pose.pose.position.x - global_plan[j].pose.position.x;
            double y_diff      = robot_pose.pose.position.y - global_plan[j].pose.position.y;
            double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
            if (new_sq_dist > sq_dist_threshold) break;  // force stop if we have reached the costmap border

            if (new_sq_dist < sq_dist)  // find closest distance
            {
                sq_dist = new_sq_dist;
                i       = j;
            }
        }

        geometry_msgs::PoseStamped newer_pose;

        double plan_length = 0;  // check cumulative Euclidean distance along the plan

        // now we'll transform until points are outside of our distance threshold
        while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
        {
            const geometry_msgs::PoseStamped& pose = global_plan[i];
            tf2::doTransform(pose, newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
            double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
            sq_dist       = x_diff * x_diff + y_diff * y_diff;

            // caclulate distance to previous pose
            if (i > 0 && max_plan_length > 0)
                plan_length += teb_local_planner::distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

            ++i;
        }

        // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
        // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
        if (transformed_plan.empty())
        {
            tf2::doTransform(global_plan.back(), newer_pose, plan_to_global_transform);

            transformed_plan.push_back(newer_pose);

            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = int(global_plan.size()) - 1;
        }
        else
        {
            // Return the index of the current goal point (inside the distance threshold)
            if (current_goal_idx) *current_goal_idx = i - 1;  // subtract 1, since i was increased once before leaving the loop
        }

        // Return the transformation from the global plan to the global planning frame if desired
        if (tf_plan_to_global) *tf_plan_to_global = plan_to_global_transform;
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("No Transform available Error: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR("Connectivity Error: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("Extrapolation Error: %s\n", ex.what());
        if (global_plan.size() > 0)
            ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(),
                      global_plan[0].header.frame_id.c_str());

        return false;
    }

    return true;
}

double MpcLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                        const geometry_msgs::PoseStamped& local_goal, int current_goal_idx,
                                                        const geometry_msgs::TransformStamped& tf_plan_to_global, int moving_average_length) const
{
    int n = (int)global_plan.size();

    // check if we are near the global goal already
    if (current_goal_idx > n - moving_average_length - 2)
    {
        if (current_goal_idx >= n - 1)  // we've exactly reached the goal
        {
            return tf2::getYaw(local_goal.pose.orientation);
        }
        else
        {
            tf2::Quaternion global_orientation;
            tf2::convert(global_plan.back().pose.orientation, global_orientation);
            tf2::Quaternion rotation;
            tf2::convert(tf_plan_to_global.transform.rotation, rotation);
            // TODO(roesmann): avoid conversion to tf2::Quaternion
            return tf2::getYaw(rotation * global_orientation);
        }
    }

    // reduce number of poses taken into account if the desired number of poses is not available
    moving_average_length =
        std::min(moving_average_length, n - current_goal_idx - 1);  // maybe redundant, since we have checked the vicinity of the goal before

    std::vector<double> candidates;
    geometry_msgs::PoseStamped tf_pose_k = local_goal;
    geometry_msgs::PoseStamped tf_pose_kp1;

    int range_end = current_goal_idx + moving_average_length;
    for (int i = current_goal_idx; i < range_end; ++i)
    {
        // Transform pose of the global plan to the planning frame
        tf2::doTransform(global_plan.at(i + 1), tf_pose_kp1, tf_plan_to_global);

        // calculate yaw angle
        candidates.push_back(
            std::atan2(tf_pose_kp1.pose.position.y - tf_pose_k.pose.position.y, tf_pose_kp1.pose.position.x - tf_pose_k.pose.position.x));

        if (i < range_end - 1) tf_pose_k = tf_pose_kp1;
    }
    return teb_local_planner::average_angles(candidates);
}

void MpcLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
{
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!",
                  opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
}

void MpcLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr& obst_msg)
{
    std::lock_guard<std::mutex> l(_custom_obst_mutex);
    _custom_obstacle_msg = *obst_msg;
}

void MpcLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr& via_points_msg)
{
    ROS_INFO_ONCE("Via-points received. This message is printed once.");
    if (_params.global_plan_viapoint_sep > 0)
    {
        ROS_WARN(
            "Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
            "Ignoring custom via-points.");
        _custom_via_points_active = false;
        return;
    }

    std::lock_guard<std::mutex> lock(_via_point_mutex);
    _via_points.clear();
    for (const geometry_msgs::PoseStamped& pose : via_points_msg->poses)
    {
        _via_points.emplace_back(pose.pose);
    }
    _custom_via_points_active = !_via_points.empty();
}


teb_local_planner::RobotFootprintModelPtr MpcLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle& nh,
                                                                                               costmap_2d::Costmap2DROS* costmap_ros)
{
    std::string model_name;
    if (!nh.getParam("footprint_model/type", model_name))
    {
        ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
        return boost::make_shared<teb_local_planner::PointRobotFootprint>();
    }

    // from costmap_2d
    if (model_name.compare("costmap_2d") == 0)
    {
        if (!costmap_ros)
        {
            ROS_WARN_STREAM("Costmap 2d pointer is null. Using point model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        ROS_INFO("Footprint model loaded from costmap_2d for trajectory optimization.");
        return getRobotFootprintFromCostmap2d(*costmap_ros);
    }

    // point
    if (model_name.compare("point") == 0)
    {
        ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
        return boost::make_shared<teb_local_planner::PointRobotFootprint>();
    }

    // circular
    if (model_name.compare("circular") == 0)
    {
        // get radius
        double radius;
        if (!nh.getParam("footprint_model/radius", radius))
        {
            ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '"
                             << nh.getNamespace() << "/footprint_model/radius' does not exist. Using point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
        return boost::make_shared<teb_local_planner::CircularRobotFootprint>(radius);
    }

    // line
    if (model_name.compare("line") == 0)
    {
        // check parameters
        if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
        {
            ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '"
                             << nh.getNamespace() << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        // get line coordinates
        std::vector<double> line_start, line_end;
        nh.getParam("footprint_model/line_start", line_start);
        nh.getParam("footprint_model/line_end", line_end);
        if (line_start.size() != 2 || line_end.size() != 2)
        {
            ROS_ERROR_STREAM(
                "Footprint model 'line' cannot be loaded for trajectory optimization, since param '"
                << nh.getNamespace()
                << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }

        ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: [" << line_end[0] << ","
                                                                << line_end[1] << "]m) loaded for trajectory optimization.");
        double min_obstacle_dist = 0.5;
        nh.param("collision_avoidance/min_obstacle_dist", min_obstacle_dist, min_obstacle_dist);
        return boost::make_shared<teb_local_planner::LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()),
                                                                         Eigen::Map<const Eigen::Vector2d>(line_end.data()),
                                                                         min_obstacle_dist);
    }

    // two circles
    if (model_name.compare("two_circles") == 0)
    {
        // check parameters
        if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") ||
            !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
        {
            ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '"
                             << nh.getNamespace()
                             << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using "
                                "point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        double front_offset, front_radius, rear_offset, rear_radius;
        nh.getParam("footprint_model/front_offset", front_offset);
        nh.getParam("footprint_model/front_radius", front_radius);
        nh.getParam("footprint_model/rear_offset", rear_offset);
        nh.getParam("footprint_model/rear_radius", rear_radius);
        ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius
                                                                        << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius
                                                                        << "m) loaded for trajectory optimization.");
        return boost::make_shared<teb_local_planner::TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }

    // polygon
    if (model_name.compare("polygon") == 0)
    {

        // check parameters
        XmlRpc::XmlRpcValue footprint_xmlrpc;
        if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc))
        {
            ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '"
                             << nh.getNamespace() << "/footprint_model/vertices' does not exist. Using point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
        // get vertices
        if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            try
            {
                Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
                ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
                return boost::make_shared<teb_local_planner::PolygonRobotFootprint>(polygon);
            }
            catch (const std::exception& ex)
            {
                ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what()
                                                                                                            << ". Using point-model instead.");
                return boost::make_shared<teb_local_planner::PointRobotFootprint>();
            }
        }
        else
        {
            ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '"
                             << nh.getNamespace()
                             << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
            return boost::make_shared<teb_local_planner::PointRobotFootprint>();
        }
    }

    // otherwise
    ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace()
                                                                               << "/footprint_model/type'. Using point model instead.");
    return boost::make_shared<teb_local_planner::PointRobotFootprint>();
}

teb_local_planner::RobotFootprintModelPtr MpcLocalPlannerROS::getRobotFootprintFromCostmap2d(costmap_2d::Costmap2DROS& costmap_ros)
{
    Point2dContainer footprint;
    Eigen::Vector2d pt;
    geometry_msgs::Polygon polygon = costmap_ros.getRobotFootprintPolygon();

    for (int i = 0; i < polygon.points.size(); ++i)
    {
        pt.x() = polygon.points[i].x;
        pt.y() = polygon.points[i].y;

        footprint.push_back(pt);
    }
    return boost::make_shared<teb_local_planner::PolygonRobotFootprint>(footprint);
}

teb_local_planner::Point2dContainer MpcLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue& footprint_xmlrpc,
                                                                                const std::string& full_param_name)
{
    // Make sure we have an array of at least 3 elements.
    if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray || footprint_xmlrpc.size() < 3)
    {
        ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s", full_param_name.c_str(),
                  std::string(footprint_xmlrpc).c_str());
        throw std::runtime_error(
            "The footprint must be specified as list of lists on the parameter server with at least "
            "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    Point2dContainer footprint;
    Eigen::Vector2d pt;

    for (int i = 0; i < footprint_xmlrpc.size(); ++i)
    {
        // Make sure each element of the list is an array of size 2. (x and y coordinates)
        XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
        if (point.getType() != XmlRpc::XmlRpcValue::TypeArray || point.size() != 2)
        {
            ROS_FATAL(
                "The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                full_param_name.c_str());
            throw std::runtime_error(
                "The footprint must be specified as list of lists on the parameter server eg: "
                "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
        }

        pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
        pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

        footprint.push_back(pt);
    }
    return footprint;
}

double MpcLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt && value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
        std::string& value_string = value;
        ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.", full_param_name.c_str(), value_string.c_str());
        throw std::runtime_error("Values in the footprint specification must be numbers");
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}

}  // end namespace mpc_local_planner
