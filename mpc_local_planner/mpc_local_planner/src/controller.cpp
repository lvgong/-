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

#include <mpc_local_planner/controller.h>

#include <corbo-optimal-control/functions/hybrid_cost.h>
#include <corbo-optimal-control/functions/minimum_time.h>
#include <corbo-optimal-control/functions/quadratic_control_cost.h>
#include <corbo-optimal-control/structured_ocp/discretization_grids/finite_differences_variable_grid.h>
#include <corbo-optimal-control/structured_ocp/structured_optimal_control_problem.h>
#include <corbo-optimization/hyper_graph/hyper_graph_optimization_problem_edge_based.h>
#include <corbo-optimization/solver/levenberg_marquardt_sparse.h>
#include <corbo-optimization/solver/nlp_solver_ipopt.h>
#include <corbo-optimization/solver/qp_solver_osqp.h>
#include <mpc_local_planner/optimal_control/fd_collocation_se2.h>
#include <mpc_local_planner/optimal_control/final_state_conditions_se2.h>
#include <mpc_local_planner/optimal_control/finite_differences_variable_grid_se2.h>
#include <mpc_local_planner/optimal_control/min_time_via_points_cost.h>
#include <mpc_local_planner/optimal_control/quadratic_cost_se2.h>
#include <mpc_local_planner/optimal_control/stage_inequality_se2.h>
#include <mpc_local_planner/systems/kinematic_bicycle_model.h>
#include <mpc_local_planner/systems/simple_car.h>
#include <mpc_local_planner/systems/unicycle_robot.h>
#include <mpc_local_planner/utils/time_series_se2.h>

#include <mpc_local_planner_msgs/OptimalControlResult.h>
#include <mpc_local_planner_msgs/StateFeedback.h>

#include <corbo-communication/utilities.h>
#include <corbo-core/console.h>

#include <tf2/utils.h>

#include <memory>
#include <mutex>

namespace mpc_local_planner {


// 控制器配置
bool Controller::configure(ros::NodeHandle& nh, const teb_local_planner::ObstContainer& obstacles,
                           teb_local_planner::RobotFootprintModelPtr robot_model, const std::vector<teb_local_planner::PoseSE2>& via_points)
{
    // 配置了车辆类型，并把参数储存在了_dynamics中
    _dynamics = configureRobotDynamics(nh);//差速车
    if (!_dynamics) return false;  // we may need state and control dimensions to check other parameters

    // 用配置网格函数configureGrid，参数储存在_grid中
    _grid   = configureGrid(nh);
    // 调用配置求解器函数configureSolver，参数储存在_solver中
    _solver = configureSolver(nh);//ipopt求解器
    // 调用配置优化求解函数configureOcp，参数储存在_structured_ocp中，之后把 _structured_ocp复制一份到父成员的变量 _ ocp当中
    _structured_ocp = configureOcp(nh, obstacles, robot_model, via_points);//最优化问题构造
    _ocp            = _structured_ocp;  // copy pointer also to parent member

    int outer_ocp_iterations = 1;
    nh.param("controller/outer_ocp_iterations", outer_ocp_iterations, outer_ocp_iterations);
    setNumOcpIterations(outer_ocp_iterations);

    // further goal opions
    nh.param("controller/force_reinit_new_goal_dist", _force_reinit_new_goal_dist, _force_reinit_new_goal_dist);
    nh.param("controller/force_reinit_new_goal_angular", _force_reinit_new_goal_angular, _force_reinit_new_goal_angular);

    nh.param("controller/allow_init_with_backward_motion", _guess_backwards_motion, _guess_backwards_motion);
    nh.param("controller/force_reinit_num_steps", _force_reinit_num_steps, _force_reinit_num_steps);

    // custom feedback:
    nh.param("controller/prefer_x_feedback", _prefer_x_feedback, _prefer_x_feedback);
    // 检查新的测量值，即通过"state_feedback"话题得到的测量值
    _x_feedback_sub = nh.subscribe("state_feedback", 1, &Controller::stateFeedbackCallback, this);

    // result publisher:
    _ocp_result_pub = nh.advertise<mpc_local_planner_msgs::OptimalControlResult>("ocp_result", 100);
    nh.param("controller/publish_ocp_results", _publish_ocp_results, _publish_ocp_results);
    nh.param("controller/print_cpu_time", _print_cpu_time, _print_cpu_time);

    setAutoUpdatePreviousControl(false);  // we want to update the previous control value manually

    // 初始化 _ ocp指针，判断初始化是否成功
    if (_ocp->initialize())
        ROS_INFO("OCP initialized.");
    else
    {
        ROS_ERROR("OCP initialization failed");
        return false;
    }
    return _grid && _dynamics && _solver && _structured_ocp;
}


// 有两个，区别是状态和控制轨迹的输入形式。
bool Controller::step(const Controller::PoseSE2& start, const Controller::PoseSE2& goal, const geometry_msgs::Twist& vel, double dt, ros::Time t,
                      corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
{
    std::vector<geometry_msgs::PoseStamped> initial_plan(2);
    start.toPoseMsg(initial_plan.front().pose);
    goal.toPoseMsg(initial_plan.back().pose);
    return step(initial_plan, vel, dt, t, u_seq, x_seq);
}

// 输入参数：部分全局规划，时间间隔，时间，输出参数，控制序列，状态序列，速度。
bool Controller::step(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist& vel, double dt, ros::Time t,
                      corbo::TimeSeries::Ptr u_seq, corbo::TimeSeries::Ptr x_seq)
{
/*
（1）. 配置检查
 检查是否已正确配置了控制器。如果未正确配置，则会记录错误信息并返回 false，表示无法执行路径规划。
配置包括检查是否已设置动力学模型 _dynamics、网格 _grid 和结构化优化控制问题 _structured_ocp。*/
    if (!_dynamics || !_grid || !_structured_ocp)//1、首先检查动力学模型、网格、求解器设置是否到位。
    {
        ROS_ERROR("Controller must be configured before invoking step().");
        return false;
    }
/*（2）初始路径检查
   代码检查初始路径（initial_plan）是否包含至少两个姿态点。初始路径通常是全局路径规划器生成的路径，必须包含起始点和目标点。*/
    if (initial_plan.size() < 2)//2、检查全局规划维度是否足够。
    {
        ROS_ERROR("Controller::step(): initial plan must contain at least two poses.");
        return false;
    }
/*（3）提取起始和目标姿态
   代码提取了初始路径的第一个和最后一个姿态点，分别用 start和 goal表示。这些姿态点将用于路径规划和目标设定。*/
    PoseSE2 start(initial_plan.front().pose);//3、获取部分全局规划的起点和终点，储存在start和goal中。
    PoseSE2 goal(initial_plan.back().pose);
/*（4）目标状态格式转换
   调用 _dynamics->getSteadyStateFromPoseSE2 将PoseSE2格式的变量goal转换为Eigen::VectorXd格式的变量xf*/
    Eigen::VectorXd xf(_dynamics->getStateDimension());//4、获取目标点参考状态xf，维度同状态变量数
    _dynamics->getSteadyStateFromPoseSE2(goal, xf);//目标点转为eigen格式

    // retrieve or estimate current state
/*（5）初始化当前状态向量，存储当前状态*/
//getStateDimension()返回的值为3
    Eigen::VectorXd x(_dynamics->getStateDimension());
    // check for new measurements
/* （6）合并当前状态的测量和预测值
   若 _recent_x_feedback 容器中有数据，且距离最近一次状态更新的时间是否小于 2.0 * dt 秒，则表示有新的状态数据可用。
将_recent_x_feedback的值赋给当前状态x，如果没有新的状态数据可用，且（没有时间序列数据 _x_ts，或者该时间序列为空，或者无法在时间间隔 dt 内插值得到状态 x ），则直接将PoseSE2格式的变量start转换为 Eigen::VectorXd格式的变量 x。
此外，如果需要，将状态反馈与里程计反馈合并。*/
    bool new_x = false;
    {
/*  // 这里的{}用于限定互斥锁的作用范围，以确保互斥锁仅在需要时锁住。
    {
        //创建一个名为 lock 的 std::lock_guard 对象，它使用 名为_x_feedback_mutex的 互斥锁。
        // 这个锁的作用是确保在同一时刻只有一个线程可以访问被锁住的代码块*/
        std::lock_guard<std::mutex> lock(_x_feedback_mutex);
        // 5、从_recent_x_feedback获取最新的机器人状态x，要求检查时间间隔。如果获取失败，就直接把start点当做当前状态。
/*        // _recent_x_feedback的值在另一个线程中进行更新，在本线程中获取其值，使用互锁可以保证不会同时对_recent_x_feedback执行读写操作
        // 这行代码用于判断是否有新的状态数据可用。它包含两个条件：
        // _recent_x_feedback.size() > 0：检查 _recent_x_feedback 容器中是否有数据。
        //(t - _recent_x_time).toSec() < 2.0 * dt：检查距离最近一次状态更新的时间是否小于 2.0 * dt 秒，其中 t 表示当前时间，_recent_x_time 表示最近一次状态数据更新的时间，而 dt 表示时间步长。
        //如果这两个条件都满足，将 new_x 设置为 true，表示有新的状态数据可用。*/
        new_x = _recent_x_feedback.size() > 0 && (t - _recent_x_time).toSec() < 2.0 * dt;
/*        // 如果 new_x 的值为 true，即有新的状态数据可用，程序会将 _recent_x_feedback 复制给 x，从而更新机器人的当前状态。*/
        if (new_x) x = _recent_x_feedback;
    }
/*    // 如果没有新的状态数据可用，且（没有时间序列数据 _x_ts，或者该时间序列为空，或者无法在时间间隔 dt 内插值得到状态 x ），
    // 则执行条件中的程序，直接将PoseSE2格式的变量start转换为 Eigen::VectorXd格式的变量 x*/
    if (!new_x && (!_x_ts || _x_ts->isEmpty() || !_x_ts->getValuesInterpolate(dt, x)))  // predict with previous state sequence
    {
        // 否则将起始点位置赋值给测量值
        //起始点根据状态反馈，来选择是用传入的start点，还是用反馈的状态点，还是用odom点。
        _dynamics->getSteadyStateFromPoseSE2(start, x);  // otherwise initialize steady state
    }
    if (!new_x || !_prefer_x_feedback)
    {
         //如果需要，将状态反馈与里程计反馈合并。
         //注意，一些模型（如独轮车）会通过odom反馈覆盖整个状态，除非_preferr_x_measurement设置为true。
        // Merge state feedback with odometry feedback if desired.
        // Note, some models like unicycle overwrite the full state by odom feedback unless _prefer_x_measurement is set to true.
        _dynamics->mergeStateFeedbackAndOdomFeedback(start, vel, x);
    }

    // now check goal
/*（7）检查是否需要重新初始化
   代码检查是否需要重新初始化路径规划。如果目标与上一个目标之间的距离或角度变化大于阈值，将清除路径规划数据 _grid。这是为了确保机器人能够适应新的目标或路径。*/
/*6、检查目标点
检查和上一次目标的距离，两个参数约束force_reinit_new_goal_dist、force_reinit_new_goal_angular。*/
    if (_force_reinit_num_steps > 0 && _ocp_seq % _force_reinit_num_steps == 0) _grid->clear();
    // 如果目标与上一个目标之间的距离或角度变化大于阈值，将清除路径规划数据 _grid。这是为了确保机器人能够适应新的目标或路径
    if (!_grid->isEmpty() && ((goal.position() - _last_goal.position()).norm() > _force_reinit_new_goal_dist ||
                              std::abs(normalize_theta(goal.theta() - _last_goal.theta())) > _force_reinit_new_goal_angular))
    {
        // goal pose diverges a lot from the previous one, so force reinit
        _grid->clear();
    }
/*（8）生成初始状态轨迹
   如果路径规划数据为空（_grid->isEmpty()），则代码会生成初始状态轨迹。初始状态轨迹是一组状态的序列，从起始状态 x`开始，以目标状态 xf 结束。这个过程通常包括路径规划算法，以生成适当的状态序列。
   先检查目标点是否在当前点的后方，并估计是否需要执行后退运动，参数_guess_backwards_motion是是否启用该功能的开关。开启该功能时，计算机器人的初始位置到目标位置的矢量与初始方向的点积，如果点积小于0，表示矢量与方向相反，因此机器人可能需要执行反向运动。
   然后， 将给定的初始路径initial_plan，添加时间序列信息以及姿态信息，从而转换为初始轨迹，并采用线性差值对相邻两个轨迹点之间的轨迹进行差值，生成的轨迹存放在controller类的变量_x_seq_init中*/    
    if (_grid->isEmpty())//网格路径是否是空
    {
        // generate custom initialization based on initial_plan
        // check if the goal is behind the start pose (w.r.t. start orientation)
        // 7、检查一下是前进还是倒退，然后进入函数generateInitialStateTrajectory。
        /*接下来检查目标点，如果俩目标点之间的角度或者距离相差过大，则清空轨迹，重新计算。
在重新计算时，首先确定目标点是否在起始点后方：*/
        // 检查目标点是否在当前点的后方，并估计是否需要执行后退运动，参数_guess_backwards_motion是是否启用该功能的开关
        // 开启该功能时，计算机器人的初始位置到目标位置的矢量与初始方向的点积，如果点积小于0，表示矢量与方向相反，因此机器人可能需要执行反向运动。
        bool backward = _guess_backwards_motion && (goal.position() - start.position()).dot(start.orientationUnitVec()) < 0; //是否需要倒车
        //添加时间序列信息以及姿态信息，从而转换为初始轨迹，并采用线性差值对相邻两个轨迹点之间的轨迹进行差值，生成的轨迹存放在controller类的变量_x_seq_init中
// 将给定的初始路径initial_plan，添加时间序列信息以及姿态信息，从而转换为初始轨迹，并采用线性差值对相邻两个轨迹点之间的轨迹进行差值
// 生成的轨迹存放在controller类的变量_x_seq_init中
        generateInitialStateTrajectory(x, xf, initial_plan, backward);//生成参考轨迹
    }
    corbo::Time time(t.toSec());
    _x_seq_init.setTimeFromStart(time);

    corbo::StaticReference xref(xf);  // currently, we only support point-to-point transitions in ros（当前只支持ROS中的点到点转换）
    corbo::ZeroReference uref(_dynamics->getInputDimension());//0参考

/*（9）路径规划执行
   代码调用 PredictiveController::step 函数来执行路径规划。函数接收当前状态 x、目标状态 xref、控制输入 uref以及时间步长等信息，计算出下一步的状态 x_seq 和控制输入 u_seq。如果路径规划成功，返回 true，否则返回 false。*/
// 8、启动预测控制器的step函数。
// 其中x为当前位置，xref为目标点。   //当前点，目标点，参考U，时间，离散时间，输入队列，状态队列
//在control_box_rst功能包中
    _ocp_successful = PredictiveController::step(x, xref, uref, corbo::Duration(dt), time, u_seq, x_seq, nullptr, nullptr, &_x_seq_init);//求解问题
    // publish results if desired
    /* （10）发布结果
   如果 _publish_ocp_results 参数设置为真，代码会发布路径规划的结果。 这包括机器人的状态轨迹、速度命令等信息，用于监测和可视化路径规划的效果。*/
    if (_publish_ocp_results) publishOptimalControlResult();  // TODO(roesmann): we could also pass time t from above
    //  如果 _print_cpu_time 参数设置为真，代码会记录 CPU 执行时间的信息，用于性能监测。
    ROS_INFO_STREAM_COND(_print_cpu_time, "Cpu time: " << _statistics.step_time.toSec() * 1000.0 << " ms.");
    /*（12）更新序列计数器
   代码递增 _ocp_seq，以跟踪执行的路径规划次数。*/
    ++_ocp_seq;
    /*（13）更新上一个目标
   代码记录当前的目标 goal 作为下一次的参考目标。*/
    _last_goal = goal;
    //  最后，代码返回路径规划是否成功的布尔值 _ocp_successful。
    return _ocp_successful;
}

void Controller::stateFeedbackCallback(const mpc_local_planner_msgs::StateFeedback::ConstPtr& msg)
{
    if (!_dynamics) return;

    if ((int)msg->state.size() != _dynamics->getStateDimension())
    {
        ROS_ERROR_STREAM("stateFeedbackCallback(): state feedback dimension does not match robot state dimension: "
                         << msg->state.size() << " != " << _dynamics->getStateDimension());
        return;
    }

    std::lock_guard<std::mutex> lock(_x_feedback_mutex);
    _recent_x_time     = msg->header.stamp;
    _recent_x_feedback = Eigen::Map<const Eigen::VectorXd>(msg->state.data(), (int)msg->state.size());
}

void Controller::publishOptimalControlResult()
{
    if (!_dynamics) return;
    mpc_local_planner_msgs::OptimalControlResult msg;
    msg.header.stamp           = ros::Time::now();
    msg.header.seq             = static_cast<unsigned int>(_ocp_seq);
    msg.dim_states             = _dynamics->getStateDimension();
    msg.dim_controls           = _dynamics->getInputDimension();
    msg.optimal_solution_found = _ocp_successful;
    msg.cpu_time               = _statistics.step_time.toSec();

    if (_x_ts && !_x_ts->isEmpty())
    {
        msg.time_states = _x_ts->getTime();
        msg.states      = _x_ts->getValues();
    }

    if (_u_ts && !_u_ts->isEmpty())
    {
        msg.time_controls = _u_ts->getTime();
        msg.controls      = _u_ts->getValues();
    }

    _ocp_result_pub.publish(msg);
}

void Controller::reset() { PredictiveController::reset(); }


corbo::DiscretizationGridInterface::Ptr Controller::configureGrid(const ros::NodeHandle& nh)
{
    if (!_dynamics) return {};

    std::string grid_type = "fd_grid";
    nh.param("grid/type", grid_type, grid_type);

    if (grid_type == "fd_grid")
    {
        // 首先申请临时变量grid，作为函数内部的临时变量。
        FiniteDifferencesGridSE2::Ptr grid;

        bool variable_grid = true;
        // 然后根据"grid/variable_grid/enable"判断是否使用可变分辨率网格，不是的话直接用grid接收类FiniteDifferencesGridSE2的指针。
        nh.param("grid/variable_grid/enable", variable_grid, variable_grid);
        if (variable_grid)
        {
            // 是的话，再申请一个临时变量var_grid，接收类FiniteDifferencesVariableGridSE2的指针，
            FiniteDifferencesVariableGridSE2::Ptr var_grid = std::make_shared<FiniteDifferencesVariableGridSE2>();

            double min_dt = 0.0;
            nh.param("grid/variable_grid/min_dt", min_dt, min_dt);
            double max_dt = 10.0;
            nh.param("grid/variable_grid/max_dt", max_dt, max_dt);
            var_grid->setDtBounds(min_dt, max_dt);

            bool grid_adaptation = true;
            nh.param("grid/variable_grid/grid_adaptation/enable", grid_adaptation, grid_adaptation);

            if (grid_adaptation)
            {
                int max_grid_size = 50;
                nh.param("grid/variable_grid/grid_adaptation/max_grid_size", max_grid_size, max_grid_size);
                double dt_hyst_ratio = 0.1;
                nh.param("grid/variable_grid/grid_adaptation/dt_hyst_ratio", dt_hyst_ratio, dt_hyst_ratio);
                var_grid->setGridAdaptTimeBasedSingleStep(max_grid_size, dt_hyst_ratio, true);

                int min_grid_size = 2;
                nh.param("grid/variable_grid/grid_adaptation/min_grid_size", min_grid_size, min_grid_size);
                var_grid->setNmin(min_grid_size);
            }
            else
            {
                var_grid->disableGridAdaptation();
            }
            grid = var_grid;
        }
        else
        {
            grid = std::make_shared<FiniteDifferencesGridSE2>();
        }
        // common grid parameters
        int grid_size_ref = 20;
        nh.param("grid/grid_size_ref", grid_size_ref, grid_size_ref);
        grid->setNRef(grid_size_ref);

        double dt_ref = 0.3;
        nh.param("grid/dt_ref", dt_ref, dt_ref);
        grid->setDtRef(dt_ref);

        std::vector<bool> xf_fixed = {true, true, true};
        nh.param("grid/xf_fixed", xf_fixed, xf_fixed);
        if ((int)xf_fixed.size() != _dynamics->getStateDimension())
        {
            ROS_ERROR_STREAM("Array size of `xf_fixed` does not match robot state dimension(): " << xf_fixed.size()
                                                                                                 << " != " << _dynamics->getStateDimension());
            return {};
        }
        Eigen::Matrix<bool, -1, 1> xf_fixed_eigen(xf_fixed.size());  // we cannot use Eigen::Map as vector<bool> does not provide raw access
        for (int i = 0; i < (int)xf_fixed.size(); ++i) xf_fixed_eigen[i] = xf_fixed[i];
        grid->setXfFixed(xf_fixed_eigen);

        bool warm_start = true;
        nh.param("grid/warm_start", warm_start, warm_start);
        grid->setWarmStart(warm_start);

        std::string collocation_method = "forward_differences";
        nh.param("grid/collocation_method", collocation_method, collocation_method);

        if (collocation_method == "forward_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<ForwardDiffCollocationSE2>());
        }
        else if (collocation_method == "midpoint_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<MidpointDiffCollocationSE2>());
        }
        else if (collocation_method == "crank_nicolson_differences")
        {
            grid->setFiniteDifferencesCollocationMethod(std::make_shared<CrankNicolsonDiffCollocationSE2>());
        }
        else
        {
            ROS_ERROR_STREAM("Unknown collocation method '" << collocation_method << "' specified. Falling back to default...");
        }

        std::string cost_integration_method = "left_sum";
        nh.param("grid/cost_integration_method", cost_integration_method, cost_integration_method);

        if (cost_integration_method == "left_sum")
        {
            grid->setCostIntegrationRule(FullDiscretizationGridBaseSE2::CostIntegrationRule::LeftSum);
        }
        else if (cost_integration_method == "trapezoidal_rule")
        {
            grid->setCostIntegrationRule(FullDiscretizationGridBaseSE2::CostIntegrationRule::TrapezoidalRule);
        }
        else
        {
            ROS_ERROR_STREAM("Unknown cost integration method '" << cost_integration_method << "' specified. Falling back to default...");
        }

        return std::move(grid);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown grid type '" << grid_type << "' specified.");
    }

    return {};
}

// 判断车辆类型并且读取参数
RobotDynamicsInterface::Ptr Controller::configureRobotDynamics(const ros::NodeHandle& nh)
{
    _robot_type = "unicycle";
    nh.param("robot/type", _robot_type, _robot_type);

    if (_robot_type == "unicycle")
    {
        return std::make_shared<UnicycleModel>();
    }
    else if (_robot_type == "simple_car")
    {
        double wheelbase = 0.5;
        nh.param("robot/simple_car/wheelbase", wheelbase, wheelbase);
        bool front_wheel_driving = false;
        nh.param("robot/simple_car/front_wheel_driving", front_wheel_driving, front_wheel_driving);
        if (front_wheel_driving)
            return std::make_shared<SimpleCarFrontWheelDrivingModel>(wheelbase);
        else
            return std::make_shared<SimpleCarModel>(wheelbase);
    }
    else if (_robot_type == "kinematic_bicycle_vel_input")
    {
        double length_rear = 1.0;
        nh.param("robot/kinematic_bicycle_vel_input/length_rear", length_rear, length_rear);
        double length_front = 1.0;
        nh.param("robot/kinematic_bicycle_vel_input/length_front", length_front, length_front);
        return std::make_shared<KinematicBicycleModelVelocityInput>(length_rear, length_front);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown robot type '" << _robot_type << "' specified.");
    }

    return {};
}


corbo::NlpSolverInterface::Ptr Controller::configureSolver(const ros::NodeHandle& nh)
{

    std::string solver_type = "ipopt";
    nh.param("solver/type", solver_type, solver_type);

    if (solver_type == "ipopt")
    {
        corbo::SolverIpopt::Ptr solver = std::make_shared<corbo::SolverIpopt>();
        solver->initialize();  // requried for setting parameters afterward

        int iterations = 100;
        nh.param("solver/ipopt/iterations", iterations, iterations);
        solver->setIterations(iterations);//迭代次数

        double max_cpu_time = -1.0;
        nh.param("solver/ipopt/max_cpu_time", max_cpu_time, max_cpu_time);
        solver->setMaxCpuTime(max_cpu_time);//最大计算时间

        // now check for additional ipopt options
        std::map<std::string, double> numeric_options;
        nh.param("solver/ipopt/ipopt_numeric_options", numeric_options, numeric_options);
        for (const auto& item : numeric_options)
        {//对应SetNumericValue() 最小迭代阈值
            if (!solver->setIpoptOptionNumeric(item.first, item.second)) ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
        }

        std::map<std::string, std::string> string_options;
        nh.param("solver/ipopt/ipopt_string_options", string_options, string_options);
        for (const auto& item : string_options)
        {//对应SetStringValue
            if (!solver->setIpoptOptionString(item.first, item.second)) ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
        }

        std::map<std::string, int> integer_options;
        nh.param("solver/ipopt/ipopt_integer_options", integer_options, integer_options);
        for (const auto& item : integer_options)
        {
            if (!solver->setIpoptOptionInt(item.first, item.second)) ROS_WARN_STREAM("Ipopt option " << item.first << " could not be set.");
        }

        return std::move(solver);
    }
    //    else if (solver_type == "sqp")
    //    {
    //        corbo::SolverSQP::Ptr solver = std::make_shared<corbo::SolverSQP>();
    //        solver->setUseObjectiveHessian(true);
    //        // solver->setQpZeroWarmstart(false);
    //        solver->setVerbose(true);
    //        corbo::SolverOsqp::Ptr qp_solver      = std::make_shared<corbo::SolverOsqp>();
    //        qp_solver->getOsqpSettings()->verbose = 1;
    //        solver->setQpSolver(qp_solver);
    //        corbo::LineSearchL1::Ptr ls = std::make_shared<corbo::LineSearchL1>();
    //        ls->setVerbose(true);
    //        ls->setEta(1e-6);
    //        solver->setLineSearchAlgorithm(ls);

    //        return std::move(solver);
    //    }
    else if (solver_type == "lsq_lm")
    {
        corbo::LevenbergMarquardtSparse::Ptr solver = std::make_shared<corbo::LevenbergMarquardtSparse>();

        int iterations = 10;
        nh.param("solver/lsq_lm/iterations", iterations, iterations);
        solver->setIterations(iterations);

        double weight_init_eq = 2;
        nh.param("solver/lsq_lm/weight_init_eq", weight_init_eq, weight_init_eq);
        double weight_init_ineq = 2;
        nh.param("solver/lsq_lm/weight_init_ineq", weight_init_ineq, weight_init_ineq);
        double weight_init_bounds = 2;
        nh.param("solver/lsq_lm/weight_init_bounds", weight_init_bounds, weight_init_bounds);

        solver->setPenaltyWeights(weight_init_eq, weight_init_ineq, weight_init_bounds);

        double weight_adapt_factor_eq = 1;
        nh.param("solver/lsq_lm/weight_adapt_factor_eq", weight_adapt_factor_eq, weight_adapt_factor_eq);
        double weight_adapt_factor_ineq = 1;
        nh.param("solver/lsq_lm/weight_adapt_factor_ineq", weight_adapt_factor_ineq, weight_adapt_factor_ineq);
        double weight_adapt_factor_bounds = 1;
        nh.param("solver/lsq_lm/weight_adapt_factor_bounds", weight_adapt_factor_bounds, weight_adapt_factor_bounds);

        double weight_adapt_max_eq = 500;
        nh.param("solver/lsq_lm/weight_adapt_max_eq", weight_adapt_max_eq, weight_adapt_max_eq);
        double weight_adapt_max_ineq = 500;
        nh.param("solver/lsq_lm/weight_init_eq", weight_adapt_max_ineq, weight_adapt_max_ineq);
        double weight_adapt_max_bounds = 500;
        nh.param("solver/lsq_lm/weight_adapt_max_bounds", weight_adapt_max_bounds, weight_adapt_max_bounds);

        solver->setWeightAdapation(weight_adapt_factor_eq, weight_adapt_factor_ineq, weight_adapt_factor_bounds, weight_adapt_max_eq,
                                   weight_adapt_max_ineq, weight_adapt_max_bounds);

        return std::move(solver);
    }
    else
    {
        ROS_ERROR_STREAM("Unknown solver type '" << solver_type << "' specified.");
    }

    return {};
}

// 输入nh、障碍物指针容器、机器人模型、控制点四个参数
corbo::StructuredOptimalControlProblem::Ptr Controller::configureOcp(const ros::NodeHandle& nh, const teb_local_planner::ObstContainer& obstacles,
                                                                     teb_local_planner::RobotFootprintModelPtr robot_model,
                                                                     const std::vector<teb_local_planner::PoseSE2>& via_points)
{
    // 首先获取hg和ocp两个指针
    // 构建一个超图最优化问题框架
    corbo::BaseHyperGraphOptimizationProblem::Ptr hg = std::make_shared<corbo::HyperGraphOptimizationProblemEdgeBased>();
    //构建一个最优控制问题框架,相当于在上面的框架上再套一层
    corbo::StructuredOptimalControlProblem::Ptr ocp = std::make_shared<corbo::StructuredOptimalControlProblem>(_grid, _dynamics, hg, _solver);

    // 获取状态变量和控制变量的维度x_dim和u_dim
    const int x_dim = _dynamics->getStateDimension();
    const int u_dim = _dynamics->getInputDimension();
    
    // 按三种车型分类处理，即前进后退和转向速度的上界：
    if (_robot_type == "unicycle")//独轮车unicycle
    {
        double max_vel_x = 0.4;
        nh.param("robot/unicycle/max_vel_x", max_vel_x, max_vel_x);
        double max_vel_x_backwards = 0.2;
        nh.param("robot/unicycle/max_vel_x_backwards", max_vel_x_backwards, max_vel_x_backwards);
        if (max_vel_x_backwards < 0)
        {
            ROS_WARN("max_vel_x_backwards must be >= 0");
            max_vel_x_backwards *= -1;
        }
        double max_vel_theta = 0.3;
        nh.param("robot/unicycle/max_vel_theta", max_vel_theta, max_vel_theta);

        // ocp->setBounds(Eigen::Vector3d(-corbo::CORBO_INF_DBL, -corbo::CORBO_INF_DBL, -corbo::CORBO_INF_DBL),
        //               Eigen::Vector3d(corbo::CORBO_INF_DBL, corbo::CORBO_INF_DBL, corbo::CORBO_INF_DBL),
        //               Eigen::Vector2d(-max_vel_x_backwards, -max_vel_theta), Eigen::Vector2d(max_vel_x, max_vel_theta));
        //控制输入边界
        ocp->setControlBounds(Eigen::Vector2d(-max_vel_x_backwards, -max_vel_theta), Eigen::Vector2d(max_vel_x, max_vel_theta));
    }
    else if (_robot_type == "simple_car")//阿克曼车simple_car
    {
        double max_vel_x = 0.4;
        nh.param("robot/simple_car/max_vel_x", max_vel_x, max_vel_x);
        double max_vel_x_backwards = 0.2;
        nh.param("robot/simple_car/max_vel_x_backwards", max_vel_x_backwards, max_vel_x_backwards);
        if (max_vel_x_backwards < 0)
        {
            ROS_WARN("max_vel_x_backwards must be >= 0");
            max_vel_x_backwards *= -1;
        }
        double max_steering_angle = 1.5;
        nh.param("robot/simple_car/max_steering_angle", max_steering_angle, max_steering_angle);

        ocp->setControlBounds(Eigen::Vector2d(-max_vel_x_backwards, -max_steering_angle), Eigen::Vector2d(max_vel_x, max_steering_angle));
    }
    else if (_robot_type == "kinematic_bicycle_vel_input")//动力自行车kinematic_bicycle_vel_input
    {
        double max_vel_x = 0.4;
        nh.param("robot/kinematic_bicycle_vel_input/max_vel_x", max_vel_x, max_vel_x);
        double max_vel_x_backwards = 0.2;
        nh.param("robot/kinematic_bicycle_vel_input/max_vel_x_backwards", max_vel_x_backwards, max_vel_x_backwards);
        if (max_vel_x_backwards < 0)
        {
            ROS_WARN("max_vel_x_backwards must be >= 0");
            max_vel_x_backwards *= -1;
        }
        double max_steering_angle = 1.5;
        nh.param("robot/kinematic_bicycle_vel_input/max_steering_angle", max_steering_angle, max_steering_angle);

        ocp->setControlBounds(Eigen::Vector2d(-max_vel_x_backwards, -max_steering_angle), Eigen::Vector2d(max_vel_x, max_steering_angle));
    }
    else
    {
        ROS_ERROR_STREAM("Cannot configure OCP for unknown robot type " << _robot_type << ".");
        return {};
    }

    // 按目标代价函数类型分类处理，主要是读取权重：
    std::string objective_type = "minimum_time";
    nh.param("planning/objective/type", objective_type, objective_type);
    bool lsq_solver = _solver->isLsqSolver();

    if (objective_type == "minimum_time")
    {   
        ocp->setStageCost(std::make_shared<corbo::MinimumTime>(lsq_solver));
    }
    //二次型目标函数cost
    else if (objective_type == "quadratic_form")
    {
/*代码首先从参数服务器中获取状态权重 state_weights 和控制权重 control_weights。状态权重用于定义状态变量在目标函数中的重要性，控制权重则定义控制变量在目标函数中的重要性。*/
        std::vector<double> state_weights;
        nh.param("planning/objective/quadratic_form/state_weights", state_weights, state_weights);
        Eigen::MatrixXd Q;
/*接着根据状态变量维度 x_dim 和控制变量维度 u_dim，构造状态权重矩阵 Q 和控制权重矩阵 R。如果参数设置正确，则会构造对应维度的对角矩阵，并用参数中的值填充对角线元素。*/
        if (state_weights.size() == x_dim)
        {
            Q = Eigen::Map<Eigen::VectorXd>(state_weights.data(), x_dim).asDiagonal();
        }
        else if (state_weights.size() == x_dim * x_dim)
        {
            Q = Eigen::Map<Eigen::MatrixXd>(state_weights.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            ROS_ERROR_STREAM("State weights dimension invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }
        std::vector<double> control_weights;
        nh.param("planning/objective/quadratic_form/control_weights", control_weights, control_weights);
        Eigen::MatrixXd R;
        if (control_weights.size() == u_dim)
        {
            R = Eigen::Map<Eigen::VectorXd>(control_weights.data(), u_dim).asDiagonal();
        }
        else if (control_weights.size() == u_dim * u_dim)
        {
            R = Eigen::Map<Eigen::MatrixXd>(control_weights.data(), u_dim, u_dim);  // Eigens default is column major
        }
        else
        {
            ROS_ERROR_STREAM("Control weights dimension invalid. Must be either " << u_dim << " x 1 or " << u_dim << " x " << u_dim << ".");
            return {};
        }
/*然后，根据参数配置是否使用积分形式 integral_form 和混合最小时间成本 hybrid_cost_minimum_time，以及状态权重和控制权重是否为零，来决定设置何种类型的阶段代价函数。*/
        bool integral_form = false;
        nh.param("planning/objective/quadratic_form/integral_form", integral_form, integral_form);
        bool hybrid_cost_minimum_time = false;
        nh.param("planning/objective/quadratic_form/hybrid_cost_minimum_time", hybrid_cost_minimum_time, hybrid_cost_minimum_time);

        bool q_zero = Q.isZero();
        bool r_zero = R.isZero();
        // 如果状态权重和控制权重均不为零，则设置二次形式代价函数 QuadraticFormCostSE2，该代价函数考虑了状态和控制的加权和
        if (!q_zero && !r_zero)
        {
            PRINT_ERROR_COND(hybrid_cost_minimum_time,
                             "Hybrid minimum time and quadratic form cost is currently only supported for non-zero control weights only. Falling "
                             "back to quadratic form.");
            ocp->setStageCost(std::make_shared<QuadraticFormCostSE2>(Q, R, integral_form, lsq_solver));
        }
        // 如果状态权重不为零且控制权重为零，则设置仅包含状态权重的二次形式代价函数 QuadraticStateCostSE2，此时只考虑状态变量的影响
        else if (!q_zero && r_zero)
        {
            PRINT_ERROR_COND(hybrid_cost_minimum_time,
                             "Hybrid minimum time and quadratic form cost is currently only supported for non-zero control weights only. Falling "
                             "back to only quadratic state cost.");
            ocp->setStageCost(std::make_shared<QuadraticStateCostSE2>(Q, integral_form, lsq_solver));
        }
        // 如果状态权重为零且控制权重不为零，根据是否设置了 hybrid_cost_minimum_time 参数，选择设置最小时间代价函数 MinTimeQuadraticControls 或者二次形式控制代价函数 QuadraticControlCost。
        else if (q_zero && !r_zero)
        {
            if (hybrid_cost_minimum_time)
            {
                ocp->setStageCost(std::make_shared<corbo::MinTimeQuadraticControls>(R, integral_form, lsq_solver));
            }
            else
            {
                ocp->setStageCost(std::make_shared<corbo::QuadraticControlCost>(R, integral_form, lsq_solver));
            }
        }
    }
    else if (objective_type == "minimum_time_via_points")
    {
        bool via_points_ordered = false;
        nh.param("planning/objective/minimum_time_via_points/via_points_ordered", via_points_ordered, via_points_ordered);
        double position_weight = 1.0;
        nh.param("planning/objective/minimum_time_via_points/position_weight", position_weight, position_weight);
        double orientation_weight = 0.0;
        nh.param("planning/objective/minimum_time_via_points/orientation_weight", orientation_weight, orientation_weight);
        ocp->setStageCost(std::make_shared<MinTimeViaPointsCost>(via_points, position_weight, orientation_weight, via_points_ordered));
        // TODO(roesmann): lsq version
    }
    else
    {
        ROS_ERROR_STREAM("Unknown objective type '" << objective_type << "' specified ('planning/objective/type').");
        return {};
    }

    // 终端cost类型
    std::string terminal_cost = "none";//无终端代价
    nh.param("planning/terminal_cost/type", terminal_cost, terminal_cost);

    if (terminal_cost == "none")
    {//终端cost
        ocp->setFinalStageCost({});
    }
    else if (terminal_cost == "quadratic")//二次型终端代价
    {
        std::vector<double> state_weights;
        nh.param("planning/terminal_cost/quadratic/final_state_weights", state_weights, state_weights);
        Eigen::MatrixXd Qf;
        if (state_weights.size() == x_dim)
        {
            Qf = Eigen::Map<Eigen::VectorXd>(state_weights.data(), x_dim).asDiagonal();
        }
        else if (state_weights.size() == x_dim * x_dim)
        {
            Qf = Eigen::Map<Eigen::MatrixXd>(state_weights.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            ROS_ERROR_STREAM("Final state weights dimension invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }
        ocp->setFinalStageCost(std::make_shared<QuadraticFinalStateCostSE2>(Qf, lsq_solver));
    }
    else
    {
        ROS_ERROR_STREAM("Unknown terminal_cost type '" << terminal_cost << "' specified ('planning/terminal_cost/type').");
        return {};
    }

    // 终端约束类型
    std::string terminal_constraint = "none";
    nh.param("planning/terminal_constraint/type", terminal_constraint, terminal_constraint);

    if (terminal_constraint == "none")//无终端约束
    {
        ocp->setFinalStageConstraint({});
    }
    else if (terminal_constraint == "l2_ball")//l2_ball终端约束
    {
        std::vector<double> weight_matrix;
        nh.param("planning/terminal_constraint/l2_ball/weight_matrix", weight_matrix, weight_matrix);
        Eigen::MatrixXd S;
        if (weight_matrix.size() == x_dim)
        {
            S = Eigen::Map<Eigen::VectorXd>(weight_matrix.data(), x_dim).asDiagonal();
        }
        else if (weight_matrix.size() == x_dim * x_dim)
        {
            S = Eigen::Map<Eigen::MatrixXd>(weight_matrix.data(), x_dim, x_dim);  // Eigens default is column major
        }
        else
        {
            ROS_ERROR_STREAM("l2-ball weight_matrix dimensions invalid. Must be either " << x_dim << " x 1 or " << x_dim << " x " << x_dim << ".");
            return {};
        }
        double radius = 1.0;
        nh.param("planning/terminal_constraint/l2_ball/radius", radius, radius);
        ocp->setFinalStageConstraint(std::make_shared<TerminalBallSE2>(S, radius));
    }
    else
    {
        ROS_ERROR_STREAM("Unknown terminal_constraint type '" << terminal_constraint << "' specified ('planning/terminal_constraint/type').");
        return {};
    }

    _inequality_constraint = std::make_shared<StageInequalitySE2>();
    //障碍物不等式约束
    _inequality_constraint->setObstacleVector(obstacles);
    //footprint不等式约束
    _inequality_constraint->setRobotFootprintModel(robot_model);

    // configure collision avoidance
    // 配置不等式约束，用于避障；
    double min_obstacle_dist = 0.5;
    nh.param("collision_avoidance/min_obstacle_dist", min_obstacle_dist, min_obstacle_dist);
    //设置障碍物最小距离
    _inequality_constraint->setMinimumDistance(min_obstacle_dist);

    bool enable_dynamic_obstacles = false;
    nh.param("collision_avoidance/enable_dynamic_obstacles", enable_dynamic_obstacles, enable_dynamic_obstacles);
    //是否开启动态障碍物
    _inequality_constraint->setEnableDynamicObstacles(enable_dynamic_obstacles);

    double force_inclusion_dist = 0.5;
    nh.param("collision_avoidance/force_inclusion_dist", force_inclusion_dist, force_inclusion_dist);
    double cutoff_dist = 2;
    nh.param("collision_avoidance/cutoff_dist", cutoff_dist, cutoff_dist);
    //障碍物过滤
    _inequality_constraint->setObstacleFilterParameters(force_inclusion_dist, cutoff_dist);

    // configure control deviation bounds
// 配置控制量偏差范围，即限制加速度和减速度的范围：
    if (_robot_type == "unicycle")//独轮车unicycle：
    {
        double acc_lim_x = 0.0;
        nh.param("robot/unicycle/acc_lim_x", acc_lim_x, acc_lim_x);
        double dec_lim_x = 0.0;
        nh.param("robot/unicycle/dec_lim_x", dec_lim_x, dec_lim_x);
        if (dec_lim_x < 0)
        {
            ROS_WARN("dec_lim_x must be >= 0");
            dec_lim_x *= -1;
        }
        double acc_lim_theta = 0.0;
        nh.param("robot/unicycle/acc_lim_theta", acc_lim_theta, acc_lim_theta);

        if (acc_lim_x <= 0) acc_lim_x = corbo::CORBO_INF_DBL;
        if (dec_lim_x <= 0) dec_lim_x = corbo::CORBO_INF_DBL;
        if (acc_lim_theta <= 0) acc_lim_theta = corbo::CORBO_INF_DBL;
        //加速度约束
        Eigen::Vector2d ud_lb(-dec_lim_x, -acc_lim_theta);
        Eigen::Vector2d ud_ub(acc_lim_x, acc_lim_theta);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else if (_robot_type == "simple_car")
    {
        double acc_lim_x = 0.0;
        nh.param("robot/simple_car/acc_lim_x", acc_lim_x, acc_lim_x);
        double dec_lim_x = 0.0;
        nh.param("robot/simple_car/dec_lim_x", dec_lim_x, dec_lim_x);
        if (dec_lim_x < 0)
        {
            ROS_WARN("dec_lim_x must be >= 0");
            dec_lim_x *= -1;
        }
        double max_steering_rate = 0.0;
        nh.param("robot/simple_car/max_steering_rate", max_steering_rate, max_steering_rate);

        if (acc_lim_x <= 0) acc_lim_x = corbo::CORBO_INF_DBL;
        if (dec_lim_x <= 0) dec_lim_x = corbo::CORBO_INF_DBL;
        if (max_steering_rate <= 0) max_steering_rate = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-dec_lim_x, -max_steering_rate);
        Eigen::Vector2d ud_ub(acc_lim_x, max_steering_rate);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else if (_robot_type == "kinematic_bicycle_vel_input")
    {
        double acc_lim_x = 0.0;
        nh.param("robot/kinematic_bicycle_vel_input/acc_lim_x", acc_lim_x, acc_lim_x);
        double dec_lim_x = 0.0;
        nh.param("robot/kinematic_bicycle_vel_input/dec_lim_x", dec_lim_x, dec_lim_x);
        if (dec_lim_x < 0)
        {
            ROS_WARN("dec_lim_x must be >= 0");
            dec_lim_x *= -1;
        }
        double max_steering_rate = 0.0;
        nh.param("robot/kinematic_bicycle_vel_input/max_steering_rate", max_steering_rate, max_steering_rate);

        if (acc_lim_x <= 0) acc_lim_x = corbo::CORBO_INF_DBL;
        if (dec_lim_x <= 0) dec_lim_x = corbo::CORBO_INF_DBL;
        if (max_steering_rate <= 0) max_steering_rate = corbo::CORBO_INF_DBL;
        Eigen::Vector2d ud_lb(-dec_lim_x, -max_steering_rate);
        Eigen::Vector2d ud_ub(acc_lim_x, max_steering_rate);
        _inequality_constraint->setControlDeviationBounds(ud_lb, ud_ub);
    }
    else
    {
        ROS_ERROR_STREAM("Cannot configure control deviation bounds for unknown robot type " << _robot_type << ".");
        return {};
    }
    //不等式约束传入最优控制器里
    ocp->setStageInequalityConstraint(_inequality_constraint);

    return ocp;
}


/*输入当前实际状态x0，目标状态xf，全局参考轨迹initial_plan，前进后退信标。
从起点到终点生成一系列状态轨迹，并与时间绑定成对。*/
bool Controller::generateInitialStateTrajectory(const Eigen::VectorXd& x0, const Eigen::VectorXd& xf,
                                                const std::vector<geometry_msgs::PoseStamped>& initial_plan, bool backward)
{
    if (initial_plan.size() < 2 || !_dynamics) return false;

    TimeSeriesSE2::Ptr ts = std::make_shared<TimeSeriesSE2>();

    int n_init = (int)initial_plan.size();//位姿数
    int n_ref  = _grid->getInitialN();//N=11，步数
    if (n_ref < 2)
    {
        ROS_ERROR("Controller::generateInitialStateTrajectory(): grid not properly initialized");
        return false;
    }
    ts->add(0.0, x0);//（时刻，当前状态）

    double dt_ref = _grid->getInitialDt();//全局参考轨迹的时间间隔 dt_ref 
    double tf_ref = (double)(n_ref - 1) * dt_ref;//总时间 tf_ref

    Eigen::VectorXd x(_dynamics->getStateDimension());//当前状态

    // we initialize by assuming equally distributed poses
    // 在generateInitialStateTrajectory()函数中，假设所有pose均匀分布。
    double dt_init = tf_ref / double(n_init - 1);//时间间隔

    double t = dt_init;
    /*遍历初始规划点，根据设定的方法获取每个规划点的航向角 yaw，并根据航向角得到中间状态 x，并将中间状态 x 和对应的时间 t 添加到 TimeSeriesSE2 对象 ts 中*/
    for (int i = 1; i < n_init - 1; ++i)
    {
        // get yaw from the orientation of the distance vector between pose_{i+1} and pose_{i}
        double yaw;
        if (_initial_plan_estimate_orientation)
        {
            // 如果目标点不在起始点后方，直接得到i时刻到i+1时刻的yaw：
            double dx = initial_plan[i + 1].pose.position.x - initial_plan[i].pose.position.x;
            double dy = initial_plan[i + 1].pose.position.y - initial_plan[i].pose.position.y;
            yaw       = std::atan2(dy, dx);
            // 如果在起始点后方的话，在计算yaw时：
            if (backward) normalize_theta(yaw + M_PI);
        }
        else
        {
            yaw = tf2::getYaw(initial_plan[i].pose.orientation);
        }
        PoseSE2 intermediate_pose(initial_plan[i].pose.position.x, initial_plan[i].pose.position.y, yaw);
        _dynamics->getSteadyStateFromPoseSE2(intermediate_pose, x);//由PoseSE2类型的intermediate_pose计算向量类型的x
        ts->add(t, x);//中间点
        t += dt_init;//时间间隔
    }

    ts->add(tf_ref, xf);//目标点

    _x_seq_init.setTrajectory(ts, corbo::TimeSeries::Interpolation::Linear);
    return true;
}


// 状态轨迹可行性检查。检查两个位姿之间的距离是否大于机器人的半径，或者方位差是否大于指定的阈值，并在这种情况下进行插值。(如果障碍物将两个连续的姿势推开，两个连续姿势之间的中心可能与障碍物重合;-)!
bool Controller::isPoseTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                          double inscribed_radius, double circumscribed_radius, double min_resolution_collision_check_angular,
                                          int look_ahead_idx)
{
    if (!_grid)
    {
        ROS_ERROR("Controller must be configured before invoking step().");
        return false;
    }
    if (_grid->getN() < 2) return false;

    // we currently require a full discretization grid as we want to have fast access to
    // individual states without requiring any new simulation.
    // Alternatively, other grids could be used in combination with method getStateAndControlTimeSeries()
    const FullDiscretizationGridBaseSE2* fd_grid = dynamic_cast<const FullDiscretizationGridBaseSE2*>(_grid.get());
    if (!fd_grid)
    {
        ROS_ERROR("isPoseTrajectoriyFeasible is currently only implemented for fd grids");
        return true;
    }

    if (look_ahead_idx < 0 || look_ahead_idx >= _grid->getN()) look_ahead_idx = _grid->getN() - 1;

    for (int i = 0; i <= look_ahead_idx; ++i)
    {
        if (costmap_model->footprintCost(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2], footprint_spec, inscribed_radius,
                                         circumscribed_radius) == -1)
        {
            return false;
        }
        // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
        // and interpolates in that case.
        // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
        if (i < look_ahead_idx)
        {
            double delta_rot           = normalize_theta(fd_grid->getState(i + 1)[2] - fd_grid->getState(i)[2]);
            Eigen::Vector2d delta_dist = fd_grid->getState(i + 1).head(2) - fd_grid->getState(i).head(2);
            if (std::abs(delta_rot) > min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
            {
                int n_additional_samples = std::max(std::ceil(std::abs(delta_rot) / min_resolution_collision_check_angular),
                                                    std::ceil(delta_dist.norm() / inscribed_radius)) -
                                           1;

                PoseSE2 intermediate_pose(fd_grid->getState(i)[0], fd_grid->getState(i)[1], fd_grid->getState(i)[2]);
                for (int step = 0; step < n_additional_samples; ++step)
                {
                    intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
                    intermediate_pose.theta()    = g2o::normalize_theta(intermediate_pose.theta() + delta_rot / (n_additional_samples + 1.0));
                    if (costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(), footprint_spec,
                                                     inscribed_radius, circumscribed_radius) == -1)
                    {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

}  // namespace mpc_local_planner
