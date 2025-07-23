#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float32MultiArray, ColorRGBA
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
import threading
import numpy as np
from visualization_msgs.msg import Marker
import casadi as ca

controller_ls = ["None", "MPC-DC", "MPC-SCBF", "MPC-DCBF", "MPC-ACBF"]


# 测距
def distance_global(c1, c2):
    return np.sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) + (c1[1] - c2[1]) * (c1[1] - c2[1]))


class Local_Planner():

    def __init__(self, controller_n:int):
        # for replan frequency
        self.replan_period = 1/rospy.get_param('/local_planner/mpc_frequency', 5)
        # for control type
        self.controller = controller_ls[controller_n]
        rospy.logwarn('----------the controller is: {}----------'.format(self.controller))

        self.z = 0
        # self.N = 20
        self.N  = rospy.get_param('/local_planner/pre_step', 25) #预测步长，就是这个，最好和NMPC那边一样
        self.Ts = rospy.get_param('/local_planner/step_time', 0.15) #时间间隔，就是这个，最好和NMPC那边一样

        self.goal_state = np.zeros([self.N, 3]) #N行3列
        self.curr_state = None
        self.global_path = None

        self.last_input = []
        self.last_state = []
        self.mpc_success = None

        # 互斥锁
        self.curr_pose_lock = threading.Lock()
        self.global_path_lock = threading.Lock()
        self.obstacle_lock = threading.Lock()
        # self.detect_lock = threading.Lock()
        
        # 障碍列表
        self.ob = []

        self.nums_of_planning = 1
        self.cost_time_sum = 0.0

        # 主流程定时器
        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
        # 先关闭定时器，等工作切换时再开启
        self.__timer_replan.shutdown()
        self.__timer_replan = None  # 将定时器对象设为None

        # 小车当前位姿
        self.__sub_curr_state = rospy.Subscriber('/curr_state', Float32MultiArray, self.__curr_pose_cb, queue_size=10)
        # 障碍物及其轨迹
        self.__sub_obs = rospy.Subscriber('/obs_Manager_node/obs_predict_pub', Float32MultiArray, self.__obs_cb, queue_size=100)
        # MPC-CBF的参考路径
        self.__sub_goal = rospy.Subscriber('/global_path', Path, self.__global_path_cb, queue_size=1)
        # 车轮廓可视化
        self.__pub_local_path_vis = rospy.Publisher('/pub_path_vis', Marker, queue_size=10)
        # 预测路径可视化
        self.__pub_local_path = rospy.Publisher('/local_path', Path, queue_size=10)
        # 控制信息供controller控制
        self.__pub_local_plan = rospy.Publisher('/local_plan', Float32MultiArray, queue_size=10)

        # self.__pub_start = rospy.Publisher('/cmd_move', Bool, queue_size=10)
        
        # 订阅预测头位姿
        self.__sub_mpc_head = rospy.Subscriber('/head_pose', PoseStamped, self.__detect_state_cb, queue_size=1)
        # 发布是否切换工作状态指令，由MPC->MPC-D-CBF
        self.__pub_change2dcbf = rospy.Publisher('/change2dcbf', Bool, queue_size=1)
        # 发布切换工作态，由MPC-D-CBF->MPC
        self.__pub_change2mpc = rospy.Publisher('/change2mpc', Bool, queue_size=1)
        # 订阅是否切换工作状态指令，由MPC-D-CBF->MPC
        # self.__sub_change2mpc = rospy.Subscriber('/change2mpc', Bool, self.__change2mpc_cb, queue_size=1)


    # 切换工作状态至mpc
    def __change2mpc_cb(self, data:Bool):
        # 定时器停止
        # 关闭定时器只是停止了后续的定时器回调触发，而不会中断当前正在执行的回调函数
        self.__timer_replan.shutdown()
        self.__timer_replan = None  # 将定时器对象设为None
        self.global_path = [] #清空全局路径
        self.global_path = None
        # change_state = Bool()
        # change_state.data = True
        # self.__pub_change2mpc.publish(change_state)
        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return False #加这条语句，还能使定时器回调立即退出
        
    # 检测MPC预测头是否碰触障碍
    def __detect_state_cb(self, data:PoseStamped):
        # 用来判断是否有障碍物阻挡了机器人前进的路径
        # 前进视野的障碍物
        # self.detect_lock.acquire()
        # 首先判断是否已经装载了障碍物
        if len(self.ob) > 0:  # self.ob存储处理后的障碍物数据
            # 遍历所有装载的障碍
            num_obs = int(len(self.ob)/self.N) #障碍物数量
            # rospy.loginfo('num_obs: %d', num_obs)
            for j in range(num_obs):
                # 遍历每一个障碍物的预测第一步哦
                ob_ = self.ob[self.N*j]

                # 将障碍物的位置坐标存储在 ob_vec 中
                ob_vec = np.array([ob_[0], ob_[1]])
                # 计算预测的目标点和障碍物之间的向量 center_vec
                # center_vec = self.goal_state[self.N-1, :2] - ob_vec
                # 取我们MPC预测的最后一步
                goal_state_ = np.array([data.pose.position.x, data.pose.position.y])
                # rospy.loginfo('x: %f', data.pose.position.x)
                # rospy.loginfo('y: %f', data.pose.position.y)
                center_vec = self.curr_state[:2] - ob_vec
                # 计算向量 center_vec 的模长，即目标点和障碍物之间的距离
                d = np.linalg.norm(center_vec)
                # 计算交叉点 cross_pt，这里可能是通过将障碍物向目标点方向移动一定距离来找到交叉点的方法
                cross_pt = ob_vec + 1.1*ob_[2]/d*center_vec
                # cross_pt = ob_vec
                # 计算两个向量，分别是目标点到交叉点和当前位置到交叉点的向量
                # vec1 = self.goal_state[self.N-1, :2] - cross_pt
                vec1 = goal_state_ - cross_pt
                vec2 = self.curr_state[:2] - cross_pt

                # 障碍物和目标点之间的角度
                # 计算这两个向量的点积，用来判断障碍物和目标点之间的角度关系
                theta = np.dot(vec1.T, vec2)
                # 根据计算得到的角度 theta 是否大于0，返回一个布尔值，表示是否存在障碍物位于前进视野中

                # rospy.loginfo('theta: %f', theta)
                # rospy.loginfo('theta')

                # 靠近障碍就会变小，直至变成<0
                if theta <= 0:
                    self.global_path = [] #清空全局路径
                    self.global_path = None
                    # 同时开启__replan_cb定时器
                    # 如果没开启定时器，则开启
                    # if not self.__timer_replan.is_alive():
                    if self.__timer_replan is None:  # 如果定时器未启动
                        # self.__timer_replan.start()
                        # 在需要重新启动定时器的地方（例如工作切换时），再次启动定时器
                        self.__timer_replan = rospy.Timer(rospy.Duration(self.replan_period), self.__replan_cb)
                   
                    local_plan = Float32MultiArray()
                    # # 只需要传输一对小数
                    local_plan.data.append(0.0) #保存控制量，线速度归零
                    local_plan.data.append(0.0) #角速度归零
                                        # 让小车速度归零
                    self.__pub_local_plan.publish(local_plan)
                    # 发布状态切换
                    change_state = Bool()
                    change_state.data = True
                    self.__pub_change2dcbf.publish(change_state)


    # 周期执行控制器主流程                
    def __replan_cb(self, event):
        if self.choose_goal_state():  # 获取当前目标的位姿信息，如果不满足后面不执行
            # add angle information
            for i in range(self.N - 1):
                y_diff = self.goal_state[i+1, 1]-self.goal_state[i, 1] #纵向差
                x_diff = self.goal_state[i+1, 0]-self.goal_state[i, 0] #横向差
                if x_diff != 0 and y_diff != 0:
                    self.goal_state[i, 2] = np.arctan2(y_diff, x_diff) #goal_state第3列存yaw
                elif i != 0:
                    self.goal_state[i, 2] = self.goal_state[i-1, 2]
                else:
                    self.goal_state[i, 2] = 0
            # 将self.goal_state中倒数第二行的第三列的值赋给最后一行的第三列
            self.goal_state[-1, 2] = self.goal_state[-2, 2] #就是最后一个路点的yaw赋前面一个的

            plan_time_start = rospy.Time.now()  # 规划开始时间

            # MPC规划-核心
            states_sol, input_sol = self.MPC_ellip()

            plan_time_spend = (rospy.Time.now() - plan_time_start).to_sec()  # 规划结束时间
            # self.cost_time_sum += plan_time_spend
            # average_time = self.cost_time_sum / self.nums_of_planning
            rospy.loginfo("plan_time_spend:= %s", plan_time_spend) #耗时
            # rospy.loginfo("\033[32m[DCBF]: Local-Trajectory Generation Spend Time := %s\033[0m", average_time) #平均耗时
            # self.nums_of_planning += 1

            # cmd_move = Bool()
            # cmd_move.data = distance_global(self.curr_state, self.global_path[-1]) > 0.1
            # self.__pub_start.publish(cmd_move)
            self.__publish_local_plan(input_sol, states_sol) #利用控制量和状态量发布各种信息


    # 获取当前状态
    def __curr_pose_cb(self, data):
        self.curr_pose_lock.acquire()
        self.curr_state = np.zeros(5)
        self.curr_state[0] = data.data[0] #x
        self.curr_state[1] = data.data[1] #y
        self.curr_state[2] = data.data[2] #yaw
        self.curr_state[3] = data.data[3] #vx
        self.curr_state[4] = data.data[4] #vy
        # rospy.loginfo("curr_state[0]:= %f", self.curr_state[0])
        self.curr_pose_lock.release()


    # 处理接收到的障碍物数据
    def __obs_cb(self, data):
        self.obstacle_lock.acquire() #使用了锁 obstacle_lock 来确保线程安全，在处理障碍物数据时先获取锁
        self.ob = [] #存储处理后的障碍物数据
        size = int(len(data.data) / 7) #计算接收到的数据中包含多少个障碍物数据，每个障碍物数据包含了7个元素
        for i in range(size):
            self.ob.append(np.array(data.data[7*i:7*i+7])) #将每个障碍物的数据（7个元素）作为一个 numpy 数组存储在 ob 列表中
        self.obstacle_lock.release() #在处理完所有障碍物数据后释放锁，确保其他线程可以继续访问


    # 获取避障的参考路径
    def __global_path_cb(self, path:Path):
        self.global_path_lock.acquire()
        size = len(path.poses) #路点数
        if size > 0: #表明有路点
            self.global_path = np.zeros([size, 3])
            for i in range(size):
                # 主要x和y
                self.global_path[i, 0] = path.poses[i].pose.position.x
                self.global_path[i, 1] = path.poses[i].pose.position.y
        # rospy.loginfo("start_x:= %f", path.poses[0].pose.position.x)
        # rospy.loginfo("end_x:= %f", path.poses[size-1].pose.position.x)
        self.global_path_lock.release()


    # 发布预测轨迹、控制量、车轮廓
    def __publish_local_plan(self, input_sol, state_sol):
        local_path = Path() #预测轨迹可视化
        local_plan = Float32MultiArray() #控制量
        # local_path_vis = Marker() ##可视化车轮廓
        # local_path_vis.type = Marker.LINE_LIST
        # local_path_vis.scale.x = 0.05
        # local_path_vis.color.g = local_path_vis.color.b = local_path_vis.color.a = 1.0

        # local_path_vis.header.stamp = rospy.Time.now()
        local_path.header.stamp = rospy.Time.now()

        local_path.header.frame_id = "map"
        # local_path_vis.header.frame_id = "map"

        for i in range(self.N):
            this_pose_stamped = PoseStamped()
            this_pose_stamped.header.seq = i
            this_pose_stamped.header.stamp = rospy.Time.now()
            this_pose_stamped.header.frame_id = "map"
            this_pose_stamped.pose.position.x = state_sol[i, 0] #预测状态
            this_pose_stamped.pose.position.y = state_sol[i, 1]
            this_pose_stamped.pose.position.z = self.z #0
            this_pose_stamped.pose.orientation.x = 0
            this_pose_stamped.pose.orientation.y = 0
            this_pose_stamped.pose.orientation.z = 0
            this_pose_stamped.pose.orientation.w = 1
            local_path.poses.append(this_pose_stamped)
            # for j in range(2):
            #     if len(input_sol[i]) != 0: #第i步的控制量
            #         # 只需要传输一对小数
            #         local_plan.data.append(input_sol[i][j]) #保存控制量

            # pt = Point()
            # pt.x = state_sol[i, 0]
            # pt.y = state_sol[i, 1]
            # pt.z = self.z #0

            # color = ColorRGBA()
            # color.r = 1
            # color.g = 0.82
            # color.b = 0.1
            # color.a = 1

            # p1 = Point()
            # p2 = Point()
            # p3 = Point()
            # p4 = Point()
            # if i < self.N-1:
            #     x_diff = state_sol[i+1, 0]-state_sol[i, 0] #x向差
            #     y_diff = state_sol[i+1, 1]-state_sol[i, 1] #y向差
            #     if x_diff != 0 and y_diff != 0:
            #         theta = np.arctan2(y_diff, x_diff) #yaw
            #     else:
            #         theta = 0
            #     w = 0.58 #车宽
            #     l = 0.72 #车长
            #     p1.z = pt.z-0.01
            #     p1.x = 0.5*(l*np.cos(theta)-w*np.sin(theta)) + pt.x
            #     p1.y = 0.5*(l*np.sin(theta)+w*np.cos(theta)) + pt.y
            #     p2.z = pt.z-0.01
            #     p2.x = 0.5*(-l*np.cos(theta)-w*np.sin(theta)) + pt.x
            #     p2.y = 0.5*(-l*np.sin(theta)+w*np.cos(theta)) + pt.y
            #     p3.z = pt.z-0.01
            #     p3.x = 0.5*(-l*np.cos(theta)+w*np.sin(theta)) + pt.x
            #     p3.y = 0.5*(-l*np.sin(theta)-w*np.cos(theta)) + pt.y
            #     p4.z = pt.z-0.01
            #     p4.x = 0.5*(l*np.cos(theta)+w*np.sin(theta)) + pt.x
            #     p4.y = 0.5*(l*np.sin(theta)-w*np.cos(theta)) + pt.y

            #     local_path_vis.points.append(p1)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p2)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p2)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p3)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p3)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p4)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p4)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.points.append(p1)
            #     local_path_vis.colors.append(color)
            #     local_path_vis.pose.orientation.x = 0
            #     local_path_vis.pose.orientation.y = 0
            #     local_path_vis.pose.orientation.z = 0
            #     local_path_vis.pose.orientation.w = 1

        for j in range(2):
            # if len(input_sol[0]) != 0: #第i步的控制量
                # 只需要传输一对小数
            local_plan.data.append(input_sol[0][j]) #保存控制量
        # local_plan.data.append(input_sol[0][0]) #保存控制量
        # local_plan.data.append(input_sol[0][1]) #保存控制量
        
        # rospy.loginfo("state_sol[0, 0]:= %f", state_sol[0, 0])
        # rospy.loginfo("state_sol[0, 1]:= %f", state_sol[0, 1])

        # self.__pub_local_path_vis.publish(local_path_vis) #可视化车轮廓
        self.__pub_local_path.publish(local_path) #预测轨迹
        self.__pub_local_plan.publish(local_plan) #控制量


    # 获取当前目标的位姿信息
    def choose_goal_state(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        # 这一步可以确保那边的全局路径有了之后才会执行这里的主流程
        if self.global_path is None or self.curr_state is None:
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False

        waypoint_num = self.global_path.shape[0] #获取global_path的形状，并返回第一个维度的大小
        # 得最近全局路点的索引，因为小车由于车速惯性可能不在规划起点
        num = np.argmin(np.array([distance_global(self.curr_state, self.global_path[i]) for i in range(waypoint_num)]))
        # rospy.loginfo('num: %d', num)

        # 当小车已经行使到目标控制点附近时，切换工作状态，回到NMPC态
        # 如何判断小车离目标点很近?必须对比当前位姿和目标点位姿（平方距离）
        if distance_global(self.curr_state, self.global_path[-1]) < 0.8: #说明小车离终点很近
            # 切换工作状态
            # 定时器停止
            # 关闭定时器只是停止了后续的定时器回调触发，而不会中断当前正在执行的回调函数
            self.__timer_replan.shutdown()
            self.__timer_replan = None  # 将定时器对象设为None
            self.global_path = [] #清空全局路径
            self.global_path = None
            change_state = Bool()
            change_state.data = True
            self.__pub_change2mpc.publish(change_state)
            self.curr_pose_lock.release()
            self.global_path_lock.release()
            return False #加这条语句，还能使定时器回调立即退出

        scale = 1 # 步距
        num_list = [] 
        for i in range(self.N):
            num_path = min(waypoint_num - 1, num + i * scale) #标号，限制最大为最后一个路点，如果路点少了就是最后路点的重复
            num_list.append(num_path)

        for k in range(self.N):
            self.goal_state[k] = self.global_path[num_list[k]] #某路点的位姿
        
        # rospy.loginfo("goal_state[0]_x:= %f", self.goal_state[0][0])
        # rospy.loginfo("goal_state[N-1]_x:= %f", self.goal_state[self.N-1][0])
        
        self.curr_pose_lock.release()
        self.global_path_lock.release()
        return True


    # MPC规划-核心
    def MPC_ellip(self):
        self.curr_pose_lock.acquire()
        self.global_path_lock.acquire()
        self.obstacle_lock.acquire()
        # self.detect_lock.acquire()

        opti = ca.Opti() #优化器
        # parameters for optimization
        gamma_k = 0.3
        safe_dist = 0.5 # for scout 安全距离，允许贴着过去

        # v_max = 1.3 #线速度
        v_max = 0.5 #线速度
        # v_min = 0.1
        v_min = 0.0
        # omega_max = 0.8 #角速度
        omega_max = 0.5 #角速度

        if self.controller == "None": #MPC-DCBF
            opt_x0 = opti.parameter(3)

            # state variables
            opt_states = opti.variable(self.N + 1, 3)
            opt_controls = opti.variable(self.N, 2)
            v = opt_controls[:, 0]
            omega = opt_controls[:, 1]

            def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

            def exceed_ob(ob_):
                l_long_axis = ob_[2]
                l_short_axis = ob_[3]
                long_axis = np.array([np.cos(ob_[4]) * l_long_axis, np.sin(ob_[4]) * l_long_axis])

                ob_vec = np.array([ob_[0], ob_[1]])
                center_vec = self.goal_state[self.N-1, :2] - ob_vec
                dist_center = np.linalg.norm(center_vec)
                cos_ = np.dot(center_vec, long_axis.T) / (dist_center * l_long_axis)

                if np.abs(cos_) > 0.1:
                    tan_square = 1 / (cos_ ** 2) - 1
                    d = np.sqrt((l_long_axis ** 2 * l_short_axis ** 2 * (1 + tan_square) / (
                        l_short_axis ** 2 + l_long_axis ** 2 * tan_square)))
                else:
                    d = l_short_axis

                cross_pt = ob_vec + d * center_vec / dist_center

                vec1 = self.goal_state[self.N-1, :2] - cross_pt
                vec2 = self.curr_state[:2] - cross_pt
                theta = np.dot(vec1.T, vec2)

                return theta > 0

            def h(curpos_, ob_):
                safe_dist = 0.8 # for scout
                # safe_dist = 0.5 # for jackal

                c = ca.cos(ob_[4])
                s = ca.sin(ob_[4])
                a = ca.MX([ob_[2]])
                b = ca.MX([ob_[3]])

                ob_vec = ca.MX([ob_[0], ob_[1]])
                center_vec = curpos_[:2] - ob_vec.T

                dist = b * (ca.sqrt((c ** 2 / a ** 2 + s ** 2 / b ** 2) * center_vec[0] ** 2 + (s ** 2 / a ** 2 + c ** 2 / b ** 2) *
                                    center_vec[1] ** 2 + 2 * c * s * (1 / a ** 2 - 1 / b ** 2) * center_vec[0] * center_vec[1]) - 1) - safe_dist
                
                return dist

            def quadratic(x, A):
                return ca.mtimes([x, A, x.T])

            # init_condition
            opti.subject_to(opt_states[0, :] == opt_x0.T)

            # Position Boundaries
            if(distance_global(self.curr_state, self.global_path[-1, :2]) > 1):
                opti.subject_to(opti.bounded(v_min, v, v_max))
            else:
                opti.subject_to(opti.bounded(-v_min, v, v_max))

            opti.subject_to(opti.bounded(-omega_max, omega, omega_max))

            # System Model constraints
            for i in range(self.N):
                x_next = opt_states[i, :] + self.Ts * f(opt_states[i, :], opt_controls[i, :]).T
                opti.subject_to(opt_states[i + 1, :] == x_next)

            num_obs = int(len(self.ob)/self.N) # 这才是真正障碍物的数量

            # rospy.loginfo("num_obs := %d", num_obs)

            for j in range(num_obs): #遍历所有障碍物
                if not exceed_ob(self.ob[self.N*j]): #遍历的是所有障碍物的预测第一步
                    for i in range(self.N-1):
                        opti.subject_to(h(opt_states[i + 1, :], self.ob[j * self.N + i + 1]) >=
                                        (1 - gamma_k) * h(opt_states[i, :], self.ob[j * self.N + i]))

            obj = 0
            R = np.diag([0.1, 0.05])
            A = np.diag([0.1, 0.02])
            for i in range(self.N):
                Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
                if i < self.N-1:
                    obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
                else:
                    obj += 0.1 * quadratic(opt_states[i, :] - self.goal_state[[i]], Q)
            Q = np.diag([1.0,1.0, 0.02])*5
            obj += quadratic(opt_states[self.N-1, :] - self.goal_state[[self.N-1]], Q)

            opti.minimize(obj)
            opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                            'ipopt.acceptable_obj_change_tol': 1e-3}
            opti.solver('ipopt', opts_setting)
            opti.set_value(opt_x0, self.curr_state[:3])

        else: #有控制器
            opt_x0 = opti.parameter(5) #初始状态量
            # states[x, y, theta, dx, dy]
            opt_states = opti.variable(self.N+1, 5) #状态量
            opt_controls = opti.variable(self.N, 2) #控制量
            tau_list        = ca.MX.zeros(10, self.N)
            v = opt_controls[:, 0] #线速度
            omega = opt_controls[:, 1]  #角速度      

            # 软约束松弛因子
            lambda_ = opti.variable(self.N, 1)  

            # 运动学约束
            def f(x_, u_): return ca.vertcat(*[u_[0] * ca.cos(x_[2]) * self.Ts, u_[0] * ca.sin(x_[2]) * self.Ts, u_[1] * self.Ts, 
                                               u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2])])
            
            # 障碍物约束1
            def h(curpos_:ca.SX, ob_:tuple)->ca.SX:
                x_obs, y_obs, r_obs = ob_
                h = ca.sqrt((x_obs-curpos_[0])**2+(y_obs-curpos_[1])**2)-(safe_dist+r_obs)
                return h
            
            # 障碍物约束2
            def h1(curpos_:ca.SX, ob_:tuple, vr_:tuple, tau_=0.0)->ca.SX:
                x_obs, y_obs, r_obs = ob_
                h = ca.sqrt((x_obs-curpos_[0]+vr_[0]*tau_)**2 + (y_obs - curpos_[1]+vr_[1]*tau_)**2) - (safe_dist + r_obs)
                return h
            
            def h2(curpos_:ca.SX, ob_:tuple, vr_:ca.SX, tau_:ca.SX)->ca.SX:
                x_obs, y_obs, r_obs = ob_
                h = ca.sqrt((x_obs-curpos_[0]+vr_[0]*tau_)**2 + (y_obs - curpos_[1]+vr_[1]*tau_)**2) - (safe_dist + r_obs)
                return h
            
            # 向量内积
            def quadratic(x, A):
                return ca.mtimes([x, A, x.T])
            
            def compute_tau0(curpos_:ca.SX, ob_:tuple, ob_v_:tuple, v_r_:ca.SX)->ca.SX:
                ro_p = curpos_[:2]
                obs_p = np.array(ob_[:2]).reshape(1,2)
                obs_v = np.array(ob_v_).reshape(1,2)

                pr = obs_p - ro_p
                vr = v_r_
                tau_max = ca.fabs((ca.norm_2(pr)-ob_[2])*ca.norm_2(pr)/ca.dot(pr,vr))
                tau_0 = ca.if_else(ca.dot(pr,vr)<0.0, tau_max, 0.0)
                tau_1 = ca.if_else(tau_0>0.65, 0.65, tau_0)
                
                obs_scale   = 1-(ca.fabs(ca.dot(pr,obs_v))/ca.norm_2(pr)/1.5-1)**2
                ro_scale    = 1-(ca.norm_2(pr)/6.0-1)**2
                tau_ = obs_scale*ro_scale*tau_1
                return tau_
            
            def compute_tau(curpos_:ca.SX, ob_:tuple, ob_v_:tuple, v_r_:ca.SX)->ca.SX:
                ro_p = curpos_[:2]
                obs_p = np.array(ob_[:2]).reshape(1,2)
                obs_v = np.array(ob_v_).reshape(1,2)

                pr = obs_p - ro_p
                vr = v_r_
                tau_max     = ca.fabs((ca.norm_2(pr)-ob_[2]-0.9)*ca.norm_2(pr)/ca.dot(pr,vr))  
                tau_0       = ca.if_else(ca.dot(pr,vr)<0.0, tau_max, 0.0)             
                tau_1       = ca.if_else(tau_0<2.0, tau_0, 2.0)             
                # obs_scale0  = 1-(ca.fabs(ca.dot(pr,obs_v))/ca.norm_2(pr)/1.5-1)**2
                tau_ = tau_1*0.50
                return tau_
                  
            # 障碍物筛选函数，判断一个障碍物是否在路径前进视野中
            def exceed_ob(ob_:list):
                case = 1
                if case == 1:
                    # 前进视野的障碍物
                    ob_vec = np.array([ob_[0], ob_[1]]) #表示障碍物的位置向量
                    # center_vec = self.goal_state[self.N-1, :2] - ob_vec
                    center_vec = self.curr_state[:2] - ob_vec #从当前状态到障碍物的向量
                    d = np.linalg.norm(center_vec) #计算中心向量的模 ，从当前状态到障碍物的欧几里得距离
                    # 计算交点:
                    # 根据障碍物的半径 ob_[2] 和中心向量 center_vec，计算出一个新的点 cross_pt，该点略微偏离障碍物的边缘。
                    cross_pt = ob_vec + 1.1*ob_[2]/d*center_vec #这里的 1.1 是一个扩展系数，使得 cross_pt 位于障碍物边缘之外。
                    # cross_pt = ob_vec
                    # 目标状态向量:
                    vec1 = self.goal_state[self.N-1, :2] - cross_pt #从目标状态（self.goal_state[self.N-1, :2]）到 cross_pt 的向量。
                    # 当前状态向量:
                    vec2 = self.curr_state[:2] - cross_pt #从当前状态（self.curr_state[:2]）到 cross_pt 的向量。

                    # 计算夹角余弦:
                    theta = np.dot(vec1.T, vec2) #vec1 和 vec2 的点积，用于判断两个向量之间的夹角。
                    # 如果 theta 大于0，则表示两个向量夹角小于90度，障碍物不在路径前进视野中。
                    return theta > 0 #返回布尔值，表示障碍物是否在路径前进视野中。
                elif case == 2:
                    # 机器人周围的障碍物
                    ob_vec = np.array([ob_[0], ob_[1]])
                    vec1 = ob_vec - self.curr_state[:2]                         #相对位移
                    vec2 = np.array([ob_[5], ob_[6]]) - self.curr_state[3:]     #相对速度
                    vec3 = self.global_path[-1, :2] - self.curr_state[:2]
                    col_time = np.linalg.norm(vec1)*(np.linalg.norm(vec1)-0.4-0.6)/np.dot(vec1,vec2)
                    
                    # sign = np.linalg.norm(vec1) >= 6.0 or np.dot(vec1, vec2) >= 0 or col_time <= -2.0 or np.dot(vec1, vec3) <= 0
                    # sign = np.linalg.norm(vec1)-0.4 >= 5.5 or np.dot(vec1, vec3) < 0
                    sign = np.linalg.norm(vec1)-0.4 >= 5.5
                    if sign == False:
                        print(np.linalg.norm(vec1))
                    return sign
            
            # tau值函数（ACBF用）
            def updata_paramter(ob_:np.ndarray) -> float: 
                tau_ = 0.0
                ro_p    = self.curr_state[:2]
                ro_v    = self.curr_state[3:]
                obs_p   = ob_[:2]
                obs_v   = ob_[5:]

                p_r     = obs_p - ro_p
                v_r     = obs_v - ro_v
                tau_max = abs((np.linalg.norm(p_r)-safe_dist-ob_[2])*np.linalg.norm(p_r)/(-np.dot(p_r, v_r)))
                print("tau_max := {:.3f}".format(tau_max))
                if (np.dot(p_r, v_r) < 0.0) & (tau_max < 2.0):
                    tau_max = min(tau_max, 0.65)
                    obs_scale = 1-(abs(np.dot(p_r,obs_v)/np.linalg.norm(p_r)/1.5)-1)**2
                    ro_scale = 1-(np.linalg.norm(p_r)/6.0-1)**2
                    scale = obs_scale * ro_scale
                    print("obs_scale :={:.3f}".format(obs_scale))
                    print("ro_scale :={:.3f}".format(ro_scale))
                else:
                    scale = 0.0
                # scale = 0.5
                
                tau_ = tau_max * scale
                return tau_


            # init_condition
            opti.subject_to(opt_states[0, :] == opt_x0.T)

            # Position Boundaries
            # self.global_path[-1, :2]:从 self.global_path 中获取最后一个点的坐标。根据代码片段来看，:2 表示获取该点的前两个维度（通常是 x 和 y 坐标）
            # if(distance_global(self.curr_state, self.global_path[-1, :2]) > 0.8): #当前位姿和全局路径最后一个位姿之间的距离
            opti.subject_to(opti.bounded(v_min, v, v_max)) #速度约束
            # else:
                # opti.subject_to(opti.bounded(-v_min, v, v_max))

            opti.subject_to(opti.bounded(-omega_max, omega, omega_max)) #角速度约束
            
            # 松弛变量的范围约束
            opti.subject_to(opti.bounded(0.0, lambda_, 0.1))

            # System Model constraints 运动学约束
            fa = np.zeros([5,5])
            fa[:3, :3] = np.eye(3) 
            for i in range(self.N):
                x_next = opt_states[i, :]@fa + f(opt_states[i, :], opt_controls[i, :]).T
                opti.subject_to(opt_states[i + 1, :] == x_next)

            num_obs = int(len(self.ob)/self.N) #障碍物数量
            exceed_num = 0
            # rospy.loginfo("num_obs := %d", num_obs)
            # Safety constraint 安全约束
            for j in range(num_obs):
                # print(j)
                # print(self.ob[self.N*j]) self.ob[self.N*j]：只是取了第一步去预测有没有障碍阻挡
                if not exceed_ob(self.ob[self.N*j]): #表示前方路线上有障碍物阻挡
                    # print('choose obs{}'.format(j))
                    # 如果障碍物在前进视野中，增加 exceed_num 计数器。
                    exceed_num = exceed_num+1
                    if exceed_num > 2:
                        break
                    print('\033[34m---------- choose obs{} ----------\033[0m'.format(j))


                    if self.controller == "MPC-ACBF":
                        v_r = self.ob[self.N*j][5:] - self.curr_state[3:]
                        p_r = self.ob[self.N*j][:2] - self.curr_state[:2]
                        obs_scale   = 1-(abs(np.dot(p_r, self.ob[self.N*j][5:]))/np.linalg.norm(p_r)/1.5-1)**2
                        ro_scale    = 1-(np.linalg.norm(p_r)/5.5-1)**2 
                        tau_scale   = obs_scale*ro_scale
                        print('\033[34m---------- obs_tau_scale:= {} ----------\033[0m'.format(tau_scale))

                    obs = (self.ob[self.N*j][0], self.ob[self.N*j][1], self.ob[self.N*j][2])
                    
                    for i in range(self.N-1):
                        if self.controller == "MPC-DC":
                            hk      = h(opt_states[i, :], obs)
                            opti.subject_to(hk>=0)
                        elif self.controller == "MPC-SCBF":
                            hk      = h(opt_states[i, :], obs)
                            hk1     = h(opt_states[i+1,:], obs)
                            st_cbf  = -hk1 + (1-gamma_k)*hk
                            opti.subject_to(st_cbf<=0)
                        elif self.controller == "MPC-DCBF":
                            obs     = (self.ob[j*self.N+i][0], self.ob[j*self.N+i][1], self.ob[j*self.N+i][2])
                            obs1    = (self.ob[j*self.N+i+1][0], self.ob[j*self.N+i+1][1], self.ob[j*self.N+i+1][2])
                            hk      = h(opt_states[i, :], obs)
                            hk1     = h(opt_states[i+1,:], obs1)
                            # st_cbf  = -hk1 + (1-gamma_k)*hk
                            st_cbf  = -hk1 + (1-gamma_k)*hk - lambda_[i, 0]
                            opti.subject_to(st_cbf<=0)
                        elif self.controller == "MPC-ACBF":
                            obs     = (self.ob[j*self.N+i][0], self.ob[j*self.N+i][1], self.ob[j*self.N+i][2])
                            obs1    = (self.ob[j*self.N+i+1][0], self.ob[j*self.N+i+1][1], self.ob[j*self.N+i+1][2])

                            obs_v   = self.ob[j*self.N+i+1][5:]
                            ro_v    = (opt_states[i+1, :2]-opt_states[i, :2])/self.Ts
                            vr      = obs_v.reshape(1,2) - ro_v
                            tau     = compute_tau(opt_states[i,:], obs, obs_v, vr)*tau_scale
                            n1 = 4
                            if i >= n1:
                                tau = 0.0
                            hk      = h2(opt_states[i, :], obs, vr, tau)
                            hk1     = h2(opt_states[i+1, :], obs1, vr, tau)
                            st_cbf  = -hk1 + (1-gamma_k)*hk - lambda_[i, 0]
                            # st_cbf  = -hk1 + (1-gamma_k)*hk
                            opti.subject_to(st_cbf<=0)
                            tau_list[j,i] = tau
                        else:
                            print('-----------------------error----------------------')
        
            # Object function
            obj = 0
            R = np.diag([0.1, 0.05]) #R矩阵
            A = np.diag([0.1, 0.02])
            for i in range(self.N):
                # 软约束DCBF松弛代价
                if self.controller == "MPC-ACBF" or "MPC-DCBF":
                    obj += lambda_[i, 0] * lambda_[i, 0] * 500.0
                    # 软约束DCBF松弛代价
                    # obj += lambda_[i,:] * lambda_[i,:] * 10000.0
                    pass

                Q = np.diag([1.0+0.05*i,1.0+0.05*i, 0.02+0.005*i])
                if i < self.N-1:
                    obj += 0.1 * quadratic(opt_states[i, :3] - self.goal_state[[i]], Q) + quadratic(opt_controls[i, :], R)
                else:
                    obj += 0.1 * quadratic(opt_states[i, :3] - self.goal_state[[i]], Q)
            Q = np.diag([1.0,1.0, 0.02])*5
            obj += quadratic(opt_states[self.N-1, :3] - self.goal_state[[self.N-1]], Q)

            opti.minimize(obj)
            opts_setting = {'ipopt.max_iter': 2000, 'ipopt.print_level': 0, 'print_time': 0, 'ipopt.acceptable_tol': 1e-3,
                            'ipopt.acceptable_obj_change_tol': 1e-3}
            opti.solver('ipopt', opts_setting)
            opti.set_value(opt_x0, self.curr_state)

        # try----
        try:
            sol = opti.solve()
            u_res = sol.value(opt_controls) #控制量
            state_res = sol.value(opt_states) #状态量
            # print(sol.value(tau_list))

            self.last_input = u_res
            self.last_state = state_res
            self.mpc_success = True # mpc成功
            # print(self.last_input)

        except:
            rospy.logerr("Infeasible Solution")
        #错误了并无其他操作了-----------------------

            if self.mpc_success:
                self.mpc_success = False #复位
            else: #失败了
                for i in range(self.N-1):
                    # 将上一次的mpc预测的后几步拿来
                    self.last_input[i] = self.last_input[i+1]
                    self.last_state[i] = self.last_state[i+1]
                    self.last_input[self.N-1] = np.zeros([1, 2]) #剩下的填充

            u_res = self.last_input
            state_res = self.last_state
        self.curr_pose_lock.release()
        self.global_path_lock.release()
        self.obstacle_lock.release()
        # self.detect_lock.release()
        return state_res, u_res



if __name__ == '__main__':
    rospy.init_node("mpc_cbf_planner")
    controller_n = rospy.get_param('/local_planner/controller_type', 0) # 需要添加前缀
    cbf_planner = Local_Planner(controller_n)

    rospy.spin()
