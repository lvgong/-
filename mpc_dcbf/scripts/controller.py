#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class Controller():

    def __init__(self):
        # self.N = 25 #预测步数
        self.rate = rospy.Rate(15) #控制频率

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # 发布小车当前位姿和转速
        self.curr_state_pub = rospy.Publisher('/curr_state', Float32MultiArray, queue_size=10)
        # 控制量
        self.local_plan_sub = rospy.Subscriber('/local_plan', Float32MultiArray, self.local_planner_cb)
        # 里程计回调函数，显示小车当前位姿和转速
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odometry_sub_cb)

        # self.__timer_localization = rospy.Timer(rospy.Duration(0.05), self.get_current_state)
        self.listener = tf.TransformListener()

        self.linear_speed = self.angular_speed = 0.0
        # self.local_plan = np.zeros([self.N, 2])
        self.local_plan = np.zeros([1, 2])
        
        # 主循环-----
        self.control_loop()


    def quart_to_rpy(self, x, y, z, w):
        r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        p = math.asin(2*(w*y-z*x))
        y = math.atan2(2*(w*z+x*y), 1-2*(z*z+y*y))
        return r, p, y


    def get_current_state(self, event):
        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            _, _, yaw = self.quart_to_rpy(rot[0], rot[1], rot[2], rot[3])
            curr_state = Float32MultiArray()
            # curr_state.data = [trans[0], trans[1], yaw, 0.0, 0.0]
            curr_state.data = [trans[0], trans[1], yaw]
            self.curr_state_pub.publish(curr_state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass


    # 发布小车速度信息
    def pub_vel(self):
        control_cmd = Twist()
        control_cmd.linear.x = self.linear_speed
        control_cmd.angular.z = self.angular_speed
        # rospy.loginfo("Linear Speed: %.1f, Angular Speed: %.1f" % (self.linear_speed, self.angular_speed))
        self.vel_pub.publish(control_cmd)


    # 控制小车
    def control_loop(self):
        while not rospy.is_shutdown():
            self.linear_speed = self.local_plan[0, 0]#取第一组控制量
            self.angular_speed = self.local_plan[0, 1]
            self.pub_vel()
            self.rate.sleep()


    # 取所有控制量
    def local_planner_cb(self, msg):
        # for i in range(self.N):
            # self.local_plan[i, 0] = msg.data[0+2*i]
            # self.local_plan[i, 1] = msg.data[1+2*i]
        self.local_plan[0, 0] = msg.data[0]
        self.local_plan[0, 1] = msg.data[1]


    # 获取小车当前位姿和转速
    def odometry_sub_cb(self, msg:Odometry):
        # odom坐标系原点不在map坐标系上，实际上我们人为给车做了偏移，所以还得加上偏移量（不仅仅是直接加偏移量，还有朝向问题呢
        # x = msg.pose.pose.position.x #该值不准确
        # y = msg.pose.pose.position.y #该值不准确
        v = msg.twist.twist.linear.x
        # roll, pitch, yaw = self.quart_to_rpy( #该值不准确
        #     msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            _, _, yaw = self.quart_to_rpy(rot[0], rot[1], rot[2], rot[3])
            # curr_state.data = [trans[0], trans[1], yaw, 0.0, 0.0]
            # self.curr_state_pub.publish(curr_state)
            curr_state = Float32MultiArray()
            curr_state.data = [trans[0], trans[1], yaw, v*np.cos(yaw), v*np.sin(yaw)]
            # rospy.loginfo('x_odom: %f', x)
            # rospy.loginfo('x_odom: %f', yaw)
            self.curr_state_pub.publish(curr_state)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass           



if __name__ == '__main__':
    rospy.init_node('control')
    controller = Controller()
