#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS节点：控制机器人走出完美的正方形轨迹
发布话题：/cmd_vel (geometry_msgs/Twist)
"""

import rospy
import math
from geometry_msgs.msg import Twist

class SquareMotionController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('square_motion_controller', anonymous=True)
        
        # 创建速度指令发布者
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 控制参数
        self.linear_speed = 0.15 # 线速度 (m/s)
        self.angular_speed = 0.5  # 角速度 (rad/s)
        self.square_side_length = 1.0  # 正方形边长 (米)
        self.rotation_angle = math.pi / 2  # 旋转角度 (90度，弧度制)
        self.delay_compensate = 0.8 # 仿真滞后补偿延时
        self.rotate_compensate = 3.4 # 旋转核心补偿
        
        # 等待发布者连接
        rospy.sleep(1)
        
    def move_straight(self, distance):
        """
        直线移动指定距离
        :param distance: 移动距离（米），正数前进，负数后退
        """
        twist = Twist()
        
        # 设置线速度（只在x轴方向移动）
        twist.linear.x = self.linear_speed if distance > 0 else -self.linear_speed
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        
        # 计算需要移动的时间 = 距离 / 速度
        move_duration = abs(distance) / self.linear_speed
        start_time = rospy.Time.now()
        
        # 持续发布速度指令直到达到指定时间
        while (rospy.Time.now() - start_time).to_sec() < move_duration:
            self.vel_pub.publish(twist)
            rospy.sleep(0.01)  # 控制发布频率
        
        # 停止移动
        twist.linear.x = 0.0
        self.vel_pub.publish(twist)
        rospy.sleep(self.delay_compensate)  # 停顿0.5秒，确保动作完成

    def rotate(self, angle):
        """
        原地旋转指定角度
        :param angle: 旋转角度（弧度），正数左转，负数右转
        """
        twist = Twist()
        
        # 设置角速度（只在z轴方向旋转）
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.angular_speed if angle > 0 else -self.angular_speed
        
        # 计算旋转时间 = 角度 / 角速度
        rotate_duration = abs(angle) / self.angular_speed
        # 关键：为Go1增加**旋转时长补偿**（仿真滞后+四足摆腿效率低）
        rotate_duration +=self.rotate_compensate
        start_time = rospy.Time.now()
        
        rospy.loginfo(f"开始旋转90°，补偿后旋转时长：{rotate_duration:.1f}秒")
        # 持续发布旋转指令直到达到指定时间
        while (rospy.Time.now() - start_time).to_sec() < rotate_duration:
            self.vel_pub.publish(twist)
            rospy.sleep(0.01)
        
        # 停止旋转
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)
        rospy.sleep(self.delay_compensate)

    def run_square(self):
        """执行正方形轨迹运动"""
        rospy.loginfo("开始执行正方形轨迹运动...")
        
        # 执行4次 "直线移动 + 旋转" 动作
        for i in range(4):
            rospy.loginfo(f"执行第{i+1}条边的运动")
            
            # 直线移动一个边长
            self.move_straight(self.square_side_length)
            
            # 旋转90度（左转），最后一次不需要旋转（可选）
            if i < 3:
                self.rotate(self.rotation_angle)
        
        rospy.loginfo("正方形轨迹运动完成！")
        
        # 最终确保机器人停止
        stop_twist = Twist()
        self.vel_pub.publish(stop_twist)

if __name__ == '__main__':
    try:
        # 创建控制器实例并执行正方形运动
        controller = SquareMotionController()
        controller.run_square()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS节点被中断")
    except Exception as e:
        rospy.logerr(f"运行出错: {str(e)}")
        # 发生异常时确保机器人停止
        vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        stop_twist = Twist()
        vel_pub.publish(stop_twist)
