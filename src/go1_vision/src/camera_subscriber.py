#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Unitree Go1 红色立方体视觉伺服闭环控制节点
# 功能：180°图像矫正 + 红色检测 + P控制转向 + 面积阈值距离控制 + /cmd_vel运动发布
# 核心：先对准（转向P控制），再前进（面积阈值），平稳追踪移动的红色立方体

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

# 全局初始化
bridge = CvBridge()
# 1. 红色HSV阈值（已优化，适配Gazebo原生红）
LOWER_RED1 = np.array([0, 80, 50])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 80, 50])
UPPER_RED2 = np.array([180, 255, 255])
KERNEL = np.ones((7, 7), np.uint8)

# 2. 相机画面参数
IMG_CENTER_X = 480  # 图像中心X（转向参考点）
IMG_CENTER_Y = 360  # 图像中心Y
IMG_WIDTH = 960
IMG_HEIGHT = 720

# 3. 闭环控制参数
KP = 0.0012         # 转向P控制器比例系数
YAW_VEL_MAX = 0.5   # 最大转向角速度
DEAD_ZONE = 30    # 转向死区阈值
MIN_YAW_VEL = 0.2  # 最小转向角速度
TARGET_AREA = 25000  # 目标面积阈值
FORWARD_SPEED = 0.4 # 前进线速度
AREA_MARGIN = 800   # 面积余量，避免频繁启停

# 4. ROS发布器：发布运动指令到/cmd_vel（Go1仿真包通用运动话题）
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# 运动指令对象初始化
twist = Twist()

def get_red_blob_feature(cv_image):
    """红色立方体特征提取：质心x、质心y、像素面积"""
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    red_mask = mask1 | mask2
    # 形态学去噪
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, KERNEL, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, KERNEL, iterations=1)
    # 轮廓检测与最大轮廓筛选
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return (0, 0, 0), red_mask
    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    # 计算质心，避免除零错误
    M = cv2.moments(max_contour)
    if M["m00"] < 100:
        return (0, 0, 0), red_mask
    c_x = int(M["m10"] / M["m00"])
    c_y = int(M["m01"] / M["m00"])
    return (c_x, c_y, area), red_mask

def visual_servo_control(c_x, c_y, area):
    # 初始化运动指令为0（默认停止）
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    # 仅当检测到有效红色目标（面积>100）时，执行控制
    if area > 100:
        # ------------ 1. 转向P控制（Yaw）------------
        error_x = c_x - IMG_CENTER_X
        if abs(error_x) > DEAD_ZONE:
            # 计算原始P控制角速度
            raw_yaw_vel = KP * error_x
            # 分段判断：大误差用P控制，小误差用最小角速度
            if abs(raw_yaw_vel) > MIN_YAW_VEL:
                # 大误差：用原P控制角速度，限幅
                twist.angular.z = raw_yaw_vel
            else:
                # 小误差：用最小角速度
                twist.angular.z = MIN_YAW_VEL * np.sign(raw_yaw_vel)
            # 最终角速度限幅（不超过最大速度）
            twist.angular.z = np.clip(twist.angular.z, -YAW_VEL_MAX, YAW_VEL_MAX)
        else:
            # 死区内：对准完成，角速度置0
            twist.angular.z = 0.0
        if area < TARGET_AREA - AREA_MARGIN:
            twist.linear.x = FORWARD_SPEED
        else:
            twist.linear.x = 0.0
        
    # 发布运动指令到Go1
    cmd_vel_pub.publish(twist)
    return twist.linear.x, twist.angular.z

def image_callback(img_msg):
    """图像回调主函数：矫正→检测→控制→可视化"""
    try:
        # ROS图像转OpenCV BGR格式
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        img_copy = cv_image.copy()
        # 图像180°旋转矫正,解决相机上下颠倒
        img_copy = cv2.rotate(img_copy, cv2.ROTATE_180)
    except CvBridgeError as e:
        rospy.logerr("图像转换失败: %s" % e)
        return

    # 步骤1：提取红色立方体特征
    (c_x, c_y, area), red_mask = get_red_blob_feature(img_copy)
    # 步骤2：执行视觉伺服闭环控制，获取运动指令
    linear_x, angular_z = visual_servo_control(c_x, c_y, area)
    # 步骤3：可视化标注（检测结果+控制指令，方便调试）
    if area > 100:
        # 画立方体轮廓、质心、图像中心十字线
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img_copy, [max_contour], -1, (0, 255, 0), 4)
        cv2.circle(img_copy, (c_x, c_y), 8, (255, 0, 0), -1)
        cv2.line(img_copy, (IMG_CENTER_X, 0), (IMG_CENTER_X, IMG_HEIGHT), (0, 0, 255), 2)
        cv2.line(img_copy, (0, IMG_CENTER_Y), (IMG_WIDTH, IMG_CENTER_Y), (0, 0, 255), 2)
        # 标注质心、面积、运动指令
        cv2.putText(img_copy, f"Centroid: ({c_x},{c_y})", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(img_copy, f"Area: {int(area)} px", (20, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(img_copy, f"Speed: {linear_x:.1f} m/s", (20, 140), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(img_copy, f"Yaw: {angular_z:.2f} rad/s", (20, 190), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
    else:
        # 未检测到目标，标注提示，发布停止指令
        cv2.putText(img_copy, "NO RED TARGET", (20, 40), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    # 放大显示窗口
    cv2.imshow("Go1 Visual Servo Control (Track Red Cube)", cv2.resize(img_copy, (960, 720)))
    cv2.imshow("Red Mask", cv2.resize(red_mask, (960, 720)))
    cv2.waitKey(1)

    # 终端打印关键信息
    if area > 100:
        rospy.loginfo_throttle(1, 
            f"✅ 追踪中 | 质心: ({c_x},{c_y}) | 面积: {int(area)} px | 前进速度: {linear_x:.1f} m/s | 转向角速度: {angular_z:.2f} rad/s"
        )
    else:
        rospy.logwarn_throttle(1, "未检测到红色立方体")

if __name__ == "__main__":
    try:
        # 初始化ROS节点，设置匿名=True避免重名
        rospy.init_node("go1_visual_servo_track_node", anonymous=True)
        # 订阅Go1头部正脸相机话题
        rospy.Subscriber(
            "/camera_face/color/image_raw",
            Image,
            image_callback,
            queue_size=10
        )
        rospy.loginfo("Go1红色立方体视觉追踪节点启动")
        rospy.loginfo(f"控制参数：KP={KP} | 目标面积={TARGET_AREA}px | 前进速度={FORWARD_SPEED}m/s")
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        # 节点中断，发布停止指令，关闭窗口
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        cv2.destroyAllWindows()
        rospy.loginfo("视觉追踪节点已中断，机器狗已停止")
    except Exception as e:
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        cv2.destroyAllWindows()
        rospy.logerr(f"节点运行错误: {e}")
