#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Unitree Go1 视觉伺服核心节点：HSV红色分割 + 质心&像素面积提取
# 适配自带相机 /camera_face/color/image_raw，集成所有核心算法逻辑

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# 全局初始化
bridge = CvBridge()
# 红色HSV阈值
LOWER_RED1 = np.array([0, 120, 70])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 120, 70])
UPPER_RED2 = np.array([180, 255, 255])
# 形态学核（去噪用，消除小光斑）
KERNEL = np.ones((5, 5), np.uint8)

def get_red_blob_feature(cv_image):
    """
    核心算法：提取红色色块的质心和像素面积
    :param cv_image: OpenCV格式的BGR彩色图像
    :return: (centroid_x, centroid_y, area) 质心x/y，像素面积；未检测到返回(0,0,0)
    """
    # 步骤1：BGR转HSV（颜色分割的标准操作，抗光照干扰）
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    
    # 步骤2：根据HSV阈值创建红色掩码（合并两个红色区间）
    mask1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
    mask2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
    red_mask = mask1 | mask2  # 合并两个掩码，提取所有红色区域
    
    # 步骤3：形态学操作去噪（开运算：先腐蚀后膨胀，消除小噪点）
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, KERNEL)
    # 闭运算：先膨胀后腐蚀，填充色块内部小孔
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, KERNEL)
    
    # 步骤4：轮廓检测（找到红色色块的轮廓）
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # 步骤5：筛选最大轮廓（避免小噪点轮廓干扰，只取红色立方体的主轮廓）
    if not contours:
        return (0, 0, 0), red_mask  # 未检测到红色，返回默认值+掩码
    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)  # 计算像素面积
    
    # 步骤6：计算最大轮廓的质心（Centroid）
    M = cv2.moments(max_contour)
    if M["m00"] == 0:  # 避免除零错误
        return (0, 0, 0), red_mask
    c_x = int(M["m10"] / M["m00"])  # 质心x坐标
    c_y = int(M["m01"] / M["m00"])  # 质心y坐标
    
    return (c_x, c_y, area), red_mask

def image_callback(img_msg):
    """图像回调函数：完整流程→订阅→转换→分割→特征提取→可视化"""
    try:
        # 基础步骤：ROS图像转OpenCV BGR格式
        cv_image = bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        img_copy = cv_image.copy()  # 复制图像，避免原图像被修改
        img_copy = cv2.rotate(img_copy, cv2.ROTATE_180)
    except CvBridgeError as e:
        rospy.logerr("CvBridge转换失败: %s" % e)
        return

    # 核心步骤：提取红色色块的质心和像素面积
    (c_x, c_y, area), red_mask = get_red_blob_feature(img_copy)
    
    # 步骤1：可视化标注（在画面中画质心、面积、轮廓，直观查看）
    if area > 100:  # 过滤极小面积（避免噪点）
        # 画红色色块的轮廓
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_contour = max(contours, key=cv2.contourArea)
        cv2.drawContours(img_copy, [max_contour], -1, (0, 255, 0), 2)  # 绿色轮廓，2px粗
        # 画质心（蓝色圆点，半径5，实心）
        cv2.circle(img_copy, (c_x, c_y), 5, (255, 0, 0), -1)
        # 画质心坐标和像素面积（白色文字）
        cv2.putText(img_copy, f"Centroid: ({c_x},{c_y})", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(img_copy, f"Area: {int(area)} px", (10, 70), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    else:
        # 未检测到红色，显示提示
        cv2.putText(img_copy, "No Red Blob Detected", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # 步骤2：多窗口显示（原始画面+红色掩码，方便调试）
    cv2.imshow("Go1 Face Camera | Red Blob Feature", img_copy)  # 标注后的主画面
    cv2.imshow("Red Mask (HSV Segmentation)", red_mask)       # 红色分割掩码画面
    cv2.waitKey(1)  # 刷新窗口，避免卡死

    # 步骤3：终端打印特征信息（1秒1次，避免刷屏）
    if area > 100:
        rospy.loginfo_throttle(1, f"检测到红色目标 | 质心: ({c_x},{c_y}) | 像素面积: {int(area)} px")
    else:
        rospy.logwarn_throttle(1, "未检测到红色立方体目标")

if __name__ == "__main__":
    try:
        # 初始化ROS节点
        rospy.init_node("go1_visual_servoing_node", anonymous=True)
        # 订阅Go1头部正脸相机话题
        rospy.Subscriber(
            "/camera_face/color/image_raw",
            Image,
            image_callback,
            queue_size=10
        )
        rospy.loginfo("Go1视觉伺服节点已启动 | 正在检测红色立方体，提取质心&面积...")
        # 保持节点运行
        rospy.spin()
    except rospy.ROSInterruptException:
        # 节点中断，关闭所有OpenCV窗口
        cv2.destroyAllWindows()
        rospy.loginfo("视觉伺服节点已中断")
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")
        cv2.destroyAllWindows()
