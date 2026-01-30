/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// 原有头文件保留，新增ROS头文件
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include "interface/CmdPanel.h"
#include "common/mathTools.h"

// 全局常量
extern const float sensitivityLeft;
extern const float sensitivityRight;


/*class KeyBoard
{
public:
    KeyBoard();
    ~KeyBoard();
    UserCommand checkCmd();
    void changeValue();
    static void* runKeyBoard(void *arg);
    void* run(void *arg);

    

    // 原有成员变量保留
    UserCommand userCmd;
    UserValue userValue;
    // ... 其他原有变量（_oldSettings/_newSettings/_tid等）
};
*/

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
    
    // 原有成员函数
    UserCommand checkCmd();
    void changeValue();
    static void* runKeyBoard(void *arg);
    void* run(void *arg);
    
    // 新增以下ROS相关
    static void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg); // ROS回调
    static float cmd_vel_linear;   // 线速度
    static float cmd_vel_angular;  // 角速度
    static bool use_ros_cmd;       // ROS控制开关
private:
    /*static void* runKeyBoard(void *arg);
    void* run(void *arg);
    UserCommand checkCmd();
    void changeValue();
*/
    pthread_t _tid;
    float sensitivityLeft = 0.05;
    float sensitivityRight = 0.05;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    int ret;
    char _c;
};


#endif  // KEYBOARD_H
