/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

// 开头添加ROS头文件
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 全局灵敏度定义
const float sensitivityLeft = 0.05;
const float sensitivityRight = 0.05;

// 新增：静态变量初始化
float KeyBoard::cmd_vel_linear = 0.0;
float KeyBoard::cmd_vel_angular = 0.0;
bool KeyBoard::use_ros_cmd = true;

// 新增：ROS回调函数
void KeyBoard::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel_linear = msg->linear.x;
    cmd_vel_angular = msg->angular.z;
    ROS_INFO_THROTTLE(1, "ROS Cmd: linear=%.2f, angular=%.2f", 
                      KeyBoard::cmd_vel_linear, KeyBoard::cmd_vel_angular);
}

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){

    if(use_ros_cmd){
        // 映射ROS指令到userValue（保留原有范围[-1,1]）
        userValue.ly = cmd_vel_linear / 0.5; // 0.5是最大线速度，可调整
        userValue.rx = cmd_vel_angular / 1.0; // 1.0是最大角速度，可调整
        // 限制范围
        userValue.ly = std::min<float>(std::max<float>(userValue.ly, -1.0), 1.0);
        userValue.rx = std::min<float>(std::max<float>(userValue.rx, -1.0), 1.0);
        return; // 跳过原有键盘逻辑
        }
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        //res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);
        // 设置select为非阻塞（最后一个参数设为NULL→阻塞 → 改为timeval设0→非阻塞）
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10ms检查一次键盘，不阻塞
        res = select( fileno( stdin )+1, &set, NULL, NULL, &tv);
        
        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        
        // ========== 核心修改2：无论是否有键盘输入，都执行ROS指令更新 ==========
        if(use_ros_cmd){ // 如果启用ROS指令模式
            changeValue(); // 持续更新ROS指令到userValue
        }
        usleep(1000);
    }
    return NULL;
}
