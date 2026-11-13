#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <vector>

class RosPackageTemplate {
public:
    RosPackageTemplate();
    void update();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher pub_;
    
    // 四个轮子的 PID
    std::vector<control_toolbox::Pid> wheel_pids_; 

    // 接收到的底盘速度指令
    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_omega_ = 0.0;

    // 麦克纳姆轮逆运动学底盘参数 (需根据实际车体尺寸修改)
    const double R_ = 0.076; // 轮子半径 (单位: 米)
    const double LX_ = 0.25; // 轮子到底盘中心的纵向距离 (单位: 米)
    const double LY_ = 0.25; // 轮子到底盘中心的横向距离 (单位: 米)
};
