#include "ros_package_template/RosPackageTemplate.hpp"

RosPackageTemplate::RosPackageTemplate() : wheel_pids_(4) {
    for (auto& pid : wheel_pids_) {
        pid.initPid(1.0, 0.0, 0.0, 1.0, -1.0);
    }
    
    // 订阅底盘速度指令
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RosPackageTemplate::cmdVelCallback, this);
    pub_ = nh_.advertise<std_msgs::Float64>("/pid_output", 10);
    
    ROS_INFO("Hero Chassis Controller Initialized with Inverse Kinematics.");
}

void RosPackageTemplate::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    target_vx_ = msg->linear.x;
    target_vy_ = msg->linear.y;
    target_omega_ = msg->angular.z;
}

void RosPackageTemplate::update() {
    double dt = 0.01; 

    // 核心：麦克纳姆轮逆运动学公式
    // 这里的顺序是：左前(FL), 右前(FR), 左后(RL), 右后(RR)
    double w_fl = (target_vx_ - target_vy_ - (LX_ + LY_) * target_omega_) / R_;
    double w_fr = (target_vx_ + target_vy_ + (LX_ + LY_) * target_omega_) / R_;
    double w_rl = (target_vx_ + target_vy_ - (LX_ + LY_) * target_omega_) / R_;
    double w_rr = (target_vx_ - target_vy_ + (LX_ + LY_) * target_omega_) / R_;

    std::vector<double> expected_vels = {w_fl, w_fr, w_rl, w_rr};

    for (size_t i = 0; i < wheel_pids_.size(); ++i) {
        // 假设当前轮子实际速度为 0 (仅为演示)
        double error = expected_vels[i] - 0.0;
        double effort = wheel_pids_[i].computeCommand(error, ros::Duration(dt));
        
        std_msgs::Float64 msg;
        msg.data = effort;
        pub_.publish(msg);
    }

    // 打印出四个轮子的期望转速，用于终端录屏展示
    ROS_INFO_THROTTLE(1.0, "Target Vx:%.2f Vy:%.2f W:%.2f | Wheels(rad/s): FL=%.2f FR=%.2f RL=%.2f RR=%.2f", 
        target_vx_, target_vy_, target_omega_, w_fl, w_fr, w_rl, w_rr);
}
