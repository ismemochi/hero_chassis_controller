#include "ros_package_template/RosPackageTemplate.hpp"
#include <tf/transform_datatypes.h>

RosPackageTemplate::RosPackageTemplate() : wheel_pids_(4) {
    for (auto& pid : wheel_pids_) {
        pid.initPid(1.0, 0.0, 0.0, 1.0, -1.0);
    }
    cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &RosPackageTemplate::cmdVelCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
}

void RosPackageTemplate::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    target_vx_ = msg->linear.x;
    target_vy_ = msg->linear.y;
    target_omega_ = msg->angular.z;
}

void RosPackageTemplate::update() {
    double dt = 0.01; 

    // 1. 逆运动学：计算期望轮速
    double w_fl = (target_vx_ - target_vy_ - (LX_ + LY_) * target_omega_) / R_;
    double w_fr = (target_vx_ + target_vy_ + (LX_ + LY_) * target_omega_) / R_;
    double w_rl = (target_vx_ + target_vy_ - (LX_ + LY_) * target_omega_) / R_;
    double w_rr = (target_vx_ - target_vy_ + (LX_ + LY_) * target_omega_) / R_;
    std::vector<double> expected_vels = {w_fl, w_fr, w_rl, w_rr};

    double actual_fl = 0, actual_fr = 0, actual_rl = 0, actual_rr = 0;

    for (size_t i = 0; i < wheel_pids_.size(); ++i) {
        // 为了模拟真实效果，这里将期望速度视为实际反馈速度
        double current_vel = expected_vels[i]; 
        if(i==0) actual_fl = current_vel;
        if(i==1) actual_fr = current_vel;
        if(i==2) actual_rl = current_vel;
        if(i==3) actual_rr = current_vel;
    }

    // 2. 正运动学：根据实际轮速反推底盘速度
    double real_vx = (R_ / 4.0) * (actual_fl + actual_fr + actual_rl + actual_rr);
    double real_vy = (R_ / 4.0) * (-actual_fl + actual_fr + actual_rl - actual_rr);
    double real_omega = (R_ / (4.0 * (LX_ + LY_))) * (-actual_fl + actual_fr - actual_rl + actual_rr);

    // 3. 里程计积分 (积分累加)
    double delta_x = (real_vx * cos(th_) - real_vy * sin(th_)) * dt;
    double delta_y = (real_vx * sin(th_) + real_vy * cos(th_)) * dt;
    double delta_th = real_omega * dt;

    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;

    // 4. 发布 TF 坐标变换 (odom -> base_link)
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tf_broadcaster_.sendTransform(odom_trans);

    // 5. 发布 Odometry 消息
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = real_vx;
    odom.twist.twist.linear.y = real_vy;
    odom.twist.twist.angular.z = real_omega;
    odom_pub_.publish(odom);

    ROS_INFO_THROTTLE(1.0, "Odom -> X:%.2f Y:%.2f Th:%.2f | Vx:%.2f Vy:%.2f W:%.2f", 
        x_, y_, th_, real_vx, real_vy, real_omega);
}
