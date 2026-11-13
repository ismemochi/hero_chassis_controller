#include "ros_package_template/RosPackageTemplate.hpp"
#include <tf/transform_datatypes.h>

RosPackageTemplate::RosPackageTemplate() : wheel_pids_(4) {
    for (auto& pid : wheel_pids_) {
        pid.initPid(1.0, 0.0, 0.0, 1.0, -1.0);
    }
    // 读取配置文件中的模式开关
    nh_.param<bool>("use_global_velocity", use_global_velocity_, true);

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

    // === 核心：TF 坐标变换 ===
    double final_vx = target_vx_;
    double final_vy = target_vy_;

    if (use_global_velocity_) {
        geometry_msgs::Vector3Stamped v_in, v_out;
        v_in.header.frame_id = "odom"; // 将指令视为世界坐标系
        v_in.header.stamp = ros::Time(0); // 最新时间
        v_in.vector.x = target_vx_;
        v_in.vector.y = target_vy_;

        try {
            // 把世界坐标系下的速度向量，转换到 base_link 坐标系
            tf_listener_.transformVector("base_link", v_in, v_out);
            final_vx = v_out.vector.x;
            final_vy = v_out.vector.y;
        } catch (tf::TransformException &ex) {
            // 如果此时还没有 TF 树（比如刚开始），使用当前角度手动旋转
            final_vx = target_vx_ * cos(th_) + target_vy_ * sin(th_);
            final_vy = -target_vx_ * sin(th_) + target_vy_ * cos(th_);
        }
    }

    // 1. 逆运动学 (使用转换后的 final_vx, final_vy)
    double w_fl = (final_vx - final_vy - (LX_ + LY_) * target_omega_) / R_;
    double w_fr = (final_vx + final_vy + (LX_ + LY_) * target_omega_) / R_;
    double w_rl = (final_vx + final_vy - (LX_ + LY_) * target_omega_) / R_;
    double w_rr = (final_vx - final_vy + (LX_ + LY_) * target_omega_) / R_;
    
    // 2. 正运动学 (计算真实速度)
    double real_vx = (R_ / 4.0) * (w_fl + w_fr + w_rl + w_rr);
    double real_vy = (R_ / 4.0) * (-w_fl + w_fr + w_rl - w_rr);
    double real_omega = (R_ / (4.0 * (LX_ + LY_))) * (-w_fl + w_fr - w_rl + w_rr);

    // 3. 里程计积分
    x_ += (real_vx * cos(th_) - real_vy * sin(th_)) * dt;
    y_ += (real_vx * sin(th_) + real_vy * cos(th_)) * dt;
    th_ += real_omega * dt;

    // 4. 发布 TF 和 Odometry
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

    ROS_INFO_THROTTLE(1.0, "Mode:%s | Odom X:%.2f Y:%.2f Th:%.2f", 
        use_global_velocity_ ? "GLOBAL" : "LOCAL", x_, y_, th_);
}
