#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>

class RosPackageTemplate {
public:
    RosPackageTemplate();
    void update();

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    
    // TF 广播与监听
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener tf_listener_;

    std::vector<control_toolbox::Pid> wheel_pids_; 

    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_omega_ = 0.0;

    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;

    bool use_global_velocity_; // 是否使用世界坐标系速度

    const double R_ = 0.076; 
    const double LX_ = 0.25; 
    const double LY_ = 0.25; 
};
