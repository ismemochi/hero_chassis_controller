#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
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
    tf::TransformBroadcaster tf_broadcaster_;

    std::vector<control_toolbox::Pid> wheel_pids_; 

    double target_vx_ = 0.0;
    double target_vy_ = 0.0;
    double target_omega_ = 0.0;

    // 里程计变量
    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;

    const double R_ = 0.076; 
    const double LX_ = 0.25; 
    const double LY_ = 0.25; 
};
