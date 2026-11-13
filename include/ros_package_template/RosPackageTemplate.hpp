#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include <vector>

class RosPackageTemplate {
public:
    RosPackageTemplate();
    void update();

private:
    ros::NodeHandle nh_;
    // 四个轮子的 PID 控制器
    std::vector<control_toolbox::Pid> wheel_pids_; 
};
