#include "ros_package_template/RosPackageTemplate.hpp"

RosPackageTemplate::RosPackageTemplate() : wheel_pids_(4) {
    // 初始化四个轮子的 PID 参数 (P, I, D, I_max, I_min)
    for (auto& pid : wheel_pids_) {
        pid.initPid(1.0, 0.0, 0.0, 1.0, -1.0);
    }
    ROS_INFO("Hero Chassis Controller Initialized with 4 PIDs.");
}

void RosPackageTemplate::update() {
    double dt = 0.01; // 假设 100Hz
    double expected_vel = 1.0;  // 期望速度 (实际开发中应从 /cmd_vel 获取)
    double current_vel = 0.0;   // 当前速度 (实际开发中应从硬件接口读取)

    for (size_t i = 0; i < wheel_pids_.size(); ++i) {
        double error = expected_vel - current_vel;
        // 核心：使用 control_toolbox::Pid 计算输出
        double effort = wheel_pids_[i].computeCommand(error, ros::Duration(dt));
        // 打印输出用于 rqt_plot 观察
        ROS_INFO_THROTTLE(1.0, "Wheel [%zu] PID Output: %f", i, effort);
    }
}
