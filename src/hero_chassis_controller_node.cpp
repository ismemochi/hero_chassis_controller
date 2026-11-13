#include <ros/ros.h>
#include "ros_package_template/RosPackageTemplate.hpp"

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "hero_chassis_controller_node");
    ros::NodeHandle nodeHandle;
    ros::Rate loop_rate(100); // 100Hz 运行频率

    // 实例化我们的控制器类（去掉了旧的命名空间）
    RosPackageTemplate controller;

    while (ros::ok()) {
        // 调用我们刚才写好的 PID update 函数
        controller.update();
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
