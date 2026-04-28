#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <signal.h>

#include "dm_ht_controller/dm_hardware_interface.h"

// 全局退出标志
bool g_quit = false;

/**
 * @brief 信号处理函数，设置退出标志
 */
void quitRequested(int sig)
{
    g_quit = true;
    ros::shutdown();
}

/**
 * @brief 主函数，初始化ROS节点，硬件接口和控制器管理器，进入达妙机械臂六轴关节控制循环
 */
int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dm_control_node");
    ros::NodeHandle nh;

    // 设置信号处理
    signal(SIGINT, quitRequested);
    signal(SIGTERM, quitRequested);

    // 创建硬件接口
    DMHardwareInterface robot(nh);

    if(!robot.init()){
        ROS_ERROR("初始化机器人硬件接口失败");
        return -1;
    }

    // 创建控制器管理器
    controller_manager::ControllerManager cm(&robot, nh);

    // 获取控制频率
    double control_frequency;
    nh.param<double>("dm_arm_hardware/control_frequency", control_frequency, 500.0);
    ros::Rate rate(control_frequency);

    ROS_INFO("开始控制循环，频率: %.1f Hz", control_frequency);

    // 异步spinner用于处理service calls等
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time last_time = ros::Time::now();

    while(ros::ok() && !g_quit){
        ros::Time current_time = ros::Time::now();
        ros::Duration period = current_time - last_time;
        last_time = current_time;

        robot.read();
        cm.update(current_time, period);
        robot.write();

        rate.sleep();
    }

    
    spinner.stop();
    ROS_INFO("控制节点正在关闭");

    return 0;
}
