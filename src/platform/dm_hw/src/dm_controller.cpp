#include <atomic>
#include <clocale>
#include <csignal>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

#include "dm_hw/dm_hardware_interface.hpp"

namespace {

std::atomic_bool g_quit{ false };

void quit_requested(int) {
    g_quit.store(true);
}

} // namespace

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dm_controller");
    ros::NodeHandle nh;

    std::signal(SIGINT, quit_requested);
    std::signal(SIGTERM, quit_requested);

    dm_hw::DmHardwareInterface robot(nh);
    if(!robot.init()) {
        ROS_ERROR("dm_hw 硬件接口初始化失败");
        return 1;
    }

    controller_manager::ControllerManager cm(&robot, nh);
    ros::ServiceServer shutdown_service = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(
        "/dm_hw/shutdown",
        [&robot](std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
            robot.shutdown(false);
            res.success = true;
            res.message = "dm_hw hardware disabled";
            return true;
        });
    ros::Rate rate(robot.control_frequency());
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("dm_hw 控制循环启动");
    ros::Time last_time = ros::Time::now();
    while(ros::ok() && !g_quit.load()) {
        const ros::Time current_time = ros::Time::now();
        const ros::Duration period = current_time - last_time;
        last_time = current_time;

        robot.read();
        cm.update(current_time, period);
        robot.write();
        rate.sleep();
    }

    spinner.stop();
    ROS_INFO("dm_hw 控制节点退出，进入硬件接口析构流程");
    return 0;
}
