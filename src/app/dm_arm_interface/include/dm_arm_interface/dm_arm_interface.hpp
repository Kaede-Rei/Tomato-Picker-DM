#ifndef _dm_arm_interface_hpp_
#define _dm_arm_interface_hpp_

#include <memory>
#include <string>
#include <vector>

#include "dm_arm_interface/arm_interface.hpp" // IWYU pragma: keep
#include "dm_arm_interface/eef_interface.hpp" // IWYU pragma: keep

namespace dm_arm {

class TasksManager;
class PickTaskAction;

class EndEffector;

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief ROS 接口开关配置
 */
struct ROSInterfaceSwitch {
    bool enable{ false };
    std::string name;
};

/**
 * @brief ROS 接口启动配置
 */
struct ROSInterfaceConfig {
    std::string arm_group_name{ "arm" };

    bool eef_enabled{ false };
    std::string eef_type{ "" };
    std::string eef_name{ "gripper" };
    std::string eef_serial_port{ "/dev/ttyACM0" };
    int eef_baud_rate{ 115200 };

    ROSInterfaceSwitch arm_move_action{ false, "arm_move_action" };
    ROSInterfaceSwitch simple_arm_move_action{ false, "simple_arm_move_action" };
    ROSInterfaceSwitch arm_config_service{ false, "arm_config_service" };
    ROSInterfaceSwitch arm_query_service{ false, "arm_query_service" };
    ROSInterfaceSwitch pick_action{ true, "pick_action" };

    ROSInterfaceSwitch eef_cmd_service{ false, "eef_cmd_service" };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class ROSInterface {
public:
    /**
     * @brief ROSInterface 构造函数
     * @param nh ROS 节点句柄
     * @param config 接口配置
     */
    ROSInterface(ros::NodeHandle& nh, const ROSInterfaceConfig& config);
    ~ROSInterface() = default;

    ROSInterface(const ROSInterface&) = delete;
    ROSInterface& operator=(const ROSInterface&) = delete;
    ROSInterface(ROSInterface&&) = delete;
    ROSInterface& operator=(ROSInterface&&) = delete;

private:
    /**
     * @brief 初始化所有可选 ROS 接口模块
     * @param nh ROS 节点句柄
     * @param config 接口配置
     */
    void init_interfaces(ros::NodeHandle& nh, const ROSInterfaceConfig& config);
    /**
     * @brief 初始化末端执行器
     * @param nh ROS 节点句柄
     * @param config 接口配置
     */
    void init_eef(ros::NodeHandle& nh, const ROSInterfaceConfig& config);
    template<typename Module, typename... Args>
    void add_interface(const ROSInterfaceSwitch& interface, Args&&... args);

private:
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<EndEffector> _eef_;
    std::shared_ptr<TasksManager> _tasks_manager_;
    std::shared_ptr<ArmCmdDispatcher> _arm_dispatcher_;
    std::shared_ptr<EefCmdDispatcher> _eef_dispatcher_;
    std::shared_ptr<PickTaskAction> _pick_action_server_;
    std::vector<std::unique_ptr<ROSModuleInterface>> _modules_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

template<typename Module, typename... Args>
void ROSInterface::add_interface(const ROSInterfaceSwitch& interface, Args&&... args) {
    if(interface.enable && !interface.name.empty()) {
        _modules_.push_back(std::make_unique<Module>(std::forward<Args>(args)..., interface.name));
        ROS_INFO("初始化接口: %s", interface.name.c_str());
    }
}

}

#endif
