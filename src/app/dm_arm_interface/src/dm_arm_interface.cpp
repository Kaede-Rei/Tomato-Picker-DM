#include "dm_arm_interface/dm_arm_interface.hpp"

#include "dm_arm_controller/eef_controller.hpp"
#include "dm_arm_task/pick_action.hpp"
#include "dm_arm_task/tasks_manager.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

ROSInterface::ROSInterface(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    _arm_ = std::make_shared<ArmController>(config.arm_group_name);
    init_eef(nh, config);
    _tasks_manager_ = std::make_shared<TasksManager>(_arm_, _eef_);
    _arm_dispatcher_ = std::make_shared<ArmCmdDispatcher>(_arm_);
    if(_eef_) {
        _eef_dispatcher_ = std::make_shared<EefCmdDispatcher>(_eef_);
    }
    init_interfaces(nh, config);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

void ROSInterface::init_eef(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    if(!config.eef_enabled) {
        ROS_INFO("EEF 未启用，跳过初始化");
        return;
    }

    try {
        if(config.eef_type == "two_finger_gripper") {
            auto eef = std::make_shared<TwoFingerGripper>(config.eef_name);
            _eef_ = eef;
        }
        else if(config.eef_type == "servo_gripper") {
            auto eef = std::make_shared<ServoGripper>(
                nh,
                config.eef_serial_port,
                config.eef_baud_rate);
            _eef_ = eef;
        }
        // TODO: CAN 末端执行器暂时直接用 urdf
        // else if(config.eef_type == "dm_arm_can_gripper") {
        //     auto eef = std::make_shared<DmArmCanGripper>(nh);
        //     _eef_ = eef;
        // }
        else {
            ROS_WARN("未知 EEF 类型: %s，跳过初始化", config.eef_type.c_str());
            return;
        }

        ROS_INFO("EEF 已初始化: type=%s, name=%s", config.eef_type.c_str(), _eef_->get_eef_name().c_str());
    }
    catch(const std::exception& e) {
        ROS_ERROR("EEF 初始化失败: %s", e.what());
        _eef_.reset();
    }
}

void ROSInterface::init_interfaces(ros::NodeHandle& nh, const ROSInterfaceConfig& config) {
    add_interface<ArmMoveAction>(config.arm_move_action, nh, _arm_, _arm_dispatcher_);
    add_interface<SimpleArmMoveAction>(config.simple_arm_move_action, nh, _arm_, _arm_dispatcher_);
    add_interface<ArmConfigService>(config.arm_config_service, nh, _arm_, _arm_dispatcher_);
    add_interface<ArmQueryService>(config.arm_query_service, nh, _arm_, _arm_dispatcher_);

    add_interface<EefCmdService>(config.eef_cmd_service, nh, _eef_, _eef_dispatcher_);

    if(config.pick_action.enable && !config.pick_action.name.empty()) {
        _pick_action_server_ = std::make_shared<PickTaskAction>(nh, _tasks_manager_, _arm_, config.pick_action.name);
        ROS_INFO("初始化接口: %s", config.pick_action.name.c_str());
    }
}

}
