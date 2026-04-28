#ifndef _arm_interface_hpp_
#define _arm_interface_hpp_

#include <memory>
#include <string>

#include <actionlib/server/simple_action_server.h>

#include "tl_optional/optional.hpp"

#include "dm_arm_interface/interface_module.hpp"
#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_commander/cmd_dispatcher.hpp"
#include "dm_arm_msgs/MoveArmAction.h"
#include "dm_arm_msgs/SimpleMoveArmAction.h"
#include "dm_arm_msgs/ConfigArm.h"
#include "dm_arm_msgs/QueryArm.h"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 机械臂运动的Action接口
 */
class ArmMoveAction : public ROSModuleInterface {
public:
    using MoveArmAS = actionlib::SimpleActionServer<dm_arm_msgs::MoveArmAction>;
    /**
     * @brief ArmMoveAction 构造函数
     * @param nh ROS 节点句柄
     * @param arm 机械臂控制器
     * @param dispatcher 命令分发器
     * @param action_name Action 名称
     */
    ArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name);
    /**
     * @brief ArmMoveAction 析构函数
     */
    ~ArmMoveAction() = default;

    ArmMoveAction(const ArmMoveAction&) = delete;
    ArmMoveAction& operator=(const ArmMoveAction&) = delete;
    ArmMoveAction(ArmMoveAction&&) = delete;
    ArmMoveAction& operator=(ArmMoveAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    tl::optional<ArmCmdRequest> convert_goal_to_request(const dm_arm_msgs::MoveArmGoal& goal);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

/**
 * @brief 简化版机械臂运动的Action接口
 */
class SimpleArmMoveAction : public ROSModuleInterface {
public:
    using MoveArmAS = actionlib::SimpleActionServer<dm_arm_msgs::SimpleMoveArmAction>;
    /**
     * @brief SimpleArmMoveAction 构造函数
     * @param nh ROS 节点句柄
     * @param arm 机械臂控制器
     * @param dispatcher 命令分发器
     * @param action_name Action 名称
     */
    SimpleArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name);
    /**
     * @brief SimpleArmMoveAction 析构函数
     */
    ~SimpleArmMoveAction() = default;

    SimpleArmMoveAction(const SimpleArmMoveAction&) = delete;
    SimpleArmMoveAction& operator=(const SimpleArmMoveAction&) = delete;
    SimpleArmMoveAction(SimpleArmMoveAction&&) = delete;
    SimpleArmMoveAction& operator=(SimpleArmMoveAction&&) = delete;

private:
    void on_goal();
    void on_preempt();
    tl::optional<ArmCmdRequest> convert_goal_to_request(const dm_arm_msgs::SimpleMoveArmGoal& goal);

private:
    std::unique_ptr<MoveArmAS> _as_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

/**
 * @brief 机械臂配置的Service接口
 */
class ArmConfigService : public ROSModuleInterface {
public:
    /**
     * @brief ArmConfigService 构造函数
     * @param nh ROS 节点句柄
     * @param arm 机械臂控制器
     * @param dispatcher 命令分发器
     * @param service_name Service 名称
     */
    ArmConfigService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name);
    /**
     * @brief ArmConfigService 析构函数
     */
    ~ArmConfigService() = default;

    ArmConfigService(const ArmConfigService&) = delete;
    ArmConfigService& operator=(const ArmConfigService&) = delete;
    ArmConfigService(ArmConfigService&&) = delete;
    ArmConfigService& operator=(ArmConfigService&&) = delete;

private:
    bool on_request(dm_arm_msgs::ConfigArm::Request& req, dm_arm_msgs::ConfigArm::Response& res);
    tl::optional<ArmCmdRequest> convert_srvreq_to_armreq(const dm_arm_msgs::ConfigArm::Request& srv_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

/**
 * @brief 机械臂查询的Service接口
 */
class ArmQueryService : public ROSModuleInterface {
public:
    /**
     * @brief ArmQueryService 构造函数
     * @param nh ROS 节点句柄
     * @param arm 机械臂控制器
     * @param dispatcher 命令分发器
     * @param service_name Service 名称
     */
    ArmQueryService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name);
    /**
     * @brief ArmQueryService 析构函数
     */
    ~ArmQueryService() = default;

    ArmQueryService(const ArmQueryService&) = delete;
    ArmQueryService& operator=(const ArmQueryService&) = delete;
    ArmQueryService(ArmQueryService&&) = delete;
    ArmQueryService& operator=(ArmQueryService&&) = delete;

private:
    bool on_request(dm_arm_msgs::QueryArm::Request& req, dm_arm_msgs::QueryArm::Response& res);
    tl::optional<ArmCmdRequest> convert_srvreq_to_armreq(const dm_arm_msgs::QueryArm::Request& srv_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<ArmController> _arm_;
    std::shared_ptr<ArmCmdDispatcher> _dispatcher_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
