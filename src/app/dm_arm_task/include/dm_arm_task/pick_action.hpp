#ifndef _pick_action_hpp_
#define _pick_action_hpp_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>

#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_msgs/PickTaskAction.h"
#include "dm_arm_task/tasks_manager.hpp"

namespace dm_arm {

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 采摘任务 Action 服务端
 */
class PickTaskAction {
public:
    using PickTaskAS = actionlib::SimpleActionServer<dm_arm_msgs::PickTaskAction>;

    /**
     * @brief PickTaskAction 构造函数
     * @param nh ROS 节点句柄
     * @param tasks_manager 任务管理器
     * @param arm 机械臂控制器
     * @param action_name Action 名称
     */
    PickTaskAction(ros::NodeHandle& nh, std::shared_ptr<TasksManager> tasks_manager, std::shared_ptr<ArmController> arm, const std::string& action_name);
    ~PickTaskAction();

    PickTaskAction(const PickTaskAction&) = delete;
    PickTaskAction& operator=(const PickTaskAction&) = delete;
    PickTaskAction(PickTaskAction&&) = delete;
    PickTaskAction& operator=(PickTaskAction&&) = delete;

private:
    /**
     * @brief 处理新的 Goal
     */
    void on_goal();

    /**
     * @brief 处理取消请求
     */
    void on_preempt();

    /**
     * @brief 处理“写入 / 更新任务”请求
     * @param goal Goal 请求
     */
    void handle_upsert_task_request(const dm_arm_msgs::PickTaskGoal& goal);

    /**
     * @brief 处理“执行任务组”请求
     * @param goal Goal 请求
     */
    void handle_execute_task_group_request(const dm_arm_msgs::PickTaskGoal& goal);

    /**
     * @brief 完成“执行任务组”请求的后台执行流程
     * @param goal Goal 请求
     */
    void finish_execute_task_group_request(const dm_arm_msgs::PickTaskGoal& goal);

    /**
     * @brief 处理“仅更新任务组配置”请求
     * @param goal Goal 请求
     */
    void handle_update_task_group_config_request(const dm_arm_msgs::PickTaskGoal& goal);

    /**
     * @brief 从 Goal 中解析任务目标
     * @param goal Goal 请求
     * @param target 输出目标
     * @param target_frame 输出目标所属坐标系
     * @return 错误码
     */
    ErrorCode resolve_goal_target(const dm_arm_msgs::PickTaskGoal& goal, tl::optional<TargetVariant>& target, std::string& target_frame) const;

    /**
     * @brief 从 Goal 中解析采摘参数
     * @param goal Goal 请求
     * @param pick_params 输出采摘参数
     * @return 错误码
     */
    ErrorCode resolve_goal_pick_params(const dm_arm_msgs::PickTaskGoal& goal, PickTaskParams& pick_params) const;

    /**
     * @brief 应用 Goal 中的任务组配置
     * @param goal Goal 请求
     * @param group_name 任务组名称
     * @return 错误码
     */
    ErrorCode apply_goal_task_group_config(const dm_arm_msgs::PickTaskGoal& goal, const std::string& group_name);

    /**
     * @brief 获取当前机械臂位姿
     * @return 当前位姿（base_link）
     */
    geometry_msgs::PoseStamped get_current_pose_stamped() const;

    /**
     * @brief 解析任务组名称
     * @param goal Goal 请求
     * @return 任务组名称
     */
    std::string resolve_group_name(const dm_arm_msgs::PickTaskGoal& goal) const;

    /**
     * @brief 解析任务 ID
     * @param goal Goal 请求
     * @return 任务 ID
     */
    unsigned int resolve_task_id(const dm_arm_msgs::PickTaskGoal& goal);

    /**
     * @brief 解析任务类型
     * @param goal Goal 请求
     * @param task_type 输出任务类型
     * @return 错误码
     */
    ErrorCode resolve_task_type(const dm_arm_msgs::PickTaskGoal& goal, TaskType& task_type) const;

    /**
     * @brief 估算任务组总步骤数量
     * @param group_name 任务组名称
     * @return 预计总步骤数量
     */
    uint32_t estimate_total_steps(const std::string& group_name) const;

private:
    std::unique_ptr<PickTaskAS> _as_;
    std::shared_ptr<TasksManager> _tasks_manager_;
    std::shared_ptr<ArmController> _arm_;

    std::atomic<bool> _cancel_requested_{ false };
    std::atomic<bool> _execute_group_running_{ false };
    std::atomic<unsigned int> _id_seed_{ 1000 };
    std::thread _execute_group_thread_;
    std::mutex _execute_group_mutex_;
};

}  // namespace dm_arm

#endif
