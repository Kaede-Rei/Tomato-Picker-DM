#ifndef _tasks_manager_hpp_
#define _tasks_manager_hpp_

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tl_optional/optional.hpp"
#include "tl_expected/expected.hpp"

#include "dm_arm_controller/types.hpp"
#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/eef_controller.hpp"  // IWYU pragma: keep
#include "dm_arm_acm_guard/acm_guard.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 任务类型枚举
 */
enum class TaskType {
    MOVE_ONLY = 0,
    PICK,
};

/**
 * @brief 任务排序方式枚举
 */
enum class SortType {
    ID,
    DIST,
};

#define PICK_STAGE_TABLE \
    X(IDLE, "空闲") \
    X(START, "开始") \
    X(MOVE_TO_PRE_PICK, "移动到预采摘位") \
    X(APPROACH_PICK, "直线接近采摘位") \
    X(PICKING, "采摘中") \
    X(RETREAT_FROM_PICK, "直线退出采摘位") \
    X(MOVE_TO_PLACE, "移动到放置位") \
    X(PLACING, "放置中") \
    X(FINISH, "完成") \
    X(GO_HOME, "回到初始位") \
    X(CANCELED, "已取消") \
    X(FAILED, "失败")

#define X(name, desc) PICK_##name,
enum class PickStage {
    PICK_STAGE_TABLE
};
#undef X

/**
 * @brief 单果采摘任务参数
 */
struct PickTaskParams {
    /// @brief 是否启用放置动作
    bool use_place_pose{ false };
    /// @brief 放置目标（设置时已冻结到底座坐标系）
    tl::optional<TargetVariant> place_target;
    /// @brief 放置目标所属坐标系（冻结后固定为 base_link）
    std::string place_frame{ "base_link" };

    /// @brief 是否执行夹爪动作
    bool use_eef{ true };
    /// @brief 取消后是否自动回到安全位
    bool go_safe_after_cancel{ true };
    /// @brief 最大重试次数（不含第一次执行）
    uint8_t retry_times{ 0 };

    /// @brief 是否使用预采摘位
    bool use_pre_pick{ true };
    /// @brief 预采摘位相对于采摘位的偏移距离（tcp 坐标系下的 z 轴偏移，单位：米）
    double pre_pick_offset{ 0.1 };
    /// @brief 退出位相对于采摘位的偏移（tcp 坐标系下的 z 轴偏移，单位：米）
    double retreat_offset{ 0.1 };
    /// @brief 直线运动时的步进距离（单位：米）
    double cartesian_eef_step{ 0.01 };
    /// @brief 直线运动的速度缩放（0-1）
    double cartesian_vel_scale{ 0.05 };
    /// @brief 直线运动的加速度缩放（0-1）
    double cartesian_acc_scale{ 0.05 };

    /// @brief 当前阶段
    PickStage current_stage{ PickStage::PICK_IDLE };
    /// @brief 是否已完成
    bool completed{ false };
    /// @brief 是否已取消
    bool canceled{ false };
};

/**
 * @brief 单个任务描述结构体
 */
struct Task {
    /// @brief 任务 ID
    unsigned int id{};
    /// @brief 任务描述
    std::string desc;
    /// @brief 任务类型
    TaskType type{ TaskType::MOVE_ONLY };

    /// @brief 任务目标（设置时已冻结到底座坐标系）
    tl::optional<TargetVariant> target;
    /// @brief 任务目标所属坐标系（冻结后固定为 base_link）
    std::string target_frame{ "base_link" };

    /// @brief 采摘参数（仅 PICK 任务使用）
    tl::optional<PickTaskParams> pick_params;
};

/**
 * @brief 任务组结构体
 */
struct TaskGroup {
    /// @brief 原始任务集合，key 为任务 ID
    std::map<unsigned int, Task> tasks;
    /// @brief 排序后的任务序列
    std::vector<Task> sorted_tasks;
    /// @brief 任务结束后是否回到初始位
    bool go_home_after_finish{ false };

    /// @brief 排序方式
    SortType sort_type{ SortType::ID };
    /// @brief 姿态权重，仅 DIST 排序方式使用
    float weight_orient{ 0.3f };
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class TasksManager {
public:
    /**
     * @brief 执行上下文
     */
    struct ExecutionContext {
        /// @brief 取消标志
        std::atomic<bool>* cancel_requested{ nullptr };
        /// @brief 反馈回调
        std::function<void(const Task& task, PickStage stage, bool last_success, ErrorCode code, const std::string& text)> feedback_cb;
    };

public:
    TasksManager(std::shared_ptr<ArmController> arm, std::shared_ptr<EndEffector> eef);
    ~TasksManager() = default;

    TasksManager(const TasksManager&) = delete;
    TasksManager& operator=(const TasksManager&) = delete;
    TasksManager(TasksManager&&) = delete;
    TasksManager& operator=(TasksManager&&) = delete;

    ErrorCode create_task_group(const std::string& group_name, SortType sort_type = SortType::ID);
    ErrorCode delete_task_group(const std::string& group_name);
    ErrorCode clear_task_group(const std::string& group_name);
    ErrorCode execute_task_group(const std::string& group_name);
    ErrorCode execute_task_group(const std::string& group_name, ExecutionContext* ctx);

    uint32_t estimate_task_group_steps(const std::string& group_name) const;
    ErrorCode set_dist_sort_weight_orient(const std::string& group_name, float weight_orient);
    ErrorCode set_task_group_sort_type(const std::string& group_name, SortType sort_type);
    ErrorCode set_task_group_go_home_after_finish(const std::string& group_name, bool go_home_after_finish);

    ErrorCode add_task(const std::string& group_name, unsigned int id, TaskType task_type = TaskType::MOVE_ONLY, const std::string& task_description = "");
    ErrorCode delete_task(const std::string& group_name, unsigned int id);
    ErrorCode set_task_target(const std::string& group_name, unsigned int id, const tl::optional<TargetVariant>& target, const std::string& target_frame = "");
    ErrorCode set_task_pick_params(const std::string& group_name, unsigned int id, const PickTaskParams& pick_params);
    ErrorCode set_task_group_pick_execution_behavior(const std::string& group_name, bool use_eef, bool go_safe_after_cancel, uint8_t retry_times);
    ErrorCode execute_task(const std::string& group_name, unsigned int id);
    ErrorCode execute_task(const std::string& group_name, unsigned int id, ExecutionContext* ctx);
    ErrorCode execute_task(Task& task);
    ErrorCode execute_task(Task& task, ExecutionContext* ctx);

private:
    tl::expected<TaskGroup*, ErrorCode> find_task_group(const std::string& group_name);
    tl::expected<Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id);
    tl::expected<const Task*, ErrorCode> find_task(const std::string& group_name, unsigned int id) const;

    ErrorCode sort_tasks(TaskGroup& task_group);
    double calculate_dist(const TargetVariant& base, const TargetVariant& target, float weight_orient = 0.3f);
    void optimize_with_2opt(std::vector<Task>& path, float weight_orient = 0.3f);

    bool is_cancel_requested(Task& task, ExecutionContext* ctx);
    void report_pick_feedback(Task& task, PickStage stage, bool ok, ErrorCode code, const std::string& text, ExecutionContext* ctx);

    ErrorCode get_frozen_task_target_in_base(const Task& task, TargetVariant& target_in_base) const;
    ErrorCode get_frozen_place_target_in_base(const Task& task, TargetVariant& place_in_base) const;
    ErrorCode freeze_target_to_base(const TargetVariant& target, const std::string& target_frame, TargetVariant& frozen_target, std::string& frozen_frame) const;
    ErrorCode freeze_target_to_base(const tl::optional<TargetVariant>& target, const std::string& target_frame, tl::optional<TargetVariant>& frozen_target, std::string& frozen_frame) const;

    uint32_t estimate_task_steps(const Task& task) const;
    ErrorCode execute_pick_task(Task& task);
    ErrorCode execute_pick_task(Task& task, ExecutionContext* ctx);

private:
    /// @brief 机械臂控制器
    std::shared_ptr<ArmController> _arm_;
    /// @brief 末端执行器
    std::shared_ptr<EndEffector> _eef_;
    /// @brief 机械臂名称
    std::string _arm_name_;
    /// @brief 末端执行器名称
    std::string _eef_name_;

    /// @brief 任务组集合
    std::map<std::string, TaskGroup> _task_groups_;

    AcmGuard _acm_guard_;
};

} /* namespace dm_arm */

#endif
