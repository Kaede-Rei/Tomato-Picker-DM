#ifndef _cmd_dispatcher_hpp_
#define _cmd_dispatcher_hpp_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "tl_optional/optional.hpp"

#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/eef_interface.hpp"

namespace dm_arm {

class EndEffector;

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 机械臂命令表，基于 X-Macro 定义，方便维护和扩展，只需修改此处即可
 * @note X(name, handler, has_fb, desc) -> X(命令名称, 处理函数后缀, 是否有反馈, 命令描述)
 * @param HOME 0: 回到初始位置
 * @param MOVE_JOINTS 1: 关节空间运动
 * @param MOVE_TARGET 2: 末端执行器空间运动
 * @param MOVE_TARGET_IN_EEF_FRAME 3: 末端执行器空间运动，目标相对于当前末端执行器位置
 * @param TELESCOPIC_END 4: 伸缩末端
 * @param ROTATE_END 5: 旋转末端
 * @param MOVE_LINE 6: 直线运动
 * @param MOVE_BEZIER 7: 贝塞尔曲线运动
 * @param MOVE_DECARTES 8: 笛卡尔空间运动，路径由多个位姿点组成
 * @param SET_ORIENTATION_CONSTRAINT 9: 设置姿态约束
 * @param SET_POSITION_CONSTRAINT 10: 设置位置约束
 * @param SET_JOINT_CONSTRAINT 11: 设置关节约束
 * @param GET_CURRENT_JOINTS 12: 获取当前关节角
 * @param GET_CURRENT_POSE 13: 获取当前位姿
 * @param MOVE_TO_ZERO 14: 重置到零点
 */
#define DM_ARM_CMD_TABLE \
    X(HOME, home, false, "回到初始位置") \
    X(MOVE_JOINTS, move_joints, true, "关节空间运动") \
    X(MOVE_TARGET, move_target, true, "末端执行器空间运动") \
    X(MOVE_TARGET_IN_EEF_FRAME, move_target_in_eef_frame, true, "末端执行器空间运动，目标相对于当前末端执行器位置") \
    X(TELESCOPIC_END, telescopic_end, true, "伸缩末端") \
    X(ROTATE_END, rotate_end, true, "旋转末端") \
    X(MOVE_LINE, move_line, true, "直线运动") \
    X(MOVE_BEZIER, move_bezier, true, "贝塞尔曲线运动") \
    X(MOVE_DECARTES, move_decartes, true, "笛卡尔空间运动，路径由多个位姿点组成") \
    X(SET_ORIENTATION_CONSTRAINT, set_orientation_constraint, false, "设置姿态约束") \
    X(SET_POSITION_CONSTRAINT, set_position_constraint, false, "设置位置约束") \
    X(SET_JOINT_CONSTRAINT, set_joint_constraint, false, "设置关节约束") \
    X(GET_CURRENT_JOINTS, get_current_joints, false, "获取当前关节角") \
    X(GET_CURRENT_POSE, get_current_pose, false, "获取当前位姿") \
    X(MOVE_TO_ZERO, move_to_zero, true, "重置到零点") \

/**
 * @brief 末端命令表，基于 X-Macro 定义，方便维护和扩展，只需修改此处即可
 * @note X(name, handler, has_fb, desc) -> X(命令名称, 处理函数后缀, 是否有反馈, 命令描述)
 * @param OPEN_GRIPPER 0: 打开夹爪
 * @param CLOSE_GRIPPER 1: 关闭夹爪
 * @param STOP_GRIPPER 2: 停止夹爪
 */
#define DM_ARM_EEF_CMD_TABLE \
    X(OPEN_GRIPPER, open_gripper, false, "打开夹爪") \
    X(CLOSE_GRIPPER, close_gripper, false, "关闭夹爪") \
    X(STOP_GRIPPER, stop_gripper, false, "停止夹爪")

/**
 * @brief 机械臂命令类型枚举（0 ~ MAX-1），每个命令类型对应 ArmCmdDispatcher 中的一个处理函数
 */
#define X(name, handler, has_fb, desc) name,
enum class ArmCmdType {
    DM_ARM_CMD_TABLE
    MAX
};
#undef X

/**
 * @brief 末端命令类型枚举（0 ~ MAX-1），每个命令类型对应 ArmCmdDispatcher 中的一个处理函数
 */
#define X(name, handler, has_fb, desc) name,
enum class EefCmdType {
    DM_ARM_EEF_CMD_TABLE
    MAX
};
#undef X

/**
 * @brief 机械臂命令请求结构体
 * @param type 命令类型
 * @param values 数值参数列表，具体含义根据命令类型而定
 * @param target 目标位姿，MOVE_TARGET 和 MOVE_TARGET_IN_EEF_FRAME 命令使用
 * @param joint_names 关节名称列表，SET_JOINT_CONSTRAINT 命令使用
 * @param joints 关节角列表，MOVE_JOINTS 命令使用
 * @param waypoints 位姿点列表，MOVE_LINE、MOVE_BEZIER 和 MOVE_DECARTES 命令使用
 */
struct ArmCmdRequest {
    ArmCmdType type;
    std::vector<double> values;

    TargetVariant target;
    std::vector<std::string> joint_names;
    std::vector<double> joints;
    std::vector<geometry_msgs::Pose> waypoints;
};

/**
 * @brief 机械臂命令结果结构体
 * @param success 命令是否成功执行
 * @param message 结果消息，成功时为提示信息，失败时为错误描述
 * @param values 数值结果列表，具体含义根据命令类型而定
 * @param error_code 错误码，成功时为 ErrorCode::SUCCESS，失败时为具体错误码
 * @param current_pose 当前位姿，GET_CURRENT_POSE 命令使用
 * @param current_joints 当前关节角列表，GET_CURRENT_JOINTS 命令使用
 */
struct ArmCmdResult {
    bool success{ false };
    std::string message;

    std::vector<double> values;
    ErrorCode error_code{ ErrorCode::SUCCESS };

    tl::optional<geometry_msgs::Pose> current_pose;
    tl::optional<std::vector<double>> current_joints;
};

/**
 * @brief 机械臂命令反馈结构体
 * @param stage 当前执行阶段，例如 "start"、"setting"、"planning"、"done" 等
 * @param progress 当前阶段的进度，范围 [0.0, 1.0]
 * @param message 当前阶段的提示信息
 */
struct ArmCmdFeedback {
    std::string stage;
    double progress{ 0.0 };
    std::string message;
};

/**
 * @brief 末端命令请求结构体
 * @param type 命令类型
 * @param values 数值参数列表，具体含义根据命令类型而定
 */
struct EefCmdRequest {
    EefCmdType type;
    std::vector<double> values;
};

/**
 * @brief 末端命令结果结构体
 * @param success 命令是否成功执行
 * @param message 结果消息，成功时为提示信息，失败时为错误描述
 * @param error_code 错误码，成功时为 ErrorCode::SUCCESS，失败时为具体错误码
 */
struct EefCmdResult {
    bool success{ false };
    std::string message;

    ErrorCode error_code{ ErrorCode::SUCCESS };
};

/**
 * @brief 末端命令反馈结构体
 * @param stage 当前执行阶段，例如 "start"、"setting"、"planning"、"done" 等
 * @param progress 当前阶段的进度，范围 [0.0, 1.0]
 * @param message 当前阶段的提示信息
 */
struct EefCmdFeedback {
    std::string stage;
    double progress{ 0.0 };
    std::string message;
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 机械臂命令分发器类，负责接收命令请求并调用 ArmController 执行相应操作
 */
class ArmCmdDispatcher {
public:
    /**
     * @brief ArmCmdDispatcher 构造函数
     * @param arm 机械臂控制器
     */
    explicit ArmCmdDispatcher(std::shared_ptr<ArmController> arm);
    /**
     * @brief ArmCmdDispatcher 析构函数
     */
    ~ArmCmdDispatcher();

    ArmCmdDispatcher(const ArmCmdDispatcher&) = delete;
    ArmCmdDispatcher& operator=(const ArmCmdDispatcher&) = delete;
    ArmCmdDispatcher(ArmCmdDispatcher&&) = delete;
    ArmCmdDispatcher& operator=(ArmCmdDispatcher&&) = delete;

    using FeedbackCb = std::function<void(const ArmCmdFeedback&)>;
    /**
     * @brief 分发机械臂命令
     * @param req 命令请求
     * @param cb 反馈回调
     * @return 命令结果
     */
    ArmCmdResult dispatch(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
    /**
     * @brief 将机械臂命令类型转换为字符串
     * @param type 命令类型
     * @return 字符串名称
     */
    std::string type_to_string(ArmCmdType type) const;

    /**
     * @brief 取消当前命令
     */
    void cancel();
    /**
     * @brief 查询是否已取消
     * @return 是否已取消
     */
    bool is_cancelled() const;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl_;
};

/**
 * @brief 末端命令分发器类，负责接收命令请求并调用 EndEffector 执行相应操作
 */
class EefCmdDispatcher {
public:
    /**
     * @brief EefCmdDispatcher 构造函数
     * @param eef 末端执行器
     */
    explicit EefCmdDispatcher(std::shared_ptr<EndEffector> eef);
    /**
     * @brief EefCmdDispatcher 析构函数
     */
    ~EefCmdDispatcher();

    EefCmdDispatcher(const EefCmdDispatcher&) = delete;
    EefCmdDispatcher& operator=(const EefCmdDispatcher&) = delete;
    EefCmdDispatcher(EefCmdDispatcher&&) = delete;
    EefCmdDispatcher& operator=(EefCmdDispatcher&&) = delete;

    using FeedbackCb = std::function<void(const EefCmdFeedback&)>;
    /**
     * @brief 分发末端命令
     * @param req 命令请求
     * @param cb 反馈回调
     * @return 命令结果
     */
    EefCmdResult dispatch(const EefCmdRequest& req, FeedbackCb cb = nullptr);
    /**
     * @brief 将末端命令类型转换为字符串
     * @param type 命令类型
     * @return 字符串名称
     */
    std::string type_to_string(EefCmdType type) const;

    /**
     * @brief 取消当前命令
     */
    void cancel();
    /**
     * @brief 查询是否已取消
     * @return 是否已取消
     */
    bool is_cancelled() const;

private:
    struct Impl;
    std::unique_ptr<Impl> _impl_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
