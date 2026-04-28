#ifndef _arm_controller_hpp_
#define _arm_controller_hpp_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <string>
#include <thread>
#include <mutex>
#include <type_traits>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/Constraints.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include "dm_arm_controller/types.hpp"
#include "tl_optional/optional.hpp"

namespace dm_arm {

using CancelChecker = std::function<bool()>;

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //

/**
 * @brief 位姿可达性搜索返回状态
 * @param SOLUTION_FOUND 找到精确可达解
 * @param APPROXIMATE_SOLUTION_FOUND 找到近似可达解
 * @param SOLUTION_NOT_FOUND 未找到可达解
 */
enum class SearchReachablePose {
    SOLUTION_FOUND = 0,
    APPROXIMATE_SOLUTION_FOUND,
    SOLUTION_NOT_FOUND
};

struct ReachablePoseResult {
    SearchReachablePose state{ SearchReachablePose::SOLUTION_NOT_FOUND };
    double score{ -1.0 };
    std::vector<double> reachable_joints;
    geometry_msgs::Pose reachable_pose;
};

/**
 * @brief A* 搜索节点
 * @param g_cost 从起点到当前节点的累计代价
 * @param h_cost 从当前节点到目标节点的启发式代价
 * @param pose 当前节点对应的末端位姿
 * @param joint_positions 当前节点对应的关节角解
 * @param depth 当前搜索深度
 */
struct AStarNode {
    double g_cost;
    double h_cost;
    geometry_msgs::Pose pose;
    std::vector<double> joint_positions;
    int depth;
};

/**
 * @brief A* 优先队列比较器（总代价小者优先）
 * @param a 节点 a
 * @param b 节点 b
 * @return 若 a 的总代价大于 b 则返回 true
 */
struct CompareAStarNode {
    bool operator()(const AStarNode& a, const AStarNode& b) {
        return (a.g_cost + a.h_cost) > (b.g_cost + b.h_cost);
    }
};

// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

/**
 * @brief 机械臂控制器类，负责目标设置、运动规划、笛卡尔轨迹与约束管理
 */
class ArmController {
public:
    /**
     * @brief ArmController 构造函数
     * @param group_name 机械臂规划组名称
     */
    explicit ArmController(const std::string& group_name = "arm");

    /**
     * @brief 析构函数
     */
    ~ArmController();

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;
    ArmController(ArmController&&) = delete;
    ArmController& operator=(ArmController&&) = delete;

    /**
     * @brief 回到初始位姿
     * @return 错误码
     */
    ErrorCode home();
    /**
     * @brief 设置关节目标
     * @param joint_values 关节角列表
     * @return 错误码
     */
    ErrorCode set_joints(const std::vector<double>& joint_values);
    /**
     * @brief 设置底座坐标系下的目标
     * @param target 目标位姿/位置/姿态
     * @return 错误码
     */
    ErrorCode set_target(const TargetVariant& target);
    /**
     * @brief 设置末端坐标系下的目标
     * @param target 目标位姿/位置/姿态
     * @return 错误码
     */
    ErrorCode set_target_in_eef_frame(const TargetVariant& target);
    /**
     * @brief 清空当前目标
     */
    void clear_target();

    /**
     * @brief 设置伸缩末端目标
     * @param length 伸缩长度
     * @return 错误码
     */
    ErrorCode telescopic_end(double length);
    /**
     * @brief 设置旋转末端目标
     * @param angle 旋转角度
     * @return 错误码
     */
    ErrorCode rotate_end(double angle);

    /**
     * @brief 规划当前目标
     * @param plan 规划输出
     * @return 错误码
     */
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan);
    /**
     * @brief 执行规划结果
     * @param plan 规划输入
     * @return 错误码
     */
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan);
    /**
     * @brief 规划并执行当前目标
     * @return 错误码
     */
    ErrorCode plan_and_execute();
    /**
     * @brief 异步规划并执行当前目标
     * @param callback 完成回调
     * @return 错误码
     */
    ErrorCode async_plan_and_execute(std::function<void(ErrorCode)> callback = nullptr);

    /**
     * @brief 带取消检查的规划
     * @param plan 规划输出
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode plan_checked(moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel);
    /**
     * @brief 带取消检查的执行
     * @param plan 规划输入
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode execute_checked(const moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel);
    /**
     * @brief 带取消检查的规划并执行
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode plan_and_execute_checked(CancelChecker should_cancel);
    /**
     * @brief 带取消检查的异步规划并执行
     * @param should_cancel 取消检查函数
     * @param callback 完成回调
     * @return 错误码
     */
    ErrorCode async_plan_and_execute(CancelChecker should_cancel, std::function<void(ErrorCode)> callback = nullptr);
    /**
     * @brief 带取消检查的目标规划
     * @param target 目标
     * @param plan 规划输出
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode plan_target_checked(const TargetVariant& target, moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel);
    /**
     * @brief 带取消检查的目标规划并执行
     * @param target 目标
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode plan_target_and_execute_checked(const TargetVariant& target, CancelChecker should_cancel);
    /**
     * @brief 停止当前运动
     */
    void stop();

    /**
     * @brief 对轨迹做时间参数化
     * @param trajectory 轨迹
     * @param method 时间参数化方法
     * @param vel_scale 速度缩放
     * @param acc_scale 加速度缩放
     * @return 错误码
     */
    ErrorCode parameterize_time(moveit_msgs::RobotTrajectory& trajectory, TimeParamMethod method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    /**
     * @brief 规划笛卡尔轨迹
     * @param waypoints 关键点
     * @param eef_step 末端步长
     * @param time_param_method 时间参数化方法
     * @param vel_scale 速度缩放
     * @param acc_scale 加速度缩放
     * @return 笛卡尔规划结果
     */
    DescartesResult plan_decartes(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step = 0.01, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    /**
     * @brief 规划直线路径
     * @param start 起点
     * @param end 终点
     * @param eef_step 末端步长
     * @param time_param_method 时间参数化方法
     * @param vel_scale 速度缩放
     * @param acc_scale 加速度缩放
     * @return 笛卡尔规划结果
     */
    DescartesResult set_line(const TargetVariant& start, const TargetVariant& end, double eef_step = 0.01, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    /**
     * @brief 规划贝塞尔曲线路径
     * @param start 起点
     * @param via 中间点
     * @param end 终点
     * @param curve_segments 曲线分段数
     * @param eef_step 末端步长
     * @param time_param_method 时间参数化方法
     * @param vel_scale 速度缩放
     * @param acc_scale 加速度缩放
     * @return 笛卡尔规划结果
     */
    DescartesResult set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments = 30, double eef_step = 0.01, TimeParamMethod time_param_method = TimeParamMethod::TOTG, double vel_scale = 0.1, double acc_scale = 0.1);
    /**
     * @brief 执行轨迹
     * @param trajectory 轨迹
     * @return 错误码
     */
    ErrorCode execute(const moveit_msgs::RobotTrajectory& trajectory);
    /**
     * @brief 异步执行轨迹
     * @param trajectory 轨迹
     * @param callback 完成回调
     * @return 错误码
     */
    ErrorCode async_execute(const moveit_msgs::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback = nullptr);
    /**
     * @brief 带取消检查的轨迹执行
     * @param trajectory 轨迹
     * @param should_cancel 取消检查函数
     * @return 错误码
     */
    ErrorCode execute_trajectory_checked(const moveit_msgs::RobotTrajectory& trajectory, CancelChecker should_cancel);
    /**
     * @brief 带取消检查的异步轨迹执行
     * @param trajectory 轨迹
     * @param should_cancel 取消检查函数
     * @param callback 完成回调
     * @return 错误码
     */
    ErrorCode async_execute(const moveit_msgs::RobotTrajectory& trajectory, CancelChecker should_cancel, std::function<void(ErrorCode)> callback = nullptr);

    /**
     * @brief 设置姿态约束
     */
    void set_orientation_constraint(const geometry_msgs::Quaternion& target_orientation, double tolerance_x = 0.1, double tolerance_y = 0.1, double tolerance_z = 0.3, double weight = 1.0);
    /**
     * @brief 设置位置约束
     */
    void set_position_constraint(const geometry_msgs::Point& target_position, const geometry_msgs::Vector3& scope_size, double weight = 1.0);
    /**
     * @brief 设置关节约束
     */
    void set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight = 1.0);
    /**
     * @brief 应用当前约束
     */
    void apply_constraints();
    /**
     * @brief 清空当前约束
     */
    void clear_constraints();

    /**
     * @brief 查询是否处于规划或执行状态
     * @return 是否忙碌
     */
    bool is_planning_or_executing() const;
    /**
     * @brief 查询是否已请求取消
     * @return 是否已请求取消
     */
    bool cancel_requested() const;
    /**
     * @brief 请求取消当前任务
     */
    void request_cancel();
    /**
     * @brief 取消异步任务
     * @return 错误码
     */
    ErrorCode cancel_async();

    /**
     * @brief 在当前姿态基础上做相对 RPY 旋转并转换为四元数
     * @param q_in 输入四元数
     * @param roll 旋转角
     * @param pitch 旋转角
     * @param yaw 旋转角
     * @return 结果四元数
     */
    geometry_msgs::Quaternion rotate_relative_rpy_to_quaternion(const geometry_msgs::Quaternion& q_in, double roll, double pitch, double yaw);
    /**
     * @brief RPY 转四元数
     * @param roll 旋转角
     * @param pitch 旋转角
     * @param yaw 旋转角
     * @return 四元数
     */
    geometry_msgs::Quaternion rpy_to_quaternion(double roll, double pitch, double yaw);
    /**
     * @brief RPY 和位置转 Pose
     * @param roll 旋转角
     * @param pitch 旋转角
     * @param yaw 旋转角
     * @param x 位置 X
     * @param y 位置 Y
     * @param z 位置 Z
     * @return 位姿
     */
    geometry_msgs::Pose rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z);
    template<class T>
    ErrorCode base_to_end_tf(const T& in, T& out);
    template<class T>
    ErrorCode end_to_base_tf(const T& in, T& out);
    /**
     * @brief 将目标解析到底座坐标系
     * @param target 输入目标
     * @param target_in_base 输出目标
     * @param source_frame 输出源坐标系
     * @return 错误码
     */
    ErrorCode resolve_target_to_base(const TargetVariant& target, TargetVariant& target_in_base, std::string* source_frame = nullptr);
    /**
     * @brief 将带默认坐标系的目标解析到底座坐标系
     * @param target 输入目标
     * @param default_source_frame 默认源坐标系
     * @param target_in_base 输出目标
     * @param source_frame 输出源坐标系
     * @return 错误码
     */
    ErrorCode resolve_target_to_base(const TargetVariant& target, const std::string& default_source_frame, TargetVariant& target_in_base, std::string* source_frame = nullptr);
    /**
     * @brief 将 PoseStamped 转到底座坐标系
     * @param pose_in 输入位姿
     * @param pose_out 输出位姿
     * @return 错误码
     */
    ErrorCode transform_pose_to_base(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped& pose_out);
    /**
     * @brief 按指定源坐标系设置目标
     * @param target 输入目标
     * @param source_frame 源坐标系
     * @return 错误码
     */
    ErrorCode set_target_in_frame(const TargetVariant& target, const std::string& source_frame);
    const std::string& get_base_link() const { return _base_link_; }
    const std::string& get_eef_link() const { return _eef_link_; }

    /**
     * @brief 获取机械臂名称
     * @return 机械臂名称
     */
    const std::string& get_arm_name() const;
    /**
     * @brief 获取当前关节角
     * @return 当前关节角列表
     */
    std::vector<double> get_current_joints() const;
    /**
     * @brief 获取当前连杆名称
     * @return 连杆名称列表
     */
    std::vector<std::string> get_current_link_names() const;
    /**
     * @brief 获取当前末端位姿
     * @return 当前位姿
     */
    geometry_msgs::Pose get_current_pose() const;

    /**
     * @brief 将关节值全部归零并执行
     * @return 错误码
     */
    ErrorCode reset_to_zero();

private:
    tl::optional<geometry_msgs::Pose> extract_pose_from_target(const TargetVariant& target) const;
    tl::optional<ReachablePoseResult> search_reachable_pose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose);

    using AsyncWork = std::function<ErrorCode()>;
    bool should_cancel(const CancelChecker& should_cancel) const;
    void reset_cancel_state();
    ErrorCode start_async_work(AsyncWork work, std::function<void(ErrorCode)> callback = nullptr);
    ErrorCode wait_async_result(CancelChecker should_cancel);
    ErrorCode run_async_and_wait(AsyncWork work, CancelChecker should_cancel);

    ErrorCode plan_impl(moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel);
    ErrorCode execute_plan_impl(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel);
    ErrorCode plan_and_execute_impl(const CancelChecker& should_cancel);
    ErrorCode execute_trajectory_impl(const moveit_msgs::RobotTrajectory& trajectory, const CancelChecker& should_cancel);
    ErrorCode plan_target_impl(const TargetVariant& target, moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel);
    ErrorCode plan_target_and_execute_impl(const TargetVariant& target, const CancelChecker& should_cancel);

private:
    /// @brief MoveGroupInterface 对象
    moveit::planning_interface::MoveGroupInterface _arm_;

    /// @brief TF 缓冲区
    tf2_ros::Buffer _tf_buffer_;
    /// @brief TF 监听器
    tf2_ros::TransformListener _tf_listener_;

    /// @brief 机械臂关节模型组指针
    const robot_state::JointModelGroup* _jmg_{ nullptr };
    /// @brief 当前机械臂状态指针
    robot_state::RobotStatePtr _current_state_{ nullptr };

    /// @brief 规划基坐标系
    const std::string _base_link_;
    /// @brief 末端坐标系
    const std::string _eef_link_;

    /// @brief 笛卡尔轨迹速度缩放
    double _vel_scale_;
    /// @brief 笛卡尔轨迹加速度缩放
    double _acc_scale_;

    /// @brief 笛卡尔步长
    double _eef_step_;
    /// @brief 关节跳跃阈值
    double _jump_threshold_;
    /// @brief 笛卡尔最小成功率
    double _min_success_rate_;

    /// @brief 约束集合
    moveit_msgs::Constraints _constraints_;

    /// @brief 是否处于异步执行状态
    std::atomic<bool> _is_planning_or_executing_{ false };
    /// @brief 是否请求取消
    std::atomic<bool> _cancel_requested_{ false };
    /// @brief 异步任务是否完成
    std::atomic<bool> _async_done_{ true };
    /// @brief 异步任务结果
    std::atomic<int> _async_result_{ static_cast<int>(ErrorCode::SUCCESS) };
    /// @brief 异步状态锁
    mutable std::mutex _async_mutex_;
    /// @brief 异步完成条件变量
    std::condition_variable _async_cv_;
    /// @brief 异步线程
    std::thread _async_thread_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //

/**
 * @brief 将输入从基坐标系变换到末端坐标系
 */
template<class T>
ErrorCode ArmController::base_to_end_tf(const T& in, T& out) {
    static_assert(
        std::is_same_v<T, geometry_msgs::Pose> ||
        std::is_same_v<T, geometry_msgs::Point> ||
        std::is_same_v<T, geometry_msgs::Quaternion> ||
        std::is_same_v<T, geometry_msgs::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        if constexpr(std::is_same_v<T, geometry_msgs::PoseStamped>) {
            auto tf_stamped = _tf_buffer_.lookupTransform(_eef_link_, _base_link_, in.header.stamp, ros::Duration(0.2));
            tf2::doTransform(in, out, tf_stamped);
        }
        else {
            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = _base_link_;
            pose_in.header.stamp = ros::Time::now();

            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            geometry_msgs::PoseStamped pose_out;
            pose_out = _tf_buffer_.transform(pose_in, _eef_link_, ros::Duration(0.2));
            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

/**
 * @brief 将输入从末端坐标系变换到底座坐标系
 */
template<class T>
ErrorCode ArmController::end_to_base_tf(const T& in, T& out) {
    static_assert(
        std::is_same_v<T, geometry_msgs::Pose> ||
        std::is_same_v<T, geometry_msgs::Point> ||
        std::is_same_v<T, geometry_msgs::Quaternion> ||
        std::is_same_v<T, geometry_msgs::PoseStamped>,
        "仅支持 Pose(Stamped)、Point 和 Quaternion 的坐标变换");

    try {
        if constexpr(std::is_same_v<T, geometry_msgs::PoseStamped>) {
            auto tf_stamped = _tf_buffer_.lookupTransform(_base_link_, _eef_link_, in.header.stamp, ros::Duration(0.2));
            tf2::doTransform(in, out, tf_stamped);
        }
        else {
            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = _eef_link_;
            pose_in.header.stamp = ros::Time::now();

            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                pose_in.pose = in;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                pose_in.pose.position = in;
                pose_in.pose.orientation.w = 1.0;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                pose_in.pose.orientation = in;
            }

            geometry_msgs::PoseStamped pose_out;
            pose_out = _tf_buffer_.transform(pose_in, _base_link_, ros::Duration(0.2));
            if constexpr(std::is_same_v<T, geometry_msgs::Pose>) {
                out = pose_out.pose;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Point>) {
                out = pose_out.pose.position;
            }
            else if constexpr(std::is_same_v<T, geometry_msgs::Quaternion>) {
                out = pose_out.pose.orientation;
            }
        }
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

} /* namespace dm_arm */

#endif
