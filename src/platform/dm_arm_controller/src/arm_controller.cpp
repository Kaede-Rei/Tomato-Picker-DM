#include "dm_arm_controller/arm_controller.hpp"

#include <cmath>
#include <cstdint>
#include <queue>
#include <sstream>
#include <unordered_map>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <shape_msgs/SolidPrimitive.h>
#include <tf2/LinearMath/Quaternion.h>

namespace dm_arm {

// ! ========================= 私 有 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 归一化输入目标为 PoseStamped 以便用于后续处理
 * @param target 输入目标
 * @param default_source_frame 当目标不带 frame 信息时使用的默认坐标系
 * @return 归一化后的 PoseStamped，若无法归一化则返回空值
 */
tl::optional<geometry_msgs::PoseStamped> normalize_target_to_pose_stamped(const TargetVariant& target, const std::string& default_source_frame) {

    return std::visit(variant_visitor{
        [&](const std::monostate&) -> tl::optional<geometry_msgs::PoseStamped> {
            return tl::nullopt;
        },
        [&](const geometry_msgs::Pose& pose) -> tl::optional<geometry_msgs::PoseStamped> {
            if(default_source_frame.empty()) return tl::nullopt;
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = default_source_frame;
            ps.header.stamp = ros::Time(0);
            ps.pose = pose;
            return ps;
        },
        [&](const geometry_msgs::Point& point) -> tl::optional<geometry_msgs::PoseStamped> {
            if(default_source_frame.empty()) return tl::nullopt;
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = default_source_frame;
            ps.header.stamp = ros::Time(0);
            ps.pose.position = point;
            ps.pose.orientation.w = 1.0;
            return ps;
        },
        [&](const geometry_msgs::Quaternion& quat) -> tl::optional<geometry_msgs::PoseStamped> {
            if(default_source_frame.empty()) return tl::nullopt;
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = default_source_frame;
            ps.header.stamp = ros::Time(0);
            ps.pose.orientation = quat;
            return ps;
        },
        [&](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<geometry_msgs::PoseStamped> {
            if(pose_stamped.header.frame_id.empty() && default_source_frame.empty()) {
                return tl::nullopt;
            }
            geometry_msgs::PoseStamped ps = pose_stamped;
            if(ps.header.frame_id.empty()) {
                ps.header.frame_id = default_source_frame;
            }
            if(ps.header.stamp == ros::Time(0)) {
                ps.header.stamp = ros::Time(0);
            }
            return ps;
        }
        }, target);
}

}

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief ArmController 构造函数：初始化 MoveGroupInterface、TF 与参数
 * @param group_name 机械臂规划组名称
 */
ArmController::ArmController(const std::string& group_name)
    : _arm_(group_name),
    _tf_listener_(_tf_buffer_),
    _base_link_(_arm_.getPlanningFrame()),
    _eef_link_(_arm_.getEndEffectorLink()) {

    ROS_INFO("Planning Frame - %s 已创建", _base_link_.c_str());
    ROS_INFO("End Effector Link - %s 已创建", _eef_link_.c_str());

    ros::NodeHandle pnh("~");

    pnh.param("decartes/vel_scale", _vel_scale_, 0.1);
    pnh.param("decartes/acc_scale", _acc_scale_, 0.1);
    pnh.param("decartes/eef_step", _eef_step_, 0.01);
    pnh.param("decartes/jump_threshold", _jump_threshold_, 0.0);
    pnh.param("decartes/min_success_rate", _min_success_rate_, 0.8);

    double planning_time = 5.0;
    int planning_attempts = 10;
    double motion_vel_scale = 0.1;
    double motion_acc_scale = 0.1;
    std::string planner_id = "RRTConnect";

    pnh.param("motion_planning/planning_time", planning_time, 5.0);
    pnh.param("motion_planning/planning_attempts", planning_attempts, 10);
    pnh.param("motion_planning/max_velocity_scaling_factor", motion_vel_scale, 0.1);
    pnh.param("motion_planning/max_acceleration_scaling_factor", motion_acc_scale, 0.1);
    pnh.param("motion_planning/planner_id", planner_id, std::string("RRTConnect"));

    _arm_.setPlanningTime(planning_time);
    _arm_.setNumPlanningAttempts(planning_attempts);
    _arm_.setMaxVelocityScalingFactor(motion_vel_scale);
    _arm_.setMaxAccelerationScalingFactor(motion_acc_scale);
    _arm_.setPlannerId(planner_id);

    ros::Duration(1.0).sleep();

    try {
        if(_tf_buffer_.canTransform(_base_link_, _eef_link_, ros::Time(0), ros::Duration(1.0))) {
            ROS_INFO("TF 变换可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
        else {
            ROS_WARN("TF 变换不可用：%s -> %s", _base_link_.c_str(), _eef_link_.c_str());
        }
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("检查 TF 变换时发生异常：%s", e.what());
    }
}

/**
 * @brief 析构函数：等待异步线程退出
 */
ArmController::~ArmController() {
    request_cancel();
    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }
}

/**
 * @brief 回到命名位姿 home
 */
ErrorCode ArmController::home() {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法执行 home");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ROS_INFO("将机械臂复位到命名位姿 zero");

    const bool target_ok = _arm_.setNamedTarget("zero");
    if(!target_ok) {
        ROS_WARN("设置命名位姿 zero 失败");
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    const moveit::core::MoveItErrorCode err_code = _arm_.move();
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行 home 失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行 home 成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置目标关节值
 * @param joint_values 关节值向量
 * @return 错误码
 */
ErrorCode ArmController::set_joints(const std::vector<double>& joint_values) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }
    bool success = _arm_.setJointValueTarget(joint_values);
    return success ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
}

/**
 * @brief 设置末端目标（底座坐标系），Pose 目标优先尝试 A* 可达性搜索
 * @param target 目标位姿/位置/姿态
 * @return 错误码
 */
ErrorCode ArmController::set_target(const TargetVariant& target) {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    ErrorCode result = ErrorCode::INVALID_TARGET_TYPE;

    std::visit(variant_visitor{
        [this, &result](const std::monostate&) {
            ROS_WARN("目标未设置");
            result = ErrorCode::INVALID_TARGET_TYPE;
        },
        [this, &result](const geometry_msgs::Pose& pose) {
            geometry_msgs::Pose current_pose = this->_arm_.getCurrentPose().pose;
            const auto reachable_result = search_reachable_pose(current_pose, pose);

            if(reachable_result &&
                (reachable_result->state == SearchReachablePose::SOLUTION_FOUND ||
                reachable_result->state == SearchReachablePose::APPROXIMATE_SOLUTION_FOUND) &&
               !reachable_result->reachable_joints.empty()) {
                const bool ok = this->_arm_.setJointValueTarget(reachable_result->reachable_joints);
                ROS_INFO("设置目标位姿（IK/A*关节解）是否成功：%s", ok ? "是" : "否");
                result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
                return;
            }

            ROS_INFO("未找到可达关节解，回退到 MoveIt PoseTarget 规划");
            const bool ok = this->_arm_.setPoseTarget(pose);
            ROS_INFO("设置目标位姿（PoseTarget）是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::Point& point) {
            const bool ok = this->_arm_.setPositionTarget(point.x, point.y, point.z);
            ROS_INFO("设置目标位置是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::Quaternion& quat) {
            const bool ok = this->_arm_.setOrientationTarget(quat.x, quat.y, quat.z, quat.w);
            ROS_INFO("设置目标姿态是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        },
        [this, &result](const geometry_msgs::PoseStamped& pose_stamped) {
            const bool ok = this->_arm_.setPoseTarget(pose_stamped);
            ROS_INFO("设置目标位姿（带时间戳）是否成功：%s", ok ? "是" : "否");
            result = ok ? ErrorCode::SUCCESS : ErrorCode::TARGET_OUT_OF_BOUNDS;
        }
        }, target);

    return result;
}

/**
 * @brief 设置末端目标（末端坐标系）
 * @param target 目标位姿/位置/姿态
 * @return 错误码
 */
ErrorCode ArmController::set_target_in_eef_frame(const TargetVariant& target) {
    return set_target_in_frame(target, _eef_link_);
}

/**
 * @brief 清空当前目标
 */
void ArmController::clear_target() {
    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法清除目标");
        return;
    }
    _arm_.clearPoseTargets();
    ROS_INFO("已清除所有目标位姿");
}

/**
 * @brief 末端伸缩设置
 * @param length 伸缩长度（米）
 * @return 错误码
 */
ErrorCode ArmController::telescopic_end(double length) {
    geometry_msgs::Pose point;
    point.position.x = 0.0;
    point.position.y = 0.0;
    point.position.z = length;
    point.orientation = rpy_to_quaternion(0.0, 0.0, 0.0);

    return set_target_in_eef_frame(point);
}

/**
 * @brief 末端旋转设置
 * @param angle 旋转角（弧度）
 * @return 错误码
 */
ErrorCode ArmController::rotate_end(double angle) {
    geometry_msgs::Pose pose;
    pose = rpy_to_pose(0.0, 0.0, angle, 0.0, 0.0, 0.0);

    return set_target_in_eef_frame(pose);
}

/**
 * @brief 执行路径规划
 * @param plan 规划输出
 * @return 错误码
 */
ErrorCode ArmController::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    return plan_checked(plan, CancelChecker{});
}

/**
 * @brief 执行规划结果
 * @param plan 规划输入
 * @return 错误码
 */
ErrorCode ArmController::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    return execute_checked(plan, CancelChecker{});
}

/**
 * @brief 规划并执行
 * @return 错误码
 */
ErrorCode ArmController::plan_and_execute() {
    return plan_and_execute_checked(CancelChecker{});
}

/**
 * @brief 异步规划并执行
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::async_plan_and_execute(std::function<void(ErrorCode)> callback) {
    return async_plan_and_execute(CancelChecker{}, std::move(callback));
}

/**
 * @brief 带取消检查的规划
 * @param plan 规划输出
 * @param should_cancel 取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_checked(moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel) {
    return run_async_and_wait([this, &plan, should_cancel]() {
        return plan_impl(plan, should_cancel);
        }, should_cancel);
}

/**
 * @brief 带取消检查的执行
 * @param plan 规划输入
 * @param should_cancel 取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::execute_checked(const moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel) {
    return run_async_and_wait([this, &plan, should_cancel]() {
        return execute_plan_impl(plan, should_cancel);
        }, should_cancel);
}

/**
 * @brief 带取消检查的规划并执行
 * @param should_cancel 取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_and_execute_checked(CancelChecker should_cancel) {
    return run_async_and_wait([this, should_cancel]() {
        return plan_and_execute_impl(should_cancel);
        }, should_cancel);
}

/**
 * @brief 带取消检查的异步规划并执行
 * @param should_cancel 取消检查函数
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::async_plan_and_execute(CancelChecker should_cancel, std::function<void(ErrorCode)> callback) {
    return start_async_work([this, should_cancel]() {
        return plan_and_execute_impl(should_cancel);
        }, std::move(callback));
}

/**
 * @brief 停止当前运动
 */
void ArmController::stop() {
    _arm_.stop();
    ROS_INFO("已停止当前运动");
}

/**
 * @brief 对轨迹做时间参数化
 * @param trajectory 轨迹输入输出
 * @param method 时间参数化方法
 * @param vel_scale 速度缩放
 * @param acc_scale 加速度缩放
 * @return 错误码
 */
ErrorCode ArmController::parameterize_time(moveit_msgs::RobotTrajectory& trajectory, TimeParamMethod method, double vel_scale, double acc_scale) {
    robot_trajectory::RobotTrajectory rt(_arm_.getRobotModel(), _arm_.getName());
    rt.setRobotTrajectoryMsg(*_arm_.getCurrentState(), trajectory);

    bool time_param_success = false;
    if(method == TimeParamMethod::TOTG) {
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        time_param_success = totg.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else if(method == TimeParamMethod::ISP) {
        trajectory_processing::IterativeSplineParameterization isp;
        time_param_success = isp.computeTimeStamps(rt, vel_scale, acc_scale);
    }
    else {
        ROS_WARN("无效的时间参数化方法");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    if(!time_param_success) {
        ROS_WARN("时间参数化失败");
        return ErrorCode::TIME_PARAM_FAILED;
    }

    rt.getRobotTrajectoryMsg(trajectory);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 笛卡尔轨迹规划
 * @param waypoints 路径点
 * @param eef_step 末端步长
 * @param time_param_method 时间参数化方法
 * @param vel_scale 速度缩放
 * @param acc_scale 加速度缩放
 * @return 规划结果
 */
DescartesResult ArmController::plan_decartes(const std::vector<geometry_msgs::Pose>& waypoints, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0;
    result.message.clear();
    result.trajectory = moveit_msgs::RobotTrajectory();

    if(_is_planning_or_executing_) {
        result.error_code = ErrorCode::ASYNC_TASK_RUNNING;
        result.message = "当前已有异步任务正在执行，无法进行新的笛卡尔规划";
        return result;
    }

    if(waypoints.empty()) {
        result.error_code = ErrorCode::EMPTY_WAYPOINTS;
        result.message = "路径点列表为空";
        return result;
    }

    if(eef_step <= 0.0) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "eef_step 必须大于 0";
        return result;
    }

    if(vel_scale <= 0.0 || vel_scale > 1.0 || acc_scale <= 0.0 || acc_scale > 1.0) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "速度/加速度缩放必须在 (0, 1] 范围内";
        return result;
    }

    moveit_msgs::RobotTrajectory trajectory;
    const double success_rate = _arm_.computeCartesianPath(
        waypoints,
        eef_step,
        trajectory);

    result.success_rate = success_rate;

    if(success_rate <= 0.0) {
        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;
        result.message = "笛卡尔路径规划失败，无法生成有效轨迹";
        return result;
    }

    if(success_rate < _min_success_rate_) {
        std::stringstream ss;
        ss << "笛卡尔路径规划成功率不足："
            << (success_rate * 100.0)
            << "%，低于阈值 "
            << (_min_success_rate_ * 100.0)
            << "%";
        result.error_code = ErrorCode::DESCARTES_PLANNING_FAILED;
        result.message = ss.str();
        return result;
    }

    const ErrorCode time_code = parameterize_time(
        trajectory, time_param_method, vel_scale, acc_scale);
    if(time_code != ErrorCode::SUCCESS) {
        result.error_code = time_code;
        result.message = "时间参数化失败";
        return result;
    }

    result.trajectory = trajectory;
    result.error_code = ErrorCode::SUCCESS;

    std::stringstream ss;
    ss << "笛卡尔路径规划成功，成功率：" << (success_rate * 100.0) << "%";
    result.message = ss.str();

    return result;
}

/**
 * @brief 规划直线路径
 * @return 规划结果
 */
DescartesResult ArmController::set_line(const TargetVariant& start, const TargetVariant& end, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0f;
    result.message.clear();

    const auto start_pose = extract_pose_from_target(start);
    const auto end_pose = extract_pose_from_target(end);

    if(!start_pose || !end_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "无法从目标中提取位姿";
        return result;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(*start_pose);
    waypoints.push_back(*end_pose);

    return plan_decartes(waypoints, eef_step, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 规划贝塞尔曲线路径
 * @return 规划结果
 */
DescartesResult ArmController::set_bezier_curve(const TargetVariant& start, const TargetVariant& via, const TargetVariant& end, int curve_segments, double eef_step, TimeParamMethod time_param_method, double vel_scale, double acc_scale) {
    DescartesResult result;
    result.error_code = ErrorCode::FAILURE;
    result.success_rate = 0.0;
    result.message.clear();

    if(curve_segments < 1) {
        result.error_code = ErrorCode::INVALID_PARAMETER;
        result.message = "curve_segments 必须大于等于 1";
        return result;
    }

    const auto start_pose = extract_pose_from_target(start);
    if(!start_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线起点无效";
        return result;
    }

    const auto via_pose = extract_pose_from_target(via);
    if(!via_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线控制点无效";
        return result;
    }

    const auto end_pose = extract_pose_from_target(end);
    if(!end_pose) {
        result.error_code = ErrorCode::INVALID_TARGET_TYPE;
        result.message = "贝塞尔曲线终点无效";
        return result;
    }

    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.reserve(static_cast<std::size_t>(curve_segments + 1));

    for(int i = 0; i <= curve_segments; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(curve_segments);

        geometry_msgs::Pose point;
        point.position.x =
            (1.0 - t) * (1.0 - t) * start_pose->position.x +
            2.0 * (1.0 - t) * t * via_pose->position.x +
            t * t * end_pose->position.x;
        point.position.y =
            (1.0 - t) * (1.0 - t) * start_pose->position.y +
            2.0 * (1.0 - t) * t * via_pose->position.y +
            t * t * end_pose->position.y;
        point.position.z =
            (1.0 - t) * (1.0 - t) * start_pose->position.z +
            2.0 * (1.0 - t) * t * via_pose->position.z +
            t * t * end_pose->position.z;

        tf2::Quaternion quat_start, quat_end, quat_interp;
        tf2::fromMsg(start_pose->orientation, quat_start);
        tf2::fromMsg(end_pose->orientation, quat_end);
        quat_interp = quat_start.slerp(quat_end, t);
        quat_interp.normalize();
        point.orientation = tf2::toMsg(quat_interp);

        waypoints.push_back(point);
    }

    return plan_decartes(waypoints, eef_step, time_param_method, vel_scale, acc_scale);
}

/**
 * @brief 执行轨迹消息
 * @param trajectory 轨迹
 * @return 错误码
 */
ErrorCode ArmController::execute(const moveit_msgs::RobotTrajectory& trajectory) {
    return execute_trajectory_checked(trajectory, CancelChecker{});
}

/**
 * @brief 异步执行轨迹消息
 * @param trajectory 轨迹
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::async_execute(const moveit_msgs::RobotTrajectory& trajectory, std::function<void(ErrorCode)> callback) {
    return async_execute(trajectory, CancelChecker{}, std::move(callback));
}

ErrorCode ArmController::execute_trajectory_checked(const moveit_msgs::RobotTrajectory& trajectory, CancelChecker should_cancel) {
    return run_async_and_wait([this, trajectory, should_cancel]() {
        return execute_trajectory_impl(trajectory, should_cancel);
        }, should_cancel);
}

ErrorCode ArmController::async_execute(const moveit_msgs::RobotTrajectory& trajectory, CancelChecker should_cancel, std::function<void(ErrorCode)> callback) {
    return start_async_work([this, trajectory, should_cancel]() {
        return execute_trajectory_impl(trajectory, should_cancel);
        }, std::move(callback));
}

/**
 * @brief 添加姿态约束
 * @param target_orientation 目标姿态四元数
 * @param tolerance_x X轴容忍度（弧度）
 * @param tolerance_y Y轴容忍度（弧度）
 * @param tolerance_z Z轴容忍度（弧度）
 * @param weight 约束权重（0-1）
 */
void ArmController::set_orientation_constraint(const geometry_msgs::Quaternion& target_orientation, double tolerance_x, double tolerance_y, double tolerance_z, double weight) {
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = _eef_link_;
    ocm.header.frame_id = _base_link_;
    ocm.header.stamp = ros::Time::now();

    ocm.orientation = target_orientation;
    ocm.absolute_x_axis_tolerance = tolerance_x;
    ocm.absolute_y_axis_tolerance = tolerance_y;
    ocm.absolute_z_axis_tolerance = tolerance_z;
    ocm.weight = weight;

    _constraints_.orientation_constraints.push_back(ocm);
    ROS_INFO("已设置末端执行器姿态约束，目标姿态：(%f, %f, %f, %f)，容忍度：(%f, %f, %f)，权重：%f",
        target_orientation.x, target_orientation.y, target_orientation.z, target_orientation.w,
        tolerance_x, tolerance_y, tolerance_z, weight);
}

/**
 * @brief 添加位置约束
 * @param target_position 目标位置
 * @param scope_size 约束范围大小（长宽高）
 * @param weight 约束权重（0-1）
 */
void ArmController::set_position_constraint(const geometry_msgs::Point& target_position, const geometry_msgs::Vector3& scope_size, double weight) {
    moveit_msgs::PositionConstraint pcm;
    pcm.link_name = _eef_link_;
    pcm.header.frame_id = _base_link_;
    pcm.header.stamp = ros::Time::now();

    pcm.target_point_offset.x = 0.0;
    pcm.target_point_offset.y = 0.0;
    pcm.target_point_offset.z = 0.0;

    shape_msgs::SolidPrimitive bounding_volume;
    bounding_volume.type = shape_msgs::SolidPrimitive::BOX;
    bounding_volume.dimensions.resize(3);
    bounding_volume.dimensions[0] = scope_size.x;
    bounding_volume.dimensions[1] = scope_size.y;
    bounding_volume.dimensions[2] = scope_size.z;

    geometry_msgs::Pose region_pose;
    region_pose.position = target_position;
    region_pose.orientation.w = 1.0;

    pcm.constraint_region.primitives.clear();
    pcm.constraint_region.primitive_poses.clear();
    pcm.constraint_region.primitives.push_back(bounding_volume);
    pcm.constraint_region.primitive_poses.push_back(region_pose);
    pcm.weight = weight;

    _constraints_.position_constraints.push_back(pcm);

    ROS_INFO("已设置末端执行器位置约束，目标位置：(%f, %f, %f)，范围大小：(%f, %f, %f)，权重：%f",
        target_position.x, target_position.y, target_position.z,
        scope_size.x, scope_size.y, scope_size.z, weight);
}
/**
 * @brief 添加关节约束
 * @param joint_name 关节名称
 * @param target_angle 目标角度（弧度）
 * @param above 上容忍度（弧度）
 * @param below 下容忍度（弧度）
 * @param weight 约束权重（0-1）
 */
void ArmController::set_joint_constraint(const std::string& joint_name, double target_angle, double above, double below, double weight) {
    moveit_msgs::JointConstraint jc;
    jc.joint_name = joint_name;
    jc.position = target_angle;
    jc.tolerance_above = above;
    jc.tolerance_below = below;
    jc.weight = weight;

    _constraints_.joint_constraints.push_back(jc);
    ROS_INFO("已设置末端执行器关节约束，关节名称：%s，目标角度：%f，上容忍度：%f，下容忍度：%f，权重：%f",
        joint_name.c_str(), target_angle, above, below, weight);
}

/**
 * @brief 应用约束
 */
void ArmController::apply_constraints() {
    _arm_.setPathConstraints(_constraints_);
    ROS_INFO("已应用所有姿态约束");
}

/**
 * @brief 清空约束
 */
void ArmController::clear_constraints() {
    _constraints_ = moveit_msgs::Constraints();
    _arm_.clearPathConstraints();
    ROS_INFO("已清除所有姿态约束");
}

/**
 * @brief 查询异步任务状态
 */
bool ArmController::is_planning_or_executing() const {
    return _is_planning_or_executing_.load();
}

/**
 * @brief 查询取消状态
 */
bool ArmController::cancel_requested() const {
    return _cancel_requested_.load();
}

/**
 * @brief 请求取消当前异步任务
 */
void ArmController::request_cancel() {
    _cancel_requested_.store(true);
    _arm_.stop();
}

/**
 * @brief 取消异步任务
 * @return 错误码
 */
ErrorCode ArmController::cancel_async() {
    request_cancel();
    return ErrorCode::SUCCESS;
}

/**
 * @brief 在当前姿态基础上相对旋转 RPY 角度并转换为四元数（底座坐标系）
 * @param q_in 当前姿态四元数
 * @param roll 旋转角度（弧度）
 * @param pitch 旋转角度（弧度）
 * @param yaw 旋转角度（弧度）
 * @return 旋转后的姿态四元数
 */
geometry_msgs::Quaternion ArmController::rotate_relative_rpy_to_quaternion(const geometry_msgs::Quaternion& q_in, double roll, double pitch, double yaw) {
    tf2::Quaternion q_in_tf2, q_relative, q_out;
    tf2::fromMsg(q_in, q_in_tf2);
    q_relative.setRPY(roll, pitch, yaw);
    q_out = q_in_tf2 * q_relative;
    q_out.normalize();

    return tf2::toMsg(q_out);
}

/**
 * @brief RPY 转四元数
 * @param roll 旋转角度（弧度）
 * @param pitch 旋转角度（弧度）
 * @param yaw 旋转角度（弧度）
 * @return 姿态四元数
 */
geometry_msgs::Quaternion ArmController::rpy_to_quaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = quat.x();
    quat_msg.y = quat.y();
    quat_msg.z = quat.z();
    quat_msg.w = quat.w();

    return quat_msg;
}

/**
 * @brief RPY + 平移 转 Pose
 * @param roll 旋转角度（弧度）
 * @param pitch 旋转角度（弧度）
 * @param yaw 旋转角度（弧度）
 * @param x 平移（米）
 * @param y 平移（米）
 * @param z 平移（米）
 * @return 位姿
 */
geometry_msgs::Pose ArmController::rpy_to_pose(double roll, double pitch, double yaw, double x, double y, double z) {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = rpy_to_quaternion(roll, pitch, yaw);

    return pose;
}

/**
 * @brief 将 PoseStamped 从任意源坐标系转换到底座坐标系
 * @param pose_in 输入位姿
 * @param pose_out 输出位姿（底座坐标系）
 * @return 错误码
 */
ErrorCode ArmController::transform_pose_to_base(const geometry_msgs::PoseStamped& pose_in, geometry_msgs::PoseStamped& pose_out) {
    geometry_msgs::PoseStamped normalized = pose_in;
    if(normalized.header.frame_id.empty()) {
        ROS_WARN("输入位姿未指定坐标系，无法进行坐标变换");
        return ErrorCode::INVALID_TARGET_TYPE;
    }
    if(normalized.header.stamp == ros::Time()) {
        normalized.header.stamp = ros::Time(0);
    }

    if(normalized.header.frame_id == _base_link_) {
        pose_out = normalized;
        pose_out.header.frame_id = _base_link_;
        return ErrorCode::SUCCESS;
    }

    try {
        pose_out = _tf_buffer_.transform(normalized, _base_link_, ros::Duration(0.2));
        pose_out.header.frame_id = _base_link_;
        return ErrorCode::SUCCESS;
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("坐标变换失败：%s", e.what());
        return ErrorCode::TF_TRANSFORM_FAILED;
    }
}

/**
 * @brief 将 TargetVariant 解析并转换为底座坐标系下的目标
 * @param target 输入目标
 * @param target_in_base 输出目标（底座坐标系）
 * @param source_frame 输出源坐标系
 * @return 错误码
 * @note 对于不带 frame 的 Pose / Point / Quaternion，默认认为其坐标系为末端坐标系
 */
ErrorCode ArmController::resolve_target_to_base(const TargetVariant& target, TargetVariant& target_in_base, std::string* source_frame) {
    return resolve_target_to_base(target, _eef_link_, target_in_base, source_frame);
}

/**
 * @brief 将 TargetVariant 按指定源坐标系解析并转换为底座坐标系下的目标
 * @param target 输入目标
 * @param default_source_frame 当目标本身不带 frame 信息时使用的默认源坐标系
 * @param target_in_base 输出目标（底座坐标系）
 * @param source_frame 输出实际使用的源坐标系
 * @return 错误码
 */
ErrorCode ArmController::resolve_target_to_base(const TargetVariant& target, const std::string& default_source_frame, TargetVariant& target_in_base, std::string* source_frame) {

    auto normalize_pose_stamped = [&](const geometry_msgs::PoseStamped& in_pose,
        geometry_msgs::PoseStamped& normalized_pose,
        std::string& used_frame) -> ErrorCode {
            normalized_pose = in_pose;
            used_frame = in_pose.header.frame_id.empty() ? default_source_frame : in_pose.header.frame_id;
            if(used_frame.empty()) {
                ROS_WARN("目标位姿未指定坐标系，且默认源坐标系也未设置，无法解析目标");
                return ErrorCode::INVALID_TARGET_TYPE;
            }
            normalized_pose.header.frame_id = used_frame;
            if(normalized_pose.header.stamp == ros::Time()) {
                normalized_pose.header.stamp = ros::Time(0);
            }
            return ErrorCode::SUCCESS;
        };

    return std::visit(variant_visitor{
        [&](const std::monostate&) -> ErrorCode {
            return ErrorCode::INVALID_TARGET_TYPE;
        },
        [&](const geometry_msgs::Pose& pose) -> ErrorCode {
            const std::string used_frame = default_source_frame;
            if(used_frame.empty()) {
                ROS_WARN("目标位姿未指定坐标系，且默认源坐标系也未设置，无法解析目标");
                return ErrorCode::INVALID_TARGET_TYPE;
            }
            if(source_frame) *source_frame = used_frame;

            if(used_frame == _base_link_) {
                target_in_base = pose;
                return ErrorCode::SUCCESS;
            }

            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = used_frame;
            pose_in.header.stamp = ros::Time(0);
            pose_in.pose = pose;

            geometry_msgs::PoseStamped pose_out;
            const ErrorCode code = transform_pose_to_base(pose_in, pose_out);
            if(code != ErrorCode::SUCCESS) return code;
            target_in_base = pose_out.pose;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::Point& point) -> ErrorCode {
            const std::string used_frame = default_source_frame;
            if(used_frame.empty()) {
                ROS_WARN("目标位置未指定坐标系，且默认源坐标系也未设置，无法解析目标");
                return ErrorCode::INVALID_TARGET_TYPE;
            }
            if(source_frame) *source_frame = used_frame;

            if(used_frame == _base_link_) {
                target_in_base = point;
                return ErrorCode::SUCCESS;
            }

            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = used_frame;
            pose_in.header.stamp = ros::Time(0);
            pose_in.pose.position = point;
            pose_in.pose.orientation.w = 1.0;

            geometry_msgs::PoseStamped pose_out;
            const ErrorCode code = transform_pose_to_base(pose_in, pose_out);
            if(code != ErrorCode::SUCCESS) return code;
            target_in_base = pose_out.pose.position;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::Quaternion& quat) -> ErrorCode {
            const std::string used_frame = default_source_frame;
            if(used_frame.empty()) {
                ROS_WARN("目标方向未指定坐标系，且默认源坐标系也未设置，无法解析目标");
                return ErrorCode::INVALID_TARGET_TYPE;
            }
            if(source_frame) *source_frame = used_frame;

            if(used_frame == _base_link_) {
                target_in_base = quat;
                return ErrorCode::SUCCESS;
            }

            geometry_msgs::PoseStamped pose_in;
            pose_in.header.frame_id = used_frame;
            pose_in.header.stamp = ros::Time(0);
            pose_in.pose.orientation = quat;

            geometry_msgs::PoseStamped pose_out;
            const ErrorCode code = transform_pose_to_base(pose_in, pose_out);
            if(code != ErrorCode::SUCCESS) return code;
            target_in_base = pose_out.pose.orientation;
            return ErrorCode::SUCCESS;
        },
        [&](const geometry_msgs::PoseStamped& pose_stamped) -> ErrorCode {
            geometry_msgs::PoseStamped normalized;
            std::string used_frame;
            const ErrorCode code = normalize_pose_stamped(pose_stamped, normalized, used_frame);
            if(code != ErrorCode::SUCCESS) return code;
            if(source_frame) *source_frame = used_frame;

            geometry_msgs::PoseStamped transformed;
            const ErrorCode tf_code = transform_pose_to_base(normalized, transformed);
            if(tf_code != ErrorCode::SUCCESS) return tf_code;
            target_in_base = transformed;
            return ErrorCode::SUCCESS;
        }
        }, target);
}

/**
 * @brief 按指定源坐标系设置目标
 * @param target 输入目标
 * @param source_frame 当目标不包含坐标系信息时使用的默认源坐标系
 * @return 错误码
 */
ErrorCode ArmController::set_target_in_frame(const TargetVariant& target, const std::string& source_frame) {

    if(_is_planning_or_executing_) {
        ROS_WARN("当前已有异步任务正在执行，无法设置新目标");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    TargetVariant base_pose;
    ErrorCode code = resolve_target_to_base(target, source_frame, base_pose, nullptr);
    if(code != ErrorCode::SUCCESS) {
        return code;
    }

    return set_target(base_pose);
}

/**
 * @brief 获取规划组名称
 */
const std::string& ArmController::get_arm_name() const {
    return _arm_.getName();
}

/**
 * @brief 获取当前关节值
 */
std::vector<double> ArmController::get_current_joints() const {
    return _arm_.getCurrentJointValues();
}

/**
 * @brief 获取当前末端位姿
 */
geometry_msgs::Pose ArmController::get_current_pose() const {
    return _arm_.getCurrentPose().pose;
}

/**
 * @brief 获取当前连杆名称
 */
std::vector<std::string> ArmController::get_current_link_names() const {
    return _arm_.getLinkNames();
}

/**
 * @brief 关节全部归零并执行
 * @return 错误码
 */
ErrorCode ArmController::reset_to_zero() {
    std::vector<double> zeroes = get_current_joints();
    if(zeroes.empty()) {
        ROS_WARN("无法获取当前关节，重置到零点失败");
        return ErrorCode::FAILURE;
    }

    for(auto& value : zeroes) {
        value = 0.0;
    }

    const ErrorCode set_code = set_joints(zeroes);
    if(set_code != ErrorCode::SUCCESS) {
        ROS_WARN("设置零点关节目标失败：%s", err_to_string(set_code).c_str());
        return set_code;
    }

    return plan_and_execute();
}

/**
 * @brief 检查是否需要取消当前异步任务
 * @param should_cancel_fn 外部取消检查函数
 * @return 是否需要取消
 */
bool ArmController::should_cancel(const CancelChecker& should_cancel_fn) const {
    return _cancel_requested_.load() || static_cast<bool>(should_cancel_fn && should_cancel_fn());
}

/**
 * @brief 重置取消状态
 */
void ArmController::reset_cancel_state() {
    _cancel_requested_.store(false);
    _async_done_.store(true);
    _async_result_.store(static_cast<int>(ErrorCode::SUCCESS));
}

/**
 * @brief 启动异步任务
 * @param work 任务函数
 * @param callback 完成回调
 * @return 错误码
 */
ErrorCode ArmController::start_async_work(AsyncWork work, std::function<void(ErrorCode)> callback) {
    if(_is_planning_or_executing_.load()) {
        ROS_WARN("当前已有异步任务正在执行，请稍后再试");
        return ErrorCode::ASYNC_TASK_RUNNING;
    }

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    reset_cancel_state();
    _async_done_.store(false);
    _async_result_.store(static_cast<int>(ErrorCode::FAILURE));
    _is_planning_or_executing_.store(true);

    _async_thread_ = std::thread([this, work = std::move(work), callback = std::move(callback)]() mutable {
        ErrorCode result = ErrorCode::FAILURE;
        try {
            result = work ? work() : ErrorCode::FAILURE;
        }
        catch(const std::exception& e) {
            ROS_ERROR("异步任务异常：%s", e.what());
            result = ErrorCode::FAILURE;
        }
        catch(...) {
            ROS_ERROR("异步任务发生未知异常");
            result = ErrorCode::FAILURE;
        }

        if(_cancel_requested_.load() && result != ErrorCode::SUCCESS) {
            result = ErrorCode::CANCELLED;
        }

        _async_result_.store(static_cast<int>(result));
        _async_done_.store(true);
        _is_planning_or_executing_.store(false);
        _async_cv_.notify_all();

        if(callback) callback(result);
        });

    return ErrorCode::SUCCESS;
}

/**
 * @brief 等待异步任务完成并获取结果
 * @param should_cancel_fn 外部取消检查函数
 * @return 任务结果错误码
 */
ErrorCode ArmController::wait_async_result(CancelChecker should_cancel_fn) {
    using namespace std::chrono_literals;

    std::unique_lock<std::mutex> lock(_async_mutex_);
    while(!_async_done_.load()) {
        if(should_cancel(should_cancel_fn)) {
            lock.unlock();
            request_cancel();
            lock.lock();
        }
        _async_cv_.wait_for(lock, 20ms, [this]() { return _async_done_.load(); });
    }
    lock.unlock();

    if(_async_thread_.joinable()) {
        _async_thread_.join();
    }

    const bool canceled = should_cancel(should_cancel_fn) || _cancel_requested_.load();
    const ErrorCode result = static_cast<ErrorCode>(_async_result_.load());

    if(canceled) {
        reset_cancel_state();
        return ErrorCode::CANCELLED;
    }

    reset_cancel_state();
    return result;
}

/**
 * @brief 运行异步任务并等待结果，期间可通过 should_cancel_fn 请求取消
 * @param work 任务函数
 * @param should_cancel_fn 外部取消检查函数
 * @return 任务结果错误码
 */
ErrorCode ArmController::run_async_and_wait(AsyncWork work, CancelChecker should_cancel_fn) {
    const bool external_cancel = static_cast<bool>(should_cancel_fn && should_cancel_fn());

    if(!external_cancel && !_is_planning_or_executing_.load() && _cancel_requested_.load()) {
        reset_cancel_state();
    }

    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    ErrorCode code = start_async_work(std::move(work), nullptr);
    if(code != ErrorCode::SUCCESS) {
        return code;
    }

    return wait_async_result(should_cancel_fn);
}

/**
 * @brief 规划实现
 * @param plan 规划结果
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_impl(moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel_fn) {
    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    ROS_INFO("正在规划...");
    moveit::core::MoveItErrorCode err_code = _arm_.plan(plan);

    if(should_cancel(should_cancel_fn)) {
        _arm_.stop();
        return ErrorCode::CANCELLED;
    }

    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("规划失败，错误码：%d", err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    ROS_INFO("规划成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行实现
 * @param plan 规划结果
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::execute_plan_impl(const moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel_fn) {
    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    ROS_INFO("正在执行规划结果...");
    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);

    if(should_cancel(should_cancel_fn)) {
        _arm_.stop();
        return ErrorCode::CANCELLED;
    }

    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划并执行实现
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_and_execute_impl(const CancelChecker& should_cancel_fn) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ErrorCode code = plan_impl(plan, should_cancel_fn);
    if(code != ErrorCode::SUCCESS) return code;
    return execute_plan_impl(plan, should_cancel_fn);
}

/**
 * @brief 执行轨迹实现
 * @param trajectory 轨迹消息
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::execute_trajectory_impl(const moveit_msgs::RobotTrajectory& trajectory, const CancelChecker& should_cancel_fn) {
    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    ROS_INFO("正在按预设轨迹执行...");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;

    moveit::core::MoveItErrorCode err_code = _arm_.execute(plan);

    if(should_cancel(should_cancel_fn)) {
        _arm_.stop();
        return ErrorCode::CANCELLED;
    }

    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("执行失败，错误码：%d", err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("执行成功");
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划目标实现
 * @param target 目标
 * @param plan 规划结果
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_target_impl(const TargetVariant& target, moveit::planning_interface::MoveGroupInterface::Plan& plan, const CancelChecker& should_cancel_fn) {
    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    clear_constraints();
    clear_target();

    ErrorCode code = set_target(target);
    if(code != ErrorCode::SUCCESS) {
        clear_target();
        return code;
    }

    code = plan_impl(plan, should_cancel_fn);
    clear_target();
    return code;
}

/**
 * @brief 规划并执行目标实现
 * @param target 目标
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_target_and_execute_impl(const TargetVariant& target, const CancelChecker& should_cancel_fn) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ErrorCode code = plan_target_impl(target, plan, should_cancel_fn);
    if(code != ErrorCode::SUCCESS) return code;
    return execute_plan_impl(plan, should_cancel_fn);
}

/**
 * @brief 规划目标实现（带检查）
 * @param target 目标
 * @param plan 规划结果
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_target_checked(const TargetVariant& target, moveit::planning_interface::MoveGroupInterface::Plan& plan, CancelChecker should_cancel_fn) {
    const bool external_cancel = static_cast<bool>(should_cancel_fn && should_cancel_fn());

    if(!external_cancel && !_is_planning_or_executing_.load() && _cancel_requested_.load()) {
        reset_cancel_state();
    }

    if(should_cancel(should_cancel_fn)) {
        return ErrorCode::CANCELLED;
    }

    clear_target();
    ErrorCode code = set_target(target);
    if(code != ErrorCode::SUCCESS) {
        clear_target();
        return code;
    }

    code = plan_checked(plan, should_cancel_fn);
    clear_target();
    return code;
}

/**
 * @brief 规划并执行目标实现（带检查）
 * @param target 目标
 * @param should_cancel_fn 外部取消检查函数
 * @return 错误码
 */
ErrorCode ArmController::plan_target_and_execute_checked(const TargetVariant& target, CancelChecker should_cancel_fn) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ErrorCode code = plan_target_checked(target, plan, should_cancel_fn);
    if(code != ErrorCode::SUCCESS) return code;
    return execute_checked(plan, should_cancel_fn);
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将 TargetVariant 提取为 Pose
 */
tl::optional<geometry_msgs::Pose> ArmController::extract_pose_from_target(const TargetVariant& target) const {
    const geometry_msgs::Pose current_pose = get_current_pose();

    return std::visit(variant_visitor{
        [](const std::monostate&) -> tl::optional<geometry_msgs::Pose> {
            return tl::nullopt;
        },
        [](const geometry_msgs::Pose& pose) -> tl::optional<geometry_msgs::Pose> {
            return pose;
        },
        [&current_pose](const geometry_msgs::Point& point) -> tl::optional<geometry_msgs::Pose> {
            geometry_msgs::Pose pose;
            pose.position = point;
            pose.orientation = current_pose.orientation;
            return pose;
        },
        [&current_pose](const geometry_msgs::Quaternion& quat) -> tl::optional<geometry_msgs::Pose> {
            geometry_msgs::Pose pose;
            pose.position = current_pose.position;
            pose.orientation = quat;
            return pose;
        },
        [](const geometry_msgs::PoseStamped& pose_stamped) -> tl::optional<geometry_msgs::Pose> {
            return pose_stamped.pose;
        }
        }, target);
}
/**
 * @brief A* 搜索最近可达目标位姿
 * @param current_pose 当前位姿
 * @param target_pose 目标位姿
 * @return 搜索结果
 */
tl::optional<ReachablePoseResult> ArmController::search_reachable_pose(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& target_pose) {
    (void)current_pose;

    ReachablePoseResult result;
    result.score = -1.0;
    result.reachable_joints.clear();
    result.reachable_pose = target_pose;

    _current_state_ = _arm_.getCurrentState(1.0);
    if(!_current_state_) {
        ROS_ERROR("无法获取当前机械臂状态，A* 可达性搜索终止。");
        return tl::nullopt;
    }

    _jmg_ = _current_state_->getJointModelGroup(_arm_.getName());
    if(!_jmg_) {
        ROS_ERROR("无法获取关节模型组 '%s'，A* 可达性搜索终止。", _arm_.getName().c_str());
        return tl::nullopt;
    }

    auto try_ik = [&](const geometry_msgs::Pose& pose_base,
        double candidate_score,
        SearchReachablePose state) {
            moveit::core::RobotState state_copy(*_current_state_);
            if(!state_copy.setFromIK(_jmg_, pose_base, 0.0)) {
                return SearchReachablePose::SOLUTION_NOT_FOUND;
            }
            state_copy.copyJointGroupPositions(_jmg_, result.reachable_joints);
            result.reachable_pose = pose_base;
            result.score = candidate_score;
            result.state = state;
            return state;
        };

    const auto exact_result = try_ik(target_pose, 0.0, SearchReachablePose::SOLUTION_FOUND);
    if(exact_result == SearchReachablePose::SOLUTION_FOUND) {
        ROS_INFO("目标位姿可直接到达，无需 A* 搜索。");
        return result;
    }

    geometry_msgs::TransformStamped tf_stamped;
    geometry_msgs::TransformStamped tf_stamped_inv;
    geometry_msgs::Pose target_pose_eef;

    try {
        tf_stamped = _tf_buffer_.lookupTransform(_eef_link_, _base_link_, ros::Time(0), ros::Duration(1.0));
        tf_stamped_inv = _tf_buffer_.lookupTransform(_base_link_, _eef_link_, ros::Time(0), ros::Duration(1.0));
        target_pose_eef = target_pose;
        tf2::doTransform(target_pose_eef, target_pose_eef, tf_stamped);
    }
    catch(const tf2::TransformException& e) {
        ROS_WARN("A* 可达性搜索 TF 变换失败：%s", e.what());
        return tl::nullopt;
    }

    ros::NodeHandle pnh("~");
    double step_deg = 5.0;
    double radius_deg = 60.0;
    int max_expand = 624;
    pnh.param("reachable_pose_search/step_deg", step_deg, 5.0);
    pnh.param("reachable_pose_search/radius_deg", radius_deg, 60.0);
    max_expand = std::pow((2 * static_cast<int>(radius_deg / step_deg) + 1), 2) - 1;

    const double step = std::max(step_deg, 0.1) * M_PI / 180.0;
    const double radius = std::max(radius_deg, step_deg) * M_PI / 180.0;

    tf2::Quaternion q_orig;
    tf2::fromMsg(target_pose_eef.orientation, q_orig);
    double roll_orig = 0.0;
    double pitch_orig = 0.0;
    double yaw_orig = 0.0;
    tf2::Matrix3x3(q_orig).getRPY(roll_orig, pitch_orig, yaw_orig);

    auto encode_key = [](int roll_idx, int pitch_idx) -> std::int64_t {
        return (static_cast<std::int64_t>(roll_idx) << 32) ^
            static_cast<std::uint32_t>(pitch_idx);
        };

    auto heuristic = [](double droll, double dpitch) {
        return std::hypot(droll, dpitch);
        };

    auto build_node = [&](double droll, double dpitch, double g_cost, int depth) {
        AStarNode node;
        node.g_cost = g_cost;
        node.h_cost = heuristic(droll, dpitch);
        node.depth = depth;
        node.pose = target_pose_eef;

        tf2::Quaternion q_candidate;
        q_candidate.setRPY(roll_orig + droll, pitch_orig + dpitch, yaw_orig);
        q_candidate.normalize();
        node.pose.orientation = tf2::toMsg(q_candidate);
        return node;
        };

    std::priority_queue<AStarNode, std::vector<AStarNode>, CompareAStarNode> open_set;
    std::unordered_map<std::int64_t, double> best_g_by_idx;

    open_set.push(build_node(0.0, 0.0, 0.0, 0));
    best_g_by_idx[encode_key(0, 0)] = 0.0;

    const int dirs[8][2] = {
        { 1, 0 }, { -1, 0 },
        { 0, 1 }, { 0, -1 },
        { 1, 1 }, { -1, -1 },
        { 1, -1 }, { -1, 1 }
    };

    int expand_count = 0;
    while(!open_set.empty() && expand_count < max_expand) {
        AStarNode current_node = open_set.top();
        open_set.pop();
        ++expand_count;

        tf2::Quaternion q_current;
        tf2::fromMsg(current_node.pose.orientation, q_current);
        double roll_now = 0.0;
        double pitch_now = 0.0;
        double yaw_now = 0.0;
        tf2::Matrix3x3(q_current).getRPY(roll_now, pitch_now, yaw_now);

        const double droll = roll_now - roll_orig;
        const double dpitch = pitch_now - pitch_orig;

        if(current_node.depth > 0) {
            geometry_msgs::Pose pose_candidate_base;
            tf2::doTransform(current_node.pose, pose_candidate_base, tf_stamped_inv);

            const auto approx_result = try_ik(
                pose_candidate_base,
                current_node.h_cost,
                SearchReachablePose::APPROXIMATE_SOLUTION_FOUND);

            if(approx_result == SearchReachablePose::APPROXIMATE_SOLUTION_FOUND) {
                ROS_INFO("A* 搜索到近似可达位姿：droll=%.2f°, dpitch=%.2f°",
                    droll * 180.0 / M_PI,
                    dpitch * 180.0 / M_PI);
                return result;
            }
        }

        const int roll_idx = static_cast<int>(std::round(droll / step));
        const int pitch_idx = static_cast<int>(std::round(dpitch / step));

        for(const auto& dir : dirs) {
            const int next_roll_idx = roll_idx + dir[0];
            const int next_pitch_idx = pitch_idx + dir[1];

            const double next_droll = next_roll_idx * step;
            const double next_dpitch = next_pitch_idx * step;
            if(std::hypot(next_droll, next_dpitch) > radius + 1e-12) {
                continue;
            }

            const double move_cost = std::hypot(next_droll - droll, next_dpitch - dpitch);
            const double next_g = current_node.g_cost + move_cost;
            const std::int64_t key = encode_key(next_roll_idx, next_pitch_idx);

            auto it = best_g_by_idx.find(key);
            if(it != best_g_by_idx.end() && it->second <= next_g) {
                continue;
            }

            best_g_by_idx[key] = next_g;
            open_set.push(build_node(next_droll, next_dpitch, next_g, current_node.depth + 1));
        }
    }

    ROS_WARN("A* 搜索未找到可达位姿（最大扩展=%d）。", max_expand);
    return tl::nullopt;
}

} /* namespace dm_arm */
