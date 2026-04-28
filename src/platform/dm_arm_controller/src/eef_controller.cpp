#include "dm_arm_controller/eef_controller.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief TwoFingerGripper 构造函数
 * @param eef_name 末端执行器规划组名称
 */
TwoFingerGripper::TwoFingerGripper(const std::string& eef_name)
    : EndEffector(eef_name), _gripper_(eef_name) {
    ROS_INFO("末端执行器控制器 [%s] 已创建，规划组：%s", get_eef_name().c_str(), _gripper_.getName().c_str());
}

/**
 * @brief 停止夹爪动作
 */
void TwoFingerGripper::stop() {
    _gripper_.stop();
    ROS_INFO("末端执行器控制器 [%s] 已停止", get_eef_name().c_str());
}

/**
 * @brief 获取夹爪规划组名称
 */
const std::string& TwoFingerGripper::get_group_name() const {
    return get_eef_name();
}

/**
 * @brief 获取夹爪 MoveGroupInterface
 */
moveit::planning_interface::MoveGroupInterface& TwoFingerGripper::get_move_group() {
    return _gripper_;
}

/**
 * @brief 打开夹爪
 */
ErrorCode TwoFingerGripper::open() {
    return execute_preset_pose("open");
}

/**
 * @brief 闭合夹爪
 */
ErrorCode TwoFingerGripper::close() {
    return execute_preset_pose("close");
}

/**
 * @brief 设置夹爪角度
 * @param angle 目标角度，单位为度
 */
ErrorCode TwoFingerGripper::set_angle(double angle) {
    // 将角度转换为关节值，假设夹爪的开合范围为 0-90 度，对应关节值 0-1
    double joint_value = angle / 90.0;
    return set_joint_value("gripper_joint", joint_value);
}

/**
 * @brief 获取夹爪当前角度
 * @return 当前夹爪角度，单位为度，如果获取失败则返回 nullopt
 */
tl::optional<double> TwoFingerGripper::get_angle() const {
    std::vector<double> joint_values = _gripper_.getCurrentJointValues();
    if(joint_values.empty()) {
        ROS_WARN("末端执行器控制器 [%s] 获取当前关节值失败", get_eef_name().c_str());
        return tl::nullopt;
    }

    double joint_value = joint_values[0];
    return joint_value * 90.0;
}

/**
 * @brief 执行夹爪预设位姿
 * @param pose_name 预设位姿名称
 */
ErrorCode TwoFingerGripper::execute_preset_pose(const std::string& pose_name) {
    ROS_INFO("末端执行器控制器 [%s] 执行预设位姿 [%s]", get_eef_name().c_str(), pose_name.c_str());
    _gripper_.setNamedTarget(pose_name);
    _gripper_.move();

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置夹爪单关节值
 * @param joint_name 关节名称
 * @param value 目标值
 */
ErrorCode TwoFingerGripper::set_joint_value(const std::string& joint_name, double value) {
    ROS_INFO("末端执行器控制器 [%s] 设置关节 [%s] 的值为 %f", get_eef_name().c_str(), joint_name.c_str(), value);

    bool success = _gripper_.setJointValueTarget(joint_name, value);
    if(!success) {
        ROS_WARN("末端执行器控制器 [%s] 设置关节值 %.4f 失败，可能超出关节限制", get_eef_name().c_str(), value);
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    ROS_INFO("末端执行器控制器 [%s] 设置关节 [%s] 的值为 %f 成功", get_eef_name().c_str(), joint_name.c_str(), value);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置夹爪全部关节值
 * @param joint_values 关节值数组
 */
ErrorCode TwoFingerGripper::set_joint_values(const std::vector<double>& joint_values) {
    ROS_INFO("末端执行器控制器 [%s] 设置关节值", get_eef_name().c_str());

    const auto& joint_names = _gripper_.getVariableNames();
    if(joint_values.size() != joint_names.size()) {
        ROS_WARN("末端执行器控制器 [%s] 设置关节值失败，提供的关节值数量 %zu 与规划组关节数量 %zu 不匹配", get_eef_name().c_str(), joint_values.size(), joint_names.size());
        return ErrorCode::INVALID_TARGET_TYPE;
    }

    bool success = _gripper_.setJointValueTarget(joint_values);
    if(!success) {
        ROS_WARN("末端执行器控制器 [%s] 设置关节值失败，可能超出关节限制", get_eef_name().c_str());
        return ErrorCode::TARGET_OUT_OF_BOUNDS;
    }

    ROS_INFO("末端执行器控制器 [%s] 设置关节值成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划夹爪轨迹
 * @param plan 规划输出
 */
ErrorCode TwoFingerGripper::plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    ROS_INFO("末端执行器控制器 [%s] 进行运动规划", get_eef_name().c_str());

    moveit::core::MoveItErrorCode err_code = _gripper_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("末端执行器控制器 [%s] 运动规划失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    ROS_INFO("末端执行器控制器 [%s] 运动规划成功，规划时间: %.4f 秒", get_eef_name().c_str(), plan.planning_time_);
    return ErrorCode::SUCCESS;
}

/**
 * @brief 执行夹爪轨迹
 * @param plan 轨迹输入
 */
ErrorCode TwoFingerGripper::execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    ROS_INFO("末端执行器控制器 [%s] 执行运动轨迹", get_eef_name().c_str());

    moveit::core::MoveItErrorCode err_code = _gripper_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("末端执行器控制器 [%s] 执行运动轨迹失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("末端执行器控制器 [%s] 执行运动轨迹成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 规划并执行夹爪动作
 */
ErrorCode TwoFingerGripper::plan_and_execute() {
    ROS_INFO("末端执行器控制器 [%s] 正在规划...", get_eef_name().c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    moveit::core::MoveItErrorCode err_code = _gripper_.plan(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("末端执行器控制器 [%s] 规划失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::PLANNING_FAILED;
    }

    ROS_INFO("末端执行器控制器 [%s] 规划成功，正在执行...", get_eef_name().c_str());
    err_code = _gripper_.execute(plan);
    if(err_code != moveit::core::MoveItErrorCode::SUCCESS) {
        ROS_WARN("末端执行器控制器 [%s] 执行失败，错误码：%d", get_eef_name().c_str(), err_code.val);
        return ErrorCode::EXECUTION_FAILED;
    }

    ROS_INFO("末端执行器控制器 [%s] 执行成功", get_eef_name().c_str());
    return ErrorCode::SUCCESS;
}

/**
 * @brief 获取当前关节值
 */
std::vector<double> TwoFingerGripper::get_current_joints() const {
    return _gripper_.getCurrentJointValues();
}

/**
 * @brief 获取当前连杆名称
 */
std::vector<std::string> TwoFingerGripper::get_current_link_names() const {
    return _gripper_.getLinkNames();
}

/**
 * @brief 获取可用力反馈名称列表
 */
std::vector<std::string> TwoFingerGripper::get_force_names() const {
    return {};
}

/**
 * @brief 获取指定力反馈值（当前为占位实现）
 * @param force_name 力反馈名称
 * @return 力反馈值，如果获取失败则返回 nullopt
 */
tl::optional<double> TwoFingerGripper::get_force(const std::string& force_name) const {
    // TODO: 实现实际的力反馈获取逻辑，目前为占位实现

    ROS_WARN("末端执行器控制器 [%s] 暂时不支持获取力反馈，力反馈名称：%s", get_eef_name().c_str(), force_name.c_str());
    return tl::nullopt;
}

/**
 * @brief ServoGripper 构造函数
 * @param nh ROS 节点句柄
 * @param tcp_offset TCP 偏移
 * @param serial_port 串口名称
 * @param baud_rate 波特率
 */
ServoGripper::ServoGripper(ros::NodeHandle& nh, const std::string& serial_port, int baud_rate)
    : EndEffector("servo_gripper"), _serialer_(nh, serial_port, baud_rate) {
    if(!_serialer_.connect()) {
        throw std::runtime_error("总线舵机末端执行器串口连接失败，串口端口: " + serial_port);
    }
    ROS_INFO("总线舵机末端执行器控制器创建成功，串口: %s, 波特率: %d", serial_port.c_str(), baud_rate);
}

/**
 * @brief 停止舵机动作
 */
void ServoGripper::stop() {
    // TODO: 发送停止命令到舵机
    ROS_INFO("总线舵机末端执行器控制器 已停止");
}

/**
 * @brief 打开舵机夹爪
 * @return 错误码
 */
ErrorCode ServoGripper::open() {
    std::string data = "$GRIPPER:OPEN#";
    if(!_serialer_.sendData(data)) {
        return ErrorCode::FAILURE;
    }
    ros::Duration(1.5).sleep();

    return ErrorCode::SUCCESS;
}

/**
 * @brief 关闭舵机夹爪
 * @return 错误码
 */
ErrorCode ServoGripper::close() {
    std::string data = "$GRIPPER:CLOSE#";
    if(!_serialer_.sendData(data)) {
        return ErrorCode::FAILURE;
    }
    ros::Duration(1.5).sleep();

    return ErrorCode::SUCCESS;
}

/**
 * @brief 设置舵机夹爪角度
 * @param angle 目标角度（0-270度）
 * @return 错误码
 */
ErrorCode ServoGripper::set_angle(double angle) {
    int angle_cmd = static_cast<int>(2000.0 * angle / 270.0 + 500);
    std::ostringstream oss;
    oss << std::setw(4) << std::setfill('0') << angle_cmd;
    std::string data = "$GRIPPER:POS:" + oss.str() + "#";
    if(!_serialer_.sendData(data)) {
        return ErrorCode::FAILURE;
    }

    _current_angle_.emplace(angle);

    return ErrorCode::SUCCESS;
}

/**
 * @brief 获取舵机夹爪当前角度
 * @param angle 输出参数，返回当前夹爪角度（0-270度）
 * @return 错误码
 */
tl::optional<double> ServoGripper::get_angle() const {
    if(!_current_angle_) {
        return tl::nullopt;
    }
    return *_current_angle_;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //



} /* namespace dm_arm */
