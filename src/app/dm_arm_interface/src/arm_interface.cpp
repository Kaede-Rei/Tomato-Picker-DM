#include "dm_arm_interface/arm_interface.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

bool xyz_sizes_match(const dm_arm_msgs::SimpleMoveArmGoal& goal);
bool rpy_sizes_match(const dm_arm_msgs::SimpleMoveArmGoal& goal);
bool command_requires_move_goal_target(ArmCmdType type);
bool command_requires_simple_goal_payload(ArmCmdType type);
geometry_msgs::Quaternion quat_from_rpy(double roll, double pitch, double yaw);
geometry_msgs::Pose pose_from_goal(const dm_arm_msgs::SimpleMoveArmGoal& goal, std::size_t index);

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 机械臂运动的 Action 接口
 * @param nh ROS 节点句柄
 * @param arm 机械臂控制器
 * @param dispatcher 机械臂命令调度器
 * @param action_name Action 名称
 */
ArmMoveAction::ArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&ArmMoveAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&ArmMoveAction::on_preempt, this));
    _as_->start();
}

/**
 * @brief 简化版机械臂运动的 Action 接口
 * @param nh ROS 节点句柄
 * @param arm 机械臂控制器
 * @param dispatcher 机械臂命令调度器
 * @param action_name Action 名称
 */
SimpleArmMoveAction::SimpleArmMoveAction(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string action_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _as_ = std::make_unique<MoveArmAS>(nh, action_name, false);
    _as_->registerGoalCallback(boost::bind(&SimpleArmMoveAction::on_goal, this));
    _as_->registerPreemptCallback(boost::bind(&SimpleArmMoveAction::on_preempt, this));
    _as_->start();
}

/**
 * @brief 机械臂配置的 Service 接口
 * @param nh ROS节点句柄
 * @param arm 机械臂控制器
 * @param dispatcher 机械臂命令调度器
 * @param service_name Service 名称
 */
ArmConfigService::ArmConfigService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _srv_ = std::make_unique<ros::ServiceServer>(nh.advertiseService(service_name, &ArmConfigService::on_request, this));
}

/**
 * @brief 机械臂查询的 Service 接口
 * @param nh ROS节点句柄
 * @param arm 机械臂控制器
 * @param dispatcher 机械臂命令调度器
 * @param service_name Service 名称
 */
ArmQueryService::ArmQueryService(ros::NodeHandle& nh, std::shared_ptr<ArmController> arm, std::shared_ptr<ArmCmdDispatcher> dispatcher, std::string service_name)
    : _arm_(std::move(arm)), _dispatcher_(std::move(dispatcher)) {
    _srv_ = std::make_unique<ros::ServiceServer>(nh.advertiseService(service_name, &ArmQueryService::on_request, this));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 将 MoveArmAction 的 Goal 转换为 ArmCmdRequest
 * @param goal MoveArmAction 的 Goal
 * @param req 转换后的 ArmCmdRequest
 * @return 转换是否成功
 */
tl::optional<ArmCmdRequest> ArmMoveAction::convert_goal_to_request(const dm_arm_msgs::MoveArmGoal& goal) {
    ArmCmdRequest req{};
    req.type = static_cast<ArmCmdType>(goal.command_type);
    if(static_cast<std::size_t>(goal.command_type) >= static_cast<std::size_t>(ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return tl::nullopt;
    }

    req.joint_names = goal.joint_names;
    req.joints = goal.joints;
    req.values = goal.values;
    req.waypoints = goal.waypoints;

    if(!command_requires_move_goal_target(req.type)) {
        return req;
    }

    if(goal.target_type == goal.TARGET_POSE) req.target = goal.pose;
    else if(goal.target_type == goal.TARGET_POINT) req.target = goal.point;
    else if(goal.target_type == goal.TARGET_QUATERNION) req.target = goal.quaternion;
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return tl::nullopt;
    }

    return req;
}

/**
 * @brief MoveArmAction 的 Goal 回调函数
 */
void ArmMoveAction::on_goal() {
    if(!_arm_ || !_dispatcher_) {
        dm_arm_msgs::MoveArmResult res;
        res.success = false;
        res.message = "控制器未初始化";
        _as_->setAborted(res, res.message);
        return;
    }

    auto goal = _as_->acceptNewGoal();
    if(!goal) {
        dm_arm_msgs::MoveArmResult res;
        res.success = false;
        res.message = "空目标";
        _as_->setAborted(res, res.message);

        return;
    }

    auto req_opt = convert_goal_to_request(*goal);
    if(!req_opt) {
        dm_arm_msgs::MoveArmResult res;
        res.success = false;
        res.message = "无效的目标";
        _as_->setAborted(res, res.message);

        return;
    }

    auto result = _dispatcher_->dispatch(*req_opt, [this](const ArmCmdFeedback& fb) {
        dm_arm_msgs::MoveArmFeedback feedback;
        feedback.stage = fb.stage;
        feedback.progress = fb.progress;
        feedback.message = fb.message;
        this->_as_->publishFeedback(feedback);
        });

    dm_arm_msgs::MoveArmResult res;
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);
    if(result.current_joints) {
        res.cur_joint = *result.current_joints;
    }
    else {
        res.cur_joint = _arm_ ? _arm_->get_current_joints() : std::vector<double>{};
    }

    if(result.current_pose) {
        res.cur_pose = *result.current_pose;
    }
    else {
        res.cur_pose = _arm_ ? _arm_->get_current_pose() : geometry_msgs::Pose{};
    }
    res.values = result.values;

    if(result.success) _as_->setSucceeded(res);
    else _as_->setAborted(res, res.message);
}

/**
 * @brief MoveArmAction 的 Preempt 回调函数
 */
void ArmMoveAction::on_preempt() {
    _dispatcher_->cancel();
    dm_arm_msgs::MoveArmResult res;
    res.success = false;
    res.message = "目标被取消";
    _as_->setPreempted(res, res.message);
}

/**
 * @brief 将 SimpleMoveArmAction 的 Goal 转换为 ArmCmdRequest
 * @param goal SimpleMoveArmAction 的 Goal
 * @param req 转换后的 ArmCmdRequest
 * @return 转换是否成功
 */
tl::optional<ArmCmdRequest> SimpleArmMoveAction::convert_goal_to_request(const dm_arm_msgs::SimpleMoveArmGoal& goal) {
    ArmCmdRequest req{};
    req.type = static_cast<ArmCmdType>(goal.command_type);

    if(static_cast<std::size_t>(goal.command_type) >=
        static_cast<std::size_t>(ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", goal.command_type);
        return tl::nullopt;
    }

    req.joint_names = goal.joint_names;
    req.joints = goal.joints;
    req.values = goal.values;

    if(!command_requires_simple_goal_payload(req.type)) {
        return req;
    }

    if(goal.target_type == goal.TARGET_POSE) {
        if(!xyz_sizes_match(goal) || !rpy_sizes_match(goal) || goal.x.empty()) {
            ROS_WARN("接收到的目标位姿数据长度不匹配");
            return tl::nullopt;
        }

        req.target = pose_from_goal(goal, 0);

        req.waypoints.reserve(goal.x.size());
        for(size_t i = 0; i < goal.x.size(); ++i) {
            req.waypoints.push_back(pose_from_goal(goal, i));
        }
    }
    else if(goal.target_type == goal.TARGET_POINT) {
        if(!xyz_sizes_match(goal) || goal.x.empty()) {
            ROS_WARN("接收到的目标点数据长度不匹配");
            return tl::nullopt;
        }

        geometry_msgs::Point point;
        point.x = goal.x[0];
        point.y = goal.y[0];
        point.z = goal.z[0];
        req.target = point;
    }
    else if(goal.target_type == goal.TARGET_ORIENTATION) {
        if(!rpy_sizes_match(goal) || goal.roll.empty()) {
            ROS_WARN("接收到的目标姿态数据长度不匹配");
            return tl::nullopt;
        }

        req.target = quat_from_rpy(goal.roll[0], goal.pitch[0], goal.yaw[0]);
    }
    else {
        ROS_WARN("接收到无效的目标类型: %d", goal.target_type);
        return tl::nullopt;
    }

    return req;
}

/**
 * @brief SimpleMoveArmAction 的 Goal 回调函数
 */
void SimpleArmMoveAction::on_goal() {
    if(!_arm_ || !_dispatcher_) {
        dm_arm_msgs::SimpleMoveArmResult res;
        res.success = false;
        res.message = "控制器未初始化";
        _as_->setAborted(res, res.message);
        return;
    }

    auto goal = _as_->acceptNewGoal();
    if(!goal) {
        dm_arm_msgs::SimpleMoveArmResult res;
        res.success = false;
        res.message = "空目标";
        res.error_code = static_cast<int32_t>(ErrorCode::INVALID_PARAMETER);
        res.cur_x = 0.0;
        res.cur_y = 0.0;
        res.cur_z = 0.0;
        res.cur_roll = 0.0;
        res.cur_pitch = 0.0;
        res.cur_yaw = 0.0;

        _as_->setAborted(res, res.message);
        return;
    }

    auto req_opt = convert_goal_to_request(*goal);
    if(!req_opt) {
        dm_arm_msgs::SimpleMoveArmResult res;
        res.success = false;
        res.message = "无效的目标";
        _as_->setAborted(res, res.message);

        return;
    }

    auto result = _dispatcher_->dispatch(*req_opt, [this](const ArmCmdFeedback& fb) {
        dm_arm_msgs::SimpleMoveArmFeedback feedback;
        feedback.stage = fb.stage;
        feedback.progress = fb.progress;
        feedback.message = fb.message;
        this->_as_->publishFeedback(feedback);
        });

    dm_arm_msgs::SimpleMoveArmResult res;
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);
    if(result.current_joints) {
        res.cur_joint = *result.current_joints;
    }
    else {
        res.cur_joint = _arm_ ? _arm_->get_current_joints() : std::vector<double>{};
    }

    const geometry_msgs::Pose cur_pose = result.current_pose.value_or(_arm_ ? _arm_->get_current_pose() : geometry_msgs::Pose{});
    res.cur_x = cur_pose.position.x;
    res.cur_y = cur_pose.position.y;
    res.cur_z = cur_pose.position.z;
    tf2::Quaternion quat(cur_pose.orientation.x, cur_pose.orientation.y, cur_pose.orientation.z, cur_pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    res.cur_roll = roll;
    res.cur_pitch = pitch;
    res.cur_yaw = yaw;

    res.values = result.values;

    if(result.success) _as_->setSucceeded(res);
    else _as_->setAborted(res, res.message);
}

/**
 * @brief SimpleMoveArmAction 的 Preempt 回调函数
 */
void SimpleArmMoveAction::on_preempt() {
    _dispatcher_->cancel();
    dm_arm_msgs::SimpleMoveArmResult res;
    res.success = false;
    res.message = "目标被取消";
    _as_->setPreempted(res, res.message);
}

/**
 * @brief 将 ConfigArm Service 的请求转换为 ArmCmdRequest
 * @param srv_req ConfigArm Service 的请求
 * @param arm_req 转换后的 ArmCmdRequest
 * @return 转换是否成功
 */
tl::optional<ArmCmdRequest> ArmConfigService::convert_srvreq_to_armreq(const dm_arm_msgs::ConfigArm::Request& srv_req) {
    ArmCmdRequest arm_req{};
    arm_req.type = static_cast<ArmCmdType>(srv_req.command_type);
    if(static_cast<std::size_t>(srv_req.command_type) >= static_cast<std::size_t>(ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", srv_req.command_type);
        return tl::nullopt;
    }

    if(srv_req.command_type == srv_req.SET_ORIENTATION_CONSTRAINT) arm_req.target = srv_req.quaternion;
    else if(srv_req.command_type == srv_req.SET_POSITION_CONSTRAINT) arm_req.target = srv_req.point;
    else if(srv_req.command_type == srv_req.SET_JOINT_CONSTRAINT) {
        arm_req.joint_names = srv_req.joint_names;
        arm_req.joints = srv_req.joints;
    }
    else {
        ROS_WARN("接收到无效的命令类型: %d", srv_req.command_type);
        return tl::nullopt;
    }
    arm_req.values = srv_req.values;

    return arm_req;
}

/**
 * @brief ConfigArm Service 的请求回调函数
 * @param req ConfigArm Service 的请求
 * @param res ConfigArm Service 的响应
 * @return 是否成功处理请求
 */
bool ArmConfigService::on_request(dm_arm_msgs::ConfigArm::Request& req, dm_arm_msgs::ConfigArm::Response& res) {
    if(!_arm_ || !_dispatcher_) {
        res.success = false;
        res.message = "控制器未初始化";
        return true;
    }

    auto arm_req_opt = convert_srvreq_to_armreq(req);
    if(!arm_req_opt) {
        res.success = false;
        res.message = "无效的请求";
        return true;
    }

    auto result = _dispatcher_->dispatch(*arm_req_opt);
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);

    return true;
}

/**
 * @brief 将 QueryArm Service 的请求转换为 ArmCmdRequest
 * @param srv_req QueryArm Service 的请求
 * @param arm_req 转换后的 ArmCmdRequest
 * @return 转换是否成功
 */
tl::optional<ArmCmdRequest> ArmQueryService::convert_srvreq_to_armreq(const dm_arm_msgs::QueryArm::Request& srv_req) {
    ArmCmdRequest arm_req{};
    arm_req.type = static_cast<ArmCmdType>(srv_req.command_type);
    if(static_cast<std::size_t>(srv_req.command_type) >= static_cast<std::size_t>(ArmCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", srv_req.command_type);
        return tl::nullopt;
    }

    arm_req.values = srv_req.values;

    return arm_req;
}

/**
 * @brief QueryArm Service 的请求回调函数
 * @param req QueryArm Service 的请求
 * @param res QueryArm Service 的响应
 * @return 是否成功处理请求
 */
bool ArmQueryService::on_request(dm_arm_msgs::QueryArm::Request& req, dm_arm_msgs::QueryArm::Response& res) {
    if(!_arm_ || !_dispatcher_) {
        res.success = false;
        res.message = "控制器未初始化";
        return true;
    }

    auto arm_req_opt = convert_srvreq_to_armreq(req);
    if(!arm_req_opt) {
        res.success = false;
        res.message = "无效的请求";
        return true;
    }

    auto result = _dispatcher_->dispatch(*arm_req_opt);
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);
    res.cur_pose = result.current_pose.value_or(_arm_ ? _arm_->get_current_pose() : geometry_msgs::Pose{});
    res.cur_joint = result.current_joints.value_or(_arm_ ? _arm_->get_current_joints() : std::vector<double>{});

    return true;
}

bool xyz_sizes_match(const dm_arm_msgs::SimpleMoveArmGoal& goal) {
    return goal.x.size() == goal.y.size() &&
        goal.x.size() == goal.z.size();
}

bool rpy_sizes_match(const dm_arm_msgs::SimpleMoveArmGoal& goal) {
    return goal.roll.size() == goal.pitch.size() &&
        goal.roll.size() == goal.yaw.size();
}

bool command_requires_move_goal_target(ArmCmdType type) {
    switch(type) {
        case ArmCmdType::MOVE_TARGET:
        case ArmCmdType::MOVE_TARGET_IN_EEF_FRAME:
        case ArmCmdType::SET_ORIENTATION_CONSTRAINT:
        case ArmCmdType::SET_POSITION_CONSTRAINT:
            return true;
        default:
            return false;
    }
}

bool command_requires_simple_goal_payload(ArmCmdType type) {
    switch(type) {
        case ArmCmdType::MOVE_TARGET:
        case ArmCmdType::MOVE_TARGET_IN_EEF_FRAME:
        case ArmCmdType::MOVE_LINE:
        case ArmCmdType::MOVE_BEZIER:
        case ArmCmdType::MOVE_DECARTES:
        case ArmCmdType::SET_ORIENTATION_CONSTRAINT:
        case ArmCmdType::SET_POSITION_CONSTRAINT:
            return true;
        default:
            return false;
    }
}

geometry_msgs::Quaternion quat_from_rpy(double roll, double pitch, double yaw) {
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    quat.normalize();

    geometry_msgs::Quaternion out;
    out.x = quat.x();
    out.y = quat.y();
    out.z = quat.z();
    out.w = quat.w();
    return out;
}

geometry_msgs::Pose pose_from_goal(const dm_arm_msgs::SimpleMoveArmGoal& goal, std::size_t index) {
    geometry_msgs::Pose pose;
    pose.position.x = goal.x[index];
    pose.position.y = goal.y[index];
    pose.position.z = goal.z[index];
    pose.orientation = quat_from_rpy(goal.roll[index], goal.pitch[index], goal.yaw[index]);
    return pose;
}

}
