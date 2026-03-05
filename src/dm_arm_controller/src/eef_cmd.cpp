#include "dm_arm_controller/eef_cmd.hpp"

#include <cmath>
#include <queue>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>

// ! ========================= 宏 定 义 ========================= ! //



namespace dm_arm {
    // ! ========================= 接 口 量 声 明 ========================= ! //



    // ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



    // ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

    /**
     * @brief 机械臂末端执行器位姿控制类构造函数
     * @param nh ROS节点句柄
     * @param plan_group_name 机械臂规划组名称
     */
    EefPoseCmd::EefPoseCmd(const ros::NodeHandle& nh, const std::string& plan_group_name)
        : _nh_(nh), _arm_(plan_group_name), _tf_listener_(_tf_buffer_) {
        // 获取规划参考坐标系
        _plan_frame_ = _arm_.getPlanningFrame();
        ROS_INFO_STREAM("规划坐标系为：" << _plan_frame_);

        // 获取末端坐标系名称
        _eef_frame_ = _arm_.getEndEffectorLink();
        ROS_INFO_STREAM("末端执行器坐标系为：" << _eef_frame_);

        // 允许重新规划、设置目标位姿容忍度、设置最大速度和加速度比例
        bool allow_replanning;
        double goal_pos_tol, goal_ori_tol;
        double vel_scale, acc_scale;

        _nh_.param<bool>("moveit/allow_replanning", allow_replanning, true);
        _nh_.param<double>("end_effector/goal_position_tolerance", goal_pos_tol, 0.05);
        _nh_.param<double>("end_effector/goal_orientation_tolerance", goal_ori_tol, 0.05);
        _nh_.param<double>("end_effector/velocity_scaling", vel_scale, 1.0);
        _nh_.param<double>("end_effector/acceleration_scaling", acc_scale, 1.0);

        _arm_.allowReplanning(allow_replanning);
        _arm_.setGoalPositionTolerance(goal_pos_tol);
        _arm_.setGoalOrientationTolerance(goal_ori_tol);
        _arm_.setMaxVelocityScalingFactor(vel_scale);
        _arm_.setMaxAccelerationScalingFactor(acc_scale);

        // 规划器ID、规划时间、规划尝试次数、执行超时时间
        std::string planner_id;
        double planning_time;
        int num_planning_attempts;
        double execution_timeout;

        _nh_.param<std::string>("moveit/planner_id", planner_id, "RRTConnect");
        _nh_.param<double>("moveit/planning_time", planning_time, 5.0);
        _nh_.param<int>("moveit/num_planning_attempts", num_planning_attempts, 10);
        _nh_.param<double>("moveit/execution_timeout", execution_timeout, 15.0);

        _arm_.setPlannerId(planner_id);
        _arm_.setPlanningTime(planning_time);
        _arm_.setNumPlanningAttempts(num_planning_attempts);

        // 读取末端执行器参数
        _nh_.param<double>("end_effector/max_reach", _max_reach_, 0.6);
        _nh_.param<double>("end_effector/min_reach", _min_reach_, 0.1);
        _nh_.param<int>("end_effector/search/max_iterations", _max_iterations_, 100);

        // 初始化场景监控器与规划场景指针
        _scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        _scene_monitor_->startSceneMonitor();
        _scene_monitor_->startWorldGeometryMonitor();
        _scene_monitor_->startStateMonitor();

        ros::Duration(1.0).sleep();
        _planning_scene_ = _scene_monitor_->getPlanningScene();

        // 获取机械臂关节模型组指针
        _jmg_ = _arm_.getCurrentState()->getJointModelGroup(plan_group_name);
    }

    /**
     * @brief 将末端执行器位姿从eef坐标系转换到base坐标系
     * @param[in] target_pose_eef 目标位姿（eef坐标系）
     * @param[out] target_pose_base 目标位姿（base坐标系）
     */
    void EefPoseCmd::eefTfBase(geometry_msgs::PoseStamped& target_pose_eef, geometry_msgs::PoseStamped& target_pose_base) {
        // 获取末端执行器相对于基座的变换矩阵
        geometry_msgs::TransformStamped tf_stamped = _tf_buffer_.lookupTransform(_plan_frame_, _eef_frame_, ros::Time(0), ros::Duration(1.0));
        // 进行坐标系转换
        tf2::doTransform(target_pose_eef, target_pose_base, tf_stamped);
    }

    /**
     * @brief 判断目标位姿是否可达
     * @param target_pose 目标位姿
     * @return 是否可达
     */
    bool EefPoseCmd::isIkValid(const geometry_msgs::Pose& target_pose) {
        return _current_state_->setFromIK(_jmg_, target_pose, 0.0);
    }

    /**
     * @brief 搜索机械臂可达的目标位姿
     * @param target_pose 目标位姿
     * @param step 搜索步长(单位: 度)
     * @param radius 搜索半径(单位: 度)
     * @return 是否找到可达位姿，找到则返回true，geometry_msgs::Pose类型的target_pose将被更新为可达位姿
     * @note 当RPY均为0时，进行前馈计算
     */
    bool EefPoseCmd::searchReachablePose(geometry_msgs::Pose& target_pose, double step, double radius) {
        // 先判断一次目标位姿是否可达
        if(isIkValid(target_pose)) return true;

        // 先判断RPY是否均为0，是则进行前馈计算
        bool need_feedforward = (
            std::abs(target_pose.orientation.w - 1.0) < 1e-8 &&
            std::abs(target_pose.orientation.x) < 1e-8 &&
            std::abs(target_pose.orientation.y) < 1e-8 &&
            std::abs(target_pose.orientation.z) < 1e-8
            );

        // 把目标位姿tf为末端坐标系下
        geometry_msgs::Pose target_pose_eef = target_pose;
        geometry_msgs::TransformStamped tf_stamped = _tf_buffer_.lookupTransform(_eef_frame_, _plan_frame_, ros::Time(0), ros::Duration(1.0));
        geometry_msgs::TransformStamped tf_stamped_inv = _tf_buffer_.lookupTransform(_plan_frame_, _eef_frame_, ros::Time(0), ros::Duration(1.0));
        tf2::doTransform(target_pose_eef, target_pose_eef, tf_stamped);

        // 将步长和半径从度转换为弧度
        step = step * M_PI / 180.0;
        radius = radius * M_PI / 180.0;

        // 获取目标位姿的欧拉角表示，并根据判断进行前馈计算
        tf2::Quaternion q_orig;
        double roll_orig, pitch_orig, yaw_orig;
        if(need_feedforward) {
            // 获取底座到末端连线向外方向单位向量，作为末端Z轴在基坐标系下的表示
            tf2::Vector3 z_axis(
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            );
            if(z_axis.length() < 1e-8) {
                ROS_ERROR("目标位姿位置无效，无法进行前馈计算。");
                return false;
            }
            z_axis.normalize();

            // 默认X轴，若平行于Z轴则改用旋转90°，再计算Y轴
            tf2::Vector3 x_axis(1, 0, 0);
            if(std::abs(z_axis.dot(x_axis)) > 0.9999) x_axis = tf2::Vector3(0, 1, 0);
            tf2::Vector3 y_axis = z_axis.cross(x_axis).normalize();

            // 构造旋转矩阵并转换为四元数
            tf2::Matrix3x3 rot_matrix(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z()
            );
            tf2::Quaternion q_feed_base;
            rot_matrix.getRotation(q_feed_base);

            // 转换到末端坐标系下得到roll/pitch偏移
            geometry_msgs::Quaternion q_feed_eef = tf2::toMsg(q_feed_base);
            tf2::doTransform(q_feed_eef, q_feed_eef, tf_stamped);
            target_pose_eef.orientation = q_feed_eef;
        }
        tf2::fromMsg(target_pose_eef.orientation, q_orig);
        tf2::Matrix3x3(q_orig).getRPY(roll_orig, pitch_orig, yaw_orig);

        // A*搜索初始化
        const int step_count = static_cast<int>(std::ceil(radius / step));
        auto heuristic = [](double droll, double dpitch) { return std::hypot(droll, dpitch); };

        std::priority_queue<AStarNode_t, std::vector<AStarNode_t>, AstarNodeCmper> open_set;
        AStarNode_t start_node = { 0.0, 0.0, 0.0, heuristic(0.0, 0.0), 0.0 };
        start_node.f = start_node.g + start_node.h;
        open_set.push(start_node);

        using Key = std::pair<int, int>;
        std::unordered_map<Key, double, PairHash_t> closed_set;
        closed_set[{0, 0}] = 0.0;

        const int dirs[8][2] = {
            {1, 0}, {-1, 0},
            {0, 1}, {0, -1},
            {1, 1}, {-1, -1},
            {1, -1}, {-1, 1}
        };

        // 使用配置的最大迭代次数作为扩展限制，如果配置为0则使用自动计算的限制
        size_t max_expand = _max_iterations_;
        if(max_expand == 0) {
            max_expand = static_cast<size_t>((2 * step_count + 1) * (2 * step_count + 1) * 10);
        }
        size_t expand_count = 0;

        // A*搜索过程
        while(!open_set.empty()) {
            if(++expand_count > max_expand) {
                ROS_ERROR("A*搜索超出最大扩展节点数 (%zu)，终止搜索。", max_expand);
                break;
            }

            // 取出当前代价最小的节点
            AStarNode_t current_node = open_set.top();
            open_set.pop();

            // 检查当前节点对应的位姿是否可达
            tf2::Quaternion q_candidate;
            double new_roll = roll_orig + current_node.droll;
            double new_pitch = pitch_orig + current_node.dpitch;
            q_candidate.setRPY(new_roll, new_pitch, 0.0);

            geometry_msgs::Pose pose_candidate = target_pose_eef;
            pose_candidate.orientation = tf2::toMsg(q_candidate);

            // 转换回底座坐标系下再进行IK检测
            tf2::doTransform(pose_candidate, pose_candidate, tf_stamped_inv);
            if(isIkValid(pose_candidate)) {
                // 碰撞检测
                moveit::core::RobotState temp_state(*_current_state_);
                temp_state.setFromIK(_jmg_, pose_candidate, 0.0);
                if(_planning_scene_->isStateColliding(temp_state, _jmg_->getName())) {
                    ROS_WARN("搜索到的位姿存在碰撞，继续搜索...");
                    continue;
                }

                // 更新目标位姿为可达位姿
                ROS_INFO("搜索到可达位姿为：roll: %.2f°, pitch: %.2f°；偏移量为：droll: %.2f°, dpitch: %.2f°",
                    new_roll * 180.0 / M_PI, new_pitch * 180.0 / M_PI, current_node.droll * 180.0 / M_PI, current_node.dpitch * 180.0 / M_PI);
                target_pose = pose_candidate;
                return true;
            }

            // 生成相邻节点
            int cur_roll_idx = static_cast<int>(std::round(current_node.droll / step));
            int cur_pitch_idx = static_cast<int>(std::round(current_node.dpitch / step));

            for(auto& dir : dirs) {
                int new_roll_idx = cur_roll_idx + dir[0];
                int new_pitch_idx = cur_pitch_idx + dir[1];

                // 生成新节点并限制在搜索半径内并检查是否已访问
                double new_droll = new_roll_idx * step;
                double new_dpitch = new_pitch_idx * step;
                if(std::hypot(new_droll, new_dpitch) > radius + 1e-12) continue;
                Key new_visited{ new_roll_idx, new_pitch_idx };

                // 计算代价：目标为原点，启发式估计使用欧几里得距离
                double move_cost = std::hypot(new_droll - current_node.droll, new_dpitch - current_node.dpitch);
                double new_g = current_node.g + move_cost;
                double new_h = heuristic(new_droll, new_dpitch);
                double new_f = new_g + new_h;

                // 检查是否已在闭集且代价更优，否则加入开放集
                auto it = closed_set.find(new_visited);
                if(it != closed_set.end() && it->second <= new_g) continue;
                closed_set[new_visited] = new_g;
                open_set.push({ new_droll, new_dpitch, new_g, new_h, new_f });
            }
        }

        // 未找到可达位姿
        ROS_ERROR("未搜索到可达位姿。");
        return false;
    }

    /**
     * @brief 移动到机械臂末端执行器目标位姿，参考坐标系为底座
     * @param target_pose 目标位姿
     * @param allow_tweak 是否允许微调(默认允许)
     * @param allow_feedforward 是否启用前馈计算(默认启用)
     * @return 是否到达成功
     * @note 前馈计算只在允许微调时可以启用
     */
    bool EefPoseCmd::setGoalPoseBase(geometry_msgs::PoseStamped& target_pose, bool allow_tweak, bool allow_feedforward) {
        target_pose.header.frame_id = _plan_frame_;
        target_pose.header.stamp = ros::Time::now();

        // 把开始状态设置为当前状态
        _arm_.setStartStateToCurrentState();
        _current_state_ = _arm_.getCurrentState();

        // 检查是否允许微调
        if(allow_tweak) {
            // 检查是否启用前馈计算
            if(allow_feedforward) {
                target_pose.pose.orientation.w = 1.0;
                target_pose.pose.orientation.x = 0.0;
                target_pose.pose.orientation.y = 0.0;
                target_pose.pose.orientation.z = 0.0;
            }

            ROS_INFO("搜索可达目标位姿中...");
            double step;
            double radius;
            _nh_.param<double>("end_effector/search/step", step, 5.0);
            _nh_.param<double>("end_effector/search/radius", radius, 45.0);

            geometry_msgs::Pose pose_candidate = target_pose.pose;
            if(!searchReachablePose(pose_candidate, step, radius)) return false;

            target_pose.pose = pose_candidate;
        }
        else {
            ROS_INFO("移动到目标位姿中...");
            if(!isIkValid(target_pose.pose)) {
                ROS_ERROR("目标位姿不可达，移动失败。");
                return false;
            }
        }

        // 规划并执行运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        _arm_.setPoseTarget(target_pose);

        bool success = (_arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success) {
            ROS_ERROR("规划到目标位姿失败，移动失败。");
            return false;
        }

        success = (_arm_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!success) {
            ROS_ERROR("执行到目标位姿失败，移动失败。");
            return false;
        }

        ROS_INFO("成功移动到目标位姿。");
        return true;
    }

    /**
     * @brief 移动到机械臂末端执行器目标位姿，参考坐标系为末端
     * @param target_pose 目标位姿
     * @param allow_tweak 是否允许微调(默认允许)
     * @param allow_feedforward 是否启用前馈计算(默认启用)
     * @return 是否到达成功
     * @note 前馈计算只在允许微调时可以启用
     * @note 用于前伸与后缩时，设d为伸缩距离，则目标位姿(x,y,z,roll,pitch,yaw) = (0,0,d,0,0,0)
     * @note 位置变换随简单的加分，但姿态变换是四元数插值，不可直接修改yaw值来实现旋转
     */
    bool EefPoseCmd::setGoalPoseEef(geometry_msgs::PoseStamped& target_pose, bool allow_tweak, bool allow_feedforward) {
        // 把目标位姿从eef坐标系转换到base坐标系
        geometry_msgs::PoseStamped target_pose_base;
        eefTfBase(target_pose, target_pose_base);

        // 调用基坐标系下的移动函数
        return setGoalPoseBase(target_pose_base, allow_tweak, allow_feedforward);
    }

    /**
     * @brief 机械臂末端执行器前伸或后缩
     * @param distance 伸缩距离，正值为前伸，负值为后缩
     * @return 是否到达成功
     */
    bool EefPoseCmd::eefStretch(double distance) {
        geometry_msgs::PoseStamped target_pose_eef;
        target_pose_eef.pose.position.x = 0.0;
        target_pose_eef.pose.position.y = 0.0;
        target_pose_eef.pose.position.z = distance;
        target_pose_eef.pose.orientation.w = 1.0;
        target_pose_eef.pose.orientation.x = 0.0;
        target_pose_eef.pose.orientation.y = 0.0;
        target_pose_eef.pose.orientation.z = 0.0;

        return setGoalPoseEef(target_pose_eef, true, false);
    }

    /**
     * @brief 机械臂末端执行器旋转
     * @param angle 旋转角度，单位为度，正值为逆时针旋转，负值为顺时针旋转
     * @return 是否到达成功
     */
    bool EefPoseCmd::eefRotate(double angle) {
        angle = angle * M_PI / 180.0; // 转换为弧度

        geometry_msgs::Pose current = getCurrentEefPose();

        tf2::Quaternion q_current;
        tf2::fromMsg(current.orientation, q_current);

        tf2::Quaternion q_delta;
        q_delta.setRPY(0, 0, angle);

        tf2::Quaternion q_new = q_current * q_delta;

        geometry_msgs::PoseStamped target;
        target.header.frame_id = _plan_frame_;
        target.pose = current;
        target.pose.orientation = tf2::toMsg(q_new);

        return setGoalPoseBase(target, true, false);
    }

    /**
     * @brief 重置机械臂到初始位置
     */
    void EefPoseCmd::resetToZero(void) {
        ROS_INFO("重置机械臂到初始位置...");
        _arm_.setNamedTarget("zero");
        _arm_.move();
    }

    /**
     * @brief 获取当前机械臂末端执行器位姿
     * @return 末端执行器位姿
     */
    geometry_msgs::Pose EefPoseCmd::getCurrentEefPose(void) {
        return _arm_.getCurrentPose().pose;
    }

    /**
     * @brief 获取当前机械臂关节位置
     * @return 关节位置向量
     */
    std::vector<double> EefPoseCmd::getCurrentJointPose(void) {
        return _arm_.getCurrentJointValues();
    }

    /**
     * @brief 任务组规划器构造函数
     * @param eef_cmd 机械臂末端执行器位姿控制类引用
     */
    TaskGroupPlanner::TaskGroupPlanner(EefPoseCmd& eef_cmd)
        : _eef_cmd_(eef_cmd) {
        ros::NodeHandle nh;
        nh.param<bool>("task_planner/enable_optimization", _enable_optimization_, true);
        nh.param<std::string>("task_planner/optimization_method", _optimization_method_, "greedy");
        nh.param<double>("task_planner/pick_height_offset", _pick_height_offset_, 0.1);
        nh.param<double>("task_planner/place_height_offset", _place_height_offset_, 0.1);
        nh.param<double>("task_planner/approach_distance", _approach_distance_, 0.05);
        nh.param<double>("task_planner/retreat_distance", _retreat_distance_, 0.05);
        nh.param<double>("task_planner/default_wait_time", _default_wait_time_, 0.5);
        nh.param<double>("task_planner/gripper_action_time", _gripper_action_time_, 1.0);
    }

    /**
     * @brief 添加任务目标到任务列表
     * @param target 任务目标
     */
    void TaskGroupPlanner::add(const TaskTarget_t& target) {
        _task_list_.push_back(target);
    }

    /**
     * @brief 清空任务列表
     */
    void TaskGroupPlanner::clear() {
        _task_list_.clear();
    }

    /**
     * @brief 执行路径优化后的所有任务
     * @note 采用贪婪最近邻算法对点位进行排序，实现最短的移动路径
     */
    void TaskGroupPlanner::executeAll() {
        if(_task_list_.empty()) {
            ROS_ERROR("任务列表为空，无法执行任务。");
            return;
        }

        ROS_INFO("开始规划任务组，共 %lu 个目标点。", _task_list_.size());

        // 初始化排序结果、待处理任务列表与当前末端位姿
        std::vector<TaskTarget_t> sorted_tasks;
        std::vector<TaskTarget_t> pending_tasks = _task_list_;
        geometry_msgs::Pose current_pose = _eef_cmd_.getCurrentEefPose();

        // 距离计算Lambda函数
        auto calc_dist = [](const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2) {
            const double dx = pose1.position.x - pose2.position.x;
            const double dy = pose1.position.y - pose2.position.y;
            const double dz = pose1.position.z - pose2.position.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
            };

        // 执行排序
        while(!pending_tasks.empty()) {
            // 找到距离当前排序位姿最近的任务
            auto nearest_it = std::min_element(
                pending_tasks.begin(),
                pending_tasks.end(),
                [&](const auto& a, const auto& b) {
                    return calc_dist(current_pose, a.pose) < calc_dist(current_pose, b.pose);
                }
            );
            // 更新排序结果与当前排序位姿
            sorted_tasks.push_back(*nearest_it);
            current_pose = nearest_it->pose;
            pending_tasks.erase(nearest_it);
        }

        // 依次执行排序后的任务
        std::size_t task_index = 1;
        for(const auto& task : sorted_tasks) {
            ROS_INFO("任务 [%zu/%zu] 开始执行。", task_index, sorted_tasks.size());
            bool success = false;

            geometry_msgs::PoseStamped target_pose;
            target_pose.pose = task.pose;
            success = _eef_cmd_.setGoalPoseBase(target_pose);

            if(!success) ROS_WARN("任务 [%zu] 失败。", task_index);

            // 执行任务动作
            if(task.action == TargetAction_e::PICK && success) {

                // TODO: 执行采摘动作
            }
            else if(task.action == TargetAction_e::STRETCH && success) {

                // TODO: 根据视觉反馈调整伸缩参数
                if(task.param1 == 0.0) {

                }

                success = _eef_cmd_.eefStretch(task.param1);
                if(!success) ROS_WARN("任务 [%zu] 伸缩动作失败。", task_index);
            }
            else if(task.action == TargetAction_e::ROTATE && success) {

                // TODO: 根据视觉反馈调整旋转参数
                if(task.param1 == 0.0) {

                }

                success = _eef_cmd_.eefRotate(task.param1);
                if(!success) ROS_WARN("任务 [%zu] 旋转动作失败。", task_index);
            }

            // 等待时间：优先使用任务指定的等待时间，否则使用默认等待时间
            double wait = (task.wait_time > 0.0) ? task.wait_time : _default_wait_time_;
            if(wait > 0.0) ros::Duration(wait).sleep();

            ++task_index;
        }

        ROS_INFO("所有任务执行完毕。");
        clear();
    }

}
