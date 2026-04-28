#include "dm_ht_controller/dm_hardware_interface.h"

// LEAD = 直线导轨转换比 (m / rav)
constexpr double LEAD = 0.053;
constexpr double TWO_PI = 2.0 * M_PI;

/**
 * @brief 构造函数，初始化成员变量
 * @param nh ROS节点句柄
 */
DMHardwareInterface::DMHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh),
    control_frequency_(500.0),
    use_mit_mode_(false),
    kp_(30.0),
    kd_(1.0),
    max_position_change_(0.5),
    max_velocity_(3.0),
    enable_write_(true) {}

/**
 * @brief 析构函数，归零并失能所有电机
 */
DMHardwareInterface::~DMHardwareInterface() {
    // 归零
    returnZero();

    // 失能所有电机
    for(auto& motor : motors_) {
        try {
            motor_controller_->disable(*motor);
        }
        catch(const std::exception& e) {
            ROS_ERROR("失能电机时出错: %s", e.what());
        }
    }
}

/**
 * @brief 初始化硬件接口，读取参数，创建电机对象并使能电机
 */
bool DMHardwareInterface::init() {
    // 读取参数
    nh_.param<std::string>("dm_arm_hardware/serial_port", serial_port_, "/dev/ttyACM0");
    nh_.param<int>("dm_arm_hardware/baudrate", baudrate_, 921600);
    nh_.param<double>("dm_arm_hardware/control_frequency", control_frequency_, 500.0);
    nh_.param<bool>("dm_arm_hardware/use_mit_mode", use_mit_mode_, false);
    nh_.param<double>("dm_arm_hardware/kp", kp_, 30.0);
    nh_.param<double>("dm_arm_hardware/kd", kd_, 1.0);
    nh_.param<double>("dm_arm_hardware/max_position_change", max_position_change_, 0.5);
    nh_.param<double>("dm_arm_hardware/max_velocity", max_velocity_, 3.0);
    nh_.param<bool>("dm_arm_hardware/enable_write", enable_write_, true);

    if(!nh_.getParam("joints/names", joint_names_)) {
        ROS_ERROR("获取 joints/names 参数失败");
        return false;
    }

    if(!nh_.getParam("joints/motor_ids", motor_ids_)) {
        ROS_ERROR("获取 joints/motor_ids 参数失败");
        return false;
    }

    if(!nh_.getParam("joints/motor_types", motor_types_)) {
        ROS_ERROR("获取 joints/motor_types 参数失败");
        return false;
    }

    if(joint_names_.size() != motor_ids_.size() ||
        joint_names_.size() != motor_types_.size()) {
        ROS_ERROR("尺寸不匹配: joint_names, motor_ids 和 motor_types 必须具有相同的长度");
        return false;
    }

    int num_joints = joint_names_.size();

    // 初始化数据容器
    joint_position_.resize(num_joints, 0.0);
    joint_velocity_.resize(num_joints, 0.0);
    joint_effort_.resize(num_joints, 0.0);
    joint_position_command_.resize(num_joints, 0.0);
    joint_velocity_command_.resize(num_joints, 0.0);
    joint_position_command_prev_.resize(num_joints, 0.0);

    // 创建串口对象
    try {
        // 将 serial 保存到成员变量 serial_ 中
        serial_ = std::make_shared<SerialPort>(serial_port_, baudrate_);
        motor_controller_ = std::make_shared<damiao::Motor_Control>(serial_);
        ROS_INFO("串口 %s 打开成功", serial_port_.c_str());
    }
    catch(const std::exception& e) {
        ROS_ERROR("打开串口失败: %s", e.what());
        return false;
    }

    // 创建电机对象
    for(size_t i = 0; i < num_joints; ++i) {
        auto motor_type = static_cast<damiao::DM_Motor_Type>(motor_types_[i]);
        auto motor = std::make_shared<damiao::Motor>(
            motor_type,
            motor_ids_[i],
            0x00
        );

        motors_.push_back(motor);
        motor_controller_->addMotor(motor.get());

        ROS_INFO("添加电机: 关节=%s, ID=%d, 类型=%d",
            joint_names_[i].c_str(), motor_ids_[i], motor_types_[i]);
    }

    // 使能电机
    for(size_t i = 0; i < motors_.size(); ++i) {
        try {
            motor_controller_->enable(*motors_[i]);
            ROS_INFO("已使能电机 %s (ID=%d)",
                joint_names_[i].c_str(), motor_ids_[i]);
        }
        catch(const std::exception& e) {
            ROS_ERROR("使能电机 %s 失败: %s",
                joint_names_[i].c_str(), e.what());
            return false;
        }
    }

    // 除了 JOINT1 外切换到位置速度模式
    if(!use_mit_mode_) {
        for(size_t i = 1; i < motors_.size(); ++i) {
            try {
                bool success = motor_controller_->switchControlMode(
                    *motors_[i], damiao::POS_VEL_MODE);

                if(success) {
                    ROS_INFO("电机 %s 已切换到位置速度模式",
                        joint_names_[i].c_str());
                }
                else {
                    ROS_WARN("电机 %s 可能未正确切换模式",
                        joint_names_[i].c_str());
                }
                usleep(200 * 1000);
            }
            catch(const std::exception& e) {
                ROS_ERROR("切换电机 %s 控制模式失败: %s",
                    joint_names_[i].c_str(), e.what());
            }
        }
    }

    // 读取初始位置
    usleep(200 * 1000);
    std::vector<double> pos_sum(num_joints, 0.0);
    int read_count = 5;

    for(int j = 0; j < read_count; ++j) {
        read();
        for(size_t i = 0; i < num_joints; ++i) {
            pos_sum[i] += joint_position_[i];
        }
        usleep(20 * 1000);
    }

    // 计算平均值并设置为初始命令
    for(size_t i = 0; i < num_joints; ++i) {
        joint_position_[i] = pos_sum[i] / read_count;
        joint_position_command_[i] = joint_position_[i];
        joint_position_command_prev_[i] = joint_position_[i];
        joint_velocity_command_[i] = 0.0;

        ROS_INFO("关节 %s 初始位置: %.3f rad",
            joint_names_[i].c_str(), joint_position_[i]);
    }

    // 注册到hardware_interface
    for(size_t i = 0; i < num_joints; ++i) {
        // 注册joint_state_interface
        hardware_interface::JointStateHandle state_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]
        );
        joint_state_interface_.registerHandle(state_handle);

        // 注册position_joint_interface
        hardware_interface::JointHandle position_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]
        );
        position_joint_interface_.registerHandle(position_handle);
    }

    // 注册接口
    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);

    ROS_INFO("DM 硬件接口初始化完成，共 %d 个关节", num_joints);
    ROS_INFO("控制模式: %s, 频率: %.1f Hz",
        use_mit_mode_ ? "MIT" : "位置速度", control_frequency_);

    return true;
}

/**
 * @brief 从电机读取状态数据，更新关节位置、速度和力矩
 */
void DMHardwareInterface::read() {
    // 从电机读取状态
    for(size_t i = 0; i < motors_.size(); ++i) {
        try {
            joint_position_[i] = motors_[i]->Get_Position();
            if(joint_names_[i] == "gripper_left") joint_position_[i] = joint_position_[i] / TWO_PI * LEAD;
            joint_velocity_[i] = motors_[i]->Get_Velocity();
            joint_effort_[i] = motors_[i]->Get_tau();

            // 检测异常值
            if(std::isnan(joint_position_[i]) || std::isinf(joint_position_[i])) {
                ROS_WARN_THROTTLE(1.0, "关节 %s 读取到无效位置值",
                    joint_names_[i].c_str());
                joint_position_[i] = joint_position_command_prev_[i];
            }
        }
        catch(const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "读取电机 %s 错误: %s",
                joint_names_[i].c_str(), e.what());
        }
    }
}

/**
 * @brief 向电机发送控制命令，根据模式计算目标速度并应用安全限制
 */
void DMHardwareInterface::write() {
    if(!enable_write_) return;

    double dt = 1.0 / control_frequency_;

    for(size_t i = 0; i < motors_.size(); ++i) {
        try {
            // 判断是否为直线导轨关节
            const bool is_gripper = (joint_names_[i] == "gripper_left");
            const double scale_to_motor = is_gripper ? (TWO_PI / LEAD) : 1.0;

            // 转换为电机单位（rad）
            double cmd_motor = joint_position_command_[i] * scale_to_motor;
            double prev_motor = joint_position_command_prev_[i] * scale_to_motor;
            double meas_motor = joint_position_[i] * scale_to_motor;

            // 计算误差与变化量（电机单位）
            double position_error_motor = cmd_motor - meas_motor;
            double position_change_motor = cmd_motor - prev_motor;

            // 安全限制：限制单次位置变化（电机单位：rad）
            if(std::abs(position_change_motor) > max_position_change_) {
                ROS_WARN_THROTTLE(1.0,
                    "关节 %s: 位置变化过大 (%.3f rad), 限制为 %.3f rad",
                    joint_names_[i].c_str(), position_change_motor, max_position_change_);
                position_change_motor = std::copysign(max_position_change_, position_change_motor);
                cmd_motor = prev_motor + position_change_motor;
            }

            // 目标速度（电机单位）：MIT与位置速度模式
            double target_velocity_motor;
            if(use_mit_mode_) {
                target_velocity_motor = position_change_motor / dt;
            }
            else {
                const double kp_tracking = 10.0;
                target_velocity_motor = kp_tracking * position_error_motor;

                if(std::abs(position_change_motor) > 1e-4) {
                    target_velocity_motor += position_change_motor / dt;
                }
            }

            // 限速（电机单位）
            target_velocity_motor = std::max(-max_velocity_, std::min(max_velocity_, target_velocity_motor));

            // 下发命令（电机单位）
            if(use_mit_mode_) {
                motor_controller_->control_mit(
                    *motors_[i],
                    kp_,
                    kd_,
                    cmd_motor,
                    target_velocity_motor,
                    0.0f
                );
            }
            else {
                if(i == 0) {
                    motor_controller_->control_mit(
                        *motors_[i],
                        kp_,
                        kd_,
                        cmd_motor,
                        target_velocity_motor,
                        0.0f
                    );
                }
                else {
                    motor_controller_->control_pos_vel(
                        *motors_[i],
                        cmd_motor,
                        target_velocity_motor
                    );
                }
            }

            // 更新上一周期命令
            joint_position_command_prev_[i] = cmd_motor / scale_to_motor;
        }
        catch(const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "控制电机 %s 错误: %s",
                joint_names_[i].c_str(), e.what());
        }
    }
}

/**
 * @brief 将所有关节位置命令设为零位置，平滑过渡以确保安全
 */
void DMHardwareInterface::returnZero() {
    ROS_INFO("正在返回零位姿态以准备关闭...");

    // Zero姿态
    std::vector<double> target_pos = { 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000 };

    if(joint_position_.size() != target_pos.size()) {
        ROS_WARN("关节数量不匹配，跳过归零操作。");
        return;
    }

    // 计算最大偏差
    read();
    double max_diff = 0.0;
    for(size_t i = 0; i < joint_position_.size(); ++i) {
        double diff = std::abs(joint_position_[i] - target_pos[i]);
        if(diff > max_diff) max_diff = diff;
    }

    // 规划时间
    double safe_vel = 0.5;
    double duration = max_diff / safe_vel;

    // 限制偏差较大时的最小时间，保证平滑且不过慢
    if(duration < 2.0) duration = 2.0;
    if(max_diff < 0.05) duration = 0.5;

    ROS_INFO("最大关节偏差: %.3f rad. 将在 %.1f 秒内返回零位。", max_diff, duration);

    int steps = static_cast<int>(duration * control_frequency_);
    std::vector<double> start_pos = joint_position_;

    for(int step = 0; step <= steps; ++step) {
        // 读取当前状态以更新反馈
        read();

        double t = static_cast<double>(step) / steps;
        // 使用平滑插值 (Smoothstep): 3t^2 - 2t^3
        double alpha = t * t * (3.0 - 2.0 * t);

        for(size_t i = 0; i < joint_position_.size(); ++i) {
            joint_position_command_[i] = start_pos[i] + (target_pos[i] - start_pos[i]) * alpha;
        }

        // 发送控制指令
        write();

        usleep(static_cast<useconds_t>(1000000.0 / control_frequency_));
    }

    // 保持阶段：延长时间并监控误差
    ROS_INFO("保持位置以稳定...");
    int hold_steps = static_cast<int>(2.5 * control_frequency_);
    for(int i = 0; i < hold_steps; ++i) {
        read();

        // 持续发送目标位置
        for(size_t j = 0; j < joint_position_.size(); ++j) {
            joint_position_command_[j] = target_pos[j];
        }
        write();

        usleep(static_cast<useconds_t>(1000000.0 / control_frequency_));
    }

    ROS_INFO("已返回零位姿态。");
}
