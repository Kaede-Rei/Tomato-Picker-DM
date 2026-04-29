#include "dm_hw/dm_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <stdexcept>
#include <unistd.h>

namespace dm_hw {
namespace {

constexpr double kMinShutdownDuration = 0.5;

} // namespace

DmHardwareInterface::DmHardwareInterface(ros::NodeHandle& nh)
    : nh_(nh) {}

DmHardwareInterface::~DmHardwareInterface() {
    if(return_zero_on_shutdown_) {
        try {
            returnZero();
        }
        catch(const std::exception& e) {
            ROS_ERROR("关机回零失败: %s", e.what());
        }
    }

    if(motor_controller_) {
        for(auto& motor : motors_) {
            try {
                motor_controller_->disable(*motor);
            }
            catch(const std::exception& e) {
                ROS_ERROR("失能电机失败: %s", e.what());
            }
        }
    }
}

bool DmHardwareInterface::init() {
    if(!load_params()) return false;
    if(!connect_hardware()) return false;
    if(!setup_motors()) return false;

    const std::size_t joint_count = joint_names_.size();
    joint_position_.assign(joint_count, 0.0);
    joint_velocity_.assign(joint_count, 0.0);
    joint_effort_.assign(joint_count, 0.0);
    joint_position_command_.assign(joint_count, 0.0);
    joint_position_command_prev_.assign(joint_count, 0.0);

    usleep(200 * 1000);
    for(int sample = 0; sample < 5; ++sample) {
        read();
        usleep(20 * 1000);
    }

    joint_position_command_ = joint_position_;
    joint_position_command_prev_ = joint_position_;

    register_ros_control_interfaces();

    ROS_INFO("dm_hw 初始化完成: joints=%zu, control_frequency=%.1f Hz, mode=%s, enable_write=%s",
             joint_names_.size(),
             control_frequency_,
             use_mit_mode_ ? "MIT" : "POS_VEL",
             enable_write_ ? "true" : "false");
    return true;
}

bool DmHardwareInterface::load_params() {
    nh_.param<std::string>("dm_arm_hardware/serial_port", serial_port_, serial_port_);
    nh_.param<int>("dm_arm_hardware/baudrate", baudrate_, baudrate_);
    nh_.param<double>("dm_arm_hardware/control_frequency", control_frequency_, control_frequency_);
    nh_.param<bool>("dm_arm_hardware/use_mit_mode", use_mit_mode_, use_mit_mode_);
    nh_.param<double>("dm_arm_hardware/kp", kp_, kp_);
    nh_.param<double>("dm_arm_hardware/kd", kd_, kd_);
    nh_.param<double>("dm_arm_hardware/max_position_change", max_position_change_, max_position_change_);
    nh_.param<double>("dm_arm_hardware/max_velocity", max_velocity_, max_velocity_);
    nh_.param<bool>("dm_arm_hardware/enable_write", enable_write_, enable_write_);
    nh_.param<bool>("dm_arm_hardware/enable_read_refresh", enable_read_refresh_, enable_read_refresh_);
    nh_.param<bool>("dm_arm_hardware/return_zero_on_shutdown", return_zero_on_shutdown_, return_zero_on_shutdown_);

    if(!nh_.getParam("dm_arm_hardware/joints/names", joint_names_)) {
        ROS_ERROR("缺少参数 dm_arm_hardware/joints/names");
        return false;
    }
    if(!nh_.getParam("dm_arm_hardware/joints/motor_ids", motor_ids_)) {
        ROS_ERROR("缺少参数 dm_arm_hardware/joints/motor_ids");
        return false;
    }
    if(!nh_.getParam("dm_arm_hardware/joints/motor_types", motor_types_)) {
        ROS_ERROR("缺少参数 dm_arm_hardware/joints/motor_types");
        return false;
    }

    if(!nh_.getParam("dm_arm_hardware/joints/master_ids", master_ids_)) {
        master_ids_.assign(joint_names_.size(), 0x00);
    }

    if(joint_names_.empty()) {
        ROS_ERROR("dm_hw 至少需要一个关节");
        return false;
    }

    const std::size_t joint_count = joint_names_.size();
    if(motor_ids_.size() != joint_count ||
       motor_types_.size() != joint_count ||
       master_ids_.size() != joint_count) {
        ROS_ERROR("dm_arm_hardware/joints 下 names、motor_ids、motor_types、master_ids 数量必须一致");
        return false;
    }

    if(control_frequency_ <= 0.0) {
        ROS_ERROR("control_frequency 必须大于 0");
        return false;
    }

    return true;
}

bool DmHardwareInterface::connect_hardware() {
    try {
        serial_ = std::make_shared<SerialPort>(serial_port_, baudrate_to_speed(baudrate_));
        motor_controller_ = std::make_shared<damiao::MotorControl>(serial_);
    }
    catch(const std::exception& e) {
        ROS_ERROR("dm_hw 打开硬件失败: %s", e.what());
        return false;
    }

    ROS_INFO("dm_hw 串口已打开: %s @ %d", serial_port_.c_str(), baudrate_);
    return true;
}

bool DmHardwareInterface::setup_motors() {
    for(std::size_t i = 0; i < joint_names_.size(); ++i) {
        const auto motor_type = static_cast<damiao::DmMotorType>(motor_types_[i]);
        auto motor = std::make_shared<damiao::Motor>(
            motor_type,
            static_cast<damiao::MotorId>(motor_ids_[i]),
            static_cast<damiao::MotorId>(master_ids_[i]));

        motor_controller_->add_motor(motor.get());
        motors_.push_back(motor);

        ROS_INFO("dm_hw 添加电机: joint=%s, slave_id=0x%X, master_id=0x%X, type=%d",
                 joint_names_[i].c_str(), motor_ids_[i], master_ids_[i], motor_types_[i]);
    }

    for(std::size_t i = 0; i < motors_.size(); ++i) {
        try {
            motor_controller_->enable(*motors_[i]);
            if(!use_mit_mode_) {
                const bool switched = motor_controller_->switch_control_mode(*motors_[i], damiao::POS_VEL_MODE);
                if(!switched) {
                    ROS_WARN("电机 %s 切换 POS_VEL_MODE 未收到确认", joint_names_[i].c_str());
                }
            }
            ROS_INFO("dm_hw 电机已使能: %s", joint_names_[i].c_str());
            usleep(100 * 1000);
        }
        catch(const std::exception& e) {
            ROS_ERROR("dm_hw 使能电机 %s 失败: %s", joint_names_[i].c_str(), e.what());
            return false;
        }
    }

    return true;
}

void DmHardwareInterface::register_ros_control_interfaces() {
    for(std::size_t i = 0; i < joint_names_.size(); ++i) {
        hardware_interface::JointStateHandle state_handle(
            joint_names_[i],
            &joint_position_[i],
            &joint_velocity_[i],
            &joint_effort_[i]);
        joint_state_interface_.registerHandle(state_handle);

        hardware_interface::JointHandle position_handle(
            joint_state_interface_.getHandle(joint_names_[i]),
            &joint_position_command_[i]);
        position_joint_interface_.registerHandle(position_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void DmHardwareInterface::read() {
    if(!motor_controller_) return;

    for(std::size_t i = 0; i < motors_.size(); ++i) {
        try {
            if(enable_read_refresh_) {
                motor_controller_->refresh_motor_status(*motors_[i]);
            }

            const double position = motors_[i]->get_position();
            const double velocity = motors_[i]->get_velocity();
            const double effort = motors_[i]->get_tau();

            if(std::isfinite(position)) {
                joint_position_[i] = position;
            }
            else {
                ROS_WARN_THROTTLE(1.0, "关节 %s 位置反馈无效，保持上一周期状态", joint_names_[i].c_str());
            }

            joint_velocity_[i] = std::isfinite(velocity) ? velocity : 0.0;
            joint_effort_[i] = std::isfinite(effort) ? effort : 0.0;
        }
        catch(const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "读取电机 %s 失败: %s", joint_names_[i].c_str(), e.what());
        }
    }
}

void DmHardwareInterface::write() {
    if(!enable_write_ || !motor_controller_) return;

    const double dt = 1.0 / control_frequency_;
    for(std::size_t i = 0; i < motors_.size(); ++i) {
        try {
            double cmd = joint_position_command_[i];
            const double prev = joint_position_command_prev_[i];
            const double position_change = cmd - prev;

            if(std::abs(position_change) > max_position_change_) {
                cmd = prev + std::copysign(max_position_change_, position_change);
                joint_position_command_[i] = cmd;
                ROS_WARN_THROTTLE(1.0,
                                  "关节 %s 单周期命令变化 %.4f rad 超限，已限制为 %.4f rad",
                                  joint_names_[i].c_str(), position_change, cmd - prev);
            }

            double target_velocity = (cmd - prev) / dt;
            if(std::abs(target_velocity) < 1e-6) {
                target_velocity = 10.0 * (cmd - joint_position_[i]);
            }
            target_velocity = std::clamp(target_velocity, -max_velocity_, max_velocity_);

            if(use_mit_mode_) {
                motor_controller_->control_mit(*motors_[i], kp_, kd_, static_cast<float>(cmd), static_cast<float>(target_velocity), 0.0f);
            }
            else {
                motor_controller_->control_pos_vel(*motors_[i], static_cast<float>(cmd), static_cast<float>(target_velocity));
            }

            joint_position_command_prev_[i] = cmd;
        }
        catch(const std::exception& e) {
            ROS_ERROR_THROTTLE(1.0, "控制电机 %s 失败: %s", joint_names_[i].c_str(), e.what());
        }
    }
}

void DmHardwareInterface::returnZero() {
    if(!motor_controller_ || motors_.empty() || joint_position_.empty()) return;

    ROS_INFO("dm_hw 开始回零");
    read();

    const std::vector<double> start = joint_position_;
    std::vector<double> target(start.size(), 0.0);

    double max_diff = 0.0;
    for(std::size_t i = 0; i < start.size(); ++i) {
        max_diff = std::max(max_diff, std::abs(start[i] - target[i]));
    }

    const double safe_velocity = std::max(0.1, std::min(max_velocity_, 0.5));
    const double duration = std::max(kMinShutdownDuration, max_diff / safe_velocity);
    const int steps = std::max(1, static_cast<int>(duration * control_frequency_));

    for(int step = 0; ros::ok() && step <= steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        const double alpha = t * t * (3.0 - 2.0 * t);
        for(std::size_t i = 0; i < joint_position_command_.size(); ++i) {
            joint_position_command_[i] = start[i] + (target[i] - start[i]) * alpha;
        }
        write();
        usleep(static_cast<useconds_t>(1000000.0 / control_frequency_));
    }
    ROS_INFO("dm_hw 回零完成");
}

speed_t DmHardwareInterface::baudrate_to_speed(int baudrate) const {
    switch(baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        default:
            ROS_WARN("未知波特率 %d，回退到 921600", baudrate);
            return B921600;
    }
}

} // namespace dm_hw
