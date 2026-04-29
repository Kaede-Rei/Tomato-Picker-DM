#ifndef DM_HW_DM_HARDWARE_INTERFACE_HPP
#define DM_HW_DM_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

#include "dm_hw/damiao.hpp"

namespace dm_hw {

class DmHardwareInterface : public hardware_interface::RobotHW {
public:
    explicit DmHardwareInterface(ros::NodeHandle& nh);
    ~DmHardwareInterface() override;

    bool init();
    void read();
    void write();
    void returnZero();

    double control_frequency() const { return control_frequency_; }

private:
    bool load_params();
    bool connect_hardware();
    bool setup_motors();
    void register_ros_control_interfaces();
    speed_t baudrate_to_speed(int baudrate) const;

private:
    ros::NodeHandle nh_;

    std::shared_ptr<SerialPort> serial_;
    std::shared_ptr<damiao::MotorControl> motor_controller_;
    std::vector<std::shared_ptr<damiao::Motor>> motors_;

    std::string serial_port_{ "/dev/ttyACM0" };
    int baudrate_{ 921600 };
    double control_frequency_{ 500.0 };
    bool use_mit_mode_{ false };
    double kp_{ 30.0 };
    double kd_{ 1.0 };
    double max_position_change_{ 0.5 };
    double max_velocity_{ 3.0 };
    bool enable_write_{ true };
    bool enable_read_refresh_{ true };
    bool return_zero_on_shutdown_{ false };

    std::vector<std::string> joint_names_;
    std::vector<int> motor_ids_;
    std::vector<int> motor_types_;
    std::vector<int> master_ids_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_position_command_prev_;

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
};

} // namespace dm_hw

#endif
