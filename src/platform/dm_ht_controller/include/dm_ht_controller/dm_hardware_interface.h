#ifndef DM_HARDWARE_INTERFACE_H
#define DM_HARDWARE_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "dm_ht_controller/damiao.h"
#include <vector>
#include <memory>
#include <string>

/**
 * @brief 达妙机械臂硬件接口类，实现ros_control的RobotHW接口
 * @details 支持达妙电机的MIT模式和位置速度模式控制(MIT模式对其他轴有bug)，方法有：
 *          - 构造函数：初始化成员变量
 *          - 析构函数：归零并失能电机
 *          - init(): 初始化硬件接口，读取参数，创建电机对象，注册ros_control接口
 *          - read(): 从电机读取当前状态，更新关节位置、速度和力矩
 *          - write(): 根据控制模式发送命令到电机
 *          - returnZero(): 将所有关节位置命令设为零位置
 */
class DMHardwareInterface : public hardware_interface::RobotHW {
public:
    DMHardwareInterface(ros::NodeHandle& nh);
    ~DMHardwareInterface();

    bool init();
    void read();
    void write();
    void returnZero();

private:
    ros::NodeHandle nh_;

    // 达妙电机控制器
    std::shared_ptr<damiao::Motor_Control> motor_controller_;

    // 串口对象指针
    std::shared_ptr<SerialPort> serial_;

    // 电机对象列表
    std::vector<std::shared_ptr<damiao::Motor>> motors_;

    // 配置参数
    std::string serial_port_;
    int baudrate_;
    std::vector<std::string> joint_names_;
    std::vector<int> motor_ids_;
    std::vector<int> motor_types_;

    // 关节数据
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;

    // 上一次的命令位置（用于计算速度）
    std::vector<double> joint_position_command_prev_;

    // ros_control接口
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    // 控制参数
    double control_frequency_;
    bool use_mit_mode_;     // true: MIT模式, false: 位置速度模式
    double kp_;             // MIT模式的比例增益
    double kd_;             // MIT模式的微分增益

    // 安全限制
    double max_position_change_;    // 单次最大位置变化
    double max_velocity_;           // 最大速度
    bool enable_write_;             // 是否启用写入
};

#endif
