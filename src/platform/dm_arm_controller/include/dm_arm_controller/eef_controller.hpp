#ifndef _eef_controller_hpp_
#define _eef_controller_hpp_

#include "dm_arm_controller/eef_interface.hpp"
#include "serial_driver/serial_driver.hpp"
#include "tl_optional/optional.hpp"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //


/**
 * @brief 双指夹爪末端执行器类，支持关节控制和力反馈
 */
class TwoFingerGripper : public EndEffector,
    public GripperEefInterface,
    public JointEefInterface,
    public ForceFeedbackEefInterface {
public:
    /**
     * @brief TwoFingerGripper 构造函数
     * @param eef_name 末端执行器名称
     */
    explicit TwoFingerGripper(const std::string& eef_name = "gripper");
    /**
     * @brief TwoFingerGripper 析构函数
     */
    ~TwoFingerGripper() override = default;

    TwoFingerGripper(const TwoFingerGripper&) = delete;
    TwoFingerGripper& operator=(const TwoFingerGripper&) = delete;
    TwoFingerGripper(TwoFingerGripper&&) = delete;
    TwoFingerGripper& operator=(TwoFingerGripper&&) = delete;

    /**
     * @brief 停止末端执行器动作
     */
    void stop() override;

    /**
     * @brief 获取规划组名称
     * @return 规划组名称
     */
    const std::string& get_group_name() const override;
    /**
     * @brief 获取 MoveGroupInterface 对象
     * @return MoveGroupInterface 引用
     */
    moveit::planning_interface::MoveGroupInterface& get_move_group() override;
    /**
     * @brief 打开夹爪
     * @return 错误码
     */
    ErrorCode open() override;
    /**
     * @brief 关闭夹爪
     * @return 错误码
     */
    ErrorCode close() override;
    /**
     * @brief 设置夹爪角度
     * @param angle 目标角度
     * @return 错误码
     */
    ErrorCode set_angle(double angle) override;
    /**
     * @brief 获取夹爪角度
     * @return 夹爪角度
     */
    tl::optional<double> get_angle() const override;
    /**
     * @brief 执行预设位姿
     * @param pose_name 位姿名称
     * @return 错误码
     */
    ErrorCode execute_preset_pose(const std::string& pose_name) override;
    /**
     * @brief 设置单关节值
     * @param joint_name 关节名称
     * @param value 目标值
     * @return 错误码
     */
    ErrorCode set_joint_value(const std::string& joint_name, double value) override;
    /**
     * @brief 设置全部关节值
     * @param joint_values 关节值列表
     * @return 错误码
     */
    ErrorCode set_joint_values(const std::vector<double>& joint_values) override;
    /**
     * @brief 规划末端轨迹
     * @param plan 规划输出
     * @return 错误码
     */
    ErrorCode plan(moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    /**
     * @brief 执行末端轨迹
     * @param plan 轨迹输入
     * @return 错误码
     */
    ErrorCode execute(const moveit::planning_interface::MoveGroupInterface::Plan& plan) override;
    /**
     * @brief 规划并执行末端轨迹
     * @return 错误码
     */
    ErrorCode plan_and_execute() override;

    /**
     * @brief 获取当前关节角
     * @return 当前关节角列表
     */
    std::vector<double> get_current_joints() const override;
    /**
     * @brief 获取当前连杆名称
     * @return 当前连杆名称列表
     */
    std::vector<std::string> get_current_link_names() const override;

    /**
     * @brief 获取力反馈名称列表
     * @return 力反馈名称列表
     */
    std::vector<std::string> get_force_names() const override;
    /**
     * @brief 获取指定力反馈值
     * @param force_name 力反馈名称
     * @return 力反馈值
     */
    tl::optional<double> get_force(const std::string& force_name) const override;

private:
    moveit::planning_interface::MoveGroupInterface _gripper_;
};

/**
 * @brief 总线舵机末端执行器类
 */
class ServoGripper : public EndEffector,
    public GripperEefInterface {
public:
    /**
     * @brief ServoGripper 构造函数
     * @param nh ROS 节点句柄
     * @param serial_port 串口名
     * @param baud_rate 波特率
     */
    ServoGripper(ros::NodeHandle& nh, const std::string& serial_port = "/dev/ttyACM0", int baud_rate = 115200);
    /**
     * @brief ServoGripper 析构函数
     */
    ~ServoGripper() override = default;

    ServoGripper(const ServoGripper&) = delete;
    ServoGripper& operator=(const ServoGripper&) = delete;
    ServoGripper(ServoGripper&&) = delete;
    ServoGripper& operator=(ServoGripper&&) = delete;

    /**
     * @brief 停止末端执行器动作
     */
    void stop() override;

    /**
     * @brief 打开夹爪
     * @return 错误码
     */
    ErrorCode open() override;
    /**
     * @brief 关闭夹爪
     * @return 错误码
     */
    ErrorCode close() override;
    /**
     * @brief 设置夹爪角度
     * @param angle 目标角度
     * @return 错误码
     */
    ErrorCode set_angle(double angle) override;
    /**
     * @brief 获取夹爪角度
     * @return 夹爪角度
     */
    tl::optional<double> get_angle() const override;

private:
    STM32Serial _serialer_;
    tl::optional<double> _current_angle_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! ///



} /* namespace dm_arm */

#endif
