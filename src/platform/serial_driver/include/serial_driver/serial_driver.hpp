#ifndef _serial_driver_hpp_
#define _serial_driver_hpp_

#include <ros/ros.h>

#include <serial/serial.h>
#include <linux/can.h>
#include <net/if.h>

/* ========================= Typedef / 量 定 义 ========================= */

typedef struct can_frame CanFrame_t;
typedef struct sockaddr_can CanAddr_t;
typedef struct ifreq CanIfReq_t;

/* ========================= 接 口 A P I 声 明 ========================= */

/**
 * @brief STM32 串口通信类
 * @details 封装了与 STM32 通过串口进行通信的功能，包括：
 *          - 连接/重连与断开连接
 *          - 发送与接收数据(含超时接收/全部接收)
 *          - 清空缓冲区
 */
class STM32Serial{
public:
    STM32Serial(ros::NodeHandle& nh, const std::string& port_name, int baud_rate);
    ~STM32Serial();

    bool isConnected() const { return _is_connected_; }
    bool connect();
    bool reConnect();
    void disConnect();

    bool sendData(const std::string& data);
    std::string rcvdData(uint32_t timeout_ms = 0);
    void clearBuffer(void);

private:
    /// @brief ROS 节点句柄
    ros::NodeHandle _nh_;
    /// @brief 串口对象
    serial::Serial _serial_;
    /// @brief 串口名称
    std::string _port_name_;
    /// @brief 波特率
    int _baud_rate_;
    /// @brief 连接状态
    bool _is_connected_;
};

/**
 * @brief CAN 串口通信类
 * @details 封装了通过 CAN 接口进行通信的功能，包括：
 *          - 连接/重连与断开连接
 *          - 设置与清除过滤器
 *          - 发送(标准/扩展/原帧)与接收 CAN 帧
 */
class CANSerial{
public:
    CANSerial(ros::NodeHandle& nh, const std::string& can_name = "can0");
    ~CANSerial();

    // 禁止拷贝构造，允许移动构造
    CANSerial(const CANSerial&) = delete;
    CANSerial& operator=(const CANSerial&) = delete;
    CANSerial(CANSerial&&) = default;
    CANSerial& operator=(CANSerial&&) = default;

    bool isConnected() const { return _is_connected_; }
    bool connect();
    bool reConnect();
    void disConnect();

    bool setFilter(uint32_t can_id, uint32_t can_mask);
    bool clearFilter(void);

    bool sendStdFrame(uint32_t can_id, const std::string& data);
    bool sendExtFrame(uint32_t can_id, const std::string& data);
    bool sendRawFrame(CanFrame_t& frame);
    CanFrame_t rcvdFrame(uint32_t timeout_ms = 0);

private:
    /// @brief ROS 节点句柄
    ros::NodeHandle _nh_;
    /// @brief CAN 设备名称
    std::string _name_;
    /// @brief 套接字描述符
    int _socket_;
    /// @brief 连接状态
    bool _is_connected_;
    /// @brief CAN 地址结构体
    CanAddr_t _addr_;
    /// @brief CAN 接口请求结构体
    CanIfReq_t _ifreq_;
};

#endif
