#include "serial_driver/serial_driver.hpp"

#include <sys/ioctl.h>
#include <linux/can/raw.h>

/* ========================= 接 口 量 声 明 ========================= */



/* ========================= 私 有 量 / 函 数 声 明 ========================= */

typedef struct can_filter CanFilter_t;

/* ========================= 接 口 类 / 函 数 实 现 ========================= */

/**
 * @brief STM32 串口通信类构造函数
 * @param nh ROS 节点句柄
 * @param port_name 串口名称，例如 "/dev/ttyUSB0"
 * @param baud_rate 波特率，例如 115200
 * @warning 禁止拷贝构造与移动构造
 */
STM32Serial::STM32Serial(ros::NodeHandle& nh, const std::string& port_name, int baud_rate)
    : _nh_(nh), _port_name_(port_name), _baud_rate_(baud_rate), _is_connected_(false)
{
    ROS_INFO("STM32Serial 初始化完成，串口名称：%s，波特率：%d", _port_name_.c_str(), _baud_rate_);
}

/**
 * @brief STM32 串口通信类析构函数
 */
STM32Serial::~STM32Serial()
{
    disConnect();
    ROS_INFO("STM32Serial 资源已释放");
}

/**
 * @brief 连接 STM32 串口
 * @return 连接成功返回 true，失败返回 false
 */
bool STM32Serial::connect()
{
    if(_is_connected_) return true;

    try{
        _serial_.setPort(_port_name_);
        _serial_.setBaudrate(_baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        _serial_.setTimeout(timeout);
        _serial_.open();
    }
    catch(const std::exception& e){
        ROS_ERROR("STM32Serial 连接失败：%s", e.what());
        return false;
    }

    if(!_serial_.isOpen()){
        ROS_ERROR("STM32Serial 连接失败：串口未打开");
        return false;
    }

    _is_connected_ = true;
    ROS_INFO("STM32Serial 连接成功，串口名称：%s，波特率：%d", _port_name_.c_str(), _baud_rate_);
    return true;
}

/**
 * @brief 重新连接 STM32 串口
 * @return 连接成功返回 true，失败返回 false
 */
bool STM32Serial::reConnect()
{
    disConnect();
    ros::Duration(0.5).sleep();
    return connect();
}

/**
 * @brief 断开 STM32 串口连接
 */
void STM32Serial::disConnect()
{
    if(_serial_.isOpen()){
        _serial_.close();
    }
    _is_connected_ = false;
    ROS_INFO("STM32Serial 已断开连接，串口名称：%s", _port_name_.c_str());
}

/**
 * @brief 发送数据到 STM32 串口
 * @param data 要发送的数据
 */
bool STM32Serial::sendData(const std::string& data)
{
    if(!_is_connected_){
        ROS_ERROR("STM32Serial 发送数据失败：未连接");
        return false;
    }

    size_t bytes_written = _serial_.write(data);
    ROS_INFO("STM32Serial 发送数据成功，字节数：%zu", bytes_written);
    return true;
}

/**
 * @brief 从 STM32 串口接收数据
 * @param timeout_ms 超时时间，单位为毫秒，0 表示阻塞等待
 * @return 接收到的数据
 */
std::string STM32Serial::rcvdData(uint32_t timeout_ms)
{
    std::string data;

    if(!_is_connected_){
        ROS_ERROR("STM32Serial 接收数据失败：未连接");
        return data;
    }

    if(timeout_ms > 0){
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
        _serial_.setTimeout(timeout);
    }

    size_t bytes_available = _serial_.available();
    if(bytes_available > 0){
        data = _serial_.read(bytes_available);
        ROS_INFO("STM32Serial 接收数据成功，字节数：%zu", data.size());
    }
    else{
        ROS_WARN("STM32Serial 接收数据超时，超时时间：%d ms", timeout_ms);
    }

    return data;
}

/**
 * @brief 清空 STM32 串口缓冲区
 */
void STM32Serial::clearBuffer(void)
{
    if(!_is_connected_){
        ROS_ERROR("STM32Serial 清空缓冲区失败：未连接");
        return;
    }

    _serial_.flush();
    ROS_INFO("STM32Serial 缓冲区已清空");
}

/**
 * @brief CAN 串口通信类构造函数
 * @param nh ROS 节点句柄
 * @param can_name CAN 设备名称，默认为 "can0"
 * @warning 禁止拷贝构造
 */
CANSerial::CANSerial(ros::NodeHandle& nh, const std::string& can_name)
    : _nh_(nh), _name_(can_name), _socket_(-1), _is_connected_(false)
{
    ROS_INFO("CANSerial 初始化完成，CAN 设备名称：%s", _name_.c_str());
}

/**
 * @brief CAN 串口通信类析构函数
 */
CANSerial::~CANSerial()
{
    disConnect();
    ROS_INFO("CANSerial 资源已释放");
}

/**
 * @brief 连接 CAN 设备
 * @return 连接成功返回 true，失败返回 false
 */
bool CANSerial::connect()
{
    if(_is_connected_) return true;

    _socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(_socket_ < 0){
        ROS_ERROR("CANSerial 连接失败：创建套接字失败");
        return false;
    }

    std::strncpy(_ifreq_.ifr_name, _name_.c_str(), IFNAMSIZ - 1);
    _ifreq_.ifr_name[IFNAMSIZ - 1] = '\0';

    if(ioctl(_socket_, SIOCGIFINDEX, &_ifreq_) < 0){
        ROS_ERROR("CANSerial 连接失败：获取接口索引失败");
        close(_socket_);
        _socket_ = -1;
        return false;
    }

    _addr_.can_family = AF_CAN;
    _addr_.can_ifindex = _ifreq_.ifr_ifindex;

    if(bind(_socket_, (struct sockaddr*)&_addr_, sizeof(_addr_)) < 0){
        ROS_ERROR("CANSerial 连接失败：绑定套接字失败");
        close(_socket_);
        _socket_ = -1;
        return false;
    }

    _is_connected_ = true;
    ROS_INFO("CANSerial 连接成功，CAN 设备名称：%s", _name_.c_str());
    return true;
}

/**
 * @brief 重新连接 CAN 设备
 * @return 连接成功返回 true，失败返回 false
 */
bool CANSerial::reConnect()
{
    disConnect();
    ros::Duration(0.5).sleep();
    return connect();
}

/**
 * @brief 断开 CAN 设备连接
 */
void CANSerial::disConnect()
{
    if(_socket_ >= 0){
        close(_socket_);
        _socket_ = -1;
    }
    _is_connected_ = false;
    ROS_INFO("CANSerial 已断开连接，CAN 设备名称：%s", _name_.c_str());
}

/**
 * @brief 设置 CAN 过滤器
 * @param can_id 过滤的 CAN ID
 * @param can_mask 过滤掩码
 * @return 设置成功返回 true，失败返回 false
 */
bool CANSerial::setFilter(uint32_t can_id, uint32_t can_mask)
{
    if(!_is_connected_){
        ROS_ERROR("CANSerial 设置过滤器失败：未连接");
        return false;
    }

    CanFilter_t filter;
    filter.can_id = can_id;
    filter.can_mask = can_mask;

    if(setsockopt(_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0){
        ROS_ERROR("CANSerial 设置过滤器失败：设置套接字选项失败");
        return false;
    }

    ROS_INFO("CANSerial 设置过滤器成功，CAN ID：0x%X，CAN Mask：0x%X", can_id, can_mask);
    return true;
}

/**
 * @brief 清除 CAN 过滤器
 * @return 清除成功返回 true，失败返回 false
 */
bool CANSerial::clearFilter(void)
{
    if(!_is_connected_){
        ROS_ERROR("CANSerial 清除过滤器失败：未连接");
        return false;
    }

    if(setsockopt(_socket_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) < 0){
        ROS_ERROR("CANSerial 清除过滤器失败：设置套接字选项失败");
        return false;
    }

    ROS_INFO("CANSerial 清除过滤器成功");
    return true;
}

/**
 * @brief 发送标准 CAN 帧
 * @param can_id 标准 CAN ID (11 位)
 * @param data 数据内容，长度不超过 8 字节
 * @return 发送成功返回 true，失败返回 false
 */
bool CANSerial::sendStdFrame(uint32_t can_id, const std::string& data)
{
    if(!_is_connected_){
        ROS_ERROR("CANSerial 发送标准帧失败：未连接");
        return false;
    }

    if(data.size() > 8){
        ROS_ERROR("CANSerial 发送标准帧失败：数据长度超过 8 字节");
        return false;
    }

    CanFrame_t frame;
    frame.can_id = can_id & CAN_SFF_MASK;
    frame.can_dlc = data.size();
    std::memcpy(frame.data, data.data(), data.size());

    ssize_t nbytes = write(_socket_, &frame, sizeof(frame));
    if(nbytes != sizeof(frame)){
        ROS_ERROR("CANSerial 发送标准帧失败：写入数据失败");
        return false;
    }

    ROS_INFO("CANSerial 发送标准帧成功，CAN ID：0x%X，数据长度：%d", can_id, frame.can_dlc);
    return true;
}

/**
 * @brief 发送扩展 CAN 帧
 * @param can_id 扩展 CAN ID (29 位)
 * @param data 数据内容，长度不超过 8 字节
 * @return 发送成功返回 true，失败返回 false
 */
bool CANSerial::sendExtFrame(uint32_t can_id, const std::string& data)
{
    if(!_is_connected_){
        ROS_ERROR("CANSerial 发送扩展帧失败：未连接");
        return false;
    }

    if(data.size() > 8){
        ROS_ERROR("CANSerial 发送扩展帧失败：数据长度超过 8 字节");
        return false;
    }

    CanFrame_t frame;
    frame.can_id = can_id | CAN_EFF_FLAG;
    frame.can_dlc = data.size();
    std::memcpy(frame.data, data.data(), data.size());

    ssize_t nbytes = write(_socket_, &frame, sizeof(frame));
    if(nbytes != sizeof(frame)){
        ROS_ERROR("CANSerial 发送扩展帧失败：写入数据失败");
        return false;
    }

    ROS_INFO("CANSerial 发送扩展帧成功，CAN ID：0x%X，数据长度：%d", can_id, frame.can_dlc);
    return true;
}

/**
 * @brief 发送原始 CAN 帧
 * @param frame 要发送的 CAN 帧
 * @return 发送成功返回 true，失败返回 false
 */
bool CANSerial::sendRawFrame(CanFrame_t& frame)
{
    if(!_is_connected_){
        ROS_ERROR("CANSerial 发送原始帧失败：未连接");
        return false;
    }

    ssize_t nbytes = write(_socket_, &frame, sizeof(frame));
    if(nbytes != sizeof(frame)){
        ROS_ERROR("CANSerial 发送原始帧失败：写入数据失败");
        return false;
    }

    ROS_INFO("CANSerial 发送原始帧成功，CAN ID：0x%X，数据长度：%d", frame.can_id, frame.can_dlc);
    return true;
}

/**
 * @brief 接收 CAN 帧
 * @param timeout_ms 超时时间，单位为毫秒，0 表示阻塞等待
 * @return 接收到的 CAN 帧，若接收失败则返回空帧
 */
CanFrame_t CANSerial::rcvdFrame(uint32_t timeout_ms)
{
    CanFrame_t frame;

    if(!_is_connected_){
        ROS_ERROR("CANSerial 接收帧失败：未连接");
        return frame;
    }

    if(timeout_ms > 0){
        fd_set read_fds;
        struct timeval timeout;
        FD_ZERO(&read_fds);
        FD_SET(_socket_, &read_fds);

        timeout.tv_sec = timeout_ms / 1000;
        timeout.tv_usec = (timeout_ms % 1000) * 1000;

        int ret = select(_socket_ + 1, &read_fds, NULL, NULL, (timeout_ms > 0) ? &timeout : NULL);
        if(ret < 0){
            ROS_ERROR("CANSerial 接收帧失败：select 错误");
            return frame;
        }
        else if(ret == 0){
            ROS_WARN("CANSerial 接收帧超时，超时时间：%d ms", timeout_ms);
            return frame;
        }
    }

    ssize_t nbytes = read(_socket_, &frame, sizeof(frame));
    if(nbytes != sizeof(frame)){
        ROS_ERROR("CANSerial 接收帧失败：读取数据失败");
        return frame;
    }

    ROS_INFO("CANSerial 接收帧成功，CAN ID：0x%X，数据长度：%d", frame.can_id, frame.can_dlc);
    return frame;
}
