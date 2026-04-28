#ifndef _dm_arm_interface_eef_interface_hpp_
#define _dm_arm_interface_eef_interface_hpp_

#include <memory>
#include <string>

#include "tl_optional/optional.hpp"

#include "dm_arm_interface/interface_module.hpp"
#include "dm_arm_controller/eef_interface.hpp"
#include "dm_arm_commander/cmd_dispatcher.hpp"
#include "dm_arm_msgs/CommandEef.h"

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class EefCmdService : public ROSModuleInterface {
public:
    /**
     * @brief EefCmdService 构造函数
     * @param nh ROS 节点句柄
     * @param eef 末端执行器
     * @param dispatcher 命令分发器
     * @param service_name Service 名称
     */
    EefCmdService(ros::NodeHandle& nh, std::shared_ptr<EndEffector> eef, std::shared_ptr<EefCmdDispatcher> dispatcher, std::string service_name);
    /**
     * @brief EefCmdService 析构函数
     */
    ~EefCmdService() = default;

    EefCmdService(const EefCmdService&) = delete;
    EefCmdService& operator=(const EefCmdService&) = delete;
    EefCmdService(EefCmdService&&) = delete;
    EefCmdService& operator=(EefCmdService&&) = delete;

private:
    bool on_request(dm_arm_msgs::CommandEef::Request& req, dm_arm_msgs::CommandEef::Response& res);
    tl::optional<EefCmdRequest> convert_srvreq_to_eefreq(const dm_arm_msgs::CommandEef::Request& srv_req);

private:
    std::unique_ptr<ros::ServiceServer> _srv_;
    std::shared_ptr<EndEffector> _eef_;
    std::shared_ptr<EefCmdDispatcher> _dispatcher_;
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}

#endif
