#include "dm_arm_interface/eef_interface.hpp"

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

EefCmdService::EefCmdService(ros::NodeHandle& nh, std::shared_ptr<EndEffector> eef, std::shared_ptr<EefCmdDispatcher> dispatcher, std::string service_name)
    : _eef_(std::move(eef)), _dispatcher_(std::move(dispatcher)) {
    _srv_ = std::make_unique<ros::ServiceServer>(nh.advertiseService(service_name, &EefCmdService::on_request, this));
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

tl::optional<EefCmdRequest> EefCmdService::convert_srvreq_to_eefreq(const dm_arm_msgs::CommandEef::Request& srv_req) {
    EefCmdRequest eef_req{};
    eef_req.type = static_cast<EefCmdType>(srv_req.command_type);
    if(static_cast<std::size_t>(srv_req.command_type) >= static_cast<std::size_t>(EefCmdType::MAX)) {
        ROS_WARN("接收到无效的命令类型: %d", srv_req.command_type);
        return tl::nullopt;
    }

    eef_req.values = srv_req.values;

    return eef_req;
}

bool EefCmdService::on_request(dm_arm_msgs::CommandEef::Request& req, dm_arm_msgs::CommandEef::Response& res) {
    if(!_eef_ || !_dispatcher_) {
        res.success = false;
        res.message = "控制器未初始化";
        return true;
    }

    auto eef_req_opt = convert_srvreq_to_eefreq(req);
    if(!eef_req_opt) {
        res.success = false;
        res.message = "无效的请求";
        return true;
    }

    auto result = _dispatcher_->dispatch(*eef_req_opt);
    res.success = result.success;
    res.message = result.message;
    res.error_code = static_cast<uint8_t>(result.error_code);

    return true;
}

}
