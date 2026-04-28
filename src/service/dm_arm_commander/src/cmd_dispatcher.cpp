#include "dm_arm_commander/cmd_dispatcher.hpp"

#include <array>

// ! ========================= 通 用 匿 名 工 具 ========================= ! //

namespace {

/**
 * @brief 定义命令类型与处理函数的映射关系的结构体模板，包含命令类型、名称、描述、是否有反馈以及对应的成员函数指针
 * @param EnumT 命令类型枚举，例如 ArmCmdType 或 EefCmdType
 * @param ImplT 实现类类型，例如 ArmCmdDispatcher::Impl 或 EefCmdDispatcher::Impl
 * @param ReqT 命令请求类型，例如 ArmCmdRequest 或 EefCmdRequest
 * @param ResT 命令结果类型，例如 ArmCmdResult 或 EefCmdResult
 * @param CbT 反馈回调函数类型，例如 ArmCmdDispatcher::FeedbackCb 或 EefCmdDispatcher::FeedbackCb
 */
template<typename EnumT, typename ImplT, typename ReqT, typename ResT, typename CbT>
struct DispatchEntry {
    EnumT type;
    const char* name;
    const char* desc;
    bool has_fb;
    ResT(ImplT::* handler)(const ReqT&, CbT);
};

/**
 * @brief 在命令表中查找对应命令类型的表项，返回指向该表项的指针，如果未找到则返回 nullptr
 * @param table 命令表，包含多个 DispatchEntry 条目
 * @param type 要查找的命令类型枚举值
 * @return 指向对应表项的指针，如果未找到则返回 nullptr
 */
template<typename Entry, typename EnumT, std::size_t N>
const Entry* find_entry(const std::array<Entry, N>& table, EnumT type) {
    for(const auto& entry : table) {
        if(entry.type == type) return &entry;
    }
    return nullptr;
}

/**
 * @brief 通过回调函数报告命令执行进度，构造一个反馈结构体并调用回调函数
 * @param cb 反馈回调函数，接受一个 FbT 类型的参数
 * @param stage 当前阶段的名称，例如 "start"、"setting"、"planning"、"done"
 * @param progress 当前阶段的进度，范围从 0.0 到 1.0
 * @param message 当前阶段的描述信息，例如 "正在设置目标位姿"
 */
template<typename FbT, typename FbCbT>
void report(FbCbT cb, const std::string& stage, double progress, const std::string& message) {
    if(cb) cb(FbT{ stage, progress, message });
}

};

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //

/**
 * @brief ArmCmdDispatcher 的 Pimpl 实现类，隐藏具体实现细节，减少头文件依赖，提高编译效率
 * @param _arm_ ArmController 的共享指针，用于执行机械臂操作
 * @param _is_cancelled_ 原子布尔变量，表示当前命令是否已被取消，支持线程安全的取消操作
 * @param fill_current_state 填充命令结果中的当前状态信息，包括当前位姿和关节角
 * @param make_ok 构造一个成功的命令结果
 * @param make_err 构造一个失败的命令结果
 * @param make_cancelled 构造一个被取消的命令结果
 * @param report 通过回调函数报告命令执行进度
 * @param execute_if_not_cancelled 如果命令未被取消，则继续执行后续操作
 * @param handle_xxx 处理对应命令
 */
#define X(name, handler, has_fb, desc) ArmCmdResult handle_##handler(const ArmCmdRequest& req, FeedbackCb cb = nullptr);
struct ArmCmdDispatcher::Impl {
    using FeedbackCb = ArmCmdDispatcher::FeedbackCb;
    using Entry = DispatchEntry<ArmCmdType, Impl, ArmCmdRequest, ArmCmdResult, FeedbackCb>;

    explicit Impl(std::shared_ptr<ArmController> arm) : _arm_(std::move(arm)) {}

    std::shared_ptr<ArmController> _arm_;
    std::atomic_bool _is_cancelled_{ false };
    bool is_cancelled() const { return _is_cancelled_.load(); }

    ArmCmdResult make_ok(const std::string& msg = "命令执行成功");
    ArmCmdResult make_err(ErrorCode code = ErrorCode::FAILURE, const std::string& msg = "命令执行失败");
    ArmCmdResult make_cancelled();

    ArmCmdResult execute_if_not_cancelled(ErrorCode code, FeedbackCb cb = nullptr);

    static const std::array<Entry, static_cast<std::size_t>(ArmCmdType::MAX)>& table();

    DM_ARM_CMD_TABLE
};
#undef X

#define X(name, handler, has_fb, desc) EefCmdResult handle_##handler(const EefCmdRequest& req, FeedbackCb cb = nullptr);
struct EefCmdDispatcher::Impl {
    using FeedbackCb = EefCmdDispatcher::FeedbackCb;
    using Entry = DispatchEntry<EefCmdType, Impl, EefCmdRequest, EefCmdResult, FeedbackCb>;

    explicit Impl(std::shared_ptr<EndEffector> eef) : _eef_(std::move(eef)) {}

    std::shared_ptr<EndEffector> _eef_;
    std::atomic_bool _is_cancelled_{ false };
    bool is_cancelled() const { return _is_cancelled_.load(); }

    EefCmdResult make_ok(const std::string& msg = "命令执行成功");
    EefCmdResult make_err(ErrorCode code = ErrorCode::FAILURE, const std::string& msg = "命令执行失败");
    EefCmdResult make_cancelled();

    static const std::array<Entry, static_cast<std::size_t>(EefCmdType::MAX)>& table();

    DM_ARM_EEF_CMD_TABLE
};
#undef X

// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造函数，接受一个 ArmController 的共享指针用于执行机械臂操作
 * @param arm ArmController 的共享指针，不能为空
 */
ArmCmdDispatcher::ArmCmdDispatcher(std::shared_ptr<ArmController> arm)
    : _impl_(std::make_unique<Impl>(std::move(arm))) {}
ArmCmdDispatcher::~ArmCmdDispatcher() = default;

/**
 * @brief 分发命令请求，调用对应的处理函数执行命令，并通过回调函数报告执行进度
 * @param req 命令请求
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return 命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::dispatch(const ArmCmdRequest& req, FeedbackCb cb) {
    if(!_impl_ || !_impl_->_arm_) {
        return _impl_ ? _impl_->make_err(ErrorCode::FAILURE, "ArmController 未初始化") : ArmCmdResult{ false, "ArmCmdDispatcher 未初始化", {}, ErrorCode::FAILURE };
    }
    _impl_->_is_cancelled_.store(false);

    const auto* entry = find_entry(_impl_->table(), req.type);
    if(!entry) return _impl_->make_err(ErrorCode::INVALID_PARAMETER, "未知的命令类型");

    return (_impl_.get()->*(entry->handler))(req, std::move(cb));
}

/**
 * @brief 将命令类型枚举转换为字符串，便于日志记录和调试
 * @param type 命令类型枚举值
 * @return 对应的字符串表示，如果枚举值未知则返回 "UNKNOWN"
 */
std::string ArmCmdDispatcher::type_to_string(ArmCmdType type) const {
    if(!_impl_) return "UNKNOWN";
    const auto* entry = find_entry(_impl_->table(), type);
    return entry ? entry->name : "UNKNOWN";
}

/**
 * @brief 取消当前正在执行的命令
 */
void ArmCmdDispatcher::cancel() {
    if(!_impl_) return;

    _impl_->_is_cancelled_.store(true);

    if(_impl_->_arm_) {
        _impl_->_arm_->cancel_async();
        _impl_->_arm_->stop();
    }
}

/**
 * @brief 检查当前命令是否已被取消
 * @return 如果命令已取消则返回 true，否则返回 false
 */
bool ArmCmdDispatcher::is_cancelled() const {
    if(!_impl_) return false;

    return _impl_->_is_cancelled_.load();
}

/**
 * @brief 构造函数，接受一个 EndEffector 的共享指针用于执行末端操作
 * @param eef EndEffector 的共享指针，不能为空
 */
EefCmdDispatcher::EefCmdDispatcher(std::shared_ptr<EndEffector> eef)
    : _impl_(std::make_unique<Impl>(std::move(eef))) {}
EefCmdDispatcher::~EefCmdDispatcher() = default;

/**
 * @brief 分发命令请求，调用对应的处理函数执行命令，并通过回调函数报告执行进度
 * @param req 命令请求
 * @param cb 反馈回调函数，接受 EefCmdFeedback 结构体参数
 * @return 命令执行结果
 */
EefCmdResult EefCmdDispatcher::dispatch(const EefCmdRequest& req, FeedbackCb cb) {
    if(!_impl_ || !_impl_->_eef_) {
        return _impl_ ? _impl_->make_err(ErrorCode::FAILURE, "EndEffector 未初始化") : EefCmdResult{ false, "EefCmdDispatcher 未初始化", ErrorCode::FAILURE };
    }
    _impl_->_is_cancelled_.store(false);

    const auto* entry = find_entry(_impl_->table(), req.type);
    if(!entry) return _impl_->make_err(ErrorCode::INVALID_PARAMETER, "未知的命令类型");

    return (_impl_.get()->*(entry->handler))(req, std::move(cb));
}

std::string EefCmdDispatcher::type_to_string(EefCmdType type) const {
    if(!_impl_) return "UNKNOWN";
    const auto* entry = find_entry(_impl_->table(), type);
    return entry ? entry->name : "UNKNOWN";
}

void EefCmdDispatcher::cancel() {
    if(!_impl_) return;

    _impl_->_is_cancelled_.store(true);

    if(_impl_->_eef_) {
        _impl_->_eef_->stop();
    }
}

bool EefCmdDispatcher::is_cancelled() const {
    if(!_impl_) return false;

    return _impl_->_is_cancelled_.load();
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 构造一个成功的命令结果
 * @param msg 结果消息，默认为 "命令执行成功"
 * @return ArmCmdResult 结构体，表示命令执行成功
 */
ArmCmdResult ArmCmdDispatcher::Impl::make_ok(const std::string& msg) {
    ArmCmdResult result;
    result.success = true;
    result.message = msg;
    result.error_code = ErrorCode::SUCCESS;

    return result;
}

/**
 * @brief 构造一个失败的命令结果
 * @param code 错误码，默认为 ErrorCode::FAILURE
 * @param msg 结果消息，默认为 "命令执行失败"
 * @return ArmCmdResult 结构体，表示命令执行失败
 */
ArmCmdResult ArmCmdDispatcher::Impl::make_err(ErrorCode code, const std::string& msg) {
    ArmCmdResult result;
    result.success = false;
    result.message = msg;
    result.error_code = code;

    return result;
}

/**
 * @brief 构造一个被取消的命令结果
 * @return ArmCmdResult 结构体，表示命令已取消
 */
ArmCmdResult ArmCmdDispatcher::Impl::make_cancelled() {
    ArmCmdResult result;
    result.success = false;
    result.message = "命令已取消";
    result.error_code = ErrorCode::CANCELLED;

    return result;
}

/**
 * @brief 如果命令未被取消，则继续执行后续操作
 * @param code 前一步操作的错误码，如果不为 ErrorCode::SUCCESS 则表示前一步失败
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::execute_if_not_cancelled(ErrorCode code, FeedbackCb cb) {
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "设置目标失败: " + err_to_string(code));
    }

    report<ArmCmdFeedback>(cb, "planning", 0.5, "开始规划与执行");
    code = _arm_->plan_and_execute();
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "规划或执行失败：" + err_to_string(code));
    }

    report<ArmCmdFeedback>(cb, "done", 1.0, "命令执行成功");
    return make_ok();
}

/**
 * @brief 定义命令类型与处理函数的映射表，使用 X-Macro 生成表项，便于维护和扩展
 */
#define X(name, handler, has_fb, desc) \
    Entry{ ArmCmdType::name, #name, desc, has_fb, &Impl::handle_##handler },
const std::array<ArmCmdDispatcher::Impl::Entry, static_cast<std::size_t>(ArmCmdType::MAX)>& ArmCmdDispatcher::Impl::table() {
    static const std::array<Entry, static_cast<std::size_t>(ArmCmdType::MAX)> kTable{
        {
            DM_ARM_CMD_TABLE
        }
    };
    return kTable;
}
#undef X


/**
 * @brief 构造一个成功的命令结果
 * @param msg 结果消息，默认为 "命令执行成功"
 * @return EefCmdResult 结构体，表示命令执行成功
 */
EefCmdResult EefCmdDispatcher::Impl::make_ok(const std::string& msg) {
    EefCmdResult result;
    result.success = true;
    result.message = msg;
    result.error_code = ErrorCode::SUCCESS;

    return result;
}

/**
 * @brief 构造一个失败的命令结果
 * @param code 错误码，默认为 ErrorCode::FAILURE
 * @param msg 结果消息，默认为 "命令执行失败"
 * @return EefCmdResult 结构体，表示命令执行失败
 */
EefCmdResult EefCmdDispatcher::Impl::make_err(ErrorCode code, const std::string& msg) {
    EefCmdResult result;
    result.success = false;
    result.message = msg;
    result.error_code = code;

    return result;
}

/**
 * @brief 构造一个被取消的命令结果
 * @return EefCmdResult 结构体，表示命令已取消
 */
EefCmdResult EefCmdDispatcher::Impl::make_cancelled() {
    EefCmdResult result;
    result.success = false;
    result.message = "命令已取消";
    result.error_code = ErrorCode::CANCELLED;

    return result;
}

/**
 * @brief 定义命令类型与处理函数的映射表，使用 X-Macro 生成表项，便于维护和扩展
 */
#define X(name, handler, has_fb, desc) \
    Entry{ EefCmdType::name, #name, desc, has_fb, &Impl::handle_##handler },
const std::array<EefCmdDispatcher::Impl::Entry, static_cast<std::size_t>(EefCmdType::MAX)>& EefCmdDispatcher::Impl::table() {
    static const std::array<Entry, static_cast<std::size_t>(EefCmdType::MAX)> kTable{
        {
            DM_ARM_EEF_CMD_TABLE
        }
    };
    return kTable;
}

/**
 * @brief 处理 HOME 命令，调用 ArmController 的 home() 方法将机械臂移动到初始位置
 * @param req 命令请求，HOME 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_home(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)req;
    (void)cb;

    if(is_cancelled()) {
        return make_cancelled();
    }

    const ErrorCode code = _arm_->home();
    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行 HOME 失败：" + err_to_string(code));
    }

    return make_ok();
}

/**
 * @brief 处理 MOVE_JOINTS 命令，调用 ArmController 的 set_joints() 方法设置目标关节角，并执行
 * @param req 命令请求，要求 req.joints 包含目标关节角列表
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_joints(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_JOINTS 命令");
    if(req.joints.empty()) {
        return make_err(ErrorCode::FAILURE, "关节角列表不能为空");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在设置目标关节角");
    ErrorCode code = _arm_->set_joints(req.joints);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_TARGET 命令，调用 ArmController 的 set_target() 方法设置目标位姿，并执行
 * @param req 命令请求，要求 req.target 包含目标位姿
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_target(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_TARGET 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在设置目标位姿");
    ErrorCode code = _arm_->set_target(req.target);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_TARGET_IN_EEF_FRAME 命令，调用 ArmController 的 set_target_in_eef_frame() 方法设置工具坐标系下的目标位姿，并执行
 * @param req 命令请求，要求 req.target 包含目标位姿
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_target_in_eef_frame(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_TARGET_IN_EEF_FRAME 命令");
    if(std::holds_alternative<std::monostate>(req.target)) {
        return make_err(ErrorCode::FAILURE, "目标不能为空");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在设置目标位姿（工具坐标系）");
    ErrorCode code = _arm_->set_target_in_eef_frame(req.target);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 TELESCOPIC_END 命令，调用 ArmController 的 telescopic_end() 方法设置伸缩末端目标，并执行
 * @param req 命令请求，要求 req.values 包含一个参数，表示伸缩长度
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_telescopic_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 TELESCOPIC_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "伸缩末端命令需要一个参数：伸缩长度");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在设置伸缩末端目标");
    ErrorCode code = _arm_->telescopic_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 ROTATE_END 命令，调用 ArmController 的 rotate_end() 方法设置旋转末端目标，并执行
 * @param req 命令请求，要求 req.values 包含一个参数，表示旋转角度（弧度）
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_rotate_end(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 ROTATE_END 命令");
    if(req.values.size() != 1) {
        return make_err(ErrorCode::FAILURE, "旋转末端命令需要一个参数：旋转角度（弧度）");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在设置旋转末端目标");
    ErrorCode code = _arm_->rotate_end(req.values[0]);
    return execute_if_not_cancelled(code, cb);
}

/**
 * @brief 处理 MOVE_LINE 命令，调用 ArmController 的 set_line() 方法设置线性路径的起点和终点，并执行
 * @param req 命令请求，要求 req.waypoints 包含两个位姿点，分别作为线性路径的起点和终点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_line(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_LINE 命令");
    if(req.waypoints.size() != 2) {
        return make_err(ErrorCode::FAILURE, "MOVE_LINE 命令需要两个路径点");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->set_line(req.waypoints[0], req.waypoints[1]);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report<ArmCmdFeedback>(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 MOVE_BEZIER 命令，调用 ArmController 的 set_bezier_curve() 方法设置三阶贝塞尔曲线的起点、控制点和终点，并执行
 * @param req 命令请求，要求 req.waypoints 包含三个位姿点，分别作为贝塞尔曲线的起点、控制点和终点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_bezier(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_BEZIER 命令");
    if(req.waypoints.size() != 3) {
        return make_err(ErrorCode::FAILURE, "MOVE_BEZIER 命令需要三个路径点");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->set_bezier_curve(req.waypoints[0], req.waypoints[1], req.waypoints[2]);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report<ArmCmdFeedback>(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 MOVE_DECARTES 命令，调用 ArmController 的 plan_decartes() 方法规划笛卡尔空间路径，并执行
 * @param req 命令请求，要求 req.waypoints 包含多个位姿点，作为笛卡尔空间路径的关键点
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_decartes(const ArmCmdRequest& req, FeedbackCb cb) {
    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_DECARTES 命令");
    if(req.waypoints.empty()) {
        return make_err(ErrorCode::FAILURE, "路径点列表不能为空");
    }

    report<ArmCmdFeedback>(cb, "setting", 0.2, "正在规划路径点");
    DescartesResult result = _arm_->plan_decartes(req.waypoints);
    if(result.error_code != ErrorCode::SUCCESS) {
        return make_err(result.error_code, "规划路径点失败：" + err_to_string(result.error_code));
    }

    if(is_cancelled()) {
        return make_cancelled();
    }

    report<ArmCmdFeedback>(cb, "planning", 0.5, "正在执行");
    ErrorCode code = _arm_->execute(result.trajectory);

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "执行失败：" + err_to_string(code));
    }
    return make_ok();
}

/**
 * @brief 处理 SET_ORIENTATION_CONSTRAINT 命令，调用 ArmController 的 set_orientation_constraint() 方法设置姿态约束
 * @param req 命令请求，要求 req.target 包含一个 geometry_msgs::Quaternion 作为目标姿态
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_set_orientation_constraint(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)cb;

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(std::holds_alternative<std::monostate>(req.target) ||
        !std::holds_alternative<geometry_msgs::Quaternion>(req.target)) {
        return make_err(ErrorCode::INVALID_PARAMETER, "目标必须是一个 Quaternion");
    }

    const geometry_msgs::Quaternion target_orientation =
        std::get<geometry_msgs::Quaternion>(req.target);

    _arm_->set_orientation_constraint(target_orientation);
    _arm_->apply_constraints();

    return make_ok("姿态约束已设置并应用");
}

/**
 * @brief 处理 SET_POSITION_CONSTRAINT 命令，调用 ArmController 的 set_position_constraint() 方法设置位置约束
 * @param req 命令请求，要求 req.target 包含一个 geometry_msgs::Point 作为目标位置，req.values 包含一个三维向量参数，表示约束范围大小
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_set_position_constraint(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)cb;

    if(is_cancelled()) {
        return make_cancelled();
    }

    if(std::holds_alternative<std::monostate>(req.target) ||
        !std::holds_alternative<geometry_msgs::Point>(req.target)) {
        return make_err(ErrorCode::INVALID_PARAMETER, "目标必须是一个 Point");
    }

    if(req.values.size() != 3) {
        return make_err(ErrorCode::INVALID_PARAMETER, "位置约束需要 3 个范围参数：x/y/z");
    }

    geometry_msgs::Point target_position = std::get<geometry_msgs::Point>(req.target);
    geometry_msgs::Vector3 scope_size;
    scope_size.x = req.values[0];
    scope_size.y = req.values[1];
    scope_size.z = req.values[2];

    _arm_->set_position_constraint(target_position, scope_size);
    _arm_->apply_constraints();

    return make_ok("位置约束已设置并应用");
}

/**
 * @brief 处理 SET_JOINT_CONSTRAINT 命令，调用 ArmController 的 set_joint_constraint() 方法设置关节约束
 * @param req 命令请求，要求 req.joint_names 包含一个或多个关节名称，req.values 包含三个参数，分别表示约束范围的最小值、最大值和权重
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_set_joint_constraint(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(req.joint_names.empty()) {
        return make_err(ErrorCode::INVALID_PARAMETER, "关节约束名称列表不能为空");
    }

    if(req.joints.size() != req.joint_names.size()) {
        return make_err(ErrorCode::INVALID_PARAMETER,
            "关节约束 joint_names 与 joints 数量不一致");
    }

    if(req.values.size() != req.joint_names.size() * 2) {
        return make_err(ErrorCode::INVALID_PARAMETER,
            "关节约束 values 数量必须等于 joint_names.size() * 2");
    }

    for(std::size_t i = 0; i < req.joint_names.size(); ++i) {
        _arm_->set_joint_constraint(
            req.joint_names[i],
            req.joints[i],
            req.values[i * 2],
            req.values[i * 2 + 1]);
    }

    _arm_->apply_constraints();

    return make_ok("关节约束已设置并应用");
}

/**
 * @brief 处理 GET_CURRENT_JOINTS 命令，调用 ArmController 的 get_current_joints() 方法获取当前关节角
 * @param req 命令请求，GET_CURRENT_JOINTS 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果，current_joints 字段包含当前关节角列表
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_get_current_joints(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    auto current_joints = _arm_->get_current_joints();
    if(current_joints.empty()) {
        return make_err(ErrorCode::FAILURE, "获取当前关节角失败");
    }

    ArmCmdResult result = make_ok("获取当前关节角成功");
    result.current_joints = std::move(current_joints);
    return result;
}

/**
 * @brief 处理 GET_CURRENT_POSE 命令，调用 ArmController 的 get_current_pose() 方法获取当前位姿
 * @param req 命令请求，GET_CURRENT_POSE 命令不需要额外参数
 * @return ArmCmdResult 结构体，表示命令执行结果，current_pose 字段包含当前位姿
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_get_current_pose(const ArmCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    ArmCmdResult result = make_ok("获取当前位姿成功");
    result.current_pose = _arm_->get_current_pose();
    return result;
}

/**
 * @brief 处理 MOVE_TO_ZERO 命令，调用 ArmController 的 reset_to_zero() 方法将机械臂重置到零点位置，并执行
 * @param req 命令请求，MOVE_TO_ZERO 命令不需要额外参数
 * @param cb 反馈回调函数，接受 ArmCmdFeedback 结构体参数
 * @return ArmCmdResult 结构体，表示命令执行结果
 */
ArmCmdResult ArmCmdDispatcher::Impl::handle_move_to_zero(const ArmCmdRequest& req, FeedbackCb cb) {

    report<ArmCmdFeedback>(cb, "start", 0.0, "开始执行 MOVE_TO_ZERO 命令");

    ErrorCode code = _arm_->reset_to_zero();
    if(code != ErrorCode::SUCCESS) {
        return make_err(code, "重置到零点失败：" + err_to_string(code));
    }

    report<ArmCmdFeedback>(cb, "done", 1.0, "命令执行成功");
    return make_ok();

}

/**
 * @brief 处理 OPEN_GRIPPER 命令，调用 JointEefInterface 的 open() 方法打开夹爪
 * @param req 命令请求，OPEN_GRIPPER 命令不需要额外参数
 * @param cb 反馈回调函数，接受 EefCmdFeedback 结构体参数
 * @return EefCmdResult 结构体，表示命令执行结果
 */
EefCmdResult EefCmdDispatcher::Impl::handle_open_gripper(const EefCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(!_eef_) return make_err(ErrorCode::FAILURE, "EndEffector 未初始化");

    auto code = find_eef_interface<GripperEefInterface>(*_eef_)
        .transform([](GripperEefInterface& ref) { return ref.open(); })
        .value_or(ErrorCode::INVALID_INTERFACE);
    if(code != ErrorCode::SUCCESS) return make_err(code, "打开夹爪失败：" + err_to_string(code));

    return make_ok();
}

/**
 * @brief 处理 CLOSE_GRIPPER 命令，调用 JointEefInterface 的 close() 方法关闭夹爪
 * @param req 命令请求，CLOSE_GRIPPER 命令不需要额外参数
 * @param cb 反馈回调函数，接受 EefCmdFeedback 结构体参数
 * @return EefCmdResult 结构体，表示命令执行结果
 */
EefCmdResult EefCmdDispatcher::Impl::handle_close_gripper(const EefCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(!_eef_) return make_err(ErrorCode::FAILURE, "EndEffector 未初始化");

    auto code = find_eef_interface<GripperEefInterface>(*_eef_)
        .transform([](GripperEefInterface& ref) { return ref.close(); })
        .value_or(ErrorCode::INVALID_INTERFACE);
    if(code != ErrorCode::SUCCESS) return make_err(code, "关闭夹爪失败：" + err_to_string(code));

    return make_ok();
}

/**
 * @brief 处理 STOP_GRIPPER 命令，调用 EndEffector 的 stop() 方法停止夹爪动作
 * @param req 命令请求，STOP_GRIPPER 命令不需要额外参数
 * @param cb 反馈回调函数，接受 EefCmdFeedback 结构体参数
 * @return EefCmdResult 结构体，表示命令执行结果
 */
EefCmdResult EefCmdDispatcher::Impl::handle_stop_gripper(const EefCmdRequest& req, FeedbackCb cb) {
    (void)cb;
    if(is_cancelled()) {
        return make_cancelled();
    }

    if(!_eef_) return make_err(ErrorCode::FAILURE, "EndEffector 未初始化");
    _eef_->stop();

    return make_ok();
}

}
