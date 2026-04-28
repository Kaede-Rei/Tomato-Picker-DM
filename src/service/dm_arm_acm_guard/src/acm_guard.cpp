#include "dm_arm_acm_guard/acm_guard.hpp"

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

namespace dm_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 函 数 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 实 现 ========================= ! //

AcmGuard::AcmGuard(std::string link_name, const ros::NodeHandle& nh)
    : nh_(nh), link_name_(std::move(link_name)) {

    scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
    apply_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
}

AcmGuard::~AcmGuard(void) {
    if(allowed_) disallow();
}

bool AcmGuard::allow(void) {
    if(allowed_) return true;

    moveit_msgs::AllowedCollisionMatrix current_acm;
    if(!get_current_acm(current_acm)) {
        ROS_ERROR("获取当前 ACM 失败");
        return false;
    }

    acm_ = current_acm;
    set_link_acm(acm_, link_name_, true);

    if(!apply_acm(acm_)) {
        ROS_ERROR("应用 ACM 失败");
        return false;
    }

    allowed_ = true;
    ROS_INFO("已允许 %s 与其他物体碰撞", link_name_.c_str());
    return true;
}

bool AcmGuard::disallow(void) {
    if(!allowed_) return true;

    moveit_msgs::AllowedCollisionMatrix current_acm;
    if(!get_current_acm(current_acm)) {
        ROS_ERROR("获取当前 ACM 失败");
        return false;
    }

    acm_ = current_acm;
    set_link_acm(acm_, link_name_, false);

    if(!apply_acm(acm_)) {
        ROS_ERROR("应用 ACM 失败");
        return false;
    }

    allowed_ = false;
    ROS_INFO("已禁止 %s 与其他物体碰撞", link_name_.c_str());
    return true;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

bool AcmGuard::get_current_acm(moveit_msgs::AllowedCollisionMatrix& acm) {
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

    if(!scene_client_.waitForExistence(ros::Duration(0.5))) {
        ROS_WARN("get_planning_scene 服务不可用");
        return false;
    }

    if(!scene_client_.call(srv)) {
        ROS_ERROR("调用 get_planning_scene 服务失败");
        return false;
    }

    acm = srv.response.scene.allowed_collision_matrix;
    return true;
}

bool AcmGuard::apply_acm(const moveit_msgs::AllowedCollisionMatrix& acm) {
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene.is_diff = true;
    srv.request.scene.allowed_collision_matrix = acm;

    if(!apply_client_.waitForExistence(ros::Duration(0.5))) {
        ROS_WARN("/apply_planning_scene 服务不可用");
        return false;
    }

    if(!apply_client_.call(srv)) {
        ROS_ERROR("调用 /apply_planning_scene 服务失败");
        return false;
    }

    return true;
}

void AcmGuard::set_link_acm(moveit_msgs::AllowedCollisionMatrix& acm, const std::string& link_name, bool allowed) {
    auto it = std::find(acm.default_entry_names.begin(), acm.default_entry_names.end(), link_name);
    if(it != acm.default_entry_names.end()) {
        std::size_t index = std::distance(acm.default_entry_names.begin(), it);
        acm.default_entry_values[index] = allowed;
    }
    else {
        acm.default_entry_names.push_back(link_name);
        acm.default_entry_values.push_back(allowed);
    }
}

}
