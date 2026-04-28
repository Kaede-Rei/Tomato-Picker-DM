#pragma once

#include <ros/ros.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>

namespace dm_arm {

// ! ========================= 接 口 变 量 / 结 构 体 / 枚 举 声 明 ========================= ! //



// ! ========================= 接 口 类 / 函 数 声 明 ========================= ! //

class AcmGuard {
public:
    AcmGuard(std::string link_name, const ros::NodeHandle& nh = ros::NodeHandle());
    ~AcmGuard(void);

    bool allow(void);
    bool disallow(void);

private:
    bool get_current_acm(moveit_msgs::AllowedCollisionMatrix& acm);
    bool apply_acm(const moveit_msgs::AllowedCollisionMatrix& acm);
    void set_link_acm(moveit_msgs::AllowedCollisionMatrix& acm, const std::string& link_name, bool allowed);

private:
    ros::NodeHandle nh_;
    std::string link_name_;

    ros::ServiceClient scene_client_;
    ros::ServiceClient apply_client_;

    moveit_msgs::AllowedCollisionMatrix acm_;
    bool allowed_{ false };
};

// ! ========================= 模 版 方 法 实 现 ========================= ! //



}
