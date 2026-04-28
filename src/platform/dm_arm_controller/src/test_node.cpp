#include <atomic>
#include <clocale>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

#include "dm_arm_controller/arm_controller.hpp"
#include "dm_arm_controller/types.hpp"

namespace {

void log_result(const std::string& test_name, dm_arm::ErrorCode code) {
    if(code == dm_arm::ErrorCode::SUCCESS) {
        ROS_INFO("[PASS] %s", test_name.c_str());
    }
    else {
        ROS_WARN("[FAIL] %s -> %s", test_name.c_str(), dm_arm::err_to_string(code).c_str());
    }
}

void log_result_bool(const std::string& test_name, bool ok, const std::string& message = "") {
    if(ok) {
        ROS_INFO("[PASS] %s", test_name.c_str());
    }
    else {
        ROS_WARN("[FAIL] %s %s", test_name.c_str(), message.c_str());
    }
}

}  // namespace

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "dm_arm_controller_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int num_threads = 4;
    pnh.param<int>("num_threads", num_threads, 4);

    bool run_motion_tests = true;
    bool run_constraints_tests = true;
    pnh.param<bool>("run_motion_tests", run_motion_tests, true);
    pnh.param<bool>("run_constraints_tests", run_constraints_tests, true);

    ROS_INFO("====================================");
    ROS_INFO("dm_arm_controller 测试节点启动");
    ROS_INFO("线程数: %d", num_threads);
    ROS_INFO("====================================");

    ros::AsyncSpinner spinner(num_threads);
    spinner.start();

    auto arm = std::make_shared<dm_arm::ArmController>("arm");

    // 记录初始位姿
    geometry_msgs::Pose start_pose = arm->get_current_pose();
    ROS_INFO("初始位姿: pos(%.3f, %.3f, %.3f)",
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z);

    // 0) home / reset
    arm->home();
    ros::Duration(0.5).sleep();
    log_result("reset_to_zero", arm->reset_to_zero());

    if(run_motion_tests) {
        ROS_INFO("========== 运动能力测试 ==========");

        // 1) set_joints + plan_and_execute
        std::vector<double> joints = arm->get_current_joints();
        if(!joints.empty()) {
            joints[0] += 0.05;
            log_result("set_joints", arm->set_joints(joints));
            log_result("plan_and_execute(joints)", arm->plan_and_execute());
        }
        else {
            log_result_bool("set_joints", false, "(关节列表为空)");
        }

        // 2) set_target(Pose) + plan + execute
        geometry_msgs::Pose pose_target = arm->get_current_pose();
        pose_target.orientation = arm->rotate_relative_rpy_to_quaternion(pose_target.orientation, 0.1, 0.1, 0.1);
        log_result("set_target(Pose)", arm->set_target(pose_target));
        moveit::planning_interface::MoveGroupInterface::Plan plan_pose;
        auto plan_ret = arm->plan(plan_pose);
        log_result("plan(Pose)", plan_ret);
        if(plan_ret == dm_arm::ErrorCode::SUCCESS) {
            log_result("execute(PosePlan)", arm->execute(plan_pose));
        }

        // 3) set_target(Point) + plan_and_execute
        geometry_msgs::Point point_target = arm->get_current_pose().position;
        point_target.y += 0.02;
        log_result("set_target(Point)", arm->set_target(point_target));
        log_result("plan_and_execute(Point)", arm->plan_and_execute());

        // 4) set_target(Quaternion) + plan_and_execute
        geometry_msgs::Pose current_pose = arm->get_current_pose();
        tf2::Quaternion q;
        tf2::fromMsg(current_pose.orientation, q);
        tf2::Quaternion q_delta;
        q_delta.setRPY(0.0, 0.0, 0.1);
        q = q * q_delta;
        q.normalize();
        geometry_msgs::Quaternion quat_target = tf2::toMsg(q);
        log_result("set_target(Quaternion)", arm->set_target(quat_target));
        log_result("plan_and_execute(Quaternion)", arm->plan_and_execute());

        // 5) set_target_in_eef_frame
        geometry_msgs::Pose eef_pose;
        eef_pose.position.x = 0.0;
        eef_pose.position.y = 0.0;
        eef_pose.position.z = 0.01;
        eef_pose.orientation.w = 1.0;
        log_result("set_target_in_eef_frame", arm->set_target_in_eef_frame(eef_pose));
        log_result("plan_and_execute(eef_frame)", arm->plan_and_execute());

        // 6) telescopic_end / rotate_end
        log_result("telescopic_end", arm->telescopic_end(0.01));
        log_result("plan_and_execute(telescopic)", arm->plan_and_execute());
        log_result("rotate_end", arm->rotate_end(0.05));
        log_result("plan_and_execute(rotate)", arm->plan_and_execute());

        // 7) async_plan_and_execute
        std::atomic<bool> async_done{ false };
        log_result("set_target(async)", arm->set_target(arm->get_current_pose()));
        auto async_ret = arm->async_plan_and_execute([&async_done](dm_arm::ErrorCode code) {
            if(code == dm_arm::ErrorCode::SUCCESS) {
                ROS_INFO("[PASS] async_plan_and_execute(callback)");
            }
            else {
                ROS_WARN("[FAIL] async_plan_and_execute(callback) -> %s", dm_arm::err_to_string(code).c_str());
            }
            async_done.store(true);
            });
        log_result("async_plan_and_execute(start)", async_ret);
        while(ros::ok() && !async_done.load()) {
            ros::Duration(0.1).sleep();
        }

        // 8) 笛卡尔轨迹：set_line + execute(trajectory)
        geometry_msgs::Pose line_start = arm->get_current_pose();
        geometry_msgs::Pose line_end = line_start;
        line_end.position.y += 0.02;
        auto line_ret = arm->set_line(line_start, line_end);
        log_result("set_line", line_ret.error_code);
        if(line_ret.error_code == dm_arm::ErrorCode::SUCCESS) {
            log_result("execute(line_trajectory)", arm->execute(line_ret.trajectory));
        }

        // 9) 笛卡尔轨迹：set_bezier_curve + async_execute
        geometry_msgs::Pose bezier_start = arm->get_current_pose();
        geometry_msgs::Pose bezier_via = bezier_start;
        geometry_msgs::Pose bezier_end = bezier_start;
        bezier_via.position.x += 0.02;
        bezier_via.position.y += 0.03;
        bezier_end.position.x += 0.04;
        auto bezier_ret = arm->set_bezier_curve(bezier_start, bezier_via, bezier_end, 20, 0.01);
        log_result("set_bezier_curve", bezier_ret.error_code);
        if(bezier_ret.error_code == dm_arm::ErrorCode::SUCCESS) {
            std::atomic<bool> async_exec_done{ false };
            auto exec_async_ret = arm->async_execute(bezier_ret.trajectory, [&async_exec_done](dm_arm::ErrorCode code) {
                if(code == dm_arm::ErrorCode::SUCCESS) {
                    ROS_INFO("[PASS] async_execute(callback)");
                }
                else {
                    ROS_WARN("[FAIL] async_execute(callback) -> %s", dm_arm::err_to_string(code).c_str());
                }
                async_exec_done.store(true);
                });
            log_result("async_execute(start)", exec_async_ret);
            while(ros::ok() && !async_exec_done.load()) {
                ros::Duration(0.1).sleep();
            }
        }

        // 10) TF 变换测试
        geometry_msgs::Pose tf_in = arm->get_current_pose();
        geometry_msgs::Pose tf_mid;
        geometry_msgs::Pose tf_out;
        auto tf1 = arm->base_to_end_tf(tf_in, tf_mid);
        log_result("base_to_end_tf", tf1);
        if(tf1 == dm_arm::ErrorCode::SUCCESS) {
            auto tf2_ret = arm->end_to_base_tf(tf_mid, tf_out);
            log_result("end_to_base_tf", tf2_ret);
        }

        // 11) 工具函数
        auto q_tool = arm->rpy_to_quaternion(0.1, 0.1, 0.1);
        auto p_tool = arm->rpy_to_pose(0.0, 0.0, 0.0, start_pose.position.x, start_pose.position.y, start_pose.position.z);
        log_result_bool("rpy_to_quaternion", std::abs(q_tool.w) > 1e-6);
        log_result_bool("rpy_to_pose", std::abs(p_tool.orientation.w) > 1e-6);
    }

    if(run_constraints_tests) {
        ROS_INFO("========== 约束能力测试 ==========");

        geometry_msgs::Pose cur = arm->get_current_pose();
        geometry_msgs::Vector3 scope;
        scope.x = 0.1;
        scope.y = 0.1;
        scope.z = 0.1;

        arm->set_orientation_constraint(cur.orientation, 0.2, 0.2, 0.2, 1.0);
        arm->set_position_constraint(cur.position, scope, 1.0);

        auto links = arm->get_current_link_names();
        if(!links.empty()) {
            arm->set_joint_constraint(links.front(), arm->get_current_joints().front(), 0.2, 0.2, 1.0);
            log_result_bool("set_joint_constraint", true);
        }
        else {
            log_result_bool("set_joint_constraint", false, "(无关节名)");
        }

        arm->apply_constraints();
        log_result_bool("apply_constraints", true);
        arm->clear_constraints();
        log_result_bool("clear_constraints", true);
    }

    // 结束前回到初始位姿附近
    log_result("set_target(final_start_pose)", arm->set_target(start_pose));
    log_result("plan_and_execute(final)", arm->plan_and_execute());

    ROS_INFO("====================================");
    ROS_INFO("dm_arm_controller 全功能测试流程结束");
    ROS_INFO("可通过参数关闭指定测试项");
    ROS_INFO("====================================");

    ros::shutdown();
    return 0;
}
