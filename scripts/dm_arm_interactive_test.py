#!/usr/bin/env python3

import argparse
import math
import sys
from typing import List, Optional

import actionlib
import rospy
from dm_arm_msgs.msg import SimpleMoveArmAction, SimpleMoveArmGoal
from dm_arm_msgs.srv import CommandEef, CommandEefRequest, QueryArm, QueryArmRequest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DM-Arm interactive test client")
    parser.add_argument("--move-action", default="/simple_move_arm", help="SimpleMoveArm action name")
    parser.add_argument("--query-service", default="/arm_query", help="QueryArm service name")
    parser.add_argument("--eef-service", default="/eef_cmd", help="CommandEef service name")
    parser.add_argument("--wait", type=float, default=60.0, help="seconds to wait for interfaces")
    parser.add_argument("--yes", action="store_true", help="skip safety confirmation prompts")
    return parser.parse_args()


def confirm(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    answer = input(f"{prompt} 输入 yes 继续: ").strip()
    return answer == "yes"


def read_floats(prompt: str, count: int) -> Optional[List[float]]:
    raw = input(prompt).strip()
    if not raw:
        return None
    parts = raw.replace(",", " ").split()
    if len(parts) != count:
        print(f"需要 {count} 个数字，实际收到 {len(parts)} 个。")
        return None
    try:
        return [float(item) for item in parts]
    except ValueError:
        print("输入包含非数字。")
        return None


def rpy_to_quat(roll: float, pitch: float, yaw: float) -> List[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def print_pose(prefix: str, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> None:
    print(f"{prefix}:")
    print(f"  xyz = [{x:.6f}, {y:.6f}, {z:.6f}]")
    print(f"  rpy = [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]")


class DMArmInteractiveTester:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        rospy.init_node("dm_arm_interactive_test", anonymous=True)
        self.move_client = actionlib.SimpleActionClient(args.move_action, SimpleMoveArmAction)
        self.query_srv = None
        self.eef_srv = None

    def wait_ready(self) -> bool:
        timeout = rospy.Duration(self.args.wait)
        print(f"等待 action: {self.args.move_action}")
        if not self.move_client.wait_for_server(timeout):
            print("SimpleMoveArm action 未就绪。")
            return False

        print(f"等待 service: {self.args.query_service}")
        try:
            rospy.wait_for_service(self.args.query_service, timeout.to_sec())
            self.query_srv = rospy.ServiceProxy(self.args.query_service, QueryArm)
        except rospy.ROSException as exc:
            print(f"QueryArm service 未就绪: {exc}")
            return False

        print(f"等待 service: {self.args.eef_service}")
        try:
            rospy.wait_for_service(self.args.eef_service, min(timeout.to_sec(), 5.0))
            self.eef_srv = rospy.ServiceProxy(self.args.eef_service, CommandEef)
        except rospy.ROSException:
            print("CommandEef service 未就绪，夹爪测试项会不可用。")

        print("接口已就绪。")
        return True

    def run(self) -> int:
        if not self.wait_ready():
            return 1

        while not rospy.is_shutdown():
            self.print_menu()
            choice = input("选择: ").strip().lower()
            if choice in {"q", "quit", "exit"}:
                return 0
            if choice == "1":
                self.query_joints()
            elif choice == "2":
                self.query_pose()
            elif choice == "3":
                self.move_to_zero()
            elif choice == "4":
                self.home()
            elif choice == "5":
                self.move_joints()
            elif choice == "6":
                self.move_pose()
            elif choice == "7":
                self.move_relative_eef()
            elif choice == "8":
                self.open_gripper()
            elif choice == "9":
                self.close_gripper()
            elif choice == "10":
                self.stop_gripper()
            else:
                print("未知选项。")
        return 0

    @staticmethod
    def print_menu() -> None:
        print()
        print("==== DM-Arm 交互式测试 ====")
        print("1. 查询当前关节")
        print("2. 查询当前位姿")
        print("3. 回零 MOVE_TO_ZERO")
        print("4. HOME")
        print("5. 关节运动")
        print("6. 末端位姿运动")
        print("7. 末端坐标系相对运动")
        print("8. 打开夹爪")
        print("9. 关闭夹爪")
        print("10. 停止夹爪")
        print("q. 退出")

    def query_joints(self) -> None:
        req = QueryArmRequest()
        req.command_type = QueryArmRequest.GET_CURRENT_JOINTS
        res = self.query_srv(req)
        print(f"success={res.success} error_code={res.error_code} message={res.message}")
        if res.cur_joint:
            print("joints(rad): [" + ", ".join(f"{value:.6f}" for value in res.cur_joint) + "]")

    def query_pose(self) -> None:
        req = QueryArmRequest()
        req.command_type = QueryArmRequest.GET_CURRENT_POSE
        res = self.query_srv(req)
        print(f"success={res.success} error_code={res.error_code} message={res.message}")
        pose = res.cur_pose
        print(f"position = [{pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}]")
        print(
            "orientation(xyzw) = "
            f"[{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
            f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f}]"
        )

    def move_to_zero(self) -> None:
        if not confirm("将发送 MOVE_TO_ZERO 运动命令。", self.args.yes):
            return
        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
        self.send_goal(goal)

    def home(self) -> None:
        if not confirm("将发送 HOME 运动命令。", self.args.yes):
            return
        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.HOME
        self.send_goal(goal)

    def move_joints(self) -> None:
        values = read_floats("输入 6 个关节角(rad)，空格分隔: ", 6)
        if values is None:
            return
        if not confirm("将发送关节运动命令。", self.args.yes):
            return
        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_JOINTS
        goal.joints = values
        self.send_goal(goal)

    def move_pose(self) -> None:
        values = read_floats("输入 x y z roll pitch yaw，单位 m/rad: ", 6)
        if values is None:
            return
        if not confirm("将发送末端位姿运动命令。", self.args.yes):
            return
        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_TARGET
        goal.target_type = SimpleMoveArmGoal.TARGET_POSE
        goal.x = [values[0]]
        goal.y = [values[1]]
        goal.z = [values[2]]
        goal.roll = [values[3]]
        goal.pitch = [values[4]]
        goal.yaw = [values[5]]
        self.send_goal(goal)

    def move_relative_eef(self) -> None:
        values = read_floats("输入相对末端坐标系的 x y z roll pitch yaw，单位 m/rad: ", 6)
        if values is None:
            return
        if not confirm("将发送末端坐标系相对运动命令。", self.args.yes):
            return
        goal = SimpleMoveArmGoal()
        goal.command_type = SimpleMoveArmGoal.MOVE_TARGET_IN_EEF_FRAME
        goal.target_type = SimpleMoveArmGoal.TARGET_POSE
        goal.x = [values[0]]
        goal.y = [values[1]]
        goal.z = [values[2]]
        goal.roll = [values[3]]
        goal.pitch = [values[4]]
        goal.yaw = [values[5]]
        self.send_goal(goal)

    def open_gripper(self) -> None:
        self.send_eef(CommandEefRequest.OPEN_GRIPPER, "打开夹爪")

    def close_gripper(self) -> None:
        self.send_eef(CommandEefRequest.CLOSE_GRIPPER, "关闭夹爪")

    def stop_gripper(self) -> None:
        self.send_eef(CommandEefRequest.STOP_GRIPPER, "停止夹爪")

    def send_eef(self, command_type: int, text: str) -> None:
        if self.eef_srv is None:
            print("CommandEef service 不可用。")
            return
        if command_type != CommandEefRequest.STOP_GRIPPER:
            if not confirm(f"将发送 {text} 命令。", self.args.yes):
                return
        req = CommandEefRequest()
        req.command_type = command_type
        res = self.eef_srv(req)
        print(f"success={res.success} error_code={res.error_code} message={res.message}")

    def send_goal(self, goal: SimpleMoveArmGoal) -> None:
        print("发送 action goal ...")
        self.move_client.send_goal(goal, feedback_cb=self.on_feedback)
        if not self.move_client.wait_for_result(rospy.Duration(60.0)):
            self.move_client.cancel_goal()
            print("\n超时，已取消 goal。")
            return
        print()
        result = self.move_client.get_result()
        print(f"success={result.success} error_code={result.error_code} message={result.message}")
        if result.cur_joint:
            print("cur_joint(rad): [" + ", ".join(f"{value:.6f}" for value in result.cur_joint) + "]")
        print_pose("cur_pose", result.cur_x, result.cur_y, result.cur_z, result.cur_roll, result.cur_pitch, result.cur_yaw)

    @staticmethod
    def on_feedback(feedback) -> None:
        print(f"\r[{feedback.stage}] {feedback.progress * 100.0:5.1f}% {feedback.message}", end="", flush=True)


def main() -> int:
    args = parse_args()
    try:
        tester = DMArmInteractiveTester(args)
        return tester.run()
    except KeyboardInterrupt:
        print()
        return 130
    except rospy.ROSInterruptException:
        return 130


if __name__ == "__main__":
    sys.exit(main())
