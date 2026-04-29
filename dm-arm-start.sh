#!/usr/bin/env bash

# 启动 DM-Arm ROS1 Noetic 控制链路
#
# 环境：
#   脚本会检查当前 ROS 环境是否为 Noetic，运行前请先确保处理 ROS Noetic 环境
#   检查通过后，脚本会自动 source ./ros_env/source-dm-arm.sh
#
# 常用：
#   ./dm-arm-start.sh                 # 真机链路：MoveIt + dm_hw + 上层接口 + RViz
#   ./dm-arm-start.sh --fake          # fake controller 链路：不启动 dm_hw，先验证上层/MoveIt
#   ./dm-arm-start.sh --no-rviz       # 不启动 RViz
#   ./dm-arm-start.sh --with-camera   # 启动相机和点云感知；默认不启动
#   ./dm-arm-start.sh --fake --with-camera --no-rviz
#
# 参数：
#   --fake          使用 MoveIt fake controller，不连接真机硬件
#   --with-camera   启动 dm_arm_camera 和 dm_arm_perception
#   --no-rviz       不启动 RViz
#   --delay SEC     收到退出信号后，尝试回零并等待 SEC 秒再关闭 roslaunch
#   -h, --help      打印本说明
#
# 退出：
#   Ctrl-C/SIGTERM 时，如果 /simple_move_arm action 已就绪，脚本会先尝试发送
#   MOVE_TO_ZERO，再关闭 roslaunch；fake 模式和真机模式共用该逻辑

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

WITH_CAMERA=false
FAKE_EXECUTION=false
USE_RVIZ=true
DELAY_SEC=2
ROSLAUNCH_PID=""
SUCCESS=false
CLEANING_UP=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --with-camera)
            WITH_CAMERA=true
            shift
            ;;
        --fake)
            FAKE_EXECUTION=true
            shift
            ;;
        --no-rviz)
            USE_RVIZ=false
            shift
            ;;
        --delay|-t)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --delay 需要一个秒数参数"
                exit 1
            fi
            DELAY_SEC="$2"
            shift 2
            ;;
        --help|-h)
            grep -E '^#' "$0" | sed 's/^# //'
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            echo "使用 --help 查看用法"
            exit 1
            ;;
    esac
done

CURRENT_ROS_DISTRO="${ROS_DISTRO:-}"
if [[ -z "$CURRENT_ROS_DISTRO" ]] && command -v rosversion >/dev/null 2>&1; then
    CURRENT_ROS_DISTRO="$(rosversion -d 2>/dev/null || true)"
fi

if [[ "$CURRENT_ROS_DISTRO" != "noetic" ]]; then
    echo "错误: 当前 ROS 环境不是 noetic"
    echo "当前检测值: ${CURRENT_ROS_DISTRO:-未检测到 ROS 环境}"
    echo "请先执行: mamba-usb rosnoetic"
    exit 1
fi

if [[ -f "$SCRIPT_DIR/ros_env/source-dm-arm.sh" ]]; then
    # shellcheck source=/dev/null
    source "$SCRIPT_DIR/ros_env/source-dm-arm.sh"
elif [[ -f "$SCRIPT_DIR/devel/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$SCRIPT_DIR/devel/setup.bash"
else
    echo "未找到 dm-ws/devel/setup.bash，请先在 ROS Noetic 环境下编译工作区"
    exit 1
fi

cleanup() {
    if [[ "$CLEANING_UP" == true ]]; then
        return
    fi
    CLEANING_UP=true

    if [[ "$SUCCESS" == true ]]; then
        echo "检测到中断，尝试令机械臂回到零点 ..."
        python3 - <<'PY' || echo "跳过回零动作（Action 未就绪）"
import sys
import actionlib
import rospy
from dm_arm_msgs.msg import SimpleMoveArmAction, SimpleMoveArmGoal

def main():
    rospy.init_node("dm_arm_exit_zero_checker", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient("/simple_move_arm", SimpleMoveArmAction)

    if not client.wait_for_server(rospy.Duration(1.5)):
        return 1

    goal = SimpleMoveArmGoal()
    goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(10.0)):
        client.cancel_goal()
        print("[WARN] 回零超时")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY
        sleep "$DELAY_SEC"
    fi

    if [[ -n "$ROSLAUNCH_PID" ]]; then
        echo "[ACTION] 正在关闭 roslaunch (PID: $ROSLAUNCH_PID)..."
        kill "$ROSLAUNCH_PID" 2>/dev/null || true
        sleep 1
        kill -9 "$ROSLAUNCH_PID" 2>/dev/null || true
    fi

    echo "退出完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "================ 启动 DM-Arm 系统 ================"
roslaunch dm_arm_interface dm_arm_start.launch \
    use_camera:="$WITH_CAMERA" \
    use_fake_execution:="$FAKE_EXECUTION" \
    use_rviz:="$USE_RVIZ" &
ROSLAUNCH_PID=$!
SUCCESS=true

wait "$ROSLAUNCH_PID" || true
