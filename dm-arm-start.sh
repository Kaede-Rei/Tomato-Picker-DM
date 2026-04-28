#!/usr/bin/env bash

# 启动 DM-Arm 系统脚本。
# 默认不启动相机；需要相机和点云感知时传入 --with-camera。

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

if ! command -v mamba-usb >/dev/null 2>&1 && [[ -f "$HOME/.bashrc" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/.bashrc"
fi

if command -v mamba-usb >/dev/null 2>&1; then
    shopt -s expand_aliases
    mamba-usb rosnoetic
elif [[ -f "/media/kaede-rei/AgroTech/home/activate.sh" ]]; then
    # shellcheck source=/dev/null
    source "/media/kaede-rei/AgroTech/home/activate.sh" rosnoetic
else
    echo "[WARN] 未找到 mamba-usb，继续使用当前 shell 环境"
fi

if [[ -f "$SCRIPT_DIR/devel/setup.bash" ]]; then
    source "$SCRIPT_DIR/devel/setup.bash"
else
    echo "未找到 dm-ws/devel/setup.bash，请先在 mamba-usb rosnoetic 环境下编译工作区"
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
