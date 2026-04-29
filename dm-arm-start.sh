#!/usr/bin/env bash

# 启动 DM-Arm ROS1 Noetic 控制链路
#
# 环境：
#   脚本会检查当前 ROS 环境是否为 Noetic，运行前请先确保处于 ROS Noetic 环境
#   检查通过后，脚本会自动 source ./ros_env/source-dm-arm.sh
#
# 常用：
#   ./dm-arm-start.sh                 # 真机链路：MoveIt + dm_hw + 上层接口 + RViz
#   ./dm-arm-start.sh --fake          # fake controller 链路：不启动 dm_hw，先验证上层/MoveIt
#   ./dm-arm-start.sh --no-rviz       # 不启动 RViz
#   ./dm-arm-start.sh --with-camera   # 启动相机和点云感知；默认不启动
#   ./dm-arm-start.sh --serial-port /dev/ttyUSB0 --baudrate 921600
#   ./dm-arm-start.sh --fake --with-camera --no-rviz
#
# 参数：
#   --fake                  使用 MoveIt fake controller，不连接真机硬件
#   --with-camera           启动 dm_arm_camera 和 dm_arm_perception
#   --no-rviz               不启动 RViz
#   --serial-port P         真机链路使用的 dm_hw 串口；默认 /dev/ttyACM0
#   --baudrate B            真机链路使用的 dm_hw 波特率；默认 921600
#   --hardware-config FILE  使用指定 dm_hw 配置文件；默认 dm_hw/config/dm_controller.yaml
#   --delay SEC             收到退出信号后，尝试回零并等待 SEC 秒再关闭 roslaunch
#   --shutdown-grace SEC    发送 TERM 后等待 roslaunch 优雅退出的秒数；默认 30
#   -h, --help              打印本说明
#
# 退出：
#   Ctrl-C/SIGTERM 时，如果 /simple_move_arm action 已就绪，脚本会先尝试发送
#   MOVE_TO_ZERO，再关闭 roslaunch；fake 模式和真机模式共用该逻辑。
#   roslaunch 会运行在独立 session 中，避免终端 Ctrl-C 直接先杀掉 action server。

set -e

DM_ARM_WS_DIR="$(cd "$(dirname "$0")" && pwd)"

WITH_CAMERA=false
FAKE_EXECUTION=false
USE_RVIZ=true
DELAY_SEC=2
SHUTDOWN_GRACE_SEC=30
HARDWARE_SERIAL_PORT="/dev/ttyACM0"
HARDWARE_BAUDRATE="921600"
HARDWARE_CONFIG_FILE=""
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
        --serial-port)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --serial-port 需要一个串口路径参数"
                exit 1
            fi
            HARDWARE_SERIAL_PORT="$2"
            shift 2
            ;;
        --baudrate)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --baudrate 需要一个波特率参数"
                exit 1
            fi
            HARDWARE_BAUDRATE="$2"
            shift 2
            ;;
        --hardware-config)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --hardware-config 需要一个配置文件路径参数"
                exit 1
            fi
            HARDWARE_CONFIG_FILE="$2"
            shift 2
            ;;
        --delay|-t)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --delay 需要一个秒数参数"
                exit 1
            fi
            DELAY_SEC="$2"
            shift 2
            ;;
        --shutdown-grace)
            if [[ -z "${2:-}" ]]; then
                echo "错误: --shutdown-grace 需要一个秒数参数"
                exit 1
            fi
            SHUTDOWN_GRACE_SEC="$2"
            shift 2
            ;;
        --help|-h)
            grep -E '^#( |$)' "$0" | sed -e 's/^# //' -e 's/^#$//'
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

if [[ -f "$DM_ARM_WS_DIR/ros_env/source-dm-arm.sh" ]]; then
    # shellcheck source=/dev/null
    source "$DM_ARM_WS_DIR/ros_env/source-dm-arm.sh"
elif [[ -f "$DM_ARM_WS_DIR/devel/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$DM_ARM_WS_DIR/devel/setup.bash"
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

    if not client.wait_for_server(rospy.Duration(8.0)):
        print("[WARN] /simple_move_arm action 未就绪")
        return 1

    goal = SimpleMoveArmGoal()
    goal.command_type = SimpleMoveArmGoal.MOVE_TO_ZERO
    client.send_goal(goal)

    if not client.wait_for_result(rospy.Duration(30.0)):
        client.cancel_goal()
        print("[WARN] 回零超时")
        return 2

    result = client.get_result()
    if result is not None and hasattr(result, "success") and not result.success:
        print("[WARN] 回零失败: {}".format(getattr(result, "message", "")))
        return 3

    print("[INFO] 回零动作完成")
    return 0

if __name__ == "__main__":
    sys.exit(main())
PY
        if [[ "$FAKE_EXECUTION" != true ]]; then
            echo "尝试关闭 dm_hw 硬件接口 ..."
            python3 - <<'PY' || echo "跳过 dm_hw 主动失能（Service 未就绪）"
import sys
import rospy
from std_srvs.srv import Trigger

def main():
    rospy.init_node("dm_hw_shutdown_client", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service("/dm_hw/shutdown", timeout=5.0)
        shutdown = rospy.ServiceProxy("/dm_hw/shutdown", Trigger)
        resp = shutdown()
        print("[INFO] /dm_hw/shutdown: success={} message={}".format(resp.success, resp.message))
        return 0 if resp.success else 1
    except Exception as exc:
        print("[WARN] /dm_hw/shutdown 调用失败: {}".format(exc))
        return 1

if __name__ == "__main__":
    sys.exit(main())
PY
        fi
        sleep "$DELAY_SEC"
    fi

    if [[ -n "$ROSLAUNCH_PID" ]]; then
        echo "[ACTION] 正在关闭 roslaunch (PID: $ROSLAUNCH_PID)..."
        kill -TERM "-$ROSLAUNCH_PID" 2>/dev/null || kill "$ROSLAUNCH_PID" 2>/dev/null || true
        for ((elapsed = 0; elapsed < SHUTDOWN_GRACE_SEC; ++elapsed)); do
            if ! kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
                break
            fi
            sleep 1
        done
        if kill -0 "$ROSLAUNCH_PID" 2>/dev/null; then
            echo "[WARN] roslaunch 未在 ${SHUTDOWN_GRACE_SEC}s 内退出，强制关闭"
            kill -KILL "-$ROSLAUNCH_PID" 2>/dev/null || kill -9 "$ROSLAUNCH_PID" 2>/dev/null || true
        fi
    fi

    echo "退出完成"
    exit 0
}

trap cleanup SIGINT SIGTERM

echo "================ 启动 DM-Arm 系统 ================"
ROSLAUNCH_ARGS=(
    use_camera:="$WITH_CAMERA"
    use_fake_execution:="$FAKE_EXECUTION"
    use_rviz:="$USE_RVIZ"
    hardware_serial_port:="$HARDWARE_SERIAL_PORT"
    hardware_baudrate:="$HARDWARE_BAUDRATE"
)

if [[ -n "$HARDWARE_CONFIG_FILE" ]]; then
    ROSLAUNCH_ARGS+=(hardware_config_file:="$HARDWARE_CONFIG_FILE")
fi

if command -v setsid >/dev/null 2>&1; then
    setsid roslaunch dm_arm_interface dm_arm_start.launch "${ROSLAUNCH_ARGS[@]}" &
else
    roslaunch dm_arm_interface dm_arm_start.launch "${ROSLAUNCH_ARGS[@]}" &
fi
ROSLAUNCH_PID=$!
SUCCESS=true

wait "$ROSLAUNCH_PID" || true
