#!/usr/bin/env bash

# DM-Arm 交互式测试入口
#
# 环境：
#   脚本会检查当前 ROS 环境是否为 Noetic
#   运行前请先执行 mamba-usb rosnoetic，或手动进入等价的 ROS1 Noetic 环境
#   检查通过后，脚本会自动 source ./ros_env/source-dm-arm.sh
#
# 前置：
#   先启动控制链路，例如：
#     ./dm-arm-start.sh --fake
#   或真机：
#     ./dm-arm-start.sh
#
# 用法：
#   ./dm-arm-test.sh
#   ./dm-arm-test.sh --wait 120
#
# 常用参数：
#   --wait SEC                 等待 action/service 就绪的秒数，默认 60
#   --yes                      跳过运动命令二次确认
#   --move-action NAME         覆盖 SimpleMoveArm action 名称，默认 /simple_move_arm
#   --query-service NAME       覆盖 QueryArm service 名称，默认 /arm_query
#   --eef-service NAME         覆盖 CommandEef service 名称，默认 /eef_cmd
#   -h, --help                 打印本说明

set -e

DM_ARM_WS_DIR="$(cd "$(dirname "$0")" && pwd)"

for arg in "$@"; do
    case "$arg" in
        --help|-h)
            awk '
                NR == 1 && /^#!/ { next }
                /^$/ { next }
                /^# ?/ { sub(/^# ?/, ""); print; next }
                NR > 1 { exit }
            ' "$0"
            exit 0
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

if ! rosnode list >/dev/null 2>&1; then
    echo "错误: 无法连接 ROS master"
    echo "请先启动: ./dm-arm-start.sh --fake"
    exit 1
fi

exec python3 "$DM_ARM_WS_DIR/scripts/dm_arm_interactive_test.py" "$@"
