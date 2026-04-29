# 激活 dm-ws 的 ROS 环境
# Usage:
#   source ./ros_env/source-dm-arm.sh

DM_ARM_ROS_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$DM_ARM_ROS_ENV_DIR/../devel/setup.bash"
