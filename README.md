<div align="center">

# DM-Arm ROS: 达妙机械臂 ROS 控制系统

基于 **ROS Noetic + MoveIt + dm_hw + DM-Arm + 可选 Gemini335L 腕上相机** 的机械臂控制与采摘实验工作区

本项目由 `piper-ws` 上层架构移植而来，保留“接口层 -> 命令层 -> 控制层 -> MoveIt -> 硬件层”的分层结构，将底层硬件接口替换为 `dm_hw` 达妙电机串口/USB-CAN 控制链路；相机与点云感知链路为可选项，默认不启动

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Framework: MoveIt](https://img.shields.io/badge/Framework-MoveIt-green.svg)](https://moveit.ros.org/)
[![Hardware: dm_hw](https://img.shields.io/badge/Hardware-dm__hw-red.svg)](src/platform/dm_hw)
[![Camera: Optional](https://img.shields.io/badge/Camera-Optional-orange.svg)](src/device/dm_arm_camera)

</div>

---

## 1. 系统能力概览

- 达妙 DM-Arm ROS1 Noetic 硬件接口接入
- `dm_hw` 基于 `ros_control` 暴露 `PositionJointInterface`
- `joint1..joint6` 通过 `arm_controller` 接入 MoveIt
- `gripper_left` 通过 `end_controller` 接入 MoveIt `end` group，对应达妙电机 `0x07`
- MoveIt 规划、执行与当前状态查询
- `/move_arm`、`/simple_move_arm`、`/pick_action`、`/arm_config`、`/arm_query`、`/eef_cmd` 接口
- fake controller 链路，用于不连接真机时验证上层、MoveIt 和接口逻辑
- Gemini335L 腕上相机接入为可选项，默认关闭
- 点云生成、TF 变换、工作空间裁剪、VoxelGrid 降采样、SOR 离群点滤波
- MoveIt Octomap 点云输入开关与异步清图
- 采摘任务阶段机
- 接近/采摘/退出阶段 `link6` 临时 ACM 放行
- 取消任务、停止执行、回安全位流程

---

## 2. 推荐使用场景

当前版本适合：

- 先在 fake controller 中验证上层控制链路
- DM-Arm 真机控制链路调试
- 达妙电机 ID、类型、串口、波特率、控制模式调试
- MoveIt 规划参数、笛卡尔路径参数调试
- 可选腕上相机手眼标定、深度估计和点云避障验证
- 采摘动作任务流验证

当前版本不建议直接用于：

- 未完成真机限位和急停验证的长时间自动运行
- 未确认 `dm_hw/config/dm_controller.yaml` 中电机类型和 ID 的真机上电测试
- 底盘移动过程中持续维护全局 Octomap
- 未经人工检查的长时间自主采摘
- 未做对象分割的复杂枝叶环境精细避障

---

## 3. 硬件与软件栈

| 类别 | 当前配置 |
|---|---|
| 机械臂 | DM-Arm / 达妙电机链路 |
| 硬件接口 | `dm_hw`，串口/USB-CAN |
| 末端执行器 | `gripper_left`，默认电机 ID `0x07` |
| 相机 | Orbbec Gemini335L，可选 |
| 主机 | Ubuntu 20.04 原生 ROS Noetic，或 Ubuntu 22.04 + micromamba ROS Noetic 虚拟环境 |
| ROS | ROS Noetic |
| 规划 | MoveIt 1 |
| 控制 | ros_control, controller_manager, JointTrajectoryController |
| 点云处理 | PCL, pcl_ros, tf2_sensor_msgs |
| GUI | Python, PyQt5, OpenCV |

---

## 4. 项目结构

```text
dm-ws/
├── README.md
├── dm-arm-start.sh                  # 总启动脚本
├── dm-arm-test.sh                   # 交互式测试脚本
├── ros_env/                         # ROS Noetic 虚拟环境辅助脚本
├── scripts/                         # 工作区级辅助脚本
├── piper-ws/                        # 原始 piper-ws 参考工作区
└── src/
    ├── platform/
    │   ├── dm_hw/                   # 达妙硬件接口与 ros_control 入口
    │   ├── dm_arm_msgs/             # 自定义 Action/Service/Msg
    │   ├── dm_arm_description/      # URDF 与 meshes
    │   ├── dm_arm_moveit_config/    # MoveIt 配置
    │   └── dm_arm_controller/       # MoveIt 控制与运动规划封装
    ├── device/
    │   └── dm_arm_camera/           # Gemini335L 相机节点，可选
    ├── service/
    │   ├── dm_arm_commander/        # 命令分发
    │   ├── dm_arm_perception/       # 点云生成与 Octomap 输入控制
    │   └── dm_arm_acm_guard/        # 采摘阶段 ACM 控制
    ├── app/
    │   ├── dm_arm_interface/        # ROS Action/Service 接口层
    │   ├── dm_arm_task/             # 任务管理与采摘状态机
    │   └── dm_arm_gui/              # ROI 选点与任务下发 GUI
    ├── domain/
    │   └── trac_ik/                 # TRAC-IK
    └── infra/
        ├── serial/                  # 串口基础库
        ├── tl_expected/
        └── tl_optional/
```

---

## 5. 快速开始

### 5.1 环境选择

本工作区支持两种 ROS Noetic 使用方式：

| 场景 | 推荐方式 | 说明 |
|---|---|---|
| Ubuntu 20.04 | 原生 ROS Noetic | 使用系统 apt 安装 ROS Noetic 和依赖 |
| Ubuntu 22.04 或需要隔离依赖 | micromamba 虚拟环境 | 使用 `mamba-usb rosnoetic` 进入工作区约定的 ROS1 Noetic 环境 |

`dm-arm-start.sh` 和 `dm-arm-test.sh` 不会主动调用 `mamba-usb`；它们只检查当前环境是否为 Noetic，因此运行前需要先进入原生 Noetic shell 或虚拟环境

### 5.2 Ubuntu 20.04 原生 ROS Noetic

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-dev curl \
  ros-noetic-desktop-full \
  ros-noetic-moveit \
  ros-noetic-ros-control \
  ros-noetic-ros-controllers \
  ros-noetic-controller-manager \
  ros-noetic-joint-trajectory-controller \
  ros-noetic-joint-state-controller \
  ros-noetic-pcl-ros \
  ros-noetic-pcl-conversions \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-moveit-ros-perception

sudo usermod -aG dialout $USER
```

重新登录后进入工作区：

```bash
cd /path/to/dm-ws
source /opt/ros/noetic/setup.bash
```

### 5.3 Ubuntu 22.04 / micromamba 虚拟环境

```bash
cd /path/to/dm-ws
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
```

虚拟环境说明见：

```text
ros_env/README.md
```

### 5.4 编译

虚拟环境推荐编译方式：

```bash
cd /path/to/dm-ws
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
. ./ros_env/use-mamba-gcc.sh && catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

原生 ROS Noetic 编译方式：

```bash
cd /path/to/dm-ws
source /opt/ros/noetic/setup.bash
catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash
```

---

## 6. 启动系统

### 6.1 fake controller 链路

fake controller 不连接 `dm_hw`，用于先验证上层、MoveIt、Action/Service 和 GUI：

```bash
./dm-arm-start.sh --fake
```

不启动 RViz：

```bash
./dm-arm-start.sh --fake --no-rviz
```

手动启动：

```bash
roslaunch dm_arm_interface dm_arm_start.launch \
  use_fake_execution:=true \
  use_camera:=false \
  use_rviz:=true
```

### 6.2 真机链路

真机链路会启动 `dm_hw`、`joint_state_controller`、`arm_controller`、`end_controller` 和上层接口：

```bash
./dm-arm-start.sh
```

指定串口和波特率：

```bash
./dm-arm-start.sh --serial-port /dev/ttyUSB0 --baudrate 921600
```

指定硬件配置文件：

```bash
./dm-arm-start.sh --hardware-config /path/to/dm_controller.yaml
```

手动启动：

```bash
roslaunch dm_arm_interface dm_arm_start.launch \
  use_fake_execution:=false \
  use_camera:=false \
  use_rviz:=true \
  hardware_serial_port:=/dev/ttyUSB0 \
  hardware_baudrate:=921600
```

### 6.3 可选相机与感知链路

相机和点云感知默认不启动；需要时显式开启：

```bash
./dm-arm-start.sh --with-camera
```

fake controller 中也可以开启相机链路：

```bash
./dm-arm-start.sh --fake --with-camera
```

---

## 7. dm_hw 硬件配置

默认配置文件：

```text
src/platform/dm_hw/config/dm_controller.yaml
```

默认电机映射：

| ROS 关节 | 电机 ID | 控制器 |
|---|---:|---|
| `joint1` | `0x01` | `arm_controller` |
| `joint2` | `0x02` | `arm_controller` |
| `joint3` | `0x03` | `arm_controller` |
| `joint4` | `0x04` | `arm_controller` |
| `joint5` | `0x05` | `arm_controller` |
| `joint6` | `0x06` | `arm_controller` |
| `gripper_left` | `0x07` | `end_controller` |

关键参数：

- `dm_arm_hardware/serial_port`：USB-CAN 串口设备，启动参数可覆盖
- `dm_arm_hardware/baudrate`：串口波特率，启动参数可覆盖
- `dm_arm_hardware/control_frequency`：控制循环频率
- `dm_arm_hardware/use_mit_mode`：`true` 使用 MIT 控制；`false` 使用位置速度控制
- `dm_arm_hardware/enable_write`：真机写入开关
- `dm_arm_hardware/return_zero_on_shutdown`：节点退出时是否尝试回零
- `dm_arm_hardware/joints/motor_ids`：达妙电机 CAN ID
- `dm_arm_hardware/joints/motor_types`：`damiao::DmMotorType` 枚举值
- `dm_arm_hardware/joints/motor_to_joint_scale`：电机反馈到 ROS 关节单位的比例
- `dm_arm_hardware/joints/joint_to_motor_scale`：ROS 关节命令到电机单位的比例

`gripper_left` 是 URDF 中的 prismatic 关节，默认按丝杆导程 `0.053m/rev` 做单位换算。

---

## 8. 配置文件说明

常用配置文件总览：

| 文件 | 作用 | 常改参数 |
|---|---|---|
| `src/app/dm_arm_interface/config/config.yaml` | 上层接口、MoveIt group、EEF、规划参数 | action/service 名称、EEF group、规划时间、速度/加速度比例 |
| `src/platform/dm_hw/config/dm_controller.yaml` | 真机硬件接口和 ros_control controller | 串口、波特率、电机 ID、电机类型、回零、限速、夹爪换算 |
| `src/device/dm_arm_camera/config/dm_arm_orbbec.yaml` | Gemini335L 相机节点 | 分辨率、帧率、对齐、深度范围、手眼矩阵 |
| `src/service/dm_arm_perception/config/dm_arm_perception.yaml` | 点云生成和滤波 | 输入 topic、目标 frame、工作空间裁剪、降采样、SOR |
| `src/platform/dm_arm_moveit_config/config/*.yaml` | MoveIt 规划、控制器、限位、传感器配置 | controller 映射、joint limit、kinematics、Octomap 输入 |

### 8.1 `dm_arm_interface/config/config.yaml`

这个文件决定上层接口如何连接 MoveIt、EEF 和 ROS Action/Service：

```yaml
start:
  arm_group_name: "arm"
  eef:
    enabled: true
    type: "two_finger_gripper"
    name: "end"
```

关键参数：

- `start/arm_group_name`：机械臂 MoveIt group，必须和 SRDF 中的 `arm` 一致。
- `start/eef/enabled`：是否启用 EEF 控制接口。若只测试机械臂本体，可设为 `false`。
- `start/eef/type`：当前 DM-Arm 夹爪使用 `two_finger_gripper`，通过 MoveIt `end` group 规划执行。
- `start/eef/name`：EEF MoveIt group，当前必须为 `end`。不要改回 `two_finger_gripper`，SRDF 中没有这个 group。
- `start/*/name`：Action/Service 名称。一般保持默认，除非需要多机械臂或命名空间隔离。
- `decartes/eef_step`：笛卡尔路径插值步长，越小路径越细、规划越慢。
- `decartes/min_success_rate`：笛卡尔路径最小成功比例。
- `motion_planning/planning_time`：MoveIt 单次规划时间。
- `motion_planning/max_velocity_scaling_factor`：MoveIt 速度比例，真机首次测试建议降到 `0.1~0.3`。
- `motion_planning/max_acceleration_scaling_factor`：MoveIt 加速度比例，真机首次测试建议降到 `0.1~0.3`。
- `motion_planning/planner_id`：默认 `RRTConnect`，需要与 MoveIt OMPL 配置中的 planner 名称匹配。
- `reachable_pose_search/step_deg`：可达姿态搜索步长。
- `reachable_pose_search/radius_deg`：可达姿态搜索角度范围。

典型真机保守配置：

```yaml
motion_planning:
  planning_time: 5.0
  planning_attempts: 10
  max_velocity_scaling_factor: 0.2
  max_acceleration_scaling_factor: 0.2
  planner_id: "RRTConnect"
```

### 8.2 `dm_hw/config/dm_controller.yaml`

这个文件只在真机链路使用。fake controller 不启动 `dm_hw`，也不会读取这些硬件参数。

```yaml
dm_arm_hardware:
  serial_port: "/dev/ttyACM0"
  baudrate: 921600
  control_frequency: 500.0
```

关键参数：

- `serial_port`：默认串口。也可以启动时用 `./dm-arm-start.sh --serial-port /dev/ttyUSB0` 覆盖。
- `baudrate`：默认波特率。也可以启动时用 `./dm-arm-start.sh --baudrate 921600` 覆盖。
- `control_frequency`：`dm_hw` 控制循环频率。
- `use_mit_mode`：`false` 使用位置速度模式；`true` 使用 MIT 模式。
- `kp`、`kd`：MIT 模式增益。位置速度模式下不作为主控制增益。
- `max_position_change`：单周期最大位置命令变化，用于限制突变。
- `max_velocity`：发送给硬件的目标速度上限。
- `enable_write`：是否向电机写命令。调试读反馈时可设为 `false`。
- `enable_read_refresh`：是否每周期刷新电机状态。
- `return_zero_on_shutdown`：节点退出时先回零，再失能电机。

电机映射：

```yaml
joints:
  names: [joint1, joint2, joint3, joint4, joint5, joint6, gripper_left]
  motor_ids: [1, 2, 3, 4, 5, 6, 7]
  master_ids: [0, 0, 0, 0, 0, 0, 0]
  motor_types: [2, 2, 2, 0, 0, 0, 0]
```

配置要求：

- `names`、`motor_ids`、`master_ids`、`motor_types`、`motor_to_joint_scale`、`joint_to_motor_scale` 长度必须一致。
- `joint1..joint6` 对应 `arm_controller`。
- `gripper_left` 对应 `end_controller`，默认电机 ID 是 `0x07`。
- `motor_types` 必须按实际达妙电机型号调整，错误型号可能导致控制范围或协议解析不正确。
- `motor_to_joint_scale` 是电机反馈到 ROS 关节单位的比例。
- `joint_to_motor_scale` 是 ROS 关节命令到电机单位的比例。
- `gripper_left` 当前按 `0.053m/rev` 丝杆导程配置，若夹爪机械结构变更，需要同步调整两组 scale。

controller 配置：

- `joint_state_controller` 发布 `/joint_states`。
- `arm_controller` 接收 MoveIt 的 6 轴轨迹。
- `end_controller` 接收 MoveIt 的夹爪轨迹。

不要只在 MoveIt controller 配置里加关节而不改 `dm_controller.yaml`。真机链路里，某个关节必须同时存在于 `dm_hw` 的 `joints` 和对应的 ros_control controller 中。

### 8.3 `dm_arm_camera/config/dm_arm_orbbec.yaml`

这个文件只在启动 `--with-camera` 时使用：

```yaml
color_width: 1280
color_height: 720
color_fps: 30
depth_width: 1280
depth_height: 800
depth_fps: 30
```

关键参数：

- `color_width`、`color_height`、`color_fps`：彩色图分辨率和帧率。
- `depth_width`、`depth_height`、`depth_fps`：深度图分辨率和帧率。
- `align_mode`：深度对齐模式。当前默认 `sw`。
- `rotate_180`：相机安装倒置时可设为 `true`。
- `min_depth_m`、`max_depth_m`：相机节点发布前的深度有效范围。
- `enable_lrm`：是否发布 LRM 单点测距。
- `publish_rate`：相机节点发布频率上限。
- `color_frame_id`、`depth_frame_id`、`depth_registered_frame_id`：相机 TF frame 名称。
- `hand_eye/T_cam_to_flange`：相机到法兰的 4x4 手眼外参矩阵。
- `hand_eye/flange_id`：手眼矩阵挂载的法兰/末端 link，当前为 `link6-7`。

手眼标定更新后，通常只改：

```yaml
hand_eye:
  T_cam_to_flange:
    - [...]
    - [...]
    - [...]
    - [0.0, 0.0, 0.0, 1.0]
  flange_id: link6-7
```

### 8.4 `dm_arm_perception/config/dm_arm_perception.yaml`

这个文件控制点云输入、坐标转换和滤波：

```yaml
topics:
  color_image: /dm_arm/camera/orbbec/color/image_raw
  depth_image: /dm_arm/camera/orbbec/depth_registered/image_raw
  depth_info: /dm_arm/camera/orbbec/depth_registered/camera_info

target_frame: base_link
```

关键参数：

- `topics/color_image`：彩色图输入。
- `topics/depth_image`：对齐深度图输入，推荐使用 `depth_registered`。
- `topics/depth_info`：深度相机内参。
- `target_frame`：输出点云目标坐标系，通常为 `base_link`。
- `pixel_stride`：像素采样步长，越大点越少、CPU 越低。
- `frame_skip`：跳帧处理，越大负载越低、更新越慢。
- `min_depth`、`max_depth`：进入点云处理的深度范围。
- `min_x/max_x`、`min_y/max_y`、`min_z/max_z`：机械臂工作空间裁剪范围。
- `voxel_leaf`：VoxelGrid 降采样尺寸。
- `sor_mean_k`、`sor_stddev`：统计离群点滤波参数。
- `publish_raw`、`publish_base`、`publish_filtered`：是否发布各阶段点云。

真机调试建议：

- 先确认 `raw` 点云有数据，再看 `base` 点云 TF 是否正确。
- 工作空间裁剪先放宽，确认方向后再收窄。
- `voxel_leaf` 可从 `0.01~0.03` 之间调，数值越大越稀疏。

### 8.5 `dm_arm_moveit_config/config/*.yaml`

MoveIt 配置由多个文件组成，常见修改点如下：

| 文件 | 说明 | 修改建议 |
|---|---|---|
| `simple_moveit_controllers.yaml` | 真机 MoveIt controller 映射 | controller 名称必须与 `dm_hw/config/dm_controller.yaml` 中的 `arm_controller`、`end_controller` 一致 |
| `fake_controllers.yaml` | fake controller 映射和初始位姿 | fake 测试时使用，`initial` 中 `arm/zero`、`end/open` 必须存在于 SRDF |
| `joint_limits.yaml` | MoveIt 速度/加速度限制 | 真机首次测试建议降低关节 `max_velocity` 或全局 scaling |
| `kinematics.yaml` | IK 求解器配置 | 默认 KDL，可按需要换 TRAC-IK，但 group 名必须仍为 `arm` |
| `ompl_planning.yaml` | OMPL planner 配置 | 调规划算法和 projection evaluator 时使用 |
| `cartesian_limits.yaml` | Pilz/笛卡尔速度限制 | 需要限制直线运动速度时调整 |
| `sensors_3d.yaml` | MoveIt Octomap 点云输入 | 只有 `use_camera:=true` 时加载 3D sensor |
| `ros_controllers.yaml`、`gazebo_controllers.yaml` | ros_control/Gazebo 参考配置 | 当前主链路使用 `dm_hw/config/dm_controller.yaml` |

真机 controller 映射必须保持一致：

```yaml
controller_list:
  - name: arm_controller
    action_ns: follow_joint_trajectory
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
  - name: end_controller
    action_ns: follow_joint_trajectory
    joints: [gripper_left]
```

Octomap 输入默认来自：

```yaml
point_cloud_topic: /dm_arm/perception/cloud/filtered
filtered_cloud_topic: /dm_arm/moveit/filtered_cloud
```

如果不启动相机，`dm_arm_start.launch` 会把 `load_3d_sensors` 设为 `false`，MoveIt 不会加载 `sensors_3d.yaml`。

---

## 9. 交互式测试

启动控制链路后运行：

```bash
./dm-arm-test.sh
```

等待 action/service 更久：

```bash
./dm-arm-test.sh --wait 120
```

跳过运动命令二次确认：

```bash
./dm-arm-test.sh --yes
```

典型 fake controller 验证流程：

```bash
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
./dm-arm-start.sh --fake --no-rviz
```

另开终端：

```bash
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
./dm-arm-test.sh
```

---

## 10. ROS 接口总览

### Action

| 名称 | 类型 | 说明 |
|---|---|---|
| `/move_arm` | `dm_arm_msgs/MoveArmAction` | 完整机械臂命令接口 |
| `/simple_move_arm` | `dm_arm_msgs/SimpleMoveArmAction` | 简化机械臂命令接口 |
| `/pick_action` | `dm_arm_msgs/PickTaskAction` | 任务组与采摘任务接口 |

### Service

| 名称 | 类型 | 说明 |
|---|---|---|
| `/arm_config` | `dm_arm_msgs/ConfigArm` | 约束配置 |
| `/arm_query` | `dm_arm_msgs/QueryArm` | 当前关节/位姿查询 |
| `/eef_cmd` | `dm_arm_msgs/CommandEef` | 末端执行器命令 |
| `/dm_arm/perception/set_octomap_enabled` | `dm_arm_msgs/CommandOctomap` | Octomap 点云输入开关与异步清图 |

---

## 11. 常用验证命令

### 检查 ROS 与工作区环境

```bash
echo "$ROS_DISTRO"
rospack find dm_hw
rospack find dm_arm_interface
```

### 检查 controller

```bash
rosservice call /controller_manager/list_controllers
rostopic list | grep follow_joint_trajectory
```

### 查询机械臂状态

```bash
rosservice call /arm_query "command_type: 12
values: []"

rosservice call /arm_query "command_type: 13
values: []"
```

### 检查相机话题

```bash
rostopic list | grep /dm_arm/camera
rostopic hz /dm_arm/camera/orbbec/depth_registered/image_raw
```

### 检查点云

```bash
rostopic hz /dm_arm/perception/cloud/raw
rostopic hz /dm_arm/perception/cloud/base
rostopic hz /dm_arm/perception/cloud/filtered
```

### 控制 Octomap

```bash
rosservice call /dm_arm/perception/set_octomap_enabled "{enabled: false, clear_octomap: true}"
rosservice call /dm_arm/perception/set_octomap_enabled "{enabled: true, clear_octomap: false}"
```

---

## 12. 故障排查

### 当前环境不是 Noetic

```bash
echo "$ROS_DISTRO"
rosversion -d
```

虚拟环境：

```bash
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
```

原生环境：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 找不到 dm_arm 包

```bash
. ./ros_env/source-dm-arm.sh
rospack profile
rospack find dm_arm_interface
```

如果仍然找不到，先重新编译工作区

### 真机串口打不开

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
groups
```

检查：

- 当前用户是否在 `dialout` 组
- `--serial-port` 是否指定了正确设备
- 是否有其他进程占用串口
- 波特率是否与 USB-CAN 固件配置一致

### end controller 不可用

```bash
rosservice call /controller_manager/list_controllers
rostopic list | grep end_controller
```

检查：

- `dm_hw/config/dm_controller.yaml` 中是否包含 `gripper_left`
- `motor_ids` 是否包含 `7`
- `controller_spawner` 是否启动了 `end_controller`
- MoveIt 是否加载 `simple_moveit_controllers.yaml`

### 点云没有发布

```bash
rostopic list | grep perception
rostopic echo -n 1 /dm_arm/camera/orbbec/depth_registered/image_raw/header
```

检查：

- 启动时是否传入 `--with-camera`
- `depth_registered` 是否发布
- 深度图编码是否为 `32FC1`
- TF 是否可从相机 frame 转到 `base_link`

---

## 13. 文档

- [dm_hw 硬件接口](src/platform/dm_hw/README.md)
- [ROS Noetic 虚拟环境说明](ros_env/README.md)
- [MoveIt 配置](src/platform/dm_arm_moveit_config)
- [DM-Arm 接口层配置](src/app/dm_arm_interface/config/config.yaml)

---

## 14. 致谢

本工作区上层架构移植自 `piper-ws`，底层硬件链路替换为 `dm_hw` 达妙电机接口

同时感谢以下优秀开源项目：

- MoveIt: https://moveit.ros.org/
- ros_control: http://wiki.ros.org/ros_control
- trac-ik: https://github.com/HIRO-group/trac_ik
- serial: https://github.com/wjwwood/serial
- tl-optional: https://github.com/TartanLlama/optional
- tl-expected: https://github.com/TartanLlama/expected

---

## 15. 许可证

MIT License，详见 `LICENSE`

---

## 16. 贡献

欢迎提交 Issue / PR
