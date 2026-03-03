<div align="center">

# DM-Arm-ROS: 达妙六轴机械臂ROS控制系统

一个基于 ROS Noetic 和 MoveIt! 的达妙六轴机械臂控制框架，集成了从硬件接口、运动规划到高层服务的全流程控制系统，应用于广东省农科院樱桃番茄采摘机器人

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS: Noetic](https://img.shields.io/badge/ROS-Noetic-blue.svg)](http://wiki.ros.org/noetic)
[![Framework: MoveIt!](https://img.shields.io/badge/Framework-MoveIt!-green.svg)](https://moveit.ros.org/)
[![Hardware: DM Motor](https://img.shields.io/badge/Hardware-DM%20Motor-red.svg)](http://www.damiaotech.com/)

</div>

## 🤖 系统架构

### 硬件配置

| 组件           | 型号/要求                               | 说明                 |
| -------------- | --------------------------------------- | -------------------- |
| 机械臂         | 达妙 6-DOF 机械臂                        | 6 个达妙无刷电机驱动 |
| 电机型号       | DM4310 / DM4340                          | 支持 24V/48V 版本    |
| 夹爪           | DM4310                          | 单自由度抓取控制     |
| 电源           | 24V 工业开关电源/可调电源                | 推荐 >= 10A 输出     |
| USB-CAN 适配器 | 达妙官方 USB-CAN 盒子                    | CAN 总线通信         |
| 主控           | Ubuntu 20.04 工控机/计算机         | >= 4 核 CPU, >= 8GB RAM |
| 串口设备       | `/dev/ttyACM*`         | USB 转 CAN 通信端口  |

**硬件连接示意：**
```
[24V 电源] ──► [达妙转接板] ──► [机械臂底座]
                   │
              [USB-CAN 盒子]
                   │
            [Type-C to 工控机]
```

### 软件栈

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **运动规划**: MoveIt! + ros_control
- **硬件接口**: 自定义 `dm_ht_controller` (实现 `hardware_interface::RobotHW`)
- **控制器**: `joint_state_controller` + `joint_trajectory_controller`
- **规划器**: RRTConnect / CHOMP / STOMP / Pilz Industrial Motion Planner
- **业务服务**: `dm_arm_service` (提供末端位姿控制和任务规划服务)
- **通信协议**: CAN 总线 (达妙电机) + ROS 服务/话题

---

## 📋 快速开始

### 1. 环境准备

#### 系统依赖

```bash
# Ubuntu 20.04 系统
sudo apt update

# 安装 ROS Noetic（如果未安装）
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full

# 安装 MoveIt 和 ros_control 相关包
sudo apt install ros-noetic-moveit \
                 ros-noetic-controller-manager \
                 ros-noetic-joint-trajectory-controller \
                 ros-noetic-joint-state-controller \
                 ros-noetic-effort-controllers \
                 ros-noetic-position-controllers \
                 ros-noetic-rviz \
                 ros-noetic-ros-controllers \
                 ros-noetic-gazebo-ros-pkgs \
                 ros-noetic-gazebo-ros-control

# 配置串口权限
sudo usermod -aG dialout $USER
sudo reboot  # 重启使权限生效
```

#### ROS 环境

```bash
# 在 ~/.bashrc 中添加 ROS 环境配置
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 源码获取与编译

```bash
# 创建工作空间
mkdir -p ~/dm_arm_ws/src
cd ~/dm_arm_ws/src

# 克隆项目（替换为你的仓库地址）
git clone https://github.com/your-username/Tomato-Picker-DM.git .

# 初始化工作空间
cd ~/dm_arm_ws/src
catkin_init_workspace

# 编译
cd ~/dm_arm_ws
catkin_make

# 配置环境
source devel/setup.bash
echo "source ~/dm_arm_ws/devel/setup.bash" >> ~/.bashrc
```

### 3. 硬件连接

#### 电源连接

1. **机械臂供电**: 24V 电源 → 达妙转接板 → 机械臂底座（XT30(2+2) 端口）
2. **转接板供电**: 24V 电源 → 转接板 XT30 端口

#### 通信连接

1. **USB-CAN 盒子**: Type-C 线连接到工控机/计算机
2. **端口识别**:
   ```bash
   # 查看串口设备
   ls -l /dev/ttyACM*  # 达妙 USB-CAN 常为 /dev/ttyACM0
   ls -l /dev/ttyUSB*  # 其他串口设备
   
   # 查看设备详细信息
   dmesg | grep tty
   ```

#### 端口配置

编辑配置文件 [dm_arm_service/config/dm_arm_config.yaml](src/dm_arm_service/config/dm_arm_config.yaml)：

```yaml
dm_arm_hardware:
  serial_port: "/dev/ttyACM0"  # 根据实际连接修改
  baudrate: 921600
  control_frequency: 500.0
```

### 4. 基础测试

#### 检查硬件连接

```bash
# 测试串口通信
python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 921600); print('串口连接成功:', s)"
```

#### 启动系统

```bash
# 一键启动（包含硬件接口、MoveIt、RViz）
roslaunch dm_arm_service dm_arm_server.launch

# 可选参数：
# use_rviz:=false          # 不启动 RViz
# use_fake_execution:=true # 仅仿真模式（不连接真实硬件）
```

**启动成功标志：**
```
[INFO] [1765811909.715087159]: ====================================
[INFO] [1765811909.715137026]:    DM Arm 服务器已启动
[INFO] [1765811909.715158397]: ====================================
[INFO] [1765811909.715180956]: 可用的服务：
[INFO] [1765811909.715199953]:   * /dm_arm_server/eef_cmd
[INFO] [1765811909.715220137]:     └─ 末端位姿控制服务
[INFO] [1765811909.715239134]:   * /dm_arm_server/task_planner
[INFO] [1765811909.715258131]:     └─ 任务组规划服务
[INFO] [1765811909.715278315]: ====================================
```

#### 测试服务

```bash
# 运行测试脚本
cd ~/dm_arm_ws
source test_service.sh
```

---

## 📊 完整使用流程

### 阶段 1: RViz 手动控制

1. **启动系统**（如已启动则跳过）:
   ```bash
   roslaunch dm_arm_service dm_arm_server.launch
   ```

2. **RViz 界面操作**:
   - 左侧 **MotionPlanning** 面板
   - **Context** 标签页选择 `Planning Scene Topic` 为 `/move_group/monitored_planning_scene`
   - **Planning** 标签页:
     - **Planning Group**: 选择 `arm` 或 `end`
     - 拖动**交互标记**设置目标位姿
     - 或点击 **Random Valid** 生成随机目标
     - 点击 **Plan** 进行路径规划
     - 点击 **Execute** 执行运动

### 阶段 2: 服务端控制

#### 末端位姿控制服务

**服务名称**: `/dm_arm_server/eef_cmd`  
**服务类型**: `dm_arm_msgs_srvs::dm_arm_cmd`

##### 常用命令

| 命令          | 说明                             | 示例用法                                                      |
| ------------- | -------------------------------- | ------------------------------------------------------------- |
| `zero`        | 回到零点（初始位置）             | `rosservice call /dm_arm_server/eef_cmd "command: 'zero'"`     |
| `goal_base`   | 基座坐标系下设置末端目标位姿     | `command: 'goal_base', x: 0.3, y: 0.0, z: 0.5, roll/pitch/yaw: 0` |
| `goal_eef`    | 末端坐标系下设置相对目标位姿     | `command: 'goal_eef', x: 0.1 (末端沿x轴移动0.1米)`              |
| `stretch`     | 末端沿当前朝向伸缩               | `command: 'stretch', param1: '0.05' (伸出0.05米)`              |
| `rotate`      | 末端绕Z轴旋转                    | `command: 'rotate', param1: '90.0' (旋转90度)`                 |
| `get_pose`    | 获取当前末端位姿（基座坐标系）   | `command: 'get_pose'`                                          |
| `get_joints`  | 获取所有关节当前角度             | `command: 'get_joints'`                                        |

##### 示例：设置末端位姿

```bash
rosservice call /dm_arm_server/eef_cmd "command: 'goal_base'
x: 0.3
y: 0.0
z: 0.5
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"
```

##### 示例：获取当前位姿

```bash
rosservice call /dm_arm_server/eef_cmd "command: 'get_pose'
x: 0.0
y: 0.0
z: 0.0
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: ''
param2: ''
param3: ''"

# 响应示例：
# success: True
# message: "当前末端位姿获取成功"
# cur_x: 0.3
# cur_y: 0.0
# cur_z: 0.5
# cur_roll: 0.0
# cur_pitch: 0.0
# cur_yaw: 0.0
```

#### 任务组规划服务

**服务名称**: `/dm_arm_server/task_planner`  
**服务类型**: `dm_arm_msgs_srvs::dm_arm_cmd`

##### 任务规划命令

| 命令            | 说明                   | 参数说明                                                      |
| --------------- | ---------------------- | ------------------------------------------------------------- |
| `add_task`      | 添加任务到队列         | `x,y,z,roll,pitch,yaw`: 目标位姿<br>`param1`: 到达后等待时间(秒)<br>`param2`: 动作类型(NONE/PICK/STRETCH/ROTATE)<br>`param3`: 动作参数 |
| `clear_tasks`   | 清空任务队列           | 无需参数                                                      |
| `exe_all_tasks` | 按顺序执行所有任务     | 无需参数                                                      |

##### 动作类型说明

- `NONE`: 无动作
- `PICK`: 抓取动作
- `STRETCH`: 伸缩, `param3` 为长度(米)
- `ROTATE`: 旋转, `param3` 为角度(弧度)

##### 示例：任务序列控制

```bash
# 1. 清空任务队列
rosservice call /dm_arm_server/task_planner "command: 'clear_tasks'"

# 2. 添加任务1：移动到位置A并等待2秒
rosservice call /dm_arm_server/task_planner "command: 'add_task'
x: 0.3
y: 0.1
z: 0.4
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '2.0'
param2: 'NONE'
param3: ''"

# 3. 添加任务2：伸出0.05米
rosservice call /dm_arm_server/task_planner "command: 'add_task'
x: 0.3
y: 0.1
z: 0.4
roll: 0.0
pitch: 0.0
yaw: 0.0
param1: '1.0'
param2: 'STRETCH'
param3: '0.05'"

# 4. 执行所有任务
rosservice call /dm_arm_server/task_planner "command: 'exe_all_tasks'"
```

### 阶段 3: Python 客户端编程

#### 安装依赖

```bash
pip install rospy
```

#### 示例代码

```python
#!/usr/bin/env python3
import rospy
from dm_arm_msgs_srvs.srv import dm_arm_cmd, dm_arm_cmdRequest

def move_to_pose(x, y, z, roll=0, pitch=0, yaw=0):
    """移动机械臂末端到指定位姿"""
    rospy.wait_for_service('/dm_arm_server/eef_cmd')
    try:
        eef_cmd = rospy.ServiceProxy('/dm_arm_server/eef_cmd', dm_arm_cmd)
        req = dm_arm_cmdRequest()
        req.command = "goal_base"
        req.x = x
        req.y = y
        req.z = z
        req.roll = roll
        req.pitch = pitch
        req.yaw = yaw
        
        resp = eef_cmd(req)
        if resp.success:
            rospy.loginfo(f"成功移动到位姿: ({x}, {y}, {z})")
        else:
            rospy.logerr(f"移动失败: {resp.message}")
        return resp
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

def get_current_pose():
    """获取当前末端位姿"""
    rospy.wait_for_service('/dm_arm_server/eef_cmd')
    try:
        eef_cmd = rospy.ServiceProxy('/dm_arm_server/eef_cmd', dm_arm_cmd)
        req = dm_arm_cmdRequest()
        req.command = "get_pose"
        
        resp = eef_cmd(req)
        rospy.loginfo(f"当前位姿: x={resp.cur_x:.3f}, y={resp.cur_y:.3f}, z={resp.cur_z:.3f}")
        return resp
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {e}")

if __name__ == "__main__":
    rospy.init_node('dm_arm_client_example')
    
    # 回到零点
    move_to_pose(0, 0, 0)
    rospy.sleep(2)
    
    # 移动到目标位姿
    move_to_pose(0.3, 0.0, 0.5)
    rospy.sleep(2)
    
    # 获取当前位姿
    get_current_pose()
```

---

## 🛠️ 配置与定制

### 参数配置文件

**文件路径**: [src/dm_arm_service/config/dm_arm_config.yaml](src/dm_arm_service/config/dm_arm_config.yaml)

#### 硬件接口配置

```yaml
dm_arm_hardware:
  serial_port: "/dev/ttyACM0"       # 串口设备路径
  baudrate: 921600                  # 波特率
  control_frequency: 500.0          # 控制频率 (200~500 Hz)
  
  # 控制模式
  use_mit_mode: false               # MIT 模式（位置+速度+力矩控制）
  kp: 30.0                          # MIT 模式比例增益
  kd: 1.0                           # MIT 模式微分增益
  
  # 安全参数
  enable_write: true                # 允许写入命令
  max_position_change: 0.5          # 单次最大位置变化 (rad)
  max_velocity: 5.0                 # 最大速度 (rad/s)
```

#### 关节配置

```yaml
joints:
  names:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
    - gripper_left
  
  motor_ids: [1, 2, 3, 4, 5, 6, 7]     # 电机 CAN ID
  motor_types: [2, 2, 2, 0, 0, 0, 0]   # 电机类型（0:DM4310, 2:DM4340）
  
  # 关节限位
  limits:
    joint1: {min: -3.14, max: 3.14}
    joint2: {min: -1.57, max: 1.57}
    joint3: {min: -1.57, max: 1.57}
    joint4: {min: -3.14, max: 3.14}
    joint5: {min: -1.57, max: 1.57}
    joint6: {min: -3.14, max: 3.14}
```

#### 末端执行器配置

```yaml
end_effector:
  max_reach: 0.6                    # 最大工作半径 (米)
  min_reach: 0.1                    # 最小工作半径 (米)
  
  # 运动参数
  velocity_scaling: 0.3             # 速度缩放因子 (0.1~1.0)
  acceleration_scaling: 0.3         # 加速度缩放因子 (0.1~1.0)
  
  # 容差
  goal_position_tolerance: 0.015    # 位置目标容差 (米)
  goal_orientation_tolerance: 0.05  # 姿态目标容差 (弧度)
```

#### MoveIt 配置

```yaml
moveit:
  planning_group: "arm"             # 规划组名称
  planner_id: "RRTConnect"          # 规划器（RRTConnect/RRTstar/CHOMP/STOMP）
  planning_time: 5.0                # 规划时间限制 (秒)
  num_planning_attempts: 10         # 规划尝试次数
  
  execution_timeout: 15.0           # 执行超时 (秒)
  allow_replanning: true            # 允许重新规划
```

### 电机类型代码

| 代码 | 型号         | 电压 |
| ---- | ------------ | ---- |
| 0    | DM4310       | 24V  |
| 1    | DM4310_48V   | 48V  |
| 2    | DM4340       | 24V  |
| 3    | DM4340_48V   | 48V  |

---

## 📁 项目结构

```
Tomato-Picker-DM/
├── src/
│   ├── dm_arm_controller/                  # 机械臂控制节点
│   │   ├── include/                        # 头文件
│   │   ├── src/
│   │   │   └── eef_cmd.cpp                 # 末端执行器命令处理
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dm_arm_description/                 # 机械臂 URDF 模型
│   │   ├── meshes/                         # 3D 模型文件
│   │   │   ├── collision/                  # 碰撞检测模型
│   │   │   └── visual/                     # 可视化模型
│   │   ├── urdf/
│   │   │   └── DM-Arm-Description.urdf     # URDF 描述文件
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dm_arm_moveit_config/               # MoveIt 配置包
│   │   ├── config/                         # 配置文件
│   │   │   ├── DM-Arm-Description.srdf     # 语义机器人描述
│   │   │   ├── joint_limits.yaml           # 关节限位
│   │   │   ├── kinematics.yaml             # 运动学配置
│   │   │   ├── ompl_planning.yaml          # OMPL 规划器配置
│   │   │   ├── chomp_planning.yaml         # CHOMP 规划器配置
│   │   │   ├── stomp_planning.yaml         # STOMP 规划器配置
│   │   │   ├── ros_controllers.yaml        # ROS 控制器配置
│   │   │   └── ...
│   │   ├── launch/                         # 启动文件
│   │   │   ├── demo.launch                 # 演示启动
│   │   │   ├── demo_gazebo.launch          # Gazebo 仿真启动
│   │   │   ├── move_group.launch           # MoveIt 核心启动
│   │   │   └── ...
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dm_arm_msgs_srvs/                   # 自定义消息和服务
│   │   ├── srv/
│   │   │   └── dm_arm_cmd.srv              # 机械臂控制服务定义
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dm_arm_service/                     # 业务服务节点
│   │   ├── config/
│   │   │   └── dm_arm_config.yaml          # 主配置文件
│   │   ├── include/                        # 头文件
│   │   ├── launch/
│   │   │   └── dm_arm_server.launch        # 主启动文件
│   │   ├── src/
│   │   │   └── dm_arm_server.cpp           # 服务器主程序
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── dm_ht_controller/                   # 硬件接口节点
│   │   ├── config/
│   │   │   └── dm_controller.yaml          # 控制器配置
│   │   ├── include/                        # 头文件
│   │   ├── launch/                         # 启动文件
│   │   ├── src/                            # 源代码
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── serial_driver/                      # 串口和 CAN 驱动
│       ├── include/                        # 头文件
│       ├── src/                            # 源代码
│       ├── CMakeLists.txt
│       └── package.xml
│
├── README.md                               # 本文档
├── LICENSE                                 # MIT 许可证
├── 达妙机械臂服务器客户端接口文档.md       # API 接口详细文档
└── test_service.sh                         # 服务测试脚本
```

---

## 🛠️ 工具脚本快速参考

### 启动脚本

| 脚本                 | 说明                           | 命令                                                          |
| -------------------- | ------------------------------ | ------------------------------------------------------------- |
| 完整系统启动         | 启动硬件接口、MoveIt、RViz     | `roslaunch dm_arm_service dm_arm_server.launch`               |
| 仿真模式启动         | 不连接真实硬件，仅仿真         | `roslaunch dm_arm_service dm_arm_server.launch use_fake_execution:=true` |
| 无 RViz 启动         | 不启动 RViz 界面               | `roslaunch dm_arm_service dm_arm_server.launch use_rviz:=false` |
| Gazebo 仿真          | 在 Gazebo 中仿真               | `roslaunch dm_arm_moveit_config demo_gazebo.launch`          |
| MoveIt 演示          | MoveIt 演示模式（假执行）      | `roslaunch dm_arm_moveit_config demo.launch`                 |

### 测试脚本

| 脚本                 | 说明                           | 命令                                                          |
| -------------------- | ------------------------------ | ------------------------------------------------------------- |
| 服务测试             | 测试所有服务接口               | `source test_service.sh`                                      |
| 串口通信测试         | 测试串口连接                   | `python3 -c "import serial; s=serial.Serial('/dev/ttyACM0', 921600); print(s)"` |

---

## 📚 高级用法

### 自定义规划器

编辑 [src/dm_arm_moveit_config/config/ompl_planning.yaml](src/dm_arm_moveit_config/config/ompl_planning.yaml) 配置不同的 OMPL 规划器：

```yaml
planner_configs:
  RRTConnect:
    type: geometric::RRTConnect
    range: 0.0  # 默认使用自动范围
  RRTstar:
    type: geometric::RRTstar
    range: 0.0
    goal_bias: 0.05
  # ... 更多规划器配置
```

在服务或代码中使用时指定规划器：
```cpp
move_group.setPlannerId("RRTstar");
```

### 使用 CHOMP 规划器

CHOMP (Covariant Hamiltonian Optimization for Motion Planning) 适合需要平滑轨迹的场景：

```bash
roslaunch dm_arm_moveit_config move_group.launch \
  pipeline:=chomp
```

### 使用 STOMP 规划器

STOMP (Stochastic Trajectory Optimization for Motion Planning) 适合处理约束复杂的场景：

```bash
roslaunch dm_arm_moveit_config move_group.launch \
  pipeline:=stomp
```

### Gazebo 仿真集成

启动 Gazebo 仿真环境：

```bash
roslaunch dm_arm_moveit_config demo_gazebo.launch
```

这将启动：
- Gazebo 物理仿真器
- 机械臂 URDF 模型
- ros_control Gazebo 插件
- MoveIt 规划和执行

### 添加碰撞检测对象

在 RViz 的 **MotionPlanning** 面板中：
1. 选择 **Scene Objects** 标签页
2. 点击 **Import From Text** 或 **Publish Current Scene**
3. 添加障碍物（Box、Sphere、Cylinder 等）
4. MoveIt 会自动避开这些障碍物进行规划

### 夹爪控制

夹爪作为第 7 个关节，可以通过以下方式控制：

#### ROS 话题控制

```bash
# 发布夹爪位置命令（0.0 = 完全张开, 1.0 = 完全闭合）
rostopic pub /dm_ht_controller/joint_trajectory_controller/command trajectory_msgs/JointTrajectory "
header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['gripper_left']
points:
- positions: [0.5]
  time_from_start: {secs: 1, nsecs: 0}"
```

#### Python 代码控制

```python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def control_gripper(position, duration=1.0):
    """
    控制夹爪
    :param position: 夹爪位置 (0.0~1.0)
    :param duration: 执行时间 (秒)
    """
    pub = rospy.Publisher(
        '/dm_ht_controller/joint_trajectory_controller/command',
        JointTrajectory,
        queue_size=10
    )
    rospy.sleep(0.5)  # 等待发布者初始化
    
    traj = JointTrajectory()
    traj.joint_names = ['gripper_left']
    
    point = JointTrajectoryPoint()
    point.positions = [position]
    point.time_from_start = rospy.Duration(duration)
    
    traj.points.append(point)
    pub.publish(traj)
    rospy.loginfo(f"夹爪移动到位置: {position}")

# 使用示例
rospy.init_node('gripper_control_example')
control_gripper(0.0)  # 完全张开
rospy.sleep(2)
control_gripper(1.0)  # 完全闭合
```

---

## 🔧 故障排查

### 串口连接问题

#### 问题：找不到串口设备

```bash
# 诊断所有 USB 设备
lsusb

# 查看串口设备
ls -l /dev/ttyACM*

# 查看设备权限
ls -l /dev/ttyACM0

# 查看内核消息
dmesg | grep tty
```

**解决方案**：
1. 确认用户在 `dialout` 组：
   ```bash
   groups $USER  # 查看当前用户组
   sudo usermod -aG dialout $USER  # 添加到 dialout 组
   sudo reboot  # 重启生效
   ```

2. 检查 USB 连接和驱动：
   ```bash
   # 查看 USB 设备详细信息
   lsusb -v | grep -A 10 "达妙\|DM\|CAN"
   ```

#### 问题：串口权限不足

**错误信息**：`PermissionError: [Errno 13] Permission denied: '/dev/ttyACM0'`

**临时解决方案**：
```bash
sudo chmod 666 /dev/ttyACM0
```

**永久解决方案**：
```bash
sudo usermod -aG dialout $USER
sudo reboot
```

#### 问题：串口被其他程序占用

```bash
# 查看占用串口的进程
lsof /dev/ttyACM0

# 终止占用进程
kill -9 <PID>
```

### 硬件接口问题

#### 问题：电机无响应或错误

**诊断步骤**：
1. 检查电源连接（24V 电源）
2. 确认 CAN 总线连接正常
3. 验证电机 ID 配置与实际一致
4. 检查电机类型配置

**测试电机通信**：
```bash
# 查看日志
rostopic echo /dm_ht_controller/diagnostics
```

#### 问题：控制频率不稳定

编辑配置文件，降低控制频率：
```yaml
control_frequency: 200.0  # 从 500 降低到 200
```

#### 问题：关节超限或突然停止

检查关节限位配置 [src/dm_arm_moveit_config/config/joint_limits.yaml](src/dm_arm_moveit_config/config/joint_limits.yaml)：
```yaml
joint_limits:
  joint1:
    max_velocity: 5.0      # 降低最大速度
    max_acceleration: 3.0  # 降低最大加速度
```

### MoveIt 规划问题

#### 问题：规划失败或耗时过长

**解决方案**：
1. 增加规划时间：
   ```yaml
   planning_time: 10.0  # 从 5.0 增加到 10.0
   ```

2. 减少规划尝试次数：
   ```yaml
   num_planning_attempts: 5  # 从 10 减少到 5
   ```

3. 更换规划器：
   ```bash
   # 使用 RRTConnect（快速但不保证最优）
   move_group.setPlannerId("RRTConnect")
   
   # 使用 RRTstar（较慢但路径更优）
   move_group.setPlannerId("RRTstar")
   ```

#### 问题：末端无法到达目标位姿

**常见原因**：
- 目标位姿超出工作空间
- 目标位姿存在奇异点
- 碰撞检测阻止到达

**解决方案**：
1. 检查工作空间限制
2. 使用 `get_pose` 命令查询当前可达位姿
3. 在 RViz 中调整碰撞检测敏感度

#### 问题：执行轨迹与规划不一致

**可能原因**：
- 通信延迟
- 控制器参数不匹配
- 硬件响应延迟

**解决方案**：
调整控制器参数 [src/dm_ht_controller/config/dm_controller.yaml](src/dm_ht_controller/config/dm_controller.yaml)：
```yaml
joint_trajectory_controller:
  stopped_velocity_tolerance: 0.05  # 提高容差
  goal_time: 0.5  # 延长目标到达时间
```

### RViz 问题

#### 问题：RViz 无法启动或崩溃

**解决方案**：
1. 重新安装 RViz：
   ```bash
   sudo apt remove ros-noetic-rviz
   sudo apt install ros-noetic-rviz
   ```

2. 清除 RViz 配置：
   ```bash
   rm -rf ~/.rviz
   ```

3. 检查 OpenGL 支持：
   ```bash
   glxinfo | grep OpenGL
   ```

#### 问题：MotionPlanning 插件不显示

**解决方案**：
1. 确认 MoveIt 插件已安装：
   ```bash
   rospack find moveit_ros_visualization
   ```

2. 在 RViz 中手动添加插件：
   - 点击 **Panels** → **Add New Panel**
   - 选择 **MotionPlanning**

#### 问题：机械臂模型不显示

**解决方案**：
1. 检查 URDF 是否加载到参数服务器：
   ```bash
   rosparam get /robot_description
   ```

2. 在 RViz 中添加 **RobotModel** 显示：
   - 点击 **Add** → **RobotModel**
   - 设置 **Robot Description** 为 `/robot_description`

### 日志与调试

#### 启用详细日志

编辑 `~/.ros/config/rosconsole.config`:
```
log4j.logger.ros=DEBUG
log4j.logger.ros.moveit=DEBUG
```

或使用环境变量：
```bash
export ROSCONSOLE_CONFIG_FILE=~/my_rosconsole.config
```

#### 查看实时日志

```bash
# 查看所有日志
rostopic echo /rosout

# 查看 MoveIt 日志
rostopic echo /move_group/result

# 查看控制器状态
rostopic echo /dm_ht_controller/joint_trajectory_controller/state
```

#### 记录 rosbag

```bash
# 记录所有话题
rosbag record -a

# 记录特定话题
rosbag record /joint_states /tf /dm_arm_server/feedback

# 回放 rosbag
rosbag play your_bag.bag
```

#### 使用 rqt 工具

```bash
# 启动 rqt 图形界面
rqt

# 常用插件：
# - rqt_graph: 查看节点和话题连接图
# - rqt_plot: 实时绘制话题数据
# - rqt_console: 查看日志消息
# - rqt_reconfigure: 动态调整参数
```

---

## 📖 参考文献与资源

### 官方文档

- **ROS Noetic**: [http://wiki.ros.org/noetic](http://wiki.ros.org/noetic)
- **MoveIt!**: [https://moveit.ros.org/](https://moveit.ros.org/)
- **ros_control**: [http://wiki.ros.org/ros_control](http://wiki.ros.org/ros_control)
- **OMPL**: [https://ompl.kavrakilab.org/](https://ompl.kavrakilab.org/)

### 硬件相关

- **达妙科技官网**: [https://www.damiaokeji.com/](https://www.damiaokeji.com/)
- **达妙电机开源仓库**: [https://gitee.com/kit-miao/damiao/tree/master](https://gitee.com/kit-miao/damiao/tree/master)

---

## 📝 许可证

本项目采用 **MIT License** 开源，详见 [LICENSE](LICENSE) 文件

---

## 👥 贡献与

欢迎提交 Issues 和 Pull Requests！

---

## 📬 联系方式

如有问题或建议，请通过以下方式联系：
- GitHub Issues
- 提交讨论 (GitHub Discussions)
---

## 🙏 致谢

感谢以下项目和社区的支持：

- [ROS](http://www.ros.org/) 开源机器人操作系统
- [MoveIt!](https://moveit.ros.org/) 运动规划框架
- [达妙科技](http://www.damiaotech.com/) 提供优秀的无刷电机产品
- 所有贡献者和用户的反馈与支持

