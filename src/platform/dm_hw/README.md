# dm_hw

`dm_hw` 是当前 `dm-ws` 的 ROS1 Noetic 硬件接口包，底层复用新的达妙串口/USB-CAN C++ 驱动，上层向 `ros_control` 暴露 `PositionJointInterface`。

## 组成

- `include/dm_hw/damiao.hpp`：达妙电机协议与 `MotorControl`
- `include/dm_hw/serial_port.hpp`：USB-CAN 串口封装
- `include/dm_hw/dm_hardware_interface.hpp`：ROS1 `hardware_interface::RobotHW`
- `src/dm_controller.cpp`：`controller_manager` 控制循环节点
- `config/dm_controller.yaml`：串口、电机 ID、控制器配置
- `launch/dm_controller.launch`：真机硬件接口启动入口

## 编译

```bash
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
. ./ros_env/use-mamba-gcc.sh && catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## 真机启动

通常从总启动文件进入：

```bash
roslaunch dm_arm_interface dm_arm_start.launch use_fake_execution:=false
```

也可以只启动硬件和 `ros_control`：

```bash
roslaunch dm_hw dm_controller.launch
```

默认串口为 `/dev/ttyACM0`，默认波特率为 `921600`。根据真机连接修改 `config/dm_controller.yaml` 中的 `dm_arm_hardware`。

## 关键参数

- `dm_arm_hardware/serial_port`：USB-CAN 串口设备
- `dm_arm_hardware/baudrate`：串口波特率
- `dm_arm_hardware/control_frequency`：控制循环频率
- `dm_arm_hardware/use_mit_mode`：`true` 使用 MIT 控制；`false` 使用位置速度控制
- `dm_arm_hardware/enable_write`：真机写入开关
- `dm_arm_hardware/joints/names`：ROS 关节名
- `dm_arm_hardware/joints/motor_ids`：达妙电机 CAN ID
- `dm_arm_hardware/joints/motor_types`：`damiao::DmMotorType` 枚举值

fake controller 不会启动 `dm_hw`，用于上层和 MoveIt 链路验证。
