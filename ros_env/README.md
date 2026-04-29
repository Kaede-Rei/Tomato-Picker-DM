# DM-Arm ROS Noetic Environment

编译使用当前工作区的 ROS1 Noetic 虚拟环境：

```bash
mamba-usb rosnoetic
. ./ros_env/source-dm-arm.sh
. ./ros_env/use-mamba-gcc.sh && catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

fake controller：

```bash
roslaunch dm_arm_interface dm_arm_start.launch \
  use_fake_execution:=true \
  use_camera:=false \
  use_rviz:=true
```

或使用脚本：

```bash
./dm-arm-start.sh --fake
```

真机链路会启动 `dm_hw` 的 ROS1 `ros_control` 硬件接口：

```bash
roslaunch dm_arm_interface dm_arm_start.launch \
  use_fake_execution:=false \
  hardware_config_file:=$(rospack find dm_hw)/config/dm_controller.yaml
```

相机和点云感知默认不启动，需要时显式开启：

```bash
./dm-arm-start.sh --fake --with-camera
```
