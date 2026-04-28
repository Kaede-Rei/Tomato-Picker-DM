# DM-Arm ROS Noetic Environment

```bash
mamba-usb rosnoetic

. ./ros_env/use-mamba-gcc.sh && catkin_make \
  -DCATKIN_ENABLE_TESTING=OFF \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source devel/setup.bash
```

启动全链路：

```bash
./dm-arm-start.sh
```

相机和点云感知默认不启动，需要时显式开启：

```bash
./dm-arm-start.sh --with-camera
```
