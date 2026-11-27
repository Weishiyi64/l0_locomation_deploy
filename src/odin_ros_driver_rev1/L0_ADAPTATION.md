# L0 Odin v3.7 适配

本分支使用厂商 ROS Driver 3.7.0 和 SDK 3.6.0。发布来源见 [COMMIT_INFO.txt](COMMIT_INFO.txt)。
单 `camera0` 需求和验收项见 [升级恢复清单](docs/l0_optimization_upgrade_checklist.md)。

独立运行使用 `ros2 launch odin_ros_driver_rev1 driver.launch.py`。
整机传入 `post_process:=true undistort_mapping_only:=true`。手动去畸变使用
`post_process:=true send_undistort:=true`。默认设备前缀为 `/manifold/device0/`。
`enable_image1`、`send_image2_raw` 和 `send_rectify` 为只读兼容字段。

## 板端构建和检查

在机器人 Linux／ROS2 Humble 工作区中只放一份此包源码。使用新 SDK 源码编译，
同步更新对应的 OpenSSL 静态库与头文件。以 aarch64 工作区为例：

```bash
source /opt/ros/humble/setup.bash
cmake -S src/odin_ros_driver_rev1/module/sdk_api -B build/odin_sdk_v37 \
  -DCMAKE_BUILD_TYPE=Release -DODIN_BUILD_SAMPLES=OFF
cmake --build build/odin_sdk_v37 -j2
colcon build --symlink-install --cmake-force-configure \
  --packages-up-to odin_ros_driver_rev1
source install/setup.bash
```

优先核对实际设备广播型号。若发现日志拒绝设备，先确认固件和厂商设备实现支持，再继续整机升级。Odin2Special 需要 `libodin-device-Odin2Special.so`，
ROS2 安装规则将其与驱动一同安装，L0 launch 将安装目录提供给 SDK 的设备插件搜索。
已设置 `ODIN_DEVICE_LIB_PATH` 的系统使用该路径。回环算法插件由机器人环境提供，
核对 `libmanifold_lc_sdk.so.2` 或配置的 `ODIN_ALGO_LIB_PATH` 与依赖、ABI 和日志。
Normal、SLAM、Relocation、ReMap 和地图保存／加载均在板端验证。

运行新版本前停止旧驱动。回退时一起恢复上一版驱动、对应的部署接线和配套构建结果。
