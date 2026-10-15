# atom_ros2_deploy_baseline

## Todo List
* legged_real移植
* LeggedHardwareInterface中硬件接口底层实现
* 电机标定功能实现
* 接口延迟、系统稳定性测试


## ros1 ros2共存、切换 bash配置
```bash
 # ===== ROS environment switchers (ROS1 Noetic + ROS2 Foxy) =====
 export ROS1_SETUP=/opt/ros/noetic/setup.bash
 export ROS2_SETUP=/opt/ros/foxy/setup.bash

 # Unset ROS variables to avoid cross-contamination
 _ros_clean() {
   unset ROS_VERSION ROS_DISTRO ROS_PYTHON_VERSION
   unset ROS_MASTER_URI ROS_ROOT ROS_PACKAGE_PATH ROSLISP_PACKAGE_DIRECTORIES
   unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH
   unset CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
 }

 # Activate ROS1 (Noetic)
 set_ros1() {
   _ros_clean
   source "$ROS1_SETUP"
   echo "[OK] ROS1 activated: $ROS_DISTRO"
 }

 # Activate ROS2 (Foxy)
 set_ros2() {
   _ros_clean
   source "$ROS2_SETUP"
   echo "[OK] ROS2 activated: $ROS_DISTRO"
 }

 # Show current ROS env
 rosenv() {
   echo "ROS_DISTRO=$ROS_DISTRO"
   echo "ROS_VERSION=$ROS_VERSION"
   which roscore 2>/dev/null | sed 's/^/roscore: /' || true
   which ros2     2>/dev/null | sed 's/^/ros2: /' || true
 }
 # =============================================================
```



## ROS2北通手柄映射
```shell
axes
[0] - 左摇杆 左右
[1] - 左摇杆 上下
[2] - 右摇杆 左右
[3] - 右摇杆 上下
[4] - RT
[5] - LT
[6] - 十字键 左右
[7] - 十字键 上下
buttons
[0] - A
[1] - B
[2] -
[3] - X
[4] - Y
[5] -
[6] - LB
[7] - RB
[8] - LT
[9] - RT
[10] - back
[11] - start
[12] - home
[13] - 左摇杆 按下
[14] - 右摇杆 按下
```
雷达功能启动
最简日常启动顺序
以后设备已经安装、编译并配置好 IP 后，只需要：
# 终端0
cd ~/zy_workspace/luvbot_deploy_ros2
source install/setup.bash
ros2 launch legged_controllers sim2real.launch.py

# 终端1
cd ~/zy_workspace/luvbot_deploy_ros2
source install/setup.bash
ros2 launch odin_ros_driver_rev1 driver.launch.py \
start_rviz:=false publish_tf_map:=false

# 终端2
cd ~/zy_workspace/luvbot_deploy_ros2
source install/setup.bash
python3 src/atom_ros2_deploy_baseline/legged_controllers/scripts/odin_goal_local_converter.py

# 终端3
cd ~/zy_workspace/luvbot_deploy_ros2
source install/setup.bash
ros2 topic pub --once   /goal_pose   geometry_msgs/msg/PoseStamped   "{header: {frame_id: 'device0/odom'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}}}".
# 根据自己需求改数
