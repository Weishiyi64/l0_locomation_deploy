# 整条路径统一的时间戳和坐标系。
# header.frame_id：device0/odom。
# header.stamp：本次路径参考生成时间，与运控使用同一 ROS 时钟。
std_msgs/Header header

# 按执行顺序排列的剩余路径点，单位：米。
# 每个点包含 x、y、z；z 是高度，不是控制参数。
# 已走过的部分由导航裁剪，最后一个点为最终目标位置。
# 不包含逐点 header 和 orientation。
geometry_msgs/Point[] points
