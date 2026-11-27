import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


packageName = 'odin_ros_driver_rev1'


# ---------------------------------------------------------------------------
# Helpers / 辅助函数
# ---------------------------------------------------------------------------
def _arg(name, default, description=''):
    """Shorthand for DeclareLaunchArgument. / 简写"""
    return DeclareLaunchArgument(name, default_value=str(default), description=description)


def _pv(name, value_type):
    """Wrap a LaunchConfiguration as a typed ROS parameter value.

    LaunchConfiguration values are always strings; ParameterValue with an
    explicit value_type forces rclcpp to receive the correct primitive type.
    将 LaunchConfiguration（始终为字符串）按指定类型转成 ROS 参数值。
    """
    return ParameterValue(LaunchConfiguration(name), value_type=value_type)


# ---------------------------------------------------------------------------
# Naming policy / 命名策略
# ---------------------------------------------------------------------------
# Launch argument names are kept identical to the node parameter names. Almost
# all parameters are unique to a single node, so there is no ambiguity. The
# only parameter that exists on more than one node is `sync_queue_size`
# (pointcloud_reprojector_node + pointcloud_to_depth_node); we declare it
# once and feed both nodes from that single arg. Clients who need different
# values per node should override at the node level via `ros2 param set
# /<full_node_name> sync_queue_size <v>` or `--ros-args -r __node:=<name>
# -p sync_queue_size:=<v>`.
#
# launch arg 名称与节点参数名完全一致，绝大多数参数仅存在于单个节点，无歧义。
# 唯一跨节点同名的参数是 `sync_queue_size`（重投影节点 + 点云转深度图节点），
# 此处只声明一次，同时下发到两个节点。若客户希望分别设置，请走全路径，例如
# `ros2 param set /<完整节点名> sync_queue_size <值>`。


def _reject_unsupported_camera_options(context):
    for name in ('enable_image1', 'send_image2_raw', 'send_rectify'):
        if IfCondition(LaunchConfiguration(name)).evaluate(context):
            raise RuntimeError(f'{name} must be false on the single-camera L0 driver')
    return []


def generate_launch_description():
    pkg_share = get_package_share_directory(packageName)
    default_rviz = os.path.join(pkg_share, 'rviz', 'odin1lite_driver_view_ros2.rviz')
    device_plugin_dir = os.path.join(get_package_prefix(packageName), 'lib', packageName)

    # -----------------------------------------------------------------------
    # Launch arguments / 启动参数声明
    # -----------------------------------------------------------------------
    args = [
        # --- General / 通用 -------------------------------------------------
        _arg('start_rviz', 'false', 'Whether to start RViz / 是否启动 RViz'),
        _arg('rviz_config', default_rviz, 'Path to RViz config file / RViz 配置文件路径'),
        _arg('topic_prefix', 'manifold',
             'Topic prefix root, final prefix is /{topic_prefix}/device{N}/ '
             '/ 话题前缀根，最终前缀 /{topic_prefix}/device{N}/'),

        # --- Main driver: device / 主驱动设备 ------------------------------
        _arg('device_mode', 'normal',
             'device mode: normal | standby / 工作模式'),
        _arg('discovery_timeout_ms', '2000',
             'SDK device-discovery timeout (ms) / SDK 设备发现超时 (ms)'),

        # --- Main driver: image stream / 图像流 ----------------------------
        _arg('image_width', '1280',
             'Image width, 0 = device default / 图像宽度，0=设备默认'),
        _arg('image_height', '1088',
             'Image height, 0 = device default / 图像高度，0=设备默认'),
        _arg('image_fps', '30',
             'Image FPS, 0 = device default / 图像帧率，0=设备默认'),
        _arg('image_format', 'mjpeg',
             'Image format: mjpeg|h264|nv12 / 图像格式'),

        # --- Main driver: device-side filter parameters / 设备端滤波器参数 ----
        _arg('filter_confidence_threshold', '-1',
             'Device confidence filter threshold [0..255], -1=device default / 设备置信度滤波阈值，-1=设备默认'),
        _arg('filter_adaptive_cdf_percentile', '-1',
             'Adaptive filter CDF percentile [1..10000] (0.01%), -1=device default / 自适应滤波 CDF 百分位，-1=设备默认'),
        _arg('filter_adaptive_outdoor_thr', '-1',
             'Adaptive filter outdoor threshold (mm), -1=device default / 室外判定距离阈值，-1=设备默认'),
        _arg('filter_range_min_depth', '-1',
             'Range filter min valid depth (mm), -1=device default / 距离滤波最小有效深度，-1=设备默认'),

        # --- Main driver: frame ids / 坐标系名 -----------------------------
        _arg('raw_frame_id', 'lidar',
             'Frame id for raw cloud / 原始点云 frame_id'),
        _arg('slam_frame_id', 'odom',
             'Frame id for SLAM cloud / SLAM 点云 frame_id'),
        _arg('image_frame_id', 'camera0',
             'Frame id for left image / 左相机图像 frame_id'),
        _arg('imu_frame_id', 'imu',
             'Frame id for IMU / IMU frame_id'),
        _arg('odom_frame_id', 'odom',
             'Odometry parent frame / 里程计父帧'),

        # --- Main driver: TF republish / TF 重发 ---------------------------
        _arg('tf_publish_rate', '0',
             'TF republish rate (Hz); 0 = disable timer republish / TF 重发频率；0=禁用定时重发'),

        # --- Main driver: runtime knobs / 运行期可调旋钮（初始值） ----------
        _arg('confidence_threshold', '7',
             'Point-cloud confidence threshold (0..255) / 点云置信度阈值'),
        _arg('enable_row_filter', 'false',
             'Enable row filter: zero out points below min_row / 启用行滤波：置零 min_row 以下的点'),
        _arg('min_row', '0',
             'Minimum row index (0..191), points with row < min_row are zeroed / 最小行号（0..191），低于此行的点置零'),
        _arg('sensor_mode', '-1',
             'dToF Lidar sensor mode: -1=Ignore 0=HDR 1=High Peak 2=Low Peak 3=Adaptive; '
             '-1 = keep device current mode (do not set on start) / dToF 传感器模式；'
             '-1 = 保持设备当前模式（启动时不下发）'),
        _arg('log_level', '1',
             'Log level: 0=ERROR 1=INFO 2=WARN 3=DEBUG / 日志级别'),

        # --- Main driver: channel enable (hot-switch via rqt/service) ------
        _arg('enable_raw_point', 'true',
             'Enable raw point cloud channel (hot-switch) / 启用原始点云通道（热切换）'),
        _arg('enable_slam_point', 'true',
             'Enable SLAM point cloud channel (hot-switch) / 启用 SLAM 点云通道（热切换）'),
        _arg('enable_image0', 'true',
             'Enable camera0 image channel (hot-switch) / 启用相机0图像通道（热切换）'),
        _arg('enable_image1', 'false',
             'Unsupported on L0 / L0 单相机不支持'),
        _arg('enable_imu', 'true',
             'Enable IMU data channel (hot-switch) / 启用 IMU 数据通道（热切换）'),
        _arg('enable_odom_lf', 'true',
             'Enable low-frequency odometry channel (hot-switch) / 启用低频里程计通道（热切换）'),
        _arg('enable_odom_hf', 'true',
             'Enable high-frequency odometry channel (hot-switch) / 启用高频里程计通道（热切换）'),
        _arg('enable_tf', 'true',
             'Enable TF transform channel (hot-switch) / 启用 TF 变换通道（热切换）'),

        # --- post_process_node: post-processing flags (hot-switch) ---------
        _arg('post_process', 'false',
             'Enable post-processing node (static, not hot-switch) / 启用后处理节点（静态，非热切换）'),
        _arg('send_image_raw', 'false',
             'Publish raw camera0 image (hot-switch) / 发布原始相机0图像（热切换）'),
        _arg('send_image2_raw', 'false',
             'Unsupported on L0 / L0 单相机不支持'),
        _arg('send_cloud_render', 'false',
             'Publish cloud-rendered image (hot-switch) / 发布点云渲染图像（热切换）'),
        _arg('undistort_mapping_only', 'false',
             'Publish camera0 undistort only in confirmed SLAM/ReMap mode'),
        _arg('send_undistort', 'false',
             'Publish undistorted images (hot-switch) / 发布去畸变图像（热切换）'),
        _arg('send_rectify', 'false',
             'Unsupported on L0 / L0 单相机不支持'),

        # --- opencv_undistort_node ----------------------------------------
        _arg('send_opencv_undistort', 'false',
             'Start OpenCV undistortion node / 启动 OpenCV 去畸变节点'),

        # --- pointcloud_reprojector_node ----------------------------------
        _arg('mode', 'raw',
             'Reprojector mode: raw | slam / 重投影模式'),
        _arg('enable_reprojection', 'false',
             'Enable pointcloud_reprojector_node (static, not hot-switch) / 启用重投影节点（静态，非热切换）'),
        _arg('min_depth', '0.1',
             'Reprojector visualization min depth (m) / 可视化最小深度'),
        _arg('max_depth', '50.0',
             'Reprojector visualization max depth (m) / 可视化最大深度'),
        _arg('point_size', '2',
             'Overlay point pixel radius / 叠加点像素半径'),
        _arg('use_depth_color', 'true',
             'Color by depth (false=single color) / 是否按深度上色'),
        _arg('sync_slop', '0.1',
             'Reprojector time-sync slop (s) / 重投影节点时间同步偏差'),

        # --- pointcloud_to_depth_node -------------------------------------
        _arg('enable_pointcloud_to_depth', 'false',
             'Enable pointcloud_to_depth_node (static, not hot-switch) / 启用点云转深度图节点（静态，非热切换）'),

        # --- pointcloud_to_depth_node -------------------------------------
        _arg('scale', '7.0',
             'Projection canvas downscale factor / 投影画布下采样比'),
        _arg('z_min', '0.1',
             'Valid depth lower bound (m) / 有效深度下限'),
        _arg('z_max', '50.0',
             'Valid depth upper bound (m) / 有效深度上限'),
        _arg('dilate_radius', '1',
             '3x3 dilation radius when writing projected pixels / 投影膨胀半径'),
        _arg('use_z_buffer', 'false',
             'Enable z-buffer to prevent far-over-near overwrite / 启用 Z-buffer'),
        _arg('edge_grad_threshold', '0.75',
             'Sobel edge suppression threshold (m/pixel) / Sobel 边缘抑制阈值'),
        _arg('vis_depth_min', '0.1',
             'JET min normalization (m); <0 = per-frame auto / JET 归一化下限'),
        _arg('vis_depth_max', '5.0',
             'JET max normalization (m) / JET 归一化上限'),
        _arg('sync_slop_sec', '0.1',
             'pointcloud_to_depth time-sync slop (s) / 点云转深度图同步偏差'),

        # --- depth_to_pointcloud_node -------------------------------------
        _arg('enable_depth_to_pointcloud', 'false',
             'Enable depth_to_pointcloud_node (static, not hot-switch) / 启用深度图转点云节点（静态，非热切换）'),

        # Shared by pointcloud_reprojector_node + pointcloud_to_depth_node.
        # If you need different values, override per-node with
        # `ros2 param set /<full_node_name> sync_queue_size <v>`.
        # 重投影节点与点云转深度图节点共用；如需分别设置请走全路径。
        _arg('sync_queue_size', '10',
             'Shared time-sync queue size (reprojector + pointcloud_to_depth) '
             '/ 共享的时间同步队列长度'),

        # --- depth_to_pointcloud_node -------------------------------------
        _arg('step', '4',
             'Pixel sampling stride (smaller = denser cloud) / 像素采样步长'),
        _arg('d_min', '0.1',
             'depth_to_pointcloud valid depth lower bound (m) / 有效深度下限'),
        _arg('d_max', '100.0',
             'depth_to_pointcloud valid depth upper bound (m) / 有效深度上限'),
    ]

    # -----------------------------------------------------------------------
    # Parameter blobs / 各节点参数表
    # -----------------------------------------------------------------------
    driver_params = {
        'topic_prefix':                 _pv('topic_prefix',                 str),
        'device_mode':                  _pv('device_mode',                  str),
        'discovery_timeout_ms':         _pv('discovery_timeout_ms',         int),

        'image_width':                  _pv('image_width',                  int),
        'image_height':                 _pv('image_height',                 int),
        'image_fps':                    _pv('image_fps',                    int),
        'image_format':                 _pv('image_format',                 str),

        'filter_confidence_threshold':    _pv('filter_confidence_threshold',    int),
        'filter_adaptive_cdf_percentile': _pv('filter_adaptive_cdf_percentile', int),
        'filter_adaptive_outdoor_thr':    _pv('filter_adaptive_outdoor_thr',    int),
        'filter_range_min_depth':         _pv('filter_range_min_depth',         int),

        'raw_frame_id':                 _pv('raw_frame_id',                 str),
        'slam_frame_id':                _pv('slam_frame_id',                str),
        'image_frame_id':               _pv('image_frame_id',               str),
        'imu_frame_id':                 _pv('imu_frame_id',                 str),
        'odom_frame_id':                _pv('odom_frame_id',                str),

        'tf_publish_rate':              _pv('tf_publish_rate',              float),

        'confidence_threshold':         _pv('confidence_threshold',         int),
        'enable_row_filter':            _pv('enable_row_filter',            bool),
        'min_row':                      _pv('min_row',                      int),
        'sensor_mode':                  _pv('sensor_mode',                  int),
        'log_level':                    _pv('log_level',                    int),

        'enable_raw_point':             _pv('enable_raw_point',             bool),
        'enable_slam_point':            _pv('enable_slam_point',            bool),
        'enable_image0':                _pv('enable_image0',                bool),
        'enable_image1':                _pv('enable_image1',                bool),
        'enable_imu':                   _pv('enable_imu',                   bool),
        'enable_odom_lf':               _pv('enable_odom_lf',               bool),
        'enable_odom_hf':               _pv('enable_odom_hf',               bool),
        'enable_tf':                    _pv('enable_tf',                    bool),
    }

    reproj_params = {
        'topic_prefix':         _pv('topic_prefix',         str),
        'enable_reprojection':  _pv('enable_reprojection',  bool),
        'mode':                 _pv('mode',                 str),
        'min_depth':        _pv('min_depth',        float),
        'max_depth':        _pv('max_depth',        float),
        'point_size':       _pv('point_size',       int),
        'use_depth_color':  _pv('use_depth_color',  bool),
        'sync_queue_size':  _pv('sync_queue_size',  int),    # shared
        'sync_slop':        _pv('sync_slop',        float),
    }

    p2d_params = {
        'topic_prefix':              _pv('topic_prefix',              str),
        'enable_pointcloud_to_depth': _pv('enable_pointcloud_to_depth', bool),
        'scale':                _pv('scale',                float),
        'z_min':                _pv('z_min',                float),
        'z_max':                _pv('z_max',                float),
        'dilate_radius':        _pv('dilate_radius',        int),
        'use_z_buffer':         _pv('use_z_buffer',         bool),
        'edge_grad_threshold':  _pv('edge_grad_threshold',  float),
        'vis_depth_min':        _pv('vis_depth_min',        float),
        'vis_depth_max':        _pv('vis_depth_max',        float),
        'sync_queue_size':      _pv('sync_queue_size',      int),    # shared
        'sync_slop_sec':        _pv('sync_slop_sec',        float),
    }

    d2p_params = {
        'topic_prefix':               _pv('topic_prefix',               str),
        'enable_depth_to_pointcloud': _pv('enable_depth_to_pointcloud', bool),
        'step':         _pv('step',         int),
        'd_min':        _pv('d_min',        float),
        'd_max':        _pv('d_max',        float),
    }

    opencv_undistort_params = {
        'topic_prefix':              _pv('topic_prefix',              str),
        'send_opencv_undistort':     _pv('send_opencv_undistort',     bool),
    }

    postprocess_params = {
        'topic_prefix':      _pv('topic_prefix',      str),
        'post_process':      _pv('post_process',      bool),
        'send_image_raw':    _pv('send_image_raw',    bool),
        'send_image2_raw':   _pv('send_image2_raw',   bool),
        'send_cloud_render': _pv('send_cloud_render', bool),
        'send_undistort':    _pv('send_undistort',    bool),
        'undistort_mapping_only': _pv('undistort_mapping_only', bool),
        'send_rectify':      _pv('send_rectify',      bool),
        'send_opencv_undistort': _pv('send_opencv_undistort', bool),
    }

    # -----------------------------------------------------------------------
    # Nodes / 节点
    # -----------------------------------------------------------------------
    nodes = [
        Node(
            package=packageName,
            executable='odin_ros_driver_node',
            output='screen',
            parameters=[driver_params],
            additional_env={'ODIN_DEVICE_LIB_PATH': EnvironmentVariable(
                'ODIN_DEVICE_LIB_PATH', default_value=device_plugin_dir)},
        ),
        Node(
            package=packageName,
            executable='post_process_node',
            name='postprocess',
            output='screen',
            parameters=[postprocess_params],
            condition=IfCondition(LaunchConfiguration('post_process')),
        ),
        Node(
            package=packageName,
            executable='opencv_undistort_node',
            name='opencv_undistort',
            output='screen',
            parameters=[opencv_undistort_params],
            condition=IfCondition(LaunchConfiguration('send_opencv_undistort')),
        ),
        Node(
            package=packageName,
            executable='pointcloud_reprojector_node',
            name='pointcloud_reprojector',
            output='screen',
            parameters=[reproj_params],
            condition=IfCondition(LaunchConfiguration('enable_reprojection')),
        ),
        # PointCloud -> depth map verification node.
        # Subscribes to the synchronized cloud_raw + camera0/undistort and
        # publishes /{prefix}/device{N}/pointcloud2depth/{depth,depth_color,depth_overlay}.
        Node(
            package=packageName,
            executable='pointcloud_to_depth_node',
            name='pointcloud_to_depth',
            output='screen',
            parameters=[p2d_params],
            condition=IfCondition(LaunchConfiguration('enable_pointcloud_to_depth')),
        ),
        # Depth -> dense XYZRGB cloud back-projection node.
        # Subscribes to the synchronized pointcloud2depth/depth + camera0/undistort
        # and publishes /{prefix}/device{N}/depth2cloud/cloud_color.
        Node(
            package=packageName,
            executable='depth_to_pointcloud_node',
            name='depth_to_pointcloud',
            output='screen',
            parameters=[d2p_params],
            condition=IfCondition(LaunchConfiguration('enable_depth_to_pointcloud')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='odin_rviz',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_rviz')),
        ),
    ]

    return LaunchDescription([*args, OpaqueFunction(function=_reject_unsupported_camera_options), *nodes])
