import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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


def generate_launch_description():
    pkg_share = get_package_share_directory(packageName)
    default_rviz = os.path.join(pkg_share, 'rviz', 'driver_view_ros2.rviz')
    default_control_config = os.path.join(pkg_share, 'config', 'control_command.yaml')

    # -----------------------------------------------------------------------
    # Launch arguments / 启动参数声明
    # -----------------------------------------------------------------------
    args = [
        # --- General / 通用 -------------------------------------------------
        _arg('start_auxiliary_nodes', 'true', 'Start optional image/depth postprocessing nodes'),
        _arg('start_rviz', 'false', 'Whether to start RViz / 是否启动 RViz'),
        _arg('rviz_config', default_rviz, 'Path to RViz config file / RViz 配置文件路径'),
        _arg('topic_prefix', 'manifold',
             'Topic prefix root, final prefix is /{topic_prefix}/{model}/device{N}/ '
             '/ 话题前缀根，最终前缀 /{topic_prefix}/{model}/device{N}/'),
        _arg('control_config_path', default_control_config,
             'Data-channel enable configuration / 数据通道开关配置'),

        # --- Main driver: device / 主驱动设备 ------------------------------
        _arg('operating_mode', 'normal',
             'Operating mode: normal | standby / 工作模式'),
        _arg('discovery_timeout_ms', '2000',
             'SDK device-discovery timeout (ms) / SDK 设备发现超时 (ms)'),

        # --- Main driver: image stream / 图像流 ----------------------------
        _arg('image_width', '0',
             'Image width, 0 = device default / 图像宽度，0=设备默认'),
        _arg('image_height', '0',
             'Image height, 0 = device default / 图像高度，0=设备默认'),
        _arg('image_fps', '0',
             'Image FPS, 0 = device default / 图像帧率，0=设备默认'),
        _arg('image_format', 'mjpeg',
             'Image format: mjpeg|yuyv|nv12|nv21|rgb24 / 图像格式'),

        # --- Main driver: frame ids / 坐标系名 -----------------------------
        _arg('raw_frame_id', 'odom',
             'Frame id for raw cloud / 原始点云 frame_id'),
        _arg('slam_frame_id', 'odom',
             'Frame id for SLAM cloud / SLAM 点云 frame_id'),
        _arg('image_frame_id', 'camera0',
             'Frame id for left image / 左相机图像 frame_id'),
        _arg('image2_frame_id', 'odom',
             'Frame id for right image / 右相机图像 frame_id'),
        _arg('gray_image_frame_id', 'odom',
             'Frame id for gray image / 灰度图像 frame_id'),
        _arg('imu_frame_id', 'odom',
             'Frame id for IMU / IMU frame_id'),
        _arg('odom_frame_id', 'odom',
             'Odometry parent frame / 里程计父帧'),
        _arg('odom_child_frame_id', 'base_link',
             'Odometry child frame / 里程计子帧'),
        _arg('map_frame_id', 'map',
             'TF map frame name / TF map 帧名'),

        # --- Main driver: TF switches / TF 开关 ----------------------------
        _arg('publish_tf_odom', 'true',
             'Publish odom -> base_link TF / 是否发布 odom->base_link TF'),
        _arg('publish_tf_map', 'true',
             'Publish map -> odom TF / 是否发布 map->odom TF'),

        # --- Main driver: runtime knobs / 运行期可调旋钮（初始值） ----------
        _arg('confidence_threshold', '7',
             'Point-cloud confidence threshold (0..255) / 点云置信度阈值'),
        _arg('sensor_mode', '1',
             'dToF sensor mode: 0=HDR 1=High Peak 2=Low Peak 3=Adaptive / dToF 传感器模式'),
        _arg('log_level', '1',
             'Log level: 0=ERROR 1=INFO 2=WARN 3=DEBUG / 日志级别'),

        # --- Main driver: odometry scaling (advanced) / 里程计缩放（高级） -
        _arg('odom_position_scale', '1e-6',
             'Odometry position scale (SDK -> meters) / 里程计位置缩放'),
        _arg('odom_orientation_scale', '1e-6',
             'Odometry orientation scale / 里程计姿态缩放'),
        _arg('odom_linear_velocity_scale', '1e-6',
             'Odometry linear-velocity scale / 线速度缩放'),
        _arg('odom_angular_velocity_scale', '1e-6',
             'Odometry angular-velocity scale / 角速度缩放'),

        # --- pointcloud_reprojector_node ----------------------------------
        # NOTE: 'mode' is also overridden at runtime by reprojection_mode in
        # config/control_command.yaml. The launch-arg here is only the seed
        # default before the YAML override is applied.
        # 注意：'mode' 同时会被 control_command.yaml 中 reprojection_mode 覆盖。
        _arg('mode', 'raw',
             'Reprojector mode: raw | slam (yaml may override) / 重投影模式'),
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
        _arg('depth_rate_hz', '10.0', 'Depth update rate'),
        _arg('depth_width', '64', 'SRU depth width; preserve calibrated FOV'),
        _arg('depth_height', '40', 'SRU depth height'),
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
        _arg('raw_depth_preview_enabled', 'false',
             'Publish full-resolution mono8 depth preview / 发布全尺寸 mono8 深度预览'),
        _arg('vis_depth_min', '0.1',
             'JET min normalization (m); <0 = per-frame auto / JET 归一化下限'),
        _arg('vis_depth_max', '5.0',
             'JET max normalization (m) / JET 归一化上限'),
        _arg('sync_slop_sec', '0.1',
             'pointcloud_to_depth time-sync slop (s) / 点云转深度图同步偏差'),

        # Shared by pointcloud_reprojector_node + pointcloud_to_depth_node.
        # If you need different values, override per-node with
        # `ros2 param set /<full_node_name> sync_queue_size <v>`.
        # 重投影节点与点云转深度图节点共用；如需分别设置请走全路径。
        _arg('sync_queue_size', '5',
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
        'control_config_path':          _pv('control_config_path',          str),
        'operating_mode':               _pv('operating_mode',               str),
        'discovery_timeout_ms':         _pv('discovery_timeout_ms',         int),

        'image_width':                  _pv('image_width',                  int),
        'image_height':                 _pv('image_height',                 int),
        'image_fps':                    _pv('image_fps',                    int),
        'image_format':                 _pv('image_format',                 str),

        'raw_frame_id':                 _pv('raw_frame_id',                 str),
        'slam_frame_id':                _pv('slam_frame_id',                str),
        'image_frame_id':               _pv('image_frame_id',               str),
        'image2_frame_id':              _pv('image2_frame_id',              str),
        'gray_image_frame_id':          _pv('gray_image_frame_id',          str),
        'imu_frame_id':                 _pv('imu_frame_id',                 str),
        'odom_frame_id':                _pv('odom_frame_id',                str),
        'odom_child_frame_id':          _pv('odom_child_frame_id',          str),
        'map_frame_id':                 _pv('map_frame_id',                 str),

        'publish_tf_odom':              _pv('publish_tf_odom',              bool),
        'publish_tf_map':               _pv('publish_tf_map',               bool),

        'confidence_threshold':         _pv('confidence_threshold',         int),
        'sensor_mode':                  _pv('sensor_mode',                  int),
        'log_level':                    _pv('log_level',                    int),

        'odom_position_scale':          _pv('odom_position_scale',          float),
        'odom_orientation_scale':       _pv('odom_orientation_scale',       float),
        'odom_linear_velocity_scale':   _pv('odom_linear_velocity_scale',   float),
        'odom_angular_velocity_scale':  _pv('odom_angular_velocity_scale',  float),
    }

    reproj_params = {
        'topic_prefix':     _pv('topic_prefix',     str),
        'mode':             _pv('mode',             str),
        'min_depth':        _pv('min_depth',        float),
        'max_depth':        _pv('max_depth',        float),
        'point_size':       _pv('point_size',       int),
        'use_depth_color':  _pv('use_depth_color',  bool),
        'sync_queue_size':  _pv('sync_queue_size',  int),    # shared
        'sync_slop':        _pv('sync_slop',        float),
    }

    p2d_params = {
        'topic_prefix':         _pv('topic_prefix',         str),
        'depth_rate_hz': _pv('depth_rate_hz', float),
        'depth_width': _pv('depth_width', int),
        'depth_height': _pv('depth_height', int),
        'scale':                _pv('scale',                float),
        'z_min':                _pv('z_min',                float),
        'z_max':                _pv('z_max',                float),
        'dilate_radius':        _pv('dilate_radius',        int),
        'use_z_buffer':         _pv('use_z_buffer',         bool),
        'edge_grad_threshold':  _pv('edge_grad_threshold',  float),
        'raw_depth_preview_enabled': _pv('raw_depth_preview_enabled', bool),
        'vis_depth_min':        _pv('vis_depth_min',        float),
        'vis_depth_max':        _pv('vis_depth_max',        float),
        'sync_queue_size':      _pv('sync_queue_size',      int),    # shared
        'sync_slop_sec':        _pv('sync_slop_sec',        float),
    }

    d2p_params = {
        'topic_prefix': _pv('topic_prefix', str),
        'step':         _pv('step',         int),
        'd_min':        _pv('d_min',        float),
        'd_max':        _pv('d_max',        float),
    }

    postprocess_params = {
        'topic_prefix': _pv('topic_prefix', str),
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
        ),
        Node(
            package=packageName,
            executable='post_process_node',
            condition=IfCondition(LaunchConfiguration('start_auxiliary_nodes')),
            name='postprocess',
            output='screen',
            parameters=[postprocess_params],
        ),
        Node(
            package=packageName,
            executable='pointcloud_reprojector_node',
            condition=IfCondition(LaunchConfiguration('start_auxiliary_nodes')),
            name='pointcloud_reprojector',
            output='screen',
            parameters=[reproj_params],
        ),
        # PointCloud -> depth map verification node.
        # Subscribes to the synchronized cloud_raw + camera0/undistort and
        # publishes /{prefix}/{model}/device{N}/pointcloud2depth/{depth,depth_color,depth_overlay}.
        Node(
            package=packageName,
            executable='pointcloud_to_depth_node',
            condition=IfCondition(LaunchConfiguration('start_auxiliary_nodes')),
            name='pointcloud_to_depth',
            output='screen',
            parameters=[p2d_params],
        ),
        # Depth -> dense XYZRGB cloud back-projection node.
        # Subscribes to the synchronized pointcloud2depth/depth + camera0/undistort
        # and publishes /{prefix}/{model}/device{N}/depth2cloud/cloud_color.
        Node(
            package=packageName,
            executable='depth_to_pointcloud_node',
            condition=IfCondition(LaunchConfiguration('start_auxiliary_nodes')),
            name='depth_to_pointcloud',
            output='screen',
            parameters=[d2p_params],
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

    return LaunchDescription([*args, *nodes])
