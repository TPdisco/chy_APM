# stereo_minimal.launch.py
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node,SetParameter
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # rtabmap_odom (stereo_odometry) - 标准参数
    odom_standard = {
        # ========== 坐标系 ==========
        'frame_id': 'camera_link',      # 相机坐标系
        'odom_frame_id': 'odom',             # 里程计父坐标系
        'child_frame_id': 'base_link',  # 机器人本体（建议用base_footprint而非base_link）
        
        # ========== 订阅设置 ==========
        'subscribe_stereo': True,
        # 'subscribe_rgbd': True,
        'subscribe_odom_info': False,
        
        
        

        # ========== 时间设置 ==========
        'use_sim_time': False,
        'approx_sync': True,
        'approx_sync_max_interval': 0.05,    # 10ms同步窗口
        'sync_queue_size': 20,                    # 输入队列长度
        'sync_tolerance': '0.05',             # 同步容差 10ms
        'topic_queue_size': 100,
        



        # ========== TF设置 ==========
        'publish_tf': True,
        'wait_for_transform': 0.2,           # TF等待超时
        'publish_frequency': 30.0,           # TF发布频率
        
        # ========== 特征检测（性能关键） ==========
        'Kp/MaxFeatures': '300',             # 每帧特征点数
        'Kp/DetectorStrategy': '2',          # 2=ORB（快速）
        'Vis/FeatureType': '2',              # 2=ORB描述子
        'Vis/CorType': '0',                 # 0=词袋匹配, 1=光学流

        "Vis/MaxDepth": "8.0",            # 最大深度4米（滤除远处特征，提升精度）
        "Vis/CorNNDR": "0.8",           # 最近邻比率0.7（提升匹配质量）默认0.8
        "Vis/MinDepth": "0.1",            # 最小深度0.3米（滤除近处噪声）
        
        # ========== 运动模型 ==========
        'Odom/Strategy': '0',               # 0=帧到帧, 1=帧到地图

        # ========== 立体匹配 ==========
        'Stereo/MaxDisparity': '128.0',      # 最大视差
        'Stereo/MinDisparity': '0',
        
        # ========== 初始化与跟踪 ==========
        'Vis/MinInliers': '8',              # PnP最小内点
        'Odom/MinInliers': '8',
        
        # ========== 性能保护 ==========
        'Rtabmap/TimeThr': '0.4',            # 单帧最大处理时间200ms

        # ========== 新增：丢失恢复机制 ==========
        'Odom/ResetCountdown': '1',          # 【新增】丢失后立即重置，快速恢复
        'Vis/MaxFeatures': '600',            # 【新增】备用特征数上限
        'Vis/EstimationType': '1',           # 确保纯视觉模式稳定
    }

    # rtabmap_slam (rtabmap) - 标准参数
    slam_standard = {
        # ========== 坐标系 ==========
        'frame_id': 'camera_link',
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        
        # ========== 订阅设置 ==========
        'subscribe_stereo': True,
        # 'subscribe_rgbd': True,
        'subscribe_odom_info': True,        # 订阅里程计信息（由odom节点提供）

        # ========== 时间设置 ==========
        'use_sim_time': False,
        'approx_sync': True,
        
        # ========== 回环检测 ==========
        'Rtabmap/DetectionRate': '1',        # 处理频率2Hz
        'Rtabmap/LoopThr': '0.1',           # 回环阈值（越小越宽松）
        
        # ========== 特征与更新 ==========
        'Kp/MaxFeatures': '300',             # SLAM特征数可略高于odom
        'RGBD/LinearUpdate': '0.1',         # 最小平移更新距离(5cm)
        'RGBD/AngularUpdate': '0.1',        # 最小旋转更新角度(0.05rad≈3°)
        
        # ========== 性能优化 ==========
        'Rtabmap/TimeThr': '0.3',            # 单帧最大处理时间300ms
        'sync_queue_size': 10,                     # 短队列降低延迟
        
        'Grid/CellSize': '0.1',              # 栅格分辨率10cm
        'Mem/ReduceGraph': 'true',           # 【新增】启用图优化简化
        
        # ========== TF设置 ==========
        'wait_for_transform': 0.8,
        'publish_tf': True,                 # SLAM不发布TF（由odom发布odom->base）

        'Rtabmap/StartNewMapOnLoopClosure': 'false',
        # ============================================
        # ========== OctoMap 3D 地图生成配置 ==========
        # ============================================
        
        # ---- 核心开关 ----
        'Grid/3D': 'true',                   # 启用 3D 占用网格（OctoMap 必需）
        'Grid/RayTracing': 'true',           # 启用光线追踪填充未知空间
        
        # ---- 传感器选择 ----
        # 0=激光雷达, 1=深度图像(RGB-D), 2=两者都用
        'Grid/Sensor': '1',                  # 使用深度图像生成 OctoMap
        
        # ---- 网格分辨率 ----
        'Grid/CellSize': '0.05',             # OctoMap 分辨率 5cm（建议0.05-0.1）





        # ========== 2D栅格地图配置 ==========
        # 启用2D地图
        'Grid/FromDepth': 'true',          # 从深度图生成2D网格（必需）

        'Grid/MaxObstacleHeight': '2.5',   # 最大障碍物高度（米）
        'Grid/MinGroundHeight': '-0.3',    # 地面以下的最小高度
        'Grid/NormalsSegmentation': 'true', # 使用法向量分割地面
        'Grid/MaxGroundAngle': '30',       # 地面最大角度（度）
        'Grid/RangeMax': '5.0',           # 最大探测范围（米）
        'Grid/RangeMin': '0.2',            # 最小探测范围（米）
        
        # 2D地图发布设置
        'grid_map': 'true',                # 启用2D栅格地图发布
        'grid_unknown_space': 'true',      # 在地图中包含未知区域
        'grid_cell_size': '0.05',          # 与Grid/CellSize保持一致
        'grid_size': '50',                 # 地图大小（米），0=自动调整
        'grid_incremental': 'false',       # 设为false使用全局地图
        
        # 占用阈值
        'GridGlobal/OccupancyThr': '0.5',  # 占用概率阈值（0-1）
        'GridGlobal/ProbHit': '0.7',       # 命中概率
        'GridGlobal/ProbMiss': '0.4',      # 未命中概率
        
        # ========== 内存管理优化用于2D地图 ==========
        'Mem/IncrementalMemory': 'true',
        'Mem/STMSize': '30',
        'Mem/RehearsalSimilarity': '0.1',
        
        # 确保地图数据被保存和发布
        'publish_map_data': 'true',
        'map_cloud_output': 'true',



        # ========== 新增：鲁棒性增强 ==========
        'Rtabmap/MaxRetrieved': '2',         # 【新增】限制检索数量，加速回环检测
        'RGBD/ProximityPathMaxNeighbors': '10', # 【新增】限制邻近搜索范围
        'RGBD/ProximityBySpace': 'true',     # 【新增】启用空间邻近检测
        'RGBD/ProximityByTime': 'false',     # 【新增】关闭时间邻近，避免错误关联
        'Vis/CorNNDR': '0.8',                # 【新增】匹配阈值与odom一致
    }



    remappings=[
           ('left/image_rect', '/stereo_proc/left/image_rect'),
           ('left/camera_info', '/stereo/left/camera_info'),
           ('right/image_rect', '/stereo_proc/right/image_rect'),
           ('right/camera_info', '/stereo/right/camera_info'),
           # ('rgbd_image', '/stereo_camera/rgbd_image'),
          #('imu', '/imu'), 
        ]


    return LaunchDescription([
        
        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),
        
        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra arguments just for odometry node. If the same argument is already set in \"args\", it will be overwritten by the one in \"odom_args\".'),


        # Node(
        #     package='chy_rtabmap_slam',
        #     executable='map_pub', 
        #     name='pub_map_fixed',
        # ),
        
        # 在 slam.launch.py 中添加完整的TF变换链
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher',
        #    name='map_to_odom_tf',
        #    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        #),
        #Node(
        #    package='tf2_ros',
        #    executable='static_transform_publisher', 
        #    name='odom_to_base_tf',
        #    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        #),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link']
        ),




        Node(
            package='rtabmap_sync', executable='stereo_sync', output='screen',
            namespace='stereo_camera',
            parameters=[{
                'sync_queue_size': 30,
                }],
                remappings=[
                    ('left/image_rect', '/stereo_proc/left/image_rect'),
                    ('right/image_rect', '/stereo_proc/right/image_rect'),
                    ('left/camera_info', '/stereo/left/camera_info'),
                    ('right/camera_info', '/stereo/right/camera_info'),
                ]
            ),

        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=[odom_standard],
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args"),
                        '--ros-args', '--log-level', 'info'],
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[slam_standard],
            remappings=remappings,
            arguments=['-d', LaunchConfiguration("args"),
                    '--ros-args', '--log-level', 'info']),

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[slam_standard,
                        {'odometry_node_name': "stereo_odometry"}],
            remappings=remappings,
            arguments=['--ros-args', '--log-level', 'info'])
                
    ])
