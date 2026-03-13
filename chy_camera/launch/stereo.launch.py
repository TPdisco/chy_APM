from launch import LaunchDescription
from launch_ros.actions import Node,PushRosNamespace
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():


	# 3. 创建stereo_image_proc节点组
    stereo_processing = GroupAction(
            actions=[
                # 设置命名空间
                PushRosNamespace('stereo_proc'),
                
                # 4. 左相机图像校正节点
                Node(
                    package='image_proc',
                    # executable='image_proc',  # 有灰度图mono和彩色图color
                    executable='rectify_node',
                    name='image_proc_left1',
                    # 输入: /stereo_proc/left/image_raw, /stereo_proc/left/camera_info
                    # 输出: /stereo_proc/left/image_rect_color 等
                    # 明确指定命名空间，避免 PushRosNamespace 的影响
                    namespace='left',
                    remappings=[
                        # 输入
                        ('image', '/stereo/left/image_raw'),
                        ('camera_info', '/stereo/left/camera_info'),
                        # 输出图像
                        ('image_rect', 'image_rect'),
                        ('image_rect_color', 'image_rect_color'),
                    ]
                ),
                
                # 5. 右相机图像校正节点
                Node(
                    package='image_proc',
                    executable='rectify_node',
                    name='image_proc_right1',
                    # 明确指定命名空间，避免 PushRosNamespace 的影响
                    namespace='right',
                    remappings=[
                        # 输入
                        ('image', '/stereo/right/image_raw'),
                        ('camera_info', '/stereo/right/camera_info'),
                        # 输出图像
                        ('image_rect', 'image_rect'),
                        ('image_rect_color', 'image_rect_color'),
                    ]
                )
                
                # # 6. 视差计算节点
                # Node(
                #     package='stereo_image_proc',
                #     executable='disparity_node',
                #     name='disparity_node',
                #     parameters=[
                #         {
                #             'approximate_sync': True,
                #             'queue_size': 10,
                #             'min_disparity': 0,
                #             'max_disparity': 128,
                #         }
                #     ],
                #     # 自动订阅:
                #     # - /stereo_proc/left/image_rect_color
                #     # - /stereo_proc/right/image_rect_color
                #     # - /stereo_proc/left/camera_info
                #     # - /stereo_proc/right/camera_info
                #     # 发布: /stereo_proc/disparity
                # )
                
                # # 7. 点云生成节点（可选）
                # Node(
                #     package='stereo_image_proc',
                #     executable='point_cloud_node',
                #     name='point_cloud_node',
                #     parameters=[
                #         {
                #             'approximate_sync': True,
                #             'queue_size': 10,
                #             'use_color': True,
                #         }
                #     ],
                #    # 自动订阅:
                #    # - /stereo_proc/left/image_rect_color
                #    # - /stereo_proc/right/image_rect_color
                #    # - /stereo_proc/disparity
                #    # - /stereo_proc/left/camera_info
                #    # 发布: /stereo_proc/points2
                # )
            ]
        )


    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='stereo_camera1',
            parameters=[ '/home/disco/study/chy/src/chy_camera/config/stereo.yaml'],
            remappings=[('image_raw', '/stereo/image_raw')]
        ),
        
        # 2. 启动分割节点
        Node(
            package='chy_camera',
            executable='stereo_split_node',
            name='stereo_splitter1',
            parameters=[
                {'left_calibration_file': '/home/disco/study/chy/src/chy_camera/biaoding/left.yaml'},
                {'right_calibration_file': '/home/disco/study/chy/src/chy_camera/biaoding/right.yaml'}
            ]
        ),
        
        # # 3. stereo_image_proc（使用官方launch） 图像矫正的包
        stereo_processing
    ])
