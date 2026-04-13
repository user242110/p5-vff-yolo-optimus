import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # --- Launch arguments (sim vs real) ---
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/rgbd_camera/image',
        description='RGB image topic (sim: /rgbd_camera/image, real: /astra/color/image_raw)')

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/rgbd_camera/depth_image',
        description='Depth image topic (sim: /rgbd_camera/depth_image, real: /astra/depth/image_raw)')

    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/rgbd_camera/camera_info',
        description='Camera info topic (sim: /rgbd_camera/camera_info, real: /astra/color/camera_info)')

    laser_topic_arg = DeclareLaunchArgument(
        'laser_topic',
        default_value='/scan_raw',
        description='Laser scan topic (sim: /scan_raw, real: /scan)')

    optical_frame_arg = DeclareLaunchArgument(
        'optical_frame',
        default_value='camera_rgb_optical_frame',
        description='Camera optical frame (sim: camera_rgb_optical_frame, real: camera_color_optical_frame)')

    # --- 1. YOLO bringup (neural network inference) ---
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('yolo_bringup'),
                'launch',
                'yolo.launch.py'
            )
        ),
        launch_arguments={
            'input_image_topic': LaunchConfiguration('image_topic'),
            'input_depth_topic': LaunchConfiguration('depth_topic'),
            'input_depth_info_topic': LaunchConfiguration('camera_info_topic'),
            'target_frame': 'camera_link'
        }.items()
    )

    # --- 2. YOLO detection format converter (yolo_msgs -> Detection2DArray) ---
    yolo_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('camera'),
                'launch',
                'yolo_detection2d.launch.py'
            )
        )
    )

    # --- 3. Obstacle detector (laser -> repulsive vector) ---
    obstacle_detector_cmd = Node(
        package='vff_control',
        executable='obstacle_detector_node',
        name='obstacle_detector_node',
        output='screen',
        parameters=[{
            'min_distance': 0.5,
            'base_frame': 'base_footprint'
        }],
        remappings=[
            ('/input_laser', LaunchConfiguration('laser_topic'))
        ]
    )

    # --- 4. Follow person FSM node (detection + VFF + state machine) ---
    follow_person_cmd = Node(
        package='p5_vff_yolo',
        executable='follow_person_node',
        name='follow_person_node',
        output='screen',
        parameters=[{
            'target_class': 'person',
            'base_frame': 'base_footprint',
            'optical_frame': LaunchConfiguration('optical_frame'),
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'search_angular_speed': 0.3,
            'person_lost_timeout': 1.0,
            'repulsive_gain_factor': 1.0,
            'repulsive_influence_distance': 0.5,
        }],
        remappings=[
            ('/input_detection_2d', '/detections_2d'),
            ('/camera_info', LaunchConfiguration('camera_info_topic'))
        ]
    )

    return LaunchDescription([
        image_topic_arg,
        depth_topic_arg,
        camera_info_topic_arg,
        laser_topic_arg,
        optical_frame_arg,
        yolo_launch,
        yolo_detection_node,
        obstacle_detector_cmd,
        follow_person_cmd,
    ])
