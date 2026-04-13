import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

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
            'input_image_topic': '/rgbd_camera/image',
            'input_depth_topic': '/rgbd_camera/depth_image',
            'input_depth_info_topic': '/rgbd_camera/camera_info',
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
            ('/input_laser', '/scan_raw')
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
            'optical_frame': 'camera_rgb_optical_frame',
            'max_linear_speed': 0.3,
            'max_angular_speed': 1.0,
            'search_angular_speed': 0.3,
            'person_lost_timeout': 1.0,
            'repulsive_gain_factor': 1.0,
            'repulsive_influence_distance': 0.5,
        }],
        remappings=[
            ('/input_detection_2d', '/detections_2d'),
            ('/camera_info', '/rgbd_camera/camera_info')
        ]
    )

    return LaunchDescription([
        yolo_launch,
        yolo_detection_node,
        obstacle_detector_cmd,
        follow_person_cmd,
    ])
