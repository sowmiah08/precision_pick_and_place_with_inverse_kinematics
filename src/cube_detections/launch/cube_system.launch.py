from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
   
    # RealSense Camera 
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'pointcloud.enable': 'true'
        }.items()
    )

    # AprilTag Detection
    apriltag_node = Node(
                package='apriltag_ros',
                executable='apriltag_node',
                name='apriltag_node',
                output='screen',
                remappings=[
                    ('image_rect', '/camera/camera/color/image_raw'),
                    ('camera_info', '/camera/camera/color/camera_info')
                ],
                parameters=[{
                    'family': '16h5',
                    'size': 0.03,
                    'detector.threads': 4,
                    'detector.decimate': 1.0,
                    'detector.blur': 0.8,
                    'detector.refine': True,
                    'detector.sharpening': 0.25,
                    'max_hamming': 1,
                }]
            )
    
    # Tag to TF
    tag_to_tf = Node(
                package='cube_detections',
                executable='tag_to_tf',
                name='tag_to_tf',
                output='screen'
            )

    # Cube Detector    
    cube_detector = Node(
                package='cube_detections',
                executable='cube_detector',
                name='cube_detector',
                output='screen'
            )
    
    # Target point
    target_tranform = Node(
                package='cube_detections',
                executable='target_tranform',
                name='target_tranform',
                output='screen'
            )

    return LaunchDescription([
        realsense_launch,
        apriltag_node,
        cube_detector,
        tag_to_tf,
        target_tranform
    ])