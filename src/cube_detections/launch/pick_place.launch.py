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

    # Cube Detector    
    cube_detector = Node(
                package='cube_detections',
                executable='cube_detector',
                name='cube_detector',
                output='screen'
            )
    
    #cam_viewer
    cam_view = Node(
            package='cube_detections',
            executable='cam_view',
            name='cam_view',
            output='screen'
        )
    # tranforms node 
    tf_node = Node(
                package='cube_detections',
                executable='tf_node',
                name='tf_node',
                output='screen'
    )

    #move_to_cube
    move_to_cube = Node(
                package='cube_detections',
                executable='move_to_cube',
                name='move_to_cube',
                output='screen'
    )

    grasp = Node(
                package='cube_detections',
                executable='grasp',
                name='grasp',
                output='screen'
    )


    return LaunchDescription([
        cube_detector,
        tf_node,
        move_to_cube
    ])