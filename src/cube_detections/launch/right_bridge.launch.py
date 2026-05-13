import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    sdk_path = '/home/zozo/Downloads/FTServo_Python/scservo_sdk'

    current_pythonpath = os.environ.get('PYTHONPATH', '')

    os.environ['PYTHONPATH'] = (
        sdk_path + ':' + current_pythonpath
    )

    moveit_share = FindPackageShare('dual_arm_moveit_config')

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_share, '/launch/rsp.launch.py'
        ])
    )

    static_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_share, '/launch/static_virtual_joint_tfs.launch.py'
        ])
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_share, '/launch/move_group.launch.py'
        ])
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            moveit_share, '/launch/moveit_rviz.launch.py'
        ])
    )

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


    bridge_node = Node(
        package='cube_detections',
        executable='test_bridge',
        name='right_arm_hardware',
        output='screen'
    )

    return LaunchDescription([
        rsp_launch,
        static_tfs_launch,
        move_group_launch,
        rviz_launch,
        bridge_node,
        #realsense_launch
    ])
