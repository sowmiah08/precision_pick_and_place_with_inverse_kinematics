import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    sdk_path = os.path.expanduser(
        '~/zobot_ws/ros/sowmiya_ws/FTServo_Python'
    )

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

    cube_detector = Node(
                package='cube_detections',
                executable='cube_detector',
                name='cube_detector',
                output='screen'
            )
    
    tf_node = Node(
                package='cube_detections',
                executable='tf_node',
                name='tf_node',
                output='screen'
    )

    pick_nd_place = Node(
                package='cube_detections',
                executable='pick_nd_place',
                name='pick_nd_place',
                output='screen'
    )

    pick_nd_place_left = Node(
                package='cube_detections',
                executable='pick_nd_place_left',
                name='pick_nd_place_left',
                output='screen'
    )

    right_bridge = Node(
        package='cube_detections',
        executable='right_bridge',
        name='right_arm_hardware',
        output='screen'
    )

    left_bridge = Node(
        package='cube_detections',
        executable='left_bridge',
        name='left_arm_hardware',
        output='screen'
    )



    return LaunchDescription([
        realsense_launch,
        rsp_launch,
        static_tfs_launch,
        move_group_launch,
        rviz_launch,
        right_bridge,
        left_bridge,
        cube_detector,
        tf_node,
        pick_nd_place,
        pick_nd_place_left,

    ])
