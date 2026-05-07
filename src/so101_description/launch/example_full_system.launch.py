from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---------------- URDF ----------------
    pkg_path = get_package_share_directory('so101_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'dual_arm_final.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
            }],
        output='screen'
    )

    joint_state_publisher= Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # ---------------- RealSense ----------------
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

    # ---------------- IMPORTANT FIX ----------------
    # Link RealSense camera to your URDF camera_link
    '''
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',   # no offset (same physical camera)
            '0', '0', '0',
            'camera_link',   # URDF frame
            'camera_depth_optical_frame'  # RealSense frame (check actual name!)
        ]
    )'''

    # ---------------- Perception ----------------
    cube_detector = Node(
        package='cube_detections',
        executable='cube_detector',
        output='screen'
    )

    tf_node = Node(
        package='cube_detections',
        executable='tf_node',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        realsense_launch,
        cube_detector,
        tf_node
    ])