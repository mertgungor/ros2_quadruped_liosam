from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_path('quadruped_description')
    default_rviz_config_path = pkg_path / 'rviz/velodyne.rviz'

    urdf_path = PathJoinSubstitution(
        [FindPackageShare('quadruped_description'), 'urdf', 'anymal_b', 'anymal_b.urdf']
    ),

    return LaunchDescription([

        DeclareLaunchArgument(
            name='urdf_path', 
            default_value=urdf_path,
            description='URDF path'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('urdf_path')]),
                'use_sim_time'     : True
                }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(default_rviz_config_path)],
        )
    ])
