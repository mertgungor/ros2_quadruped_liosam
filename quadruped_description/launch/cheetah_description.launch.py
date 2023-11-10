from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_path('quadruped_description')
    default_rviz_config_path = pkg_path / 'rviz/urdf.rviz'

    robot_description = ParameterValue(Command(['xacro ', PathJoinSubstitution([str(pkg_path), 'urdf/cheetah/mini_cheetah.urdf'])]))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        #remappings=[('/joint_states', '/quadruped_gazebo/joint_states')]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(default_rviz_config_path)],
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
