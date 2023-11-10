from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

pkg_path = get_package_share_path('quadruped_description')
default_rviz_config_path = pkg_path / 'rviz/urdf.rviz'

anitkabir_path = PathJoinSubstitution(
        [FindPackageShare("quadruped_gazebo"), "worlds", "anitkabir.world"]
    )

world_path = PathJoinSubstitution(
        [FindPackageShare("quadruped_gazebo"), "worlds", "default.world"]
    )

config_path = PathJoinSubstitution(
        [FindPackageShare("quadruped_gazebo"), "config", "a1_ros_control.yaml"]
    )

gazebo_path = PathJoinSubstitution(
        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
    )

rviz_path = PathJoinSubstitution(
        [FindPackageShare("quadruped_description"), "rviz", "urdf.rviz"]
    )

description_path = PathJoinSubstitution(
        [FindPackageShare("quadruped_description"), "launch", "a1_rs_description.launch.py"]
    )


    

robot_description = ParameterValue(Command(['xacro ', PathJoinSubstitution([str(pkg_path), 'urdf/a1/a1_rs.xacro'])]))

def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            name="rviz",
            default_value="false",
            description="Whether to start RViz"
        ),

        DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Whether to use simulation time"
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_path),
        ),

        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration("use_sim_time"),
                'world': anitkabir_path,
            }.items()
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{
                "robot_description":robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                }, config_path],
            # remappings=[
            #     ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
            # ],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),
         
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_bot', '-z 0.5'],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                }],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_states_controller'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                'joint_group_effort_controller'],
            output='screen'
        ),


    ])