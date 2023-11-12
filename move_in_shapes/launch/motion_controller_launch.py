from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import LaunchConfigurationEquals, IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    launch_3d = LaunchConfiguration("launch_3d")

    arg = DeclareLaunchArgument(
        'launch_3d',
        default_value='false',
        description='Whether to launch 3d simulation instead of deafult 2d')
    ld.add_action(arg)

    sim_node_2d = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen',
        condition=UnlessCondition(launch_3d)
        )
    
    sim_node_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot4_ignition_bringup'),
                "launch/turtlebot4_ignition.launch.py")
            ),
        condition=IfCondition(launch_3d)
    )

    ld.add_action(sim_node_2d)
    ld.add_action(sim_node_3d)

    mover_node_2d = Node(
        package='move_in_shapes',
        executable='move_in_shapes',
        output='screen',
        parameters=[
            {'namespace': '/turtle1'}
        ],
        condition=UnlessCondition(launch_3d)
    )

    mover_node_3d = Node(
        package='move_in_shapes',
        executable='move_in_shapes',
        output='screen',
        condition=IfCondition(launch_3d)
    )
    ld.add_action(mover_node_2d)
    ld.add_action(mover_node_3d)

    controller_node = Node(
        package='move_in_shapes',
        executable='shape_motion_controller',
        output='screen',
        prefix='gnome-terminal --',
    )
    ld.add_action(controller_node)

    return ld
