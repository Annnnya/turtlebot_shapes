from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():
    ld = LaunchDescription()

    arg = DeclareLaunchArgument('simulation', default_value='2d', description='Choose simulation type (2d or 3d)')
    ld.add_action(arg)

    if LaunchConfigurationEquals('simulation', '2d'):
        sim_node = Node(
            package='turtlesim',
            executable='turtlesim_node',
            output='screen',
        )
    elif LaunchConfigurationEquals('simulation', '3d'):
        sim_node = Node(
            package='turtlebot4_ignition_bringup',
            executable='turtlebot4_ignition.launch.py',
            output='screen'
        )
    else:
        raise ValueError(f"Invalid value  for 'simulation': {LaunchConfiguration('simulation')}. Expected '2d' or '3d'.")

    ld.add_action(sim_node)

    mover_node = Node(
        package='move_in_shapes',
        executable='move_in_shapes',
        output='screen',
        namespace=LaunchConfiguration('simulation') if LaunchConfiguration('simulation') == '2d' else '',
        parameters=[
            {'namespace': '/turtle1'} if LaunchConfigurationEquals('simulation', '2d') else {},
        ],
    )
    ld.add_action(mover_node)

    controller_node = Node(
        package='move_in_shapes',
        executable='shape_motion_controller',
        output='screen',
        prefix='gnome-terminal --',
    )
    ld.add_action(controller_node)

    return ld
