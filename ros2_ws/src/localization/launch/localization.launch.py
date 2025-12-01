from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, Shutdown, EmitEvent
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Declare launch argument for rosbag file
    bag_in_arg = DeclareLaunchArgument(
        'bag_in',
        description='Path to the rosbag file to play'
    )

    bag_out_arg = DeclareLaunchArgument(
        'bag_out',
        description='Path to the rosbag to be generated',
        default_value='' # Make it so it's not required.
    )

    rosbag_in_file = LaunchConfiguration('bag_in') # Must be specified
    rosbag_out_file = LaunchConfiguration('bag_out') # May be empty

    # Node: DispatchNode
    dispatch_node = Node(
        package='localization',
        executable='dispatch_node',
        name='dispatch_node',
        output='screen'
    )

    # Node: particle_filter
    particle_filter_node = Node(
        package='localization',
        executable='particle_filter',
        name='particle_filter',
        parameters=[{'bag_in': rosbag_in_file}],
        output='screen'
    )

    # Play rosbag
    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', rosbag_in_file],
        output='screen'
    )

    # Record rosbag
    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', rosbag_out_file],
        output='screen',
        condition=IfCondition(PythonExpression(["'", rosbag_out_file, "' != ''"])) # Only record if an outfile is specified.
    )

    # Shutdown nodes after rosbag finishes

    shutdown_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbag_play,
            on_exit=[Shutdown()]
        )
    )

    return LaunchDescription([
        bag_in_arg,
        bag_out_arg,
        dispatch_node,
        particle_filter_node,
        rosbag_record, #Should only launch if a rosbag_out was specified
        rosbag_play,
        shutdown_nodes
    ])