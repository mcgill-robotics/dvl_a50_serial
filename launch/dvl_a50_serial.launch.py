import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    autostart = LaunchConfiguration('autostart')
    
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the lifecycle node on startup'
    )

    config_file = os.path.join(
        get_package_share_directory('dvl_a50_serial'), 'config', 'dvl_a50_serial.yaml')

    dvl_node = LifecycleNode(
        package='dvl_a50_serial',
        executable='dvl_a50_serial_node',
        name='dvl_a50_serial',
        namespace='dvl',
        output='screen',
        parameters=[config_file]
    )

    # When the inactive state is reached, make it transition to active.
    configure_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=dvl_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(dvl_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart)
    )

    emit_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(dvl_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart)
    )

    return LaunchDescription([
        autostart_arg,
        dvl_node,
        configure_event_handler,
        emit_configure_event,
    ])
