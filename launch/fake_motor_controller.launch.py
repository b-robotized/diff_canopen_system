import launch.events
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution,
)
import launch.actions

import launch_ros
import launch_ros.events
import launch_ros.events.lifecycle
from launch_ros.substitutions import FindPackageShare

import lifecycle_msgs.msg


def generate_launch_description():
    node_id_arg = launch.actions.DeclareLaunchArgument(
        "node_id",
        default_value=TextSubstitution(text="2"),
        description="CANopen node id the mock slave shall have.",
    )

    slave_config_arg = launch.actions.DeclareLaunchArgument(
        "slave_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("diff_canopen_system"),
                "config",
                "fake_motor_controller.eds",
            ]
        ),
        description="Path to eds file to be used for the slave.",
    )

    can_interface_name_arg = launch.actions.DeclareLaunchArgument(
        "can_interface_name",
        default_value=TextSubstitution(text="vcan0"),
        description="CAN interface to be used by mock slave.",
    )

    node_name_arg = launch.actions.DeclareLaunchArgument(
        "node_name",
        default_value=TextSubstitution(text="fake_canopen_inverter_node"),
        description="Name of the node.",
    )

    slave_node = launch_ros.actions.LifecycleNode(
        name=LaunchConfiguration("node_name"),
        namespace="",
        package="diff_canopen_system",
        output="screen",
        executable="fake_canopen_controller_node",
        parameters=[
            {
                "slave_config": LaunchConfiguration("slave_config"),
                "node_id": LaunchConfiguration("node_id"),
                "can_interface_name": LaunchConfiguration("can_interface_name"),
            }
        ],
    )
    slave_inactive_state_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=slave_node,
            goal_state="inactive",
            handle_once=True,
            entities=[
                launch.actions.LogInfo(
                    msg="node 'fake_canopen_inverter_node' reached the 'inactive' state, 'activating'."
                ),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(slave_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
    )
    slave_configure = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(slave_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )
    ld = launch.LaunchDescription()
    ld.add_action(node_id_arg)
    ld.add_action(slave_config_arg)
    ld.add_action(can_interface_name_arg)
    ld.add_action(node_name_arg)
    ld.add_action(slave_inactive_state_handler)
    ld.add_action(slave_node)
    ld.add_action(slave_configure)
    return ld
