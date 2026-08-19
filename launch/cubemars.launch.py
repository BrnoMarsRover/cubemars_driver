# Launch a cubemars_node for one CAN bus of CubeMars actuators.
#
# Follows the project namespace convention:  /<robot_name>_<number>/<package>/<entity>

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
from typing import Optional, List


def launch_setup(context: LaunchContext) -> Optional[List[LaunchDescriptionEntity]]:

    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    robot_number = LaunchConfiguration('robot_number')
    config = LaunchConfiguration('config')
    state = LaunchConfiguration('state')
    command = LaunchConfiguration('command')

    indexed_robot_name = [robot_name.perform(context), '_', robot_number.perform(context)] \
        if robot_number.perform(context) else [robot_name.perform(context)]
    indexed_robot_name = ''.join(indexed_robot_name)

    # Full path, supplied by the caller. The driver ships no robot-specific config
    # of its own -- which motors exist and how they behave belongs to whatever
    # package owns the hardware, not to the driver. `config/example.yaml` here
    # documents the parameters but is not meant to be launched.
    config_file = config.perform(context)
    if not os.path.isfile(config_file):
        raise RuntimeError(
            f"cubemars_driver: config file not found: {config_file}\n"
            "Pass an absolute path, e.g. config:=$(ros2 pkg prefix --share my_pkg)/config/my_motors.yaml")

    return [LaunchDescription([
        Node(
            package='cubemars_driver',
            executable='cubemars_node',
            name='cubemars_node',
            namespace=indexed_robot_name + '/cubemars_driver',
            output='screen',
            parameters=[config_file,
                        {'use_sim_time': use_sim_time}],
            remappings=[
                ('~/state', state.perform(context)),
                ('~/command', command.perform(context)),
            ],
        )
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Use simulation (Gazebo) clock if true.'
        ),
        DeclareLaunchArgument(
            name='robot_name',
            description='Robot name (ex: "freya"), used to build the node namespace.'
        ),
        DeclareLaunchArgument(
            name='robot_number',
            default_value='',
            description='Robot number, appended to the robot name if defined ("<name>_<number>").'
        ),
        DeclareLaunchArgument(
            name='config',
            description='ABSOLUTE path to the parameter file describing the motors. '
                        'Owned by the package that owns the hardware, not by this driver. '
                        'See config/example.yaml for the parameters.'
        ),
        DeclareLaunchArgument(
            name='state',
            default_value='~/state',
            description='Output JointState topic (measured position/velocity/effort).'
        ),
        DeclareLaunchArgument(
            name='command',
            default_value='~/command',
            description='Input JointState topic (commanded position/velocity/effort).'
        ),
        OpaqueFunction(function=launch_setup)
    ])
