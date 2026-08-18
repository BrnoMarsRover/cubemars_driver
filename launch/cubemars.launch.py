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

    # config/<robot_name>/<config>.yaml
    config_file = os.path.join(
        get_package_share_directory('cubemars_driver'),
        'config', robot_name.perform(context), config.perform(context))

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
            description='Robot name (ex: "freya"). Also selects the config/<robot_name>/ folder.'
        ),
        DeclareLaunchArgument(
            name='robot_number',
            default_value='',
            description='Robot number, appended to the robot name if defined ("<name>_<number>").'
        ),
        DeclareLaunchArgument(
            name='config',
            default_value='manipulator.yaml',
            description='Parameter file name inside config/<robot_name>/.'
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
