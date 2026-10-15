#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_name = 'rover_description'
    pkg_share = get_package_share_directory(pkg_name)
    ws_share_dir = os.path.join(get_package_prefix(pkg_name), 'share')

    declared_args = [
        DeclareLaunchArgument('world', default_value='empty.sdf'),
    ]
    world = LaunchConfiguration('world')

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            ws_share_dir,
            os.path.join(pkg_share, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    xacro_file = os.path.join(pkg_share, 'urdf', 'Rover.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_desc_raw = Command([
        'xacro ', xacro_file,
        ' sim:=true',
        ' controllers_file:=', controllers_yaml,
    ])
    robot_desc = ParameterValue(robot_desc_raw, value_type=str)

    bridge_params_path = os.path.join(pkg_share, 'config', 'bridge_parameters.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', world]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc_raw,
            '-name', 'rover',
            '-z', '0.35',
        ],
        output='screen'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_params_path,
            'use_sim_time': True
        }],
        output='screen'
    )

    def spawner(name):
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                name,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
            parameters=[{'use_sim_time': True}],
        )

    jsb = spawner('joint_state_broadcaster')
    steer_ctrl = spawner('steer_position_controller')
    wheel_ctrl = spawner('wheel_velocity_controller')

    controller_chain = [
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[jsb])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=jsb, on_exit=[steer_ctrl])
        ),
        RegisterEventHandler(
            OnProcessExit(target_action=steer_ctrl, on_exit=[wheel_ctrl])
        ),
    ]

    kinematics = Node(
        package='rover_control',
        executable='sterowanie_node',
        name='rover_kinematics',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'kinematics.yaml'),
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription(
        declared_args + [
            set_gz_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            ros_gz_bridge,
            kinematics,
        ] + controller_chain
    )
