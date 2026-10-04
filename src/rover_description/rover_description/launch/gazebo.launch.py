#!/usr/bin/env python3
"""
Symulacja lazika 6-kolowego (4 kola skretne) w Gazebo Harmonic.

Uruchomienie:
    ros2 launch rover_description gazebo.launch.py
    ros2 launch rover_description gazebo.launch.py world:=rover_world.sdf joy:=false
"""

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

    # ------------------------------------------------------------------
    # Argumenty
    # ------------------------------------------------------------------
    declared_args = [
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        DeclareLaunchArgument('joy', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
    ]
    world = LaunchConfiguration('world')

    # ------------------------------------------------------------------
    # GZ_SIM_RESOURCE_PATH
    #
    # Dokladam istniejaca wartosc zmiennej i katalog worlds/.
    # Twoja wersja nadpisywala zmienna w calosci - dziala, dopoki wszystko
    # siedzi w jednym prefiksie, ale gubi modele z innych pakietow
    # i z ~/.gz/models.
    # ------------------------------------------------------------------
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            ws_share_dir,
            os.path.join(pkg_share, 'worlds'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    # ------------------------------------------------------------------
    # robot_description
    #
    # Te same robot_desc_raw ida do RSP i do create. Jesli dodasz
    # argumenty xacro, dodaj je TUTAJ - inaczej symulacja i drzewo TF
    # rozjada sie na dwa rozne modele, a to debuguje sie fatalnie.
    # ------------------------------------------------------------------
    xacro_file = os.path.join(pkg_share, 'urdf', 'Rover.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_desc_raw = Command([
        'xacro ', xacro_file,
        ' sim:=true',
        ' controllers_file:=', controllers_yaml,
    ])
    robot_desc = ParameterValue(robot_desc_raw, value_type=str)

    bridge_params_path = os.path.join(pkg_share, 'config', 'bridge_parameters.yaml')

    # ------------------------------------------------------------------
    # Gazebo
    # ------------------------------------------------------------------
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

    # -string zamiast -topic, zeby ominac problem z QoS
    # (create czekal w nieskonczonosc na topic robot_description
    # przez niedopasowanie QoS transient_local vs domyslny subscriber)
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

    # ------------------------------------------------------------------
    # Kontrolery
    #
    # controller_manager startuje WEWNATRZ pluginu gz_ros2_control, czyli
    # dopiero po spawnie modelu. Dlatego lancuchowo przez OnProcessExit,
    # a nie rownolegle - inaczej wyscig i losowe padniecia spawnerow.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # Joystick + kinematyka
    #
    # Ten sam wezel poleci na sprzecie. Zadnej osobnej sciezki dla symulacji.
    # ------------------------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'deadzone': 0.08,
                     'autorepeat_rate': 20.0}],
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    kinematics = Node(
        package='rover_control',
        executable='ackermann_6wheel_node',
        name='rover_kinematics',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'kinematics.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('joy')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'rover.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        declared_args + [
            set_gz_resource_path,
            gazebo,
            robot_state_publisher,
            spawn_robot,
            ros_gz_bridge,
            joy_node,
            kinematics,
            rviz,
        ] + controller_chain
    )