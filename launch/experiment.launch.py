"""Jeden launch: symulacja + wybrany regulator + ewaluacja."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('roboracer_sim')

    ########### ARGUMENTY #########
    # controller: pid | stanley
    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Regulator: pid albo stanley',
        choices=['pid', 'stanley'],
    )
    # Przekazujemy dalej do simulation.launch.py
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Czy uruchomić RViz',
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Czy pokazać okno Gazebo (false = headless)',
    )
    body_arg = DeclareLaunchArgument(
        'body',
        default_value='shell',
        choices=['shell', 'open'],
        description='Wygląd nadwozia',
    )
    # Przekazywane do simulation.launch.py → gz service set_physics
    rtf_arg = DeclareLaunchArgument(
        'rtf',
        default_value='1.0',
        description='Gazebo real_time_factor (1=realtime, >1 szybciej, 0=max)',
    )

    ########### SYMULACJA (Gazebo + mostek + robot) #########
    # Nie kopiujemy logiki — tylko includujemy gotowy launch.
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'gui': LaunchConfiguration('gui'),
            'body': LaunchConfiguration('body'),
            'rtf': LaunchConfiguration('rtf'),
        }.items(),
    )

    ########### REGULATOR PID #########
    # Uruchamiany tylko gdy controller:=pid
    pid_node = Node(
        package='roboracer_sim',
        executable='roboracer_pid',
        name='roboracer_pid',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('controller'), "' == 'pid'"
            ])
        ),
    )

    ########### REGULATOR STANLEY #########
    # Uruchamiany tylko gdy controller:=stanley
    stanley_node = Node(
        package='roboracer_sim',
        executable='control_stanley',
        name='control_stanley',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('controller'), "' == 'stanley'"
            ])
        ),
    )

    ########### EWALUACJA #########
    # Subskrybuje /odom_gt, porównuje z line_gt.csv, raport co 10 s, max 1 okrążenie.
    evaluate_node = Node(
        package='roboracer_sim',
        executable='evaluate_run',
        name='evaluate_run',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'controller_name': LaunchConfiguration('controller'),
            'odom_topic': '/odom_gt',
            'report_period': 10.0,
        }],
    )

    ########### LISTA AKCJI LAUNCH #########
    return LaunchDescription([
        controller_arg,
        rviz_arg,
        gui_arg,
        body_arg,
        rtf_arg,
        simulation,
        pid_node,
        stanley_node,
        evaluate_node,
    ])
