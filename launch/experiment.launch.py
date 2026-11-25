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

    controller_arg = DeclareLaunchArgument(
        'controller',
        default_value='pid',
        description='Regulator: pid, stanley albo pure_pursuit',
        choices=['pid', 'stanley', 'pure_pursuit'],
    )
    easy_arg = DeclareLaunchArgument(
        'easy',
        default_value='true',
        description='true = linefollower_track_easy, false = linefollower_track',
    )
    track = PythonExpression([
        "'linefollower_track_easy' if '",
        LaunchConfiguration('easy'),
        "' == 'true' else 'linefollower_track'",
    ])
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Czy uruchomić RViz',
    )
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true',
        description='Czy pokazać okno Gazebo (false = headless)',
    )
    body_arg = DeclareLaunchArgument(
        'body', default_value='shell', choices=['shell', 'open'],
        description='Wygląd nadwozia',
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='gym', choices=['classic', 'gym'],
        description='Wariant modelu: classic (STL) albo gym (GLB)',
    )
    rtf_arg = DeclareLaunchArgument(
        'rtf', default_value='1.0',
        description='Gazebo real_time_factor (1=realtime, >1 szybciej, 0=max)',
    )
    # gt = CSV /track/get_path, camera = line_perception
    path_source_arg = DeclareLaunchArgument(
        'path_source',
        default_value='gt',
        choices=['gt', 'camera'],
        description='Źródło ścieżki: gt (CSV) albo camera (line_perception)',
    )
    use_camera = PythonExpression([
        "'true' if '", LaunchConfiguration('path_source'), "' == 'camera' else 'false'",
    ])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'track': track,
            'rviz': LaunchConfiguration('rviz'),
            'gui': LaunchConfiguration('gui'),
            'body': LaunchConfiguration('body'),
            'model': LaunchConfiguration('model'),
            'rtf': LaunchConfiguration('rtf'),
        }.items(),
    )

    line_perception_node = Node(
        package='roboracer_sim',
        executable='line_perception',
        name='line_perception',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('path_source'), "' == 'camera'"
            ])
        ),
    )

    pid_node = Node(
        package='roboracer_sim',
        executable='roboracer_pid',
        name='roboracer_pid',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'use_camera': use_camera,
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('controller'), "' == 'pid'"
            ])
        ),
    )

    stanley_node = Node(
        package='roboracer_sim',
        executable='control_stanley',
        name='control_stanley',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'path_source': LaunchConfiguration('path_source'),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('controller'), "' == 'stanley'"
            ])
        ),
    )

    pure_pursuit_node = Node(
        package='roboracer_sim',
        executable='pure_pursuit',
        name='pure_pursuit',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'path_source': LaunchConfiguration('path_source'),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('controller'), "' == 'pure_pursuit'"
            ])
        ),
    )

    evaluate_node = Node(
        package='roboracer_sim',
        executable='evaluate_run',
        name='evaluate_run',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'controller_name': LaunchConfiguration('controller'),
            'track': track,
            'odom_topic': '/odom_gt',
            'report_period': 10.0,
        }],
    )

    return LaunchDescription([
        controller_arg,
        easy_arg,
        rviz_arg,
        gui_arg,
        body_arg,
        model_arg,
        rtf_arg,
        path_source_arg,
        simulation,
        line_perception_node,
        pid_node,
        stanley_node,
        pure_pursuit_node,
        evaluate_node,
    ])
