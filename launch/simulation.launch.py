import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('roboracer_sim')
    description_dir = get_package_share_directory('roboracer_description')

    world_file = os.path.join(pkg_dir, 'worlds', 'track.sdf')
    worlds_dir = os.path.join(pkg_dir, 'worlds')

    bridge_config = os.path.join(pkg_dir, 'config', 'bridge_config.yaml')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'roboracer.urdf.xacro')
    rviz_config = os.path.join(description_dir, 'rviz', 'roboracer.rviz')

    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=worlds_dir
    )

    ########### ARGUMENTY #########
    body_arg = DeclareLaunchArgument(
        'body',
        default_value='shell',
        choices=['shell', 'open']
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Czy uruchomić RViz',
    )
    # gui:=false → Gazebo bez okna (server-only, flaga -s)
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Czy pokazać okno Gazebo (false = headless -s)',
    )
    # RTF: 1.0 = czas rzeczywisty, 3.0 = ~3× szybciej, 0 = bez limitu (max CPU)
    rtf_arg = DeclareLaunchArgument(
        'rtf',
        default_value='1.0',
        description='Gazebo real_time_factor (1=realtime, >1 szybciej, 0=max)',
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' body:=', LaunchConfiguration('body')]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    gz_sim_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )

    ########### GAZEBO Z GUI #########
    # -r = od razu start symulacji
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    ########### GAZEBO BEZ GUI (HEADLESS) #########
    # -s = tylko serwer, bez okna
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={'gz_args': '-r -s ' + world_file}.items(),
        condition=UnlessCondition(LaunchConfiguration('gui')),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'roboracer',
            '-x', '0.0',
            '-y', '0',
            '-z', '0.3'
        ],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config, '-f', 'odom'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    ########### USTAWIENIE RTF PO STARCIE ŚWIATA #########
    # Czekamy aż Gazebo wystawi /world/track/set_physics, potem ustawiamy RTF.
    set_rtf = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg=[
                'Ustawiam Gazebo real_time_factor (RTF) = ',
                LaunchConfiguration('rtf'),
            ]),
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/track/set_physics',
                    '--reqtype', 'gz.msgs.Physics',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    ['real_time_factor: ', LaunchConfiguration('rtf')],
                ],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        gz_resource_path,
        body_arg,
        rviz_arg,
        gui_arg,
        rtf_arg,
        robot_state_publisher,
        gazebo_gui,
        gazebo_headless,
        spawn_robot,
        bridge,
        rviz,
        set_rtf,
    ])
