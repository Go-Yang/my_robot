#!/usr/bin/env python3
"""启动 Gazebo Sim 仿真环境 + TurtleBot3 + ros_gz 桥接。

用法：
    ros2 launch my_robot_bringup sim.launch.py
    ros2 launch my_robot_bringup sim.launch.py headless:=true
    ros2 launch my_robot_bringup sim.launch.py rviz:=true
    ros2 launch my_robot_bringup sim.launch.py model:=waffle
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('my_robot_bringup')
    tb3_dir = get_package_share_directory('turtlebot3_gazebo')

    model = LaunchConfiguration('model').perform(context)
    world = LaunchConfiguration('world').perform(context)
    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    start_rviz = _as_bool(LaunchConfiguration('rviz').perform(context))
    software_rendering = _as_bool(
        LaunchConfiguration('software_rendering').perform(context))
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    # Gazebo 通过 GZ_SIM_RESOURCE_PATH 解析 model:// 路径：
    # 本包自己的 models/ + TurtleBot3 的 models/（turtlebot3_common 网格在这里）。
    resource_path = os.pathsep.join(
        path for path in (
            os.path.join(bringup_dir, 'models'),
            os.path.join(tb3_dir, 'models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ) if path
    )

    sdf_path = os.path.join(tb3_dir, 'models', f'turtlebot3_{model}', 'model.sdf')
    urdf_path = os.path.join(tb3_dir, 'urdf', f'turtlebot3_{model}.urdf')
    if not os.path.isfile(sdf_path) or not os.path.isfile(urdf_path):
        raise RuntimeError(
            f'找不到 TurtleBot3 模型 "{model}" 的 SDF/URDF：{sdf_path} / {urdf_path}。'
            '可用型号：burger / waffle / waffle_pi'
        )
    with open(urdf_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    bridge_config = os.path.join(bringup_dir, 'config', 'bridge.yaml')
    rviz_config = os.path.join(bringup_dir, 'rviz', 'my_robot.rviz')

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
    ]

    # 虚拟机（VMware 等）没有可用的硬件 EGL/OpenGL 时，Ogre2 会初始化失败，
    # 表现为 gpu_lidar 一直不出数据（日志里是 libEGL ... failed to create dri2 screen）。
    # 强制 Mesa 软件渲染即可；真机有 GPU 时可用 software_rendering:=false 关掉。
    if software_rendering:
        actions.append(SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'))

    actions += [

        # Gazebo 服务端（-r 立即运行，-s 只起 server）
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '-v', '2', world],
            name='gz_server',
            output='screen',
            on_exit=Shutdown(),
        ),

        # 发布 TF 与 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_description,
            }],
        ),

        # Gazebo <-> ROS 2 桥接
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
            output='screen',
        ),

        # 等 server 就绪后再生成机器人，避免 create 服务未注册导致失败
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_turtlebot3',
                    arguments=[
                        '-name', f'turtlebot3_{model}',
                        '-file', sdf_path,
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.01',
                        '-Y', yaw,
                    ],
                    output='screen',
                ),
            ],
        ),
    ]

    if not headless:
        actions.append(
            ExecuteProcess(
                cmd=['gz', 'sim', '-g', '-v', '2'],
                name='gz_client',
                output='screen',
            )
        )

    if start_rviz:
        actions.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config],
                        parameters=[{'use_sim_time': True}],
                        output='screen',
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    default_world = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'worlds',
        'room_world.world',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Gazebo world 文件的绝对路径'),
        DeclareLaunchArgument(
            'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
            description='TurtleBot3 型号：burger / waffle / waffle_pi'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true 时只启动 gz server，不启动 Gazebo GUI'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true 时同时启动 RViz2'),
        DeclareLaunchArgument(
            'software_rendering', default_value='true',
            description='强制 Mesa 软件渲染（虚拟机无 3D 加速时必需；真机可设为 false）'),
        DeclareLaunchArgument(
            'x_pose', default_value='-3.5',
            description='机器人初始 x 坐标（m）'),
        DeclareLaunchArgument(
            'y_pose', default_value='2.0',
            description='机器人初始 y 坐标（m）'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='机器人初始朝向（rad）'),
        OpaqueFunction(function=_launch_setup),
    ])
