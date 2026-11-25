#!/usr/bin/env python3
"""启动 Gazebo Sim + TurtleBot 4（官方 Clearpath 模型）。

默认使用 TB4 Lite，在 room_world.world 里出生。复用官方
turtlebot4_description / irobot_create_gz_bringup / irobot_create_nodes
组件，但只桥接本项目需要的话题：

    /scan   RPLIDAR A1M8（gz -> ROS）
    /imu    Create3 IMU（gz -> ROS，上游默认没有桥）
    /clock

/odom、/tf、/joint_states 由 gz_ros2_control 的 diff_drive_controller /
joint_state_broadcaster 直接发布，不经过 ros_gz_bridge。

用法：
    ros2 launch my_robot_bringup sim_turtlebot4.launch.py
    ros2 launch my_robot_bringup sim_turtlebot4.launch.py headless:=true rviz:=true
    ros2 launch my_robot_bringup sim_turtlebot4.launch.py tb4_model:=standard
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

ROBOT_NAME = 'turtlebot4'


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _read_world_name(world_path: str) -> str:
    """从 SDF world 文件里取 <world name="...">，gz 话题路径要用它。"""
    try:
        with open(world_path, 'r', errors='replace') as world_file:
            text = world_file.read()
    except OSError:
        return 'room_world'

    match = re.search(r'<world\b[^>]*\bname\s*=\s*"([^"]+)"', text)
    if match is None:
        match = re.search(r"<world\b[^>]*\bname\s*=\s*'([^']+)'", text)
    return match.group(1) if match else 'room_world'


def _launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('my_robot_bringup')
    tb4_description_dir = get_package_share_directory('turtlebot4_description')
    create_description_dir = get_package_share_directory('irobot_create_description')
    create_gz_bringup_dir = get_package_share_directory('irobot_create_gz_bringup')

    world = LaunchConfiguration('world').perform(context)
    world_name = _read_world_name(world)
    model = LaunchConfiguration('tb4_model').perform(context)
    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    start_rviz = _as_bool(LaunchConfiguration('rviz').perform(context))
    gui_software = _as_bool(
        LaunchConfiguration('gui_software_rendering').perform(context))
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    z_pose = str(float(LaunchConfiguration('z_pose').perform(context)) - 0.0025)

    # Gazebo 通过 GZ_SIM_RESOURCE_PATH 解析 package:// 网格；官方包要求把
    # share 目录本身（包的上一级）加进来。
    resource_path = os.pathsep.join(
        path for path in (
            os.path.join(bringup_dir, 'models'),
            os.path.dirname(tb4_description_dir),
            os.path.dirname(create_description_dir),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ) if path
    )

    imu_gz_topic = (
        f'/world/{world_name}/model/{ROBOT_NAME}'
        '/link/imu_link/sensor/imu/imu')

    create3_bridge_launch = os.path.join(
        create_gz_bringup_dir, 'launch', 'create3_ros_gz_bridge.launch.py')
    create3_nodes_launch = os.path.join(
        get_package_share_directory('irobot_create_common_bringup'),
        'launch', 'create3_nodes.launch.py')
    create3_gz_nodes_launch = os.path.join(
        create_gz_bringup_dir, 'launch', 'create3_gz_nodes.launch.py')

    # 用本包的模型副本（去掉了 OAK-D 相机，见 xacro 头部说明）。
    # xacro 里的 package:// 网格仍从 turtlebot4_description 解析。
    urdf_path = os.path.join(
        bringup_dir, 'urdf', f'turtlebot4_{model}_nocam.urdf.xacro')
    if not os.path.isfile(urdf_path):
        raise RuntimeError(f'找不到 TurtleBot4 模型文件：{urdf_path}')
    # 包装脚本会去掉 gpu_lidar 传感器（改用 world_lidar 合成扫描）以及
    # OAK-D 相机/相机支架；详见 tb4_urdf.py 头部说明。
    urdf_script = os.path.join(bringup_dir, 'scripts', 'tb4_urdf.py')
    world_lidar_script = os.path.join(
        bringup_dir, 'scripts', 'world_lidar.py')

    rviz_config = os.path.join(bringup_dir, 'rviz', 'my_robot.rviz')

    print(f'[sim_turtlebot4.launch] world={world} (name={world_name})')
    print(f'[sim_turtlebot4.launch] model={model}, '
          f'scan=world_lidar(CPU raycast), imu={imu_gz_topic}')

    # 只给 GUI / RViz 这两个显示进程设置软件渲染。无 3D 加速的 VM 必须设，
    # 否则窗口会闪一下变黑；WSL2/有 3D 的机器传 gui_software_rendering:=false，
    # 让它们走硬件 OpenGL。
    display_env = {'LIBGL_ALWAYS_SOFTWARE': '1'} if gui_software else {}

    actions = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
    ]

    actions += [
        # Gazebo 服务端（-r 立即运行，-s 只起 server）
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-s', '-v', '2', world],
            name='gz_server',
            output='screen',
            on_exit=Shutdown(),
        ),

        # 仿真时钟
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),

        # TB4 URDF -> /robot_description
        # 不要用官方 turtlebot4_description 的 robot_description.launch.py：
        # 它默认把 xacro 的 namespace 设成 robot_name（turtlebot4），
        # gz_ros2_control 会去等 /turtlebot4/robot_description，而
        # robot_state_publisher 在根命名空间，controller_manager 会一直
        # 初始化不了、/odom 没有发布者。这里自己起 RSP，namespace 留空。
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': Command(
                    ['python3 ', urdf_script, ' xacro ', urdf_path,
                     ' gazebo:=ignition']),
            }],
        ),

        # /imu：官方 TB4 仿真没有桥 IMU，这里补一条。
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            arguments=[
                imu_gz_topic + '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            remappings=[(imu_gz_topic, 'imu')],
            output='screen',
        ),

        # 激光不再用 gz 的 gpu_lidar：llvmpipe 下它会连续几十秒只输出 inf，
        # 导航只能停车。这里用 world 几何 + Gazebo 真值位姿在 CPU 上做
        # 2D 射线投射，稳定发布 /scan（10 Hz，360 点）。
        ExecuteProcess(
            cmd=['python3', world_lidar_script,
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-p', f'world_file:={world}'],
            name='world_lidar',
            output='screen',
        ),

        # Create3 底盘桥接：cmd_vel、碰撞、悬崖/红外（motion_control 用）、
        # 底盘真值位姿。world 必须传对，否则 gz 话题路径前缀不匹配。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(create3_bridge_launch),
            launch_arguments={
                'world': world_name,
                'robot_name': ROBOT_NAME,
                'dock_name': 'standard_dock',
                'namespace': '',
                'use_sim_time': 'true',
            }.items(),
        ),

        # 等 server 就绪并生成机器人，避免 create 服务未注册导致失败。
        # 这台 VM 启动 gz server 偶尔要 4-5 s，留足时间。
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    name='spawn_turtlebot4',
                    arguments=[
                        '-name', ROBOT_NAME,
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', z_pose,
                        '-Y', yaw,
                        '-topic', 'robot_description',
                    ],
                    output='screen',
                ),
            ],
        ),

        # gz_ros2_control 的 controller_manager 由模型里的插件加载，
        # 晚一点再起 spawner / motion_control，留出模型加载时间。
        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(create3_nodes_launch),
                    launch_arguments={
                        'namespace': '',
                        'gazebo': 'ignition',
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(create3_gz_nodes_launch),
                    launch_arguments={
                        'robot_name': ROBOT_NAME,
                        'dock_name': 'standard_dock',
                    }.items(),
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
                additional_env=display_env,
            )
        )

    if start_rviz:
        actions.append(
            TimerAction(
                period=8.0,
                actions=[
                    ExecuteProcess(
                        cmd=['rviz2', '-d', rviz_config,
                             '--ros-args', '-p', 'use_sim_time:=true'],
                        name='rviz2',
                        output='screen',
                        additional_env=display_env,
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
            'tb4_model', default_value='lite',
            choices=['lite', 'standard'],
            description='TurtleBot4 型号：lite / standard'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true 时只启动 gz server，不启动 Gazebo GUI'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true 时同时启动 RViz2'),
        DeclareLaunchArgument(
            'gui_software_rendering', default_value='true',
            description='给 Gazebo GUI/RViz 强制 Mesa 软件渲染；'
                        'WSL2 或有 3D 加速时设 false'),
        DeclareLaunchArgument(
            'x_pose', default_value='-3.5',
            description='机器人初始 x 坐标（m）'),
        DeclareLaunchArgument(
            'y_pose', default_value='2.0',
            description='机器人初始 y 坐标（m）'),
        DeclareLaunchArgument(
            'z_pose', default_value='0.0',
            description='机器人初始 z 坐标（m）'),
        DeclareLaunchArgument(
            'yaw', default_value='0.0',
            description='机器人初始朝向（rad）'),
        OpaqueFunction(function=_launch_setup),
    ])
