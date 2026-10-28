#!/usr/bin/env python3
"""一键启动仿真 + 本项目主节点。

    ros2 launch my_robot_bringup bringup.launch.py
    ros2 launch my_robot_bringup bringup.launch.py headless:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_robot_bringup')
    sim_launch = os.path.join(bringup_dir, 'launch', 'sim.launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')
    software_rendering = LaunchConfiguration('software_rendering')
    explore = LaunchConfiguration('explore')
    mapping_backend = LaunchConfiguration('mapping_backend')
    slam_params_file = LaunchConfiguration('slam_params_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'model': model,
            'world': world,
            'headless': headless,
            'rviz': rviz,
            'software_rendering': software_rendering,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    # 没有指定 name，launch_ros 会把参数写到 "/**"，对进程内所有节点生效
    # （main_node 里同时创建 mapping_node 和 sensor_node）
    # 注意：不要给这个 Node 指定 name，否则 --ros-args -r __node:=... 会把
    # 进程内所有节点（mapping_node / sensor_node）都改成同一个名字。
    robot_node = Node(
        package='my_robot_project',
        executable='main_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'mapping_backend': ParameterValue(mapping_backend, value_type=str),
            # SLAM 模式降低速度，避免扫描匹配跟不上导致叠图
            'max_linear_speed': ParameterValue(
                PythonExpression([
                    "0.15 if '", mapping_backend, "' == 'slam_toolbox' else 0.30"]),
                value_type=float),
            'max_angular_speed': ParameterValue(
                PythonExpression([
                    "0.50 if '", mapping_backend, "' == 'slam_toolbox' else 0.90"]),
                value_type=float),
        }],
    )

    # 可选：外部 slam_toolbox 节点（mapping_backend:=slam_toolbox 时启动）。
    # 必须等第一帧 /scan 真正到达后再启动；与仿真同时启动时 slam_toolbox
    # 会一直停在 0x0 地图（实测问题）。wait_for_scan 收到一帧后退出，
    # 再用 OnProcessExit 触发 slam_toolbox。
    wait_scan = ExecuteProcess(
        cmd=['timeout', '60', 'python3',
             os.path.join(bringup_dir, 'scripts', 'wait_for_scan.py')],
        output='screen',
        condition=IfCondition(PythonExpression(
            ["'", mapping_backend, "' == 'slam_toolbox'"])),
    )

    slam_toolbox_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
            # 关闭官方 launch 的一次性生命周期激活；高负载下它可能因服务
            # 响应超时失败。下面用带重试的脚本完成 configure/activate。
            'autostart': 'false',
        }.items(),
    )

    activate_slam = ExecuteProcess(
        cmd=['bash',
             os.path.join(bringup_dir, 'scripts', 'activate_node.sh'),
             '/slam_toolbox'],
        output='screen',
    )

    slam_toolbox_launch = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_scan,
            on_exit=[
                TimerAction(period=5.0, actions=[
                    slam_toolbox_include,
                    TimerAction(period=3.0, actions=[activate_slam]),
                ]),
            ],
        ),
    )

    # 可选：前沿自动探索，通过 /goal_pose 驱动已有的 NavigationModule
    explorer_node = Node(
        package='navigation_lib',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(explore),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='使用 Gazebo 仿真时钟'),
        DeclareLaunchArgument(
            'model', default_value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
            description='TurtleBot3 型号：burger / waffle / waffle_pi'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_dir, 'worlds', 'room_world.world'),
            description='Gazebo world 文件的绝对路径'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'software_rendering', default_value='true',
            description='强制 Mesa 软件渲染；关闭 3D 加速时必需，开启 3D 后可设 false'),
        DeclareLaunchArgument(
            'explore', default_value='false',
            description='true 时启动 frontier_explorer 自动探索（不要同时遥控）'),
        DeclareLaunchArgument(
            'mapping_backend', default_value='odom_grid',
            description='建图后端：odom_grid / slam_toolbox / custom_slam'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(bringup_dir, 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox 参数文件'),
        DeclareLaunchArgument('x_pose', default_value='-3.5'),
        DeclareLaunchArgument('y_pose', default_value='2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        sim,
        wait_scan,
        slam_toolbox_launch,
        robot_node,
        explorer_node,
    ])
