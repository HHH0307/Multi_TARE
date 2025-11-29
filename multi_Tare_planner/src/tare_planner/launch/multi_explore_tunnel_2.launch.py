import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 声明启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time (true/false)'
    ) 
    rviz_arg = DeclareLaunchArgument(
        'rviz', 
        default_value='false', 
        description='Launch RViz'
    )
    use_boundary_arg = DeclareLaunchArgument(
        'use_boundary', 
        default_value='false', 
        description='Use boundary for navigation'
    )

    # 机器人1分组（命名空间 robot_1）
    robot_1_Group = GroupAction(
        actions=[
            PushRosNamespace('robot_1'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('tare_planner'), 
                    '/multi_explore.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_num': '2',
                    'robot_name': 'robot_1',  
                    'scenario': 'tunnel',
                    'rviz': 'false' 
                }.items()
            )
        ]
    )

    # 机器人2分组（命名空间 robot_2）
    robot_2_Group = GroupAction(
        actions=[
            PushRosNamespace('robot_2'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('tare_planner'), 
                    '/multi_explore.launch.py'
                ]),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_num': '2',
                    'robot_name': 'robot_2',
                    'scenario': 'tunnel',
                    'rviz': 'false'
                }.items()
            )
        ]
    )

    # 边界导航节点
    navigation_boundary_node = Node(
        package='tare_planner',
        executable='navigationBoundary',
        name='navigationBoundary',
        output='screen',
        parameters=[
            {'boundary_file_dir': f"{get_package_share_directory('tare_planner')}/boundary.ply"},
            {'sendBoundary': True},
            {'sendBoundaryInterval': 2}
        ],
        condition=IfCondition(LaunchConfiguration('use_boundary'))
    )

    # 全局RViz节点（只启动一次）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_ground_rviz',
        arguments=[
            '-d', f"{get_package_share_directory('tare_planner')}/multi_tare_planner_ground.rviz"
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_arg,
        use_boundary_arg,
        robot_1_Group,
        robot_2_Group,
        navigation_boundary_node,
        rviz_node
    ])
