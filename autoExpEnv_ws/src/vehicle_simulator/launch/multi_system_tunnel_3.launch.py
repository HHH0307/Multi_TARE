import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                           TimerAction, GroupAction)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明启动参数
    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    
    # 机器人0参数
    robot_1_vehicleX = LaunchConfiguration('robot_1_vehicleX')
    robot_1_vehicleY = LaunchConfiguration('robot_1_vehicleY')
    robot_1_vehicleZ = LaunchConfiguration('robot_1_vehicleZ')
    robot_1_terrainZ = LaunchConfiguration('robot_1_terrainZ')
    robot_1_vehicleYaw = LaunchConfiguration('robot_1_vehicleYaw')
    
    # 机器人1参数
    robot_2_vehicleX = LaunchConfiguration('robot_2_vehicleX')
    robot_2_vehicleY = LaunchConfiguration('robot_2_vehicleY')
    robot_2_vehicleZ = LaunchConfiguration('robot_2_vehicleZ')
    robot_2_terrainZ = LaunchConfiguration('robot_2_terrainZ')
    robot_2_vehicleYaw = LaunchConfiguration('robot_2_vehicleYaw')

    # 机器人2参数
    robot_3_vehicleX = LaunchConfiguration('robot_3_vehicleX')
    robot_3_vehicleY = LaunchConfiguration('robot_3_vehicleY')
    robot_3_vehicleZ = LaunchConfiguration('robot_3_vehicleZ')
    robot_3_terrainZ = LaunchConfiguration('robot_3_terrainZ')
    robot_3_vehicleYaw = LaunchConfiguration('robot_3_vehicleYaw')
    
    # 参数声明列表
    declare_args = [
        DeclareLaunchArgument('world_name', default_value='tunnel', description='仿真环境名称'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='机器人高度'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='相机Z轴偏移量'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='是否启动Gazebo GUI'),
        DeclareLaunchArgument('checkTerrainConn', default_value='false', description='是否检查地形连接'),
        
        # 机器人0参数
        DeclareLaunchArgument('robot_1_vehicleX', default_value='0.0', description='robot_1的X坐标'),
        DeclareLaunchArgument('robot_1_vehicleY', default_value='0.0', description='robot_1的Y坐标'),
        DeclareLaunchArgument('robot_1_vehicleZ', default_value='0.0', description='0机器人Z坐标'),
        DeclareLaunchArgument('robot_1_terrainZ', default_value='0.0', description='0地形Z坐标'),
        DeclareLaunchArgument('robot_1_vehicleYaw', default_value='0.0', description='机器人0偏航角'),
        
        # 机器人1参数
        DeclareLaunchArgument('robot_2_vehicleX', default_value='2.0', description='robot_2的X坐标'),
        DeclareLaunchArgument('robot_2_vehicleY', default_value='0.0', description='robot_2的Y坐标'),
        DeclareLaunchArgument('robot_2_vehicleZ', default_value='0.0', description='1机器人Z坐标'),
        DeclareLaunchArgument('robot_2_terrainZ', default_value='0.0', description='1地形Z坐标'),
        DeclareLaunchArgument('robot_2_vehicleYaw', default_value='0.0', description='机器人1偏航角'),

        # 机器人1参数
        DeclareLaunchArgument('robot_3_vehicleX', default_value='0.0', description='robot_3的X坐标'),
        DeclareLaunchArgument('robot_3_vehicleY', default_value='1.0', description='robot_3的Y坐标'),
        DeclareLaunchArgument('robot_3_vehicleZ', default_value='0.0', description='2机器人Z坐标'),
        DeclareLaunchArgument('robot_3_terrainZ', default_value='0.0', description='2地形Z坐标'),
        DeclareLaunchArgument('robot_3_vehicleYaw', default_value='0.0', description='机器人2偏航角')
    ]

    # 启动Gazebo环境
    multi_vehicle_simulator = TimerAction(
        period=0.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'multi_vehicle_simulator_3.launch.py')),
                launch_arguments={
                    'world_name': world_name,
                    'vehicleHeight': vehicleHeight,
                    'cameraOffsetZ': cameraOffsetZ,
                    'robot_1_vehicleX': robot_1_vehicleX,
                    'robot_1_vehicleY': robot_1_vehicleY,
                    'robot_1_vehicleZ': robot_1_vehicleZ,
                    'robot_1_terrainZ': robot_1_terrainZ,
                    'robot_1_vehicleYaw': robot_1_vehicleYaw,
                    'robot_2_vehicleX': robot_2_vehicleX,
                    'robot_2_vehicleY': robot_2_vehicleY,
                    'robot_2_vehicleZ': robot_2_vehicleZ,
                    'robot_2_terrainZ': robot_2_terrainZ,
                    'robot_2_vehicleYaw': robot_2_vehicleYaw,
                    'robot_3_vehicleX': robot_3_vehicleX,
                    'robot_3_vehicleY': robot_3_vehicleY,
                    'robot_3_vehicleZ': robot_3_vehicleZ,
                    'robot_3_terrainZ': robot_3_terrainZ,
                    'robot_3_vehicleYaw': robot_3_vehicleYaw,
                    'gui': gazebo_gui,
                }.items()
            )
        ]
    )

    # ------------------------------
    # Robot 1 命名空间组（使用GroupAction隔离）
    # ------------------------------
    robot_1_group = TimerAction(
        period=2.0,
        actions=[
            GroupAction(
                actions=[
                    # 推入命名空间，该组内所有节点都会在这个命名空间下
                    PushRosNamespace('robot_1'),
                    
                    # 本地规划器
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('local_planner'), 'launch', 'multi_local_planner.launch')),
                        launch_arguments={
                            'cameraOffsetZ': cameraOffsetZ,
                            'goalX': robot_1_vehicleX,
                            'goalY': robot_1_vehicleY,
                            'robot_name': 'robot_1'
                        }.items()
                    ),
                    
                    # 地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis'), 'launch', 'multi_terrain_analysis.launch')),
                        launch_arguments={'robot_name': 'robot_1'}.items()
                    ),
                    
                    # 扩展地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis_ext'), 'launch', 'multi_terrain_analysis_ext.launch')),
                        launch_arguments={
                            'checkTerrainConn': checkTerrainConn,
                            'robot_name': 'robot_1'
                        }.items()
                    ),
                    
                    # 传感器扫描生成
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('sensor_scan_generation'), 'launch', 'multi_sensor_scan_generation.launch')),
                        launch_arguments={'robot_name': 'robot_1'}.items()
                    ),
                    
                    # 可视化工具
                    IncludeLaunchDescription(FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('visualization_tools'), 'launch', 'multi_visualization_tools.launch')),
                        launch_arguments={
                            'world_name': world_name,
                            'robot_name': 'robot_1'
                        }.items()
                    ),

                    # Node(
                    #     package='joy', 
                    #     executable='joy_node',
                    #     name='ps3_joy',
                    #     output='screen',
                    #     parameters=[{
                    #         'robot_name': 'robot_1',
                    #         'dev': "/dev/input/js0",
                    #         'deadzone': 0.12,
                    #         'autorepeat_rate': 0.0,
                    #     }]
                    # ),
                    
                    # Node(
                    #     package='rviz2',
                    #     executable='rviz2',
                    #     name='rviz_1',
                    #     arguments=['-d', os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'robot_1_vehicle_simulator.rviz'
                    #     )],
                    #     output='screen'
                    # )
                ]
            )
        ]
    )

    # ------------------------------
    # Robot 2 命名空间组（使用GroupAction隔离）
    # ------------------------------
    robot_2_group = TimerAction(
        period=2.0,  # 延迟启动，与robot_1错开
        actions=[
            GroupAction(
                actions=[
                    # 推入robot_2命名空间
                    PushRosNamespace('robot_2'),
                    
                    # 本地规划器
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('local_planner'), 'launch', 'multi_local_planner.launch')),
                        launch_arguments={
                            'cameraOffsetZ': cameraOffsetZ,
                            'goalX': robot_2_vehicleX,
                            'goalY': robot_2_vehicleY,
                            'robot_name': 'robot_2'
                        }.items()
                    ),
                    
                    # 地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis'), 'launch', 'multi_terrain_analysis.launch')),
                        launch_arguments={'robot_name': 'robot_2'}.items()
                    ),
                    
                    # 扩展地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis_ext'), 'launch', 'multi_terrain_analysis_ext.launch')),
                        launch_arguments={
                            'checkTerrainConn': checkTerrainConn,
                            'robot_name': 'robot_2'
                        }.items()
                    ),
                    
                    # 传感器扫描生成
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('sensor_scan_generation'), 'launch', 'multi_sensor_scan_generation.launch')),
                        launch_arguments={'robot_name': 'robot_2'}.items()
                    ),
                    
                    # 可视化工具
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('visualization_tools'), 'launch', 'multi_visualization_tools.launch')),
                        launch_arguments={
                            'world_name': world_name,
                            'robot_name': 'robot_2'
                        }.items()
                    ),

                    # Node(
                    #     package='rviz2',
                    #     executable='rviz2',
                    #     name='rviz_2',
                    #     arguments=['-d', os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'robot_2_vehicle_simulator.rviz'
                    #     )],
                    #     output='screen'
                    # )
                ]
            )
        ]
    )

    # ------------------------------
    # Robot 3 命名空间组（使用GroupAction隔离）
    # ------------------------------
    robot_3_group = TimerAction(
        period=2.0,  
        actions=[
            GroupAction(
                actions=[
                    # 推入robot_3命名空间
                    PushRosNamespace('robot_3'),
                    
                    # 本地规划器
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('local_planner'), 'launch', 'multi_local_planner.launch')),
                        launch_arguments={
                            'cameraOffsetZ': cameraOffsetZ,
                            'goalX': robot_3_vehicleX,
                            'goalY': robot_3_vehicleY,
                            'robot_name': 'robot_3'
                        }.items()
                    ),
                    
                    # 地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis'), 'launch', 'multi_terrain_analysis.launch')),
                        launch_arguments={'robot_name': 'robot_3'}.items()
                    ),
                    
                    # 扩展地形分析
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('terrain_analysis_ext'), 'launch', 'multi_terrain_analysis_ext.launch')),
                        launch_arguments={
                            'checkTerrainConn': checkTerrainConn,
                            'robot_name': 'robot_3'
                        }.items()
                    ),
                    
                    # 传感器扫描生成
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('sensor_scan_generation'), 'launch', 'multi_sensor_scan_generation.launch')),
                        launch_arguments={'robot_name': 'robot_3'}.items()
                    ),
                    
                    # 可视化工具
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(get_package_share_directory('visualization_tools'), 'launch', 'multi_visualization_tools.launch')),
                        launch_arguments={
                            'world_name': world_name,
                            'robot_name': 'robot_3'
                        }.items()
                    ),

                    # Node(
                    #     package='rviz2',
                    #     executable='rviz2',
                    #     name='rviz_3',
                    #     arguments=['-d', os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'robot_3_vehicle_simulator.rviz'
                    #     )],
                    #     output='screen'
                    # )
                ]
            )
        ]
    )

    robot_1_to_robot_2_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_1_to_robot_2_tf_broadcaster',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'robot_1/map',
            '--child-frame-id', 'robot_2/map'
        ]
    )
    robot_1_to_robot_3_tf_broadcaster = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='robot_1_to_robot_3_tf_broadcaster',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'robot_1/map',
            '--child-frame-id', 'robot_3/map'
        ]
    )
    
    # 构建启动描述
    ld = LaunchDescription()
    
    # 添加参数声明
    for arg in declare_args:
        ld.add_action(arg)
    
    # 添加环境和机器人节点
    ld.add_action(multi_vehicle_simulator)
    ld.add_action(robot_1_group)
    ld.add_action(robot_2_group)
    ld.add_action(robot_3_group)
    
    # 添加其他节点
    # ld.add_action(start_joy)
    ld.add_action(robot_1_to_robot_2_tf_broadcaster)
    ld.add_action(robot_1_to_robot_3_tf_broadcaster)

    return ld
