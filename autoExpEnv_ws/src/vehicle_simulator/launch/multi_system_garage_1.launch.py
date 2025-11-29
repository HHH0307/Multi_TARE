import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                           TimerAction, GroupAction)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    
    robot_1_vehicleX = LaunchConfiguration('robot_1_vehicleX')
    robot_1_vehicleY = LaunchConfiguration('robot_1_vehicleY')
    robot_1_vehicleZ = LaunchConfiguration('robot_1_vehicleZ')
    robot_1_terrainZ = LaunchConfiguration('robot_1_terrainZ')
    robot_1_vehicleYaw = LaunchConfiguration('robot_1_vehicleYaw')

    # 参数声明列表
    declare_args = [
        DeclareLaunchArgument('world_name', default_value='garage', description='仿真环境名称'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='机器人高度'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='相机Z轴偏移量'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='是否启动Gazebo GUI'),
        DeclareLaunchArgument('checkTerrainConn', default_value='false', description='是否检查地形连接'),

        DeclareLaunchArgument('robot_1_vehicleX', default_value='0.0', description='robot_1的X坐标'),
        DeclareLaunchArgument('robot_1_vehicleY', default_value='0.0', description='robot_1的Y坐标'),
        DeclareLaunchArgument('robot_1_vehicleZ', default_value='0.0', description='1机器人Z坐标'),
        DeclareLaunchArgument('robot_1_terrainZ', default_value='0.0', description='1地形Z坐标'),
        DeclareLaunchArgument('robot_1_vehicleYaw', default_value='0.0', description='机器人1偏航角'),
        
    ]

    robot_1_group = GroupAction(
        actions=[
            PushRosNamespace('robot_1'),
            
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
            #     package='rviz2',
            #     executable='rviz2',
            #     name='rviz_1',
            #     arguments=['-d', os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'robot_1_vehicle_simulator.rviz'
            #     )],
            #     output='screen'
            # )
        ]
    )


    # 启动Gazebo环境
    multi_vehicle_simulator = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'multi_vehicle_simulator_1.launch.py')),
                launch_arguments={
                    'world_name': world_name,
                    'vehicleHeight': vehicleHeight,
                    'cameraOffsetZ': cameraOffsetZ,
                    'robot_1_vehicleX': robot_1_vehicleX,
                    'robot_1_vehicleY': robot_1_vehicleY,
                    'robot_1_vehicleZ': robot_1_vehicleZ,
                    'robot_1_terrainZ': robot_1_terrainZ,
                    'robot_1_vehicleYaw': robot_1_vehicleYaw,
                    'gui': gazebo_gui,
                }.items()
            )
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

    return ld
