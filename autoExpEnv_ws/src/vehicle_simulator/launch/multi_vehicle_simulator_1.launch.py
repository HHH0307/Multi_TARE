import os
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, GroupAction)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def declare_world_action(context, world_name):
    world_name_str = str(world_name.perform(context))
    declare_world = DeclareLaunchArgument('world', default_value=[os.path.join(get_package_share_directory('vehicle_simulator'), 'world', world_name_str + '.world')], description='')
    return [declare_world]

def generate_launch_description():
    # 声明所有启动参数（全局共享）
    robot_1 = LaunchConfiguration('robot_1')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    
    # robot_1位置参数
    robot_1_vehicleX = LaunchConfiguration('robot_1_vehicleX')
    robot_1_vehicleY = LaunchConfiguration('robot_1_vehicleY')
    robot_1_vehicleZ = LaunchConfiguration('robot_1_vehicleZ')
    robot_1_terrainZ = LaunchConfiguration('robot_1_terrainZ')
    robot_1_vehicleYaw = LaunchConfiguration('robot_1_vehicleYaw')
    
    # 其他共享参数
    terrainVoxelSize = LaunchConfiguration('terrainVoxelSize')
    groundHeightThre = LaunchConfiguration('groundHeightThre')
    adjustZ = LaunchConfiguration('adjustZ')
    terrainRadiusZ = LaunchConfiguration('terrainRadiusZ')
    minTerrainPointNumZ = LaunchConfiguration('minTerrainPointNumZ')
    smoothRateZ = LaunchConfiguration('smoothRateZ')
    adjustIncl = LaunchConfiguration('adjustIncl')
    terrainRadiusIncl = LaunchConfiguration('terrainRadiusIncl')
    minTerrainPointNumIncl = LaunchConfiguration('minTerrainPointNumIncl')
    smoothRateIncl = LaunchConfiguration('smoothRateIncl')
    InclFittingThre = LaunchConfiguration('InclFittingThre')
    maxIncl = LaunchConfiguration('maxIncl')
    pause = LaunchConfiguration('pause')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    record = LaunchConfiguration('record')
    verbose = LaunchConfiguration('verbose')
    world_name = LaunchConfiguration('world_name')

    # 参数声明（全局）
    declare_args = [
        DeclareLaunchArgument('robot_1', default_value='robot_1', description=''),
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description=''),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description=''),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description=''),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_1_vehicleX', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_1_vehicleY', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_1_vehicleZ', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_1_terrainZ', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_1_vehicleYaw', default_value='0.0', description=''),
        DeclareLaunchArgument('terrainVoxelSize', default_value='0.05', description=''),
        DeclareLaunchArgument('groundHeightThre', default_value='0.1', description=''),
        DeclareLaunchArgument('adjustZ', default_value='true', description=''),
        DeclareLaunchArgument('terrainRadiusZ', default_value='1.0', description=''),
        DeclareLaunchArgument('minTerrainPointNumZ', default_value='5', description=''),
        DeclareLaunchArgument('smoothRateZ', default_value='0.5', description=''),
        DeclareLaunchArgument('adjustIncl', default_value='true', description=''),
        DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0', description=''),
        DeclareLaunchArgument('minTerrainPointNumIncl', default_value='200', description=''),
        DeclareLaunchArgument('smoothRateIncl', default_value='0.5', description=''),
        DeclareLaunchArgument('InclFittingThre', default_value='0.2', description=''),
        DeclareLaunchArgument('maxIncl', default_value='30.0', description=''),
        DeclareLaunchArgument('pause', default_value='false', description=''),
        DeclareLaunchArgument('use_sim_time', default_value='true', description=''),
        DeclareLaunchArgument('gui', default_value='false', description=''),
        DeclareLaunchArgument('record', default_value='false', description=''),
        DeclareLaunchArgument('verbose', default_value='false', description=''),
        DeclareLaunchArgument('world_name', default_value='garage', description='')
    ]

    robot_1_group = GroupAction(
        actions=[
            PushRosNamespace('robot_1'),  

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher', 
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'robot_description': Command([
                        'xacro',' ', os.path.join(
                            get_package_share_directory('vehicle_simulator'), 
                            'urdf', 'multi_lidar.urdf.xacro'
                        )
                    ])
                }]
            ),
            
            Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot_1_lidar', 
                    '-topic', '/robot_1/robot_description',
                    '-robot_namespace', 'robot_1'
                ],
                output='screen'
            ),
            
            Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=[
                    '-file', os.path.join(
                        get_package_share_directory('vehicle_simulator'), 
                        'urdf', 'multi_robot.sdf'
                    ),
                    '-entity', 'robot_1_robot',
                    '-robot_namespace', 'robot_1'
                ],
                output='screen'
            ),
            
            Node(
                package='gazebo_ros', 
                executable='spawn_entity.py',
                arguments=[
                    '-file', os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'multi_camera.urdf.xacro'),
                    '-entity', 'robot_1_camera',
                    '-robot_namespace', 'robot_1'
                ],
                output='screen'
            ),
            
            Node(
                package='vehicle_simulator', 
                executable='multi_vehicleSimulator',
                parameters=[{
                    'robot_name': robot_1,
                    'use_gazebo_time': False,
                    'sensorOffsetX': sensorOffsetX,
                    'sensorOffsetY': sensorOffsetY,
                    'vehicleHeight': vehicleHeight,
                    'cameraOffsetZ': cameraOffsetZ,
                    'vehicleX': robot_1_vehicleX,
                    'vehicleY': robot_1_vehicleY,
                    'vehicleZ': robot_1_vehicleZ,
                    'terrainZ': robot_1_terrainZ,
                    'vehicleYaw': robot_1_vehicleYaw,
                    'terrainVoxelSize': terrainVoxelSize,
                    'groundHeightThre': groundHeightThre,
                    'adjustZ': adjustZ,
                    'terrainRadiusZ': terrainRadiusZ,
                    'minTerrainPointNumZ': minTerrainPointNumZ,
                    'smoothRateZ': smoothRateZ,
                    'adjustIncl': adjustIncl,
                    'terrainRadiusIncl': terrainRadiusIncl,
                    'minTerrainPointNumIncl': minTerrainPointNumIncl,
                    'smoothRateIncl': smoothRateIncl,
                    'InclFittingThre': InclFittingThre,
                    'maxIncl': maxIncl,
                    'use_sim_time': use_sim_time
                }],
                output='screen'
            )
        ]
    )

    # 全局Gazebo环境
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )
    # 构建启动描述
    ld = LaunchDescription()
    
    # 添加所有参数声明
    for arg in declare_args:
        ld.add_action(arg)
    
    # 声明世界参数
    ld.add_action(OpaqueFunction(function=declare_world_action, args=[world_name]))
    
    # 启动全局Gazebo
    ld.add_action(start_gazebo)
    
    # 添加两个机器人分组（含所有节点）
    ld.add_action(robot_1_group)

    return ld