"""
multi_system_env.launch.py

说明：
    本文件用于在 ROS 2 中启动多车仿真系统（vehicle_simulator + 多个子模块）。
    它会根据传入的启动参数创建若干命名空间（例如 robot_1、robot_2 ...），
    并在每个命名空间中包含 local_planner、terrain_analysis、sensor_scan_generation
    和 visualization_tools 等子 launch。脚本也会启动一个 multi_vehicle_simulator
    子 launch 来管理 Gazebo 中的车辆模型。

常见使用步骤：
    1. 在工作区根目录构建并 source：
         source /opt/ros/<ros2_distro>/setup.bash
         source install/setup.bash
    2. 使用 ros2 launch 启动：
         ros2 launch vehicle_simulator multi_system_env.launch.py [参数]

常用参数示例：
    robot_count（默认3）、robot_columns、robot_name_prefix、robot_positions（详见下文）、
    robot_start_x/y/z、robot_spacing_x/y、gazebo_gui、world_name 等。

robot_positions 格式：
    "x,y,z,terrainZ,yaw; x,y,z,terrainZ,yaw; ..."
    示例："0.0,0.0,0.0,0.0,0.0; 2.0,0.0,0.0,0.0,90.0"

注意：若提供 `robot_positions`，脚本优先使用显式位置；否则按起始坐标+网格间距自动排列。

完整示例：

1) 网格布局示例（不显式列出每个位置，按起始坐标与间距自动排列 4 台车）：

     ```bash
     ros2 launch vehicle_simulator multi_system_env.launch.py \
         robot_count:=4 \
         robot_columns:=2 \
         robot_name_prefix:=robot_ \
         robot_start_x:=0.0 \
         robot_start_y:=0.0 \
         robot_spacing_x:=2.0 \
         robot_spacing_y:=1.5 \
         gazebo_gui:=true \
         world_name:=campus
     ```

     - 效果：创建 `robot_1`..`robot_4`，按 2 列网格排列，间距按 `robot_spacing_x/robot_spacing_y`。

2) 显式位置示例（为每台车指定位置与偏航角）：

     ```bash
     ros2 launch vehicle_simulator multi_system_env.launch.py \
         robot_count:=5 \
         gazebo_gui:=true \
         robot_positions:="0.0,0.0,0.0,0.0,0.0; 2.0,0.0,0.0,0.0,90.0; 4.0,0.0,0.0,0.0,180.0; 6.0,0.0,0.0,0.0,270.0; 8.0,0.0,0.0,0.0,0.0" \
         world_name:=campus
     ```

     - 格式说明：每个条目为 `x,y,z,terrainZ,yaw`，条目间以分号分隔；脚本将按条目顺序为 `robot_1`、`robot_2` ... 分配位置。
"""
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
    OpaqueFunction,
)
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from ament_index_python.packages import get_package_share_directory


def _launch_value(context, name):
    return str(LaunchConfiguration(name).perform(context))


def _launch_int(context, name):
    return int(float(_launch_value(context, name)))


def _launch_float(context, name):
    return float(_launch_value(context, name))


def _parse_robot_positions(raw_positions):
    positions = []
    for entry in raw_positions.split(';'):
        entry = entry.strip()
        if not entry:
            continue

        values = [value.strip() for value in entry.split(',')]
        if len(values) < 5:
            raise ValueError('robot_positions 每个条目至少需要 5 个值: x,y,z,terrainZ,yaw')

        positions.append({
            'vehicleX': float(values[0]),
            'vehicleY': float(values[1]),
            'vehicleZ': float(values[2]),
            'terrainZ': float(values[3]),
            'vehicleYaw': float(values[4]),
        })

    return positions


def _robot_name(index, prefix):
    return f'{prefix}{index + 1}'


def _robot_pose(context, index):
    robot_positions_raw = _launch_value(context, 'robot_positions').strip()
    explicit_positions = _parse_robot_positions(robot_positions_raw) if robot_positions_raw else []
    if index < len(explicit_positions):
        return explicit_positions[index]

    robot_columns = max(1, _launch_int(context, 'robot_columns'))
    robot_start_x = _launch_float(context, 'robot_start_x')
    robot_start_y = _launch_float(context, 'robot_start_y')
    robot_start_z = _launch_float(context, 'robot_start_z')
    robot_start_terrain_z = _launch_float(context, 'robot_start_terrain_z')
    robot_start_yaw = _launch_float(context, 'robot_start_yaw')
    robot_spacing_x = _launch_float(context, 'robot_spacing_x')
    robot_spacing_y = _launch_float(context, 'robot_spacing_y')
    robot_yaw_step = _launch_float(context, 'robot_yaw_step')

    column = index % robot_columns
    row = index // robot_columns

    return {
        'vehicleX': robot_start_x + column * robot_spacing_x,
        'vehicleY': robot_start_y + row * robot_spacing_y,
        'vehicleZ': robot_start_z,
        'terrainZ': robot_start_terrain_z,
        'vehicleYaw': robot_start_yaw + index * robot_yaw_step,
    }


def _build_robot_group(context, index):
    robot_name_prefix = _launch_value(context, 'robot_name_prefix')
    robot_name = _robot_name(index, robot_name_prefix)
    robot_pose = _robot_pose(context, index)

    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')
    world_name = LaunchConfiguration('world_name')

    local_planner_share = get_package_share_directory('local_planner')
    terrain_analysis_share = get_package_share_directory('terrain_analysis')
    terrain_analysis_ext_share = get_package_share_directory('terrain_analysis_ext')
    sensor_scan_generation_share = get_package_share_directory('sensor_scan_generation')
    visualization_tools_share = get_package_share_directory('visualization_tools')

    return TimerAction(
        period=2.0 + index * 0.1,
        actions=[
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(local_planner_share, 'launch', 'multi_local_planner.launch')),
                        launch_arguments={
                            'cameraOffsetZ': cameraOffsetZ,
                            'goalX': str(robot_pose['vehicleX']),
                            'goalY': str(robot_pose['vehicleY']),
                            'robot_name': robot_name,
                        }.items()
                    ),
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(terrain_analysis_share, 'launch', 'multi_terrain_analysis.launch')),
                        launch_arguments={'robot_name': robot_name}.items()
                    ),
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(terrain_analysis_ext_share, 'launch', 'multi_terrain_analysis_ext.launch')),
                        launch_arguments={
                            'checkTerrainConn': checkTerrainConn,
                            'robot_name': robot_name,
                        }.items()
                    ),
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(sensor_scan_generation_share, 'launch', 'multi_sensor_scan_generation.launch')),
                        launch_arguments={'robot_name': robot_name}.items()
                    ),
                    IncludeLaunchDescription(
                        FrontendLaunchDescriptionSource(os.path.join(visualization_tools_share, 'launch', 'multi_visualization_tools.launch')),
                        launch_arguments={
                            'world_name': world_name,
                            'robot_name': robot_name,
                        }.items()
                    ),
                ]
            )
        ]
    )


def _build_tf_broadcaster(base_robot_name, robot_name):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{base_robot_name}_to_{robot_name}_tf_broadcaster',
        output='screen',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', f'{base_robot_name}/map',
            '--child-frame-id', f'{robot_name}/map'
        ]
    )


def build_launch_actions(context, *args, **kwargs):
    robot_count = max(0, _launch_int(context, 'robot_count'))
    robot_name_prefix = _launch_value(context, 'robot_name_prefix')
    base_robot_name = _robot_name(0, robot_name_prefix) if robot_count > 0 else _robot_name(0, robot_name_prefix)

    world_name = LaunchConfiguration('world_name')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
    gazebo_gui = LaunchConfiguration('gazebo_gui')
    pause = LaunchConfiguration('pause')
    verbose = LaunchConfiguration('verbose')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
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
    checkTerrainConn = LaunchConfiguration('checkTerrainConn')

    multi_vehicle_simulator = TimerAction(
        period=0.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'multi_vehicle_simulator.launch.py')),
                launch_arguments={
                    'robot_count': LaunchConfiguration('robot_count'),
                    'robot_columns': LaunchConfiguration('robot_columns'),
                    'robot_name_prefix': LaunchConfiguration('robot_name_prefix'),
                    'robot_positions': LaunchConfiguration('robot_positions'),
                    'robot_start_x': LaunchConfiguration('robot_start_x'),
                    'robot_start_y': LaunchConfiguration('robot_start_y'),
                    'robot_start_z': LaunchConfiguration('robot_start_z'),
                    'robot_start_terrain_z': LaunchConfiguration('robot_start_terrain_z'),
                    'robot_start_yaw': LaunchConfiguration('robot_start_yaw'),
                    'robot_spacing_x': LaunchConfiguration('robot_spacing_x'),
                    'robot_spacing_y': LaunchConfiguration('robot_spacing_y'),
                    'robot_yaw_step': LaunchConfiguration('robot_yaw_step'),
                    'sensorOffsetX': sensorOffsetX,
                    'sensorOffsetY': sensorOffsetY,
                    'vehicleHeight': vehicleHeight,
                    'cameraOffsetZ': cameraOffsetZ,
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
                    'pause': pause,
                    'use_sim_time': use_sim_time,
                    'gui': gazebo_gui,
                    'record': LaunchConfiguration('record'),
                    'verbose': verbose,
                    'world_name': world_name,
                }.items()
            )
        ]
    )

    robot_actions = [_build_robot_group(context, index) for index in range(robot_count)]
    tf_actions = [_build_tf_broadcaster(base_robot_name, _robot_name(index, robot_name_prefix)) for index in range(1, robot_count)]

    return [multi_vehicle_simulator, *robot_actions, *tf_actions]

def generate_launch_description():
    declare_args = [
        DeclareLaunchArgument('robot_count', default_value='3', description='机器人数量'),
        DeclareLaunchArgument('robot_columns', default_value='2', description='机器人布局列数'),
        DeclareLaunchArgument('robot_name_prefix', default_value='robot_', description='机器人名称前缀'),
        DeclareLaunchArgument('robot_start_x', default_value='1.0', description='机器人起始X坐标'),
        DeclareLaunchArgument('robot_start_y', default_value='0.0', description='机器人起始Y坐标'),
        DeclareLaunchArgument('robot_start_z', default_value='0.0', description='机器人起始Z坐标'),
        DeclareLaunchArgument('robot_start_terrain_z', default_value='0.0', description='机器人起始地形高度'),
        DeclareLaunchArgument('robot_start_yaw', default_value='0.0', description='机器人起始偏航角'),
        DeclareLaunchArgument('robot_spacing_x', default_value='1.0', description='机器人X方向间距'),
        DeclareLaunchArgument('robot_spacing_y', default_value='1.0', description='机器人Y方向间距'),
        DeclareLaunchArgument('robot_yaw_step', default_value='0.0', description='机器人偏航角步进'),
        DeclareLaunchArgument('robot_positions', default_value='', description='每个机器人位姿列表: x,y,z,terrainZ,yaw;...'),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='机器人高度'),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='相机Z轴偏移量'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='是否启动Gazebo GUI'),
        DeclareLaunchArgument('checkTerrainConn', default_value='false', description='是否检查地形连接'),
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description='传感器X偏移量'),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='传感器Y偏移量'),
        DeclareLaunchArgument('terrainVoxelSize', default_value='0.05', description='地形体素大小'),
        DeclareLaunchArgument('groundHeightThre', default_value='0.1', description='地面高度阈值'),
        DeclareLaunchArgument('adjustZ', default_value='true', description='是否调整Z轴'),
        DeclareLaunchArgument('terrainRadiusZ', default_value='1.0', description='Z轴地形半径'),
        DeclareLaunchArgument('minTerrainPointNumZ', default_value='5', description='Z轴最少地形点数'),
        DeclareLaunchArgument('smoothRateZ', default_value='0.5', description='Z轴平滑率'),
        DeclareLaunchArgument('adjustIncl', default_value='true', description='是否调整坡度'),
        DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0', description='坡度地形半径'),
        DeclareLaunchArgument('minTerrainPointNumIncl', default_value='200', description='坡度最少地形点数'),
        DeclareLaunchArgument('smoothRateIncl', default_value='0.5', description='坡度平滑率'),
        DeclareLaunchArgument('InclFittingThre', default_value='0.2', description='坡度拟合阈值'),
        DeclareLaunchArgument('maxIncl', default_value='30.0', description='最大坡度'),
        DeclareLaunchArgument('pause', default_value='false', description='是否暂停Gazebo'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='是否使用仿真时间'),
        DeclareLaunchArgument('record', default_value='false', description='是否录制'),
        DeclareLaunchArgument('verbose', default_value='false', description='是否输出详细日志'),
        DeclareLaunchArgument('world_name', default_value='campus', description='仿真环境名称')
    ]
    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)
    ld.add_action(OpaqueFunction(function=build_launch_actions))
    return ld
