import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def declare_world_action(context, world_name):
    world_name_str = str(world_name.perform(context))
    declare_world = DeclareLaunchArgument('world', default_value=[os.path.join(get_package_share_directory('vehicle_simulator'), 'world', world_name_str + '.world')], description='')
    return [declare_world]


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


def build_robot_groups(context, *args, **kwargs):
    robot_count = max(0, _launch_int(context, 'robot_count'))
    robot_columns = max(1, _launch_int(context, 'robot_columns'))
    robot_name_prefix = _launch_value(context, 'robot_name_prefix')
    robot_positions_raw = _launch_value(context, 'robot_positions').strip()
    explicit_positions = _parse_robot_positions(robot_positions_raw) if robot_positions_raw else []

    robot_start_x = _launch_float(context, 'robot_start_x')
    robot_start_y = _launch_float(context, 'robot_start_y')
    robot_start_z = _launch_float(context, 'robot_start_z')
    robot_start_terrain_z = _launch_float(context, 'robot_start_terrain_z')
    robot_start_yaw = _launch_float(context, 'robot_start_yaw')
    robot_spacing_x = _launch_float(context, 'robot_spacing_x')
    robot_spacing_y = _launch_float(context, 'robot_spacing_y')
    robot_yaw_step = _launch_float(context, 'robot_yaw_step')

    sensorOffsetX = LaunchConfiguration('sensorOffsetX')
    sensorOffsetY = LaunchConfiguration('sensorOffsetY')
    vehicleHeight = LaunchConfiguration('vehicleHeight')
    cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
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
    use_sim_time = LaunchConfiguration('use_sim_time')

    vehicle_simulator_share = get_package_share_directory('vehicle_simulator')
    lidar_urdf = os.path.join(vehicle_simulator_share, 'urdf', 'multi_lidar.urdf.xacro')
    robot_sdf = os.path.join(vehicle_simulator_share, 'urdf', 'multi_robot.sdf')
    camera_urdf = os.path.join(vehicle_simulator_share, 'urdf', 'multi_camera.urdf.xacro')

    robot_groups = []
    for index in range(robot_count):
        robot_name = f'{robot_name_prefix}{index + 1}'

        if index < len(explicit_positions):
            pose = explicit_positions[index]
        else:
            grid_column = index % robot_columns
            grid_row = index // robot_columns
            pose = {
                'vehicleX': robot_start_x + grid_column * robot_spacing_x,
                'vehicleY': robot_start_y + grid_row * robot_spacing_y,
                'vehicleZ': robot_start_z,
                'terrainZ': robot_start_terrain_z,
                'vehicleYaw': robot_start_yaw + index * robot_yaw_step,
            }

        robot_groups.append(
            GroupAction(
                actions=[
                    PushRosNamespace(robot_name),
                    Node(
                        package='robot_state_publisher',
                        executable='robot_state_publisher',
                        name='robot_state_publisher',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time,
                            'robot_description': Command(['xacro ', lidar_urdf])
                        }]
                    ),
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-entity', f'{robot_name}_lidar', '-topic', f'/{robot_name}/robot_description', '-robot_namespace', robot_name],
                        output='screen'
                    ),
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-file', robot_sdf, '-entity', f'{robot_name}_robot', '-robot_namespace', robot_name],
                        output='screen'
                    ),
                    Node(
                        package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-file', camera_urdf, '-entity', f'{robot_name}_camera', '-robot_namespace', robot_name],
                        output='screen'
                    ),
                    Node(
                        package='vehicle_simulator',
                        executable='multi_vehicleSimulator',
                        parameters=[{
                            'robot_name': robot_name,
                            'use_gazebo_time': False,
                            'sensorOffsetX': sensorOffsetX,
                            'sensorOffsetY': sensorOffsetY,
                            'vehicleHeight': vehicleHeight,
                            'cameraOffsetZ': cameraOffsetZ,
                            'vehicleX': pose['vehicleX'],
                            'vehicleY': pose['vehicleY'],
                            'vehicleZ': pose['vehicleZ'],
                            'terrainZ': pose['terrainZ'],
                            'vehicleYaw': pose['vehicleYaw'],
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
        )

    return robot_groups

def generate_launch_description():
    pause = LaunchConfiguration('pause')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    record = LaunchConfiguration('record')
    verbose = LaunchConfiguration('verbose')
    world_name = LaunchConfiguration('world_name')

    declare_args = [
        DeclareLaunchArgument('robot_count', default_value='3', description=''),
        DeclareLaunchArgument('robot_columns', default_value='2', description=''),
        DeclareLaunchArgument('robot_name_prefix', default_value='robot_', description=''),
        DeclareLaunchArgument('robot_start_x', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_start_y', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_start_z', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_start_terrain_z', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_start_yaw', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_spacing_x', default_value='1.5', description=''),
        DeclareLaunchArgument('robot_spacing_y', default_value='1.5', description=''),
        DeclareLaunchArgument('robot_yaw_step', default_value='0.0', description=''),
        DeclareLaunchArgument('robot_positions', default_value='', description='每个机器人位姿列表: x,y,z,terrainZ,yaw;...'),
        DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description=''),
        DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description=''),
        DeclareLaunchArgument('vehicleHeight', default_value='0.75', description=''),
        DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description=''),
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

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'pause': pause,
            'gui': gui,
            'verbose': verbose,
        }.items()
    )

    ld = LaunchDescription()
    for arg in declare_args:
        ld.add_action(arg)

    ld.add_action(OpaqueFunction(function=declare_world_action, args=[world_name]))
    ld.add_action(start_gazebo)
    ld.add_action(OpaqueFunction(function=build_robot_groups))

    return ld