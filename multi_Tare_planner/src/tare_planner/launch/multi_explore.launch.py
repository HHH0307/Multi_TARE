import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter, PushRosNamespace


def launch_tare_node(context, scenario, robot_num, robot_name):
    scenario_str = str(scenario.perform(context))      # 解析场景和机器人名称为字符串
    robot_num_str = str(robot_num.perform(context))    # 解析机器人数量为字符串
    robot_name_str = str(robot_name.perform(context))  # 解析后的机器人名称字符串
    test_path = os.path.join(get_package_share_directory('tare_planner'), f"{scenario_str}_{robot_num_str}_{robot_name_str}.yaml")
    multi_tare_planner_node = Node(
        package='tare_planner',
        executable='multi_tare_planner_node',
        name='multi_tare_planner_node',
        output='screen',
        namespace='sensor_coverage_planner',
        parameters=[test_path]
    )
    print("###########################################"+ test_path)
    return [multi_tare_planner_node]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_num = LaunchConfiguration('robot_num')
    robot_name = LaunchConfiguration('robot_name')
    scenario = LaunchConfiguration('scenario')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    declare_robot_num = DeclareLaunchArgument(
        'robot_num',
        default_value='0',
        description='Robot number identifier')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot_1',
        description='Robot name identifier')

    declare_scenario = DeclareLaunchArgument(
        'scenario',
        default_value='garage',
        description='Scenario configuration identifier')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_ground_rviz',
        arguments=[
            '-d', f"{get_package_share_directory('tare_planner')}/tare_planner_ground.rviz"],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_robot_num,
        declare_robot_name,
        declare_scenario,
        SetParameter(name='use_sim_time', value=use_sim_time),
        declare_rviz,
        GroupAction([rviz_node]),
        OpaqueFunction(function=launch_tare_node, args=[scenario, robot_num, robot_name])
    ])

