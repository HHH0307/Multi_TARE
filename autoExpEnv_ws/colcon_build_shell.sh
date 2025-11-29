#!/bin/bash

# 定义要编译的包列表
packages=(
    "loam_interface"
    "local_planner"
    "sensor_scan_generation"
    "terrain_analysis"
    "terrain_analysis_ext"
    "vehicle_simulator"
    "velodyne_description"
    "velodyne_gazebo_plugins"
    "velodyne_simulator"
    "visualization_tools"
    "waypoint_example"
    "waypoint_rviz_plugin"
)

# 循环编译每个包
for package in "${packages[@]}"; do
    echo "开始编译功能包: $package"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select "$package"
    
    # 检查上一个命令是否成功执行
    if [ $? -ne 0 ]; then
        echo "编译功能包 $package 失败！"
        exit 1
    fi
    
    echo "功能包 $package 编译成功"
    echo "----------------------------------------"
done

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "所有功能包编译完成！"

