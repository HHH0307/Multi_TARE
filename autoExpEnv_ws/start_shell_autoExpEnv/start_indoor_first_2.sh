#!/bin/bash
echo "------------------- 开始测试 -------------------"
source /usr/share/gazebo/setup.sh
source /home/hqy/mycode/Multi_TARE/autoExpEnv_ws/install/setup.bash
ros2 launch vehicle_simulator multi_system_indoor_first_2.launch.py
