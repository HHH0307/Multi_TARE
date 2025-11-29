#!/bin/bash
echo "------------------- 开始测试 -------------------"
source /home/hqy/mycode/Multi_TARE/multi_Tare_planner/install/setup.bash
ros2 launch tare_planner multi_explore_campus_3.launch.py
