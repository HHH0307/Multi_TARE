#!/bin/bash

# 打印开始信息
echo "开始编译工作空间，使用参数：--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"
echo "----------------------------------------"

# 执行编译命令
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "----------------------------------------"
    echo "工作空间编译成功！"
else
    echo "----------------------------------------"
    echo "工作空间编译失败！"
    exit 1
fi
    
