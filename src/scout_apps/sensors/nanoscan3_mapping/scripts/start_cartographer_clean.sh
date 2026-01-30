#!/bin/bash
# 启动 Cartographer 的脚本（使用干净的库路径）

# 保存需要的 Conan 库路径（只保留 osqp 和 libunwind，如果 MPC 需要的话）
CLEAN_LD_PATH="/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/usr/local/cuda-12.1/lib64"
CLEAN_LD_PATH="$CLEAN_LD_PATH:/home/a/.conan/data/osqp/0.6.3/_/_/package/2af715f34a7c8c2aeae57b25be0a52c4110dc502/lib"
CLEAN_LD_PATH="$CLEAN_LD_PATH:/home/a/.conan/data/libunwind/1.8.0/_/_/package/c8c888b1fc83f5e0145e8890c2af3bd4e0005c98/lib"

export LD_LIBRARY_PATH="$CLEAN_LD_PATH"

source /opt/ros/noetic/setup.bash
source ~/scout_ws/devel/setup.bash
source ~/scout_ws/src/scout_apps/sensors/cartographer_ws/install_isolated/setup.bash

roslaunch nanoscan3_mapping scout_nanoscan3_cartographer_sim.launch
