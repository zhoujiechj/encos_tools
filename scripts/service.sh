#!/bin/bash

# 默认密码
# PASSWD="cx125"
PASSWD="123456"

# 缓存密码
echo ${PASSWD} | sudo -S pwd

# 当前用户
USER_NAME=$(whoami)

# 获取执行脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   

# 工程路径
PROJECT_PATH=${SCRIPT_DIR}/..

# 切换路径
cd ${PROJECT_PATH}

# 运行服务端脚本
source ./install/setup.bash
ros2 run encos_driver ec_server --ros-args -p net_name:=enx00e04c36b33e
# ros2 run encos_driver ec_server --ros-args -p net_name:=ens33

