#!/bin/bash

# 默认密码
# PASSWD="cx125"
PASSWD="123456"

# 缓存密码
echo ${PASSWD} | sudo -S pwd

# 当前用户
USER_NAME=$(whoami)

# 执行脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   

# 工程路径
PROJECT_PATH=${SCRIPT_DIR}/..

# 加载ros环境
source $PROJECT_PATH/install/setup.bash

# 限制本地隔离
export ROS_LOCALHOST_ONLY=1

# 为robot_control添加权限
# MODE="${1:-init}"
MODE="${1}"
if [ "$MODE" = "init" ]; then
	sudo setcap 'cap_net_raw,cap_net_admin+eip' ./install/encos_driver/lib/encos_driver/ec_server
fi

# 运行服务端
ros2 run encos_driver ec_server --ros-args -p net_name:=enx00e04c36b33e
