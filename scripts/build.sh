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

# 编译程序
colcon build

# 为soem相关的程序添加权限
sudo setcap 'cap_net_raw,cap_net_admin+eip' $PROJECT_PATH/install/encos_driver/lib/encos_driver/ec_server

