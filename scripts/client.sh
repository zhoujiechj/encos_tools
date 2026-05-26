#!/usr/bin/env bash

# 获取执行脚本所在路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"   

# 工程路径
PROJECT_PATH=${SCRIPT_DIR}/..

# 加载ros环境
source $PROJECT_PATH/install/setup.bash

set -e

# 关节名称（1~31）
JOINTS=(
    "IGNORE"
    "头部1"   "头部2"
    "左手1"   "左手2"   "左手3"   "左手4"   "左手5"   "左手6"   "左手7"
    "右手1"   "右手2"   "右手3"   "右手4"   "右手5"   "右手6"   "右手7"
    "腰部1"   "腰部2"   "腰部3"
    "左腿1"   "左腿2"   "左腿3"   "左腿4"   "左腿5"   "左腿6"
    "右腿1"   "右腿2"   "右腿3"   "右腿4"   "右腿5"   "右腿6"
)

# Slave ID
SLAVE_ID=(
    "IGNORE"
    3 3
    3 0 0 0 0 0 0
    3 1 1 1 1 1 1
    3 3 3
    2 2 2 2 2 2
    4 4 4 4 4 4
)

# Passage
PASSAGE=(
    "IGNORE"
    4 5
    2 1 2 3 4 5 6
    3 1 2 3 4 5 6
    "" "" 6
    1 2 3 4 5 6
    1 2 3 4 5 6
)

# 电机 ID
MOTOR_ID=(
    "IGNORE"
    1 2
    11 12 13 14 15 16 17
    21 22 23 24 25 26 27
    31 32 33
    41 42 43 44 45 46
    51 52 53 54 55 56
)

# 显示菜单
show_menu() {
  echo
  echo "============================================="
  echo "           电机零位标定（直接控制电机）"
  echo "============================================="
  echo " 1 头部1    2 头部2"
  echo " 3 左手1    4 左手2    5 左手3    6 左手4    7 左手5    8 左手6    9 左手7"
  echo "10 右手1   11 右手2   12 右手3   13 右手4   14 右手5   15 右手6   16 右手7"
  echo "17 腰部1   18 腰部2   19 腰部3"
  echo "20 左腿1   21 左腿2   22 左腿3   23 左腿4   24 左腿5   25 左腿6"
  echo "26 右腿1   27 右腿2   28 右腿3   29 右腿4   30 右腿5   31 右腿6"
  echo " q 退出"
  echo "============================================="
}

# 执行标定
run_motor() {
  local id=$1
  local s=${SLAVE_ID[$id]}
  local p=${PASSAGE[$id]}
  local m=${MOTOR_ID[$id]}
  local name=${JOINTS[$id]}

  if [[ -z $p || $p == "/" ]]; then
    echo "→ $name 无通道，跳过"
    return
  fi

  echo
  echo "→ 正在控制：$name"
  echo "[命令] ros2 run encos_driver ec_client -t set_zero -s $s -p $p -m $m"
  
  # ======================
  # 这一行是真正执行命令！
  # ======================
  ros2 run encos_driver ec_client -t set_zero -s "$s" -p "$p" -m "$m"
  ros2 run encos_driver ec_client -t get_param -c 1 -s "$s" -p "$p" -m "$m"
  ros2 run encos_driver ec_client -t get_param -c 1 -s "$s" -p "$p" -m "$m"

  echo "→ $name 执行完成"
  echo
}

# 主程序
main() {
  while true; do
    show_menu
    read -p "请选择：" sel

    if [[ $sel == q ]]; then
      echo "退出"
      exit 0
    fi

    # if [[ $sel -eq 0 ]]; then
      # echo "→ 开始全部标定..."
      # for ((i=1; i<=31; i++)); do run_motor $i; done
      # echo "→ 全部标定完成！"
    if [[ $sel -ge 1 && $sel -le 31 ]]; then
      run_motor $sel
    else
      echo "无效输入"
    fi

    # read -p "按回车继续..."
  done
}

main
