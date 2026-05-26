#!/usr/bin/env bash

pkill -f ec_server

set -eo pipefail

# ===================== 基础配置 =====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."
source "${PROJECT_PATH}/install/setup.bash"

# 以太网网卡（根据实际修改）
# NET_NAME="ens33"
NET_NAME="enx00e04c36b33e"

# ===================== 关节配置数组 =====================
JOINTS=(
    "IGNORE"
    "头部1"   "头部2"
    "左手1"   "左手2"   "左手3"   "左手4"   "左手5"   "左手6"   "左手7"
    "右手1"   "右手2"   "右手3"   "右手4"   "右手5"   "右手6"   "右手7"
    "腰部1"   "腰部2"   "腰部3"
    "左腿1"   "左腿2"   "左腿3"   "左腿4"   "左腿5"   "左腿6"
    "右腿1"   "右腿2"   "右腿3"   "右腿4"   "右腿5"   "右腿6"
)

SLAVE_ID=(
    "IGNORE"
    3 3
    3 0 0 0 0 0 0
    3 1 1 1 1 1 1
    3 3 3
    2 2 2 2 2 2
    4 4 4 4 4 4
)

PASSAGE=(
    "IGNORE"
    4 5
    2 1 2 3 4 5 6
    3 1 2 3 4 5 6
    "" "" 6
    1 2 3 4 5 6
    1 2 3 4 5 6
)

MOTOR_ID=(
    "IGNORE"
    1 2
    11 12 13 14 15 16 17
    21 22 23 24 25 26 27
    31 32 33
    41 42 43 44 45 46
    51 52 53 54 55 56
)

# 全局 PID
EC_SERVER_PID=""

# ===================== 服务控制 =====================
start_server() {
    echo -e "\n============================================="
    echo "        启动 ec_server 网口: ${NET_NAME}"
    echo -e "=============================================\n"

    ros2 run encos_driver ec_server --ros-args -p net_name:="${NET_NAME}" &
    EC_SERVER_PID=$!
    if ps -p "${EC_SERVER_PID}" &>/dev/null; then
        echo "✅ 服务端启动成功 PID: ${EC_SERVER_PID}"
    else
        echo "❌ 服务端启动失败！请检查网卡/驱动"
        exit 1
    fi
    sleep 2
}

stop_server() {
    if [[ -n "${EC_SERVER_PID}" && -d "/proc/${EC_SERVER_PID}" ]]; then
        kill "${EC_SERVER_PID}" &>/dev/null
        wait "${EC_SERVER_PID}" 2>/dev/null
        echo -e "\n🛑 服务端已安全停止"
    fi
}

# ===================== 工具函数 =====================
check_valid_id() {
    local id="$1"
    [[ "${id}" =~ ^[1-9]$|^[12][0-9]$|^3[01]$ ]]
}

# 读取位置并返回数值（自动提取数字）
get_pos() {
    local id="$1"
    local s="${SLAVE_ID[${id}]}"
    local p="${PASSAGE[${id}]}"
    local m="${MOTOR_ID[${id}]}"
    local name="${JOINTS[${id}]}"

    if [[ -z "${p}" || "${p}" == "/" ]]; then
        echo -e "\n${name} 无通道，跳过"
        return 1
    fi

    echo -e "============================================="
    echo "              🔧 读取${name}位置"
    echo -e "============================================="
    local cmd="ros2 run encos_driver ec_client -t get_param -c 1 -s ${s} -p ${p} -m ${m}"
    echo "[命令] ${cmd}"
    
    # 执行命令并捕获输出，提取位置数值（支持 position: 123.45 deg 格式）
    local output=$(${cmd})
    echo "${output}"
    
    # 提取数字（自动适配驱动输出格式）
    local pos=$(echo "${output}" | grep -oE '[+-]?[0-9]+\.[0-9]+' | head -1)
    echo "${pos}"
}

# 关节标零 + 成功/失败判断
set_zero() {
    local id="$1"
    local s="${SLAVE_ID[${id}]}"
    local p="${PASSAGE[${id}]}"
    local m="${MOTOR_ID[${id}]}"
    local name="${JOINTS[${id}]}"

    if [[ -z "${p}" || "${p}" == "/" ]]; then
        echo "${name} 无通道，跳过"
        return
    fi

    echo -e "============================================="
    echo "              🔧 ${name} 关节标零"
    echo -e "============================================="

    echo "[标零前位置]"
    get_pos "${id}" > /dev/null  # 只打印，不取值

    echo -e "\n[执行标零...]"
    local cmd="ros2 run encos_driver ec_client -t set_zero -s ${s} -p ${p} -m ${m}"
    echo "[命令] ${cmd}"
    ${cmd}
    sleep 1

    echo -e "\n[标零后位置]"
    # 获取标零后的位置数值
    local pos=$(get_pos "${id}" | tail -1)
    local pos=$(get_pos "${id}" | tail -1)
}

# ===================== 菜单 =====================
show_joints() {
    # clear
    local name="$1"
    echo -e "\n============================================="
    echo "          ${name} 关节选择 1-31"
    echo "============================================="
    echo " 1 头部1    2 头部2"
    echo " 3 左手1    4 左手2    5 左手3    6 左手4    7 左手5    8 左手6    9 左手7"
    echo "10 右手1   11 右手2   12 右手3   13 右手4   14 右手5   15 右手6   16 右手7"
    echo "17 腰部1   18 腰部2   19 腰部3"
    echo "20 左腿1   21 左腿2   22 左腿3   23 左腿4   24 左腿5   25 左腿6"
    echo "26 右腿1   27 右腿2   28 右腿3   29 右腿4   30 右腿5   31 右腿6"
    echo "============================================="
    echo "q 返回主菜单"
}

main_menu() {
    # clear
    echo "============================================="
    echo "          人形机器人电机标定工具"
    echo "============================================="
    echo " 1. 📍 关节位置"
    echo " 2. 🔧 关节标零"
    echo " q. 🛑 退出程序"
    echo "============================================="
    read -p "请选择功能: " opt

    case "${opt}" in
        1)
            while true; do
                show_joints "关节位置"
                read -p "输入关节号: " sel
                [[ "${sel}" == q ]] && break
                if check_valid_id "${sel}"; then
                    get_pos "${sel}"
                else
                    echo "❌ 无效输入！请输入 1-31"
                fi
                # read -p $'\n按回车继续...'
            done
            ;;

        2)
            while true; do
                show_joints "关节标零"
                read -p "输入关节号: " sel
                [[ "${sel}" == q ]] && break
                if check_valid_id "${sel}"; then
                    set_zero "${sel}"
                else
                    echo "❌ 无效输入！请输入 1-31"
                fi
                # read -p $'\n按回车继续...'
            done
            ;;

        q|Q)
            stop_server
            echo -e "\n👋 程序已安全退出"
            exit 0
            ;;

        *)
            echo "❌ 无效选项！"
            sleep 1
            ;;
    esac
}

# ===================== 入口 =====================
trap stop_server EXIT
start_server

while true; do
    main_menu
done
