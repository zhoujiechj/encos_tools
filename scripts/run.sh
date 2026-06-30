#!/usr/bin/env bash

set -eo pipefail

# ===================== 参数配置 =====================
# 用法：
#   ./run_ec_calib.sh                  # 默认 robot
#   ./run_ec_calib.sh robot            # 机器人整机模式
#   ./run_ec_calib.sh single           # 单板/单从站模式
#   ./run_ec_calib.sh --mode single
#   ./run_ec_calib.sh --net ens33
#   ./run_ec_calib.sh --mode robot --net enx00e04c36b33e

MODE="robot"
NET_NAME="enp1s0f3"
RMW="cyclonedds"

usage() {
    cat <<USAGE
用法: $0 [robot|single] [--mode robot|single] [--net 网卡名] [--fastdds|--cyclonedds]

参数:
  robot              使用机器人整机从站配置，默认值
  single             使用单板/单从站配置，SLAVE_ID 全部为 0
  -m, --mode MODE    指定模式: robot 或 single
  -n, --net NAME     指定以太网网卡名称，默认: ${NET_NAME}
  --cyclonedds       使用 CycloneDDS RMW（默认）
  --fastdds          使用 FastDDS RMW
  -h, --help         查看帮助

示例:
  $0
  $0 single
  $0 --mode robot --net enx00e04c36b33e --fastdds
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        robot|single)
            MODE="$1"
            shift
            ;;
        -m|--mode)
            MODE="${2:-}"
            shift 2
            ;;
        -n|--net)
            NET_NAME="${2:-}"
            shift 2
            ;;
        --cyclonedds)
            RMW="cyclonedds"
            shift
            ;;
        --fastdds)
            RMW="fastdds"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ "${MODE}" != "robot" && "${MODE}" != "single" ]]; then
    echo "❌ mode 只能是 robot 或 single，当前: ${MODE}"
    usage
    exit 1
fi

if [[ -z "${NET_NAME}" ]]; then
    echo "❌ 网卡名称不能为空"
    usage
    exit 1
fi

if [[ "${RMW}" != "cyclonedds" && "${RMW}" != "fastdds" ]]; then
    echo "❌ RMW 只能是 cyclonedds 或 fastdds，当前: ${RMW}"
    usage
    exit 1
fi

# 删除ec_server
pkill -f ec_server || true

# ===================== 基础配置 =====================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
PROJECT_PATH="${SCRIPT_DIR}/.."

setup_rmw() {
    case "${RMW}" in
        cyclonedds)
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI="file://${PROJECT_PATH}/config/cyclonedds_localhost.xml"
            unset FASTRTPS_DEFAULT_PROFILES_FILE
            unset FASTDDS_DEFAULT_PROFILES_FILE
            unset RMW_FASTRTPS_USE_QOS_FROM_XML
            unset RMW_FASTRTPS_USE_SHM
            unset FASTDDS_BUILTIN_TRANSPORTS
            ;;
        fastdds)
            unset RMW_IMPLEMENTATION
            unset CYCLONEDDS_URI
            ;;
    esac
}
setup_rmw

source "${PROJECT_PATH}/install/setup.bash"

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

# robot：整机从站配置；single：单板/单从站配置，全部 slave_id = 0
if [[ "${MODE}" == "robot" ]]; then
    SLAVE_ID=(
        "IGNORE"
        3 3
        3 0 0 0 0 0 0
        3 1 1 1 1 1 1
        3 3 3
        2 2 2 2 2 2
        4 4 4 4 4 4
    )
else
    SLAVE_ID=(
        "IGNORE"
        0 0
        0 0 0 0 0 0 0
        0 0 0 0 0 0 0
        0 0 0
        0 0 0 0 0 0
        0 0 0 0 0 0
    )
fi



if [[ "${MODE}" == "robot" ]]; then
    PASSAGE=(
        "IGNORE"
        4 5
        2 1 2 3 4 5 6
        3 1 2 3 4 5 6
        "" "" 6
        1 2 3 4 5 6
        1 2 3 4 5 6
)
else
    PASSAGE=(
        "IGNORE"
        1 1
        1 1 1 1 1 1 1
        1 1 1 1 1 1 1
        "" "" 1
        1 1 1 1 1 1
        1 1 1 1 1 1
)
fi

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
CLEANED_UP=0

# ===================== 服务控制 =====================
start_server() {
    echo -e "\n============================================="
    echo "        启动 ec_server"
    echo "        模式: ${MODE}"
    echo "        网口: ${NET_NAME}"
    echo -e "=============================================\n"

    setsid ros2 run encos_driver ec_server --ros-args -p net_name:="${NET_NAME}" &
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
    # 防止 EXIT、INT、TERM 多次触发时重复清理
    if [[ "${CLEANED_UP}" -eq 1 ]]; then
        return
    fi
    CLEANED_UP=1

    echo -e "\n🧹 正在清理 ec_server..."

    if [[ -n "${EC_SERVER_PID}" ]]; then
        # 优先按进程组清理：setsid 启动后，ec_server 会在独立进程组中
        kill -TERM -- "-${EC_SERVER_PID}" 2>/dev/null || true
        kill -TERM "${EC_SERVER_PID}" 2>/dev/null || true
        sleep 1

        # 如果仍有残留，再强制清理
        kill -KILL -- "-${EC_SERVER_PID}" 2>/dev/null || true
        kill -KILL "${EC_SERVER_PID}" 2>/dev/null || true

        wait "${EC_SERVER_PID}" 2>/dev/null || true
    fi

    # 兜底：防止 ros2 run 或 ec_server 子进程残留
    pkill -f "ros2 run encos_driver ec_server" 2>/dev/null || true
    pkill -x "ec_server" 2>/dev/null || true
    pkill -f "encos_driver.*ec_server" 2>/dev/null || true

    echo "🛑 ec_server 已停止"
}

handle_interrupt() {
    stop_server
    echo -e "\n👋 捕获 Ctrl+C，程序已安全退出"
    exit 130
}

handle_exit() {
    stop_server
}

# ===================== 工具函数 =====================
check_valid_id() {
    local id="$1"
    [[ "${id}" =~ ^[1-9]$|^[12][0-9]$|^3[01]$ ]]
}

check_valid_slave_id() {
    local slave_id="$1"
    [[ "${slave_id}" =~ ^[0-9]+$ ]]
}

check_valid_passage() {
    local passage="$1"
    [[ "${passage}" =~ ^[0-9]+$ ]]
}

check_valid_motor_id() {
    local motor_id="$1"
    [[ "${motor_id}" =~ ^[0-9]+$ ]]
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

    local output
    output=$(${cmd})
    echo "${output}"

    # 提取数字（自动适配驱动输出格式）
    # local pos
    # pos=$(echo "${output}" | grep -oE '[+-]?[0-9]+\.[0-9]+' | head -1)
    # echo "${pos}"
}

# 关节标零
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
    get_pos "${id}" || true

    echo -e "\n[执行标零...]"
    local cmd="ros2 run encos_driver ec_client -t set_zero -s ${s} -p ${p} -m ${m}"
    echo "[命令] ${cmd}"
    ${cmd}
    sleep 1

    echo -e "\n[标零后位置]"
    get_pos "${id}" || true
    get_pos "${id}" || true
}

# 获取指定从站下的电机 ID
get_motor_id() {
    local slave_id="$1"

    if ! check_valid_slave_id "${slave_id}"; then
        echo "❌ 无效 slave_id！请输入非负整数"
        return 1
    fi

    echo -e "============================================="
    echo "              🆔 读取电机 ID"
    echo "              slave_id: ${slave_id}"
    echo -e "============================================="

    local cmd="ros2 run encos_driver ec_client -t get_id -s ${slave_id}"
    echo "[命令] ${cmd}"
    ${cmd}
}

# 设置指定从站下的电机 ID
set_motor_id() {
    local slave_id="$1"
    local passage="$2"
    local motor_id_old="$3"
    local motor_id_new="$4"

    if ! check_valid_slave_id "${slave_id}"; then
        echo "❌ 无效 slave_id！请输入非负整数"
        return 1
    fi
    
    if ! check_valid_passage "${passage}"; then
        echo "❌ 无效 passage！请输入非负整数"
        return 1
    fi

    if ! check_valid_motor_id "${motor_id_old}"; then
        echo "❌ 无效旧 motor_id！请输入非负整数"
        return 1
    fi

    if ! check_valid_motor_id "${motor_id_new}"; then
        echo "❌ 无效新 motor_id！请输入非负整数"
        return 1
    fi

    if [[ "${motor_id_old}" == "${motor_id_new}" ]]; then
        echo "❌ 新旧 motor_id 相同，无需设置"
        return 1
    fi

    echo -e "============================================="
    echo "              🆔 设置电机 ID"
    echo "              slave_id: ${slave_id}"
    echo "              passage:  ${passage}"
    echo "              old_id:   ${motor_id_old}"
    echo "              new_id:   ${motor_id_new}"
    echo -e "============================================="

    local cmd="ros2 run encos_driver ec_client -t set_id -s ${slave_id} -p ${passage} -o ${motor_id_old} -n ${motor_id_new}"
    echo "[命令] ${cmd}"
    ${cmd}
    
    sleep 1
    
    local cmd="ros2 run encos_driver ec_client -t get_id -s ${slave_id}"
    echo "[命令] ${cmd}"
    ${cmd}
    ${cmd}
}

# ===================== 菜单 =====================
show_joints() {
    local name="$1"
    echo -e "\n============================================="
    echo "          ${name} 关节选择 1-31"
    echo "============================================="
    echo " 1-头部1    2-头部2"
    echo " 3-左手1    4-左手2    5-左手3    6-左手4    7-左手5    8-左手6    9-左手7"
    echo "10-右手1   11-右手2   12-右手3   13-右手4   14-右手5   15-右手6   16-右手7"
    echo "17-腰部1   18-腰部2   19-腰部3"
    echo "20-左腿1   21-左腿2   22-左腿3   23-左腿4   24-左腿5   25-左腿6"
    echo "26-右腿1   27-右腿2   28-右腿3   29-右腿4   30-右腿5   31-右腿6"
    echo "============================================="
    echo "q 返回主菜单"
}

main_menu() {
    echo "============================================="
    echo "          人形机器人电机标定工具"
    echo "          当前模式: ${MODE}"
    echo "============================================="
    echo " 1. 📍 关节位置"
    echo " 2. 🔧 关节标零"
    echo " 3. 🆔 读取电机 ID"
    echo " 4. 🆔 设置电机 ID"
    echo " q. 🛑 退出程序"
    echo "============================================="
    read -r -p "请选择功能: " opt

    case "${opt}" in
        1)
            while true; do
                show_joints "关节位置"
                read -r -p "输入关节号: " sel
                [[ "${sel}" == q || "${sel}" == Q ]] && break
                if check_valid_id "${sel}"; then
                    get_pos "${sel}"
                else
                    echo "❌ 无效输入！请输入 1-31"
                fi
            done
            ;;

        2)
            while true; do
                show_joints "关节标零"
                read -r -p "输入关节号: " sel
                [[ "${sel}" == q || "${sel}" == Q ]] && break
                if check_valid_id "${sel}"; then
                    set_zero "${sel}"
                else
                    echo "❌ 无效输入！请输入 1-31"
                fi
            done
            ;;

        3)
            read -r -p "输入 slave_id: " slave_id
            get_motor_id "${slave_id}"
            ;;

        4)
            read -r -p "输入 slave_id: " slave_id
            read -r -p "输入 passage: " passage
            read -r -p "输入当前/旧 motor_id: " motor_id_old
            read -r -p "输入目标/新 motor_id: " motor_id_new
            set_motor_id "${slave_id}" "${passage}" "${motor_id_old}" "${motor_id_new}"
            ;;

        q|Q)
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
trap handle_exit EXIT
trap handle_interrupt INT TERM
start_server

while true; do
    main_menu
done
