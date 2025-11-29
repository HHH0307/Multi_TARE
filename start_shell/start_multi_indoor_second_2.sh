#!/bin/bash
SCRIPTS_DIR="/home/hqy/mycode/Multi_TARE"
TARE_SCRIPT="$SCRIPTS_DIR/multi_Tare_planner/start_shell_tare/start_explore_indoor_second_2.sh"
AUTOEXP_SCRIPT="$SCRIPTS_DIR/autoExpEnv_ws/start_shell_autoExpEnv/start_indoor_second_2.sh"

# 颜色定义用于输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 检查脚本是否存在并设置可执行权限
setup_script() {
    local script_path="$1"
    local script_name="$2"
    
    if [[ ! -f "$script_path" ]]; then
        log_error "脚本不存在: $script_path"
        return 1
    fi
    
    if [[ ! -x "$script_path" ]]; then
        log_info "设置可执行权限: $script_name"
        if ! chmod +x "$script_path"; then
            log_error "无法设置可执行权限: $script_path"
            return 1
        fi
    fi
    
    return 0
}

# 验证所有脚本
validate_scripts() {
    log_info "验证启动脚本..."
    
    local valid=true
    
    setup_script "$TARE_SCRIPT" "Multi_TARE" || valid=false
    setup_script "$AUTOEXP_SCRIPT" "autoExpEnv" || valid=false
    
    if [[ "$valid" != "true" ]]; then
        log_error "脚本验证失败，请检查路径是否正确"
        exit 1
    fi
    
    log_info "所有脚本验证通过"
}

# 启动终端标签页
start_terminals() {
    log_info "启动终端标签页..."
    
    # 检查gnome-terminal是否可用
    if ! command -v gnome-terminal &> /dev/null; then
        log_error "未找到 gnome-terminal，请确保在GNOME环境下运行"
        exit 1
    fi
    
    # 使用数组存储命令，提高可读性
    local commands=(
        "--tab --title='Multi_TARE' -e 'bash -c \"sleep 1; $TARE_SCRIPT; exec bash\"'"
        "--tab --title='autoExpEnv' -e 'bash -c \"$AUTOEXP_SCRIPT; exec bash\"'"
    )
    
    # 构建完整的gnome-terminal命令
    local terminal_cmd="gnome-terminal"
    for cmd in "${commands[@]}"; do
        terminal_cmd+=" $cmd"
    done
    terminal_cmd+=" &"
    
    # 执行命令
    log_info "执行: $terminal_cmd"
    eval "$terminal_cmd"
    
    if [[ $? -eq 0 ]]; then
        log_info "所有终端标签页启动成功"
    else
        log_error "启动终端时出现错误"
        exit 1
    fi
}

# 主函数
main() {
    log_info "开始启动 Multi-TARE-ROS2 系统..."
    
    # 验证脚本
    validate_scripts
    
    # 启动终端
    start_terminals
    
    log_info "启动完成！"
    
    # 显示提示信息
    echo
    log_warn "注意：系统正在启动中，请等待各服务完全初始化..."
    log_info "Multi_TARE - Multi-TARE算法"
    log_info "autoExpEnv - 自动实验环境"
}

# 信号处理，确保脚本退出时清理
cleanup() {
    log_warn "正在退出..."
    exit 0
}

trap cleanup SIGINT SIGTERM

# 检查是否直接运行该脚本
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
