#!/usr/bin/env bash
# shellcheck shell=bash
# ============================================================================
# gps_ws 环境激活脚本（bash + zsh 双兼容）
#
# 用法（必须 source，直接 ./env.sh 只会报错退出）：
#     source scripts/env.sh            # 激活
#     source scripts/env.sh --quiet    # 静默激活
#     source scripts/env.sh --check    # 只体检，不改环境
#
# 为什么要写 shebang（哪怕只给 source 用）：
#     zsh 直接执行没有 shebang 的文件时会 execve 失败(ENOEXEC)，回退到
#     /bin/sh（Ubuntu 是 dash）执行。dash 不认 ${BASH_SOURCE[0]}，于是
#     直接甩一句 "env.sh: 41: Bad substitution"，下面的友好提示根本没机会跑。
#     写了 #!/usr/bin/env bash 后，直接执行会被 bash 接管，能正常打印提示。
#
# 设计目标：不污染环境
#   1. 幂等        —— 重复 source 不会把路径越拼越长
#   2. 切断父链    —— 用 local_setup.* 而非 setup.*。colcon 会把编译时
#                     source 过的环境焊进 setup.sh，你机器上 con_ws 和
#                     slam_ws 就互相焊了对方，载入任一个都会拖进全部 5 个
#                     工作空间，包括 /home/w/fastlio 里那个重复的 fast_lio
#   3. 双壳兼容    —— bash 用 setup.bash，zsh 用 setup.zsh。
#                     注意 zsh 下 source setup.bash 会静默失败：
#                     setup.bash 用 ${BASH_SOURCE[0]} 推断自身位置，
#                     在 zsh 里它是空的，于是跑去当前目录找 setup.sh
#   4. 缺包只警告  —— clean build 期间 install/ 被删掉也能安全 source
#   5. 不写 dotfile —— 只在你手动 source 时才生效
#
# 加载顺序即优先级：后加载的覆盖先加载的同名包。
# ============================================================================

# --- 识别当前 shell ---------------------------------------------------------
if [ -n "${ZSH_VERSION:-}" ]; then
    _GPS_SHELL=zsh
else
    _GPS_SHELL=bash
fi

# --- 必须用 source 运行 -----------------------------------------------------
_gps_running_directly() {
    if [ "$_GPS_SHELL" = "zsh" ]; then
        case "${ZSH_EVAL_CONTEXT:-}" in
            *:file*) return 1 ;;   # 被 source：上下文里带 file
            *)       return 0 ;;   # 直接执行
        esac
    fi
    if [ -n "${BASH_VERSION:-}" ]; then
        # ${BASH_SOURCE[0]} 是 bash 独有语法，dash/其它 sh 会 Bad substitution，
        # 所以必须先确认自己在 bash 里再展开。
        if [ "${BASH_SOURCE[0]:-}" = "${0:-}" ]; then
            return 0
        fi
        return 1
    fi
    # 既不是 zsh 也不是 bash（多半是被 /bin/sh 兜底执行了）：
    # 这里不展开任何 bash 独有语法，一律按“直接执行”处理。
    return 0
}

if _gps_running_directly; then
    echo "这个脚本要用 source 运行（子进程改不了你当前终端的环境）：" >&2
    echo "    bash: source scripts/env.sh" >&2
    echo "    zsh : source scripts/env.sh" >&2
    echo "    或者：scripts/rosenv      # 进一个加载好环境的隔离子 shell" >&2
    unset -f _gps_running_directly
    exit 1
fi
unset -f _gps_running_directly

# --- 参数解析 ---------------------------------------------------------------
_GPS_QUIET=0
_GPS_CHECK_ONLY=0
for _arg in "$@"; do
    case "$_arg" in
        --quiet|-q)   _GPS_QUIET=1 ;;
        --check)      _GPS_CHECK_ONLY=1 ;;
        --verbose|-v) _GPS_QUIET=0 ;;
    esac
done
unset _arg

_gps_log()  { [ "$_GPS_QUIET" -eq 1 ] || printf '%s\n' "$*"; }
_gps_warn() { printf '%s\n' "$*" >&2; }

# ============================================================================
# 配置区 —— 换版本 / 加工作空间只改这里
# ============================================================================
_GPS_ROS_DISTRO="${GPS_ROS_DISTRO:-/opt/ros/humble}"

# 格式： "显示名|install目录"
# 按依赖顺序排列：底层在前，本项目在最后（保证本项目优先）
_GPS_OVERLAYS=(
    "fast_lio|/home/w/counter/con_ws/install"
    "livox_ros_driver2|/home/w/ws_livox/install"
    "slam_bridge|/home/w/slam_ws/install"
    "gps+vision|/home/w/projects/projects/gps_ws/install"
)

# ⚠ /home/w/fastlio/install 被故意排除：它是另一份 fast_lio 副本，
#   和 con_ws 里的同名，同时加载会让 ros2 pkg prefix fast_lio 的结果
#   取决于 source 顺序。要改用那一份就替换上面 fast_lio 那行。
# ============================================================================

# --- zsh 命令补全 -----------------------------------------------------------
# ROS2 的补全本该由 setup.zsh 的 ament hook（share/ros2cli/package.dsv）自动
# 注册，但本机实测：走完 setup.zsh 后 _comps[ros2] 仍然是空的——函数
# _python_argcomplete 定义上了，complete 却没登记成功。所以这里显式补一次：
# bashcompinit 提供 complete 兼容层，再 source 官方补全脚本。
# bash 用户不用管，setup.bash 会自动注册好。
# 放在幂等判断之前，是为了让已经激活过的终端再敲一次 gpsenv 也能补上补全。
if [ -n "${ZSH_VERSION:-}" ] && command -v compdef >/dev/null 2>&1; then
    _ros_complete="$_GPS_ROS_DISTRO/share/ros2cli/environment/ros2-argcomplete.bash"
    if [ -f "$_ros_complete" ]; then
        autoload -U +X bashcompinit 2>/dev/null && bashcompinit
        . "$_ros_complete" 2>/dev/null
    fi
    unset _ros_complete
fi

# --- 幂等标记 ---------------------------------------------------------------
if [ "${_GPS_ENV_ACTIVE:-}" = "1" ] && [ "$_GPS_CHECK_ONLY" -eq 0 ]; then
    _gps_log "[env] 环境已激活，跳过（强制重载：unset _GPS_ENV_ACTIVE）"
    unset -f _gps_log _gps_warn
    unset _GPS_QUIET _GPS_CHECK_ONLY
    return 0 2>/dev/null || exit 0
fi

# --- 内部函数：加载单个工作空间（只用 local_setup.*） ----------------------
_gps_load_prefix() {
    _p_prefix="$1"
    _p_name="$2"

    if [ ! -d "$_p_prefix" ]; then
        _gps_warn "[env] ✗ ${_p_name}: 目录不存在 $_p_prefix（编译过它吗？）"
        return 1
    fi

    # zsh 用 local_setup.zsh，bash 用 local_setup.bash
    case "$_GPS_SHELL" in
        zsh) _p_file="$_p_prefix/local_setup.zsh" ;;
        *)   _p_file="$_p_prefix/local_setup.bash" ;;
    esac
    if [ ! -f "$_p_file" ]; then
        # 退回到 POSIX 版 local_setup.sh，zsh/bash 都能 source
        if [ -f "$_p_prefix/local_setup.sh" ]; then
            _p_file="$_p_prefix/local_setup.sh"
        else
            _gps_warn "[env] ✗ ${_p_name}: 缺 local_setup.*（未编译）"
            return 1
        fi
    fi

    # 已经在 AMENT_PREFIX_PATH 里就算加载过
    case ":${AMENT_PREFIX_PATH:-}:" in
        *":${_p_prefix}:"*|*":${_p_prefix}/"*)
            _gps_log "[env] · ${_p_name}（已加载）"
            return 0
            ;;
    esac

    # 关键：显式指定 COLCON_CURRENT_PREFIX，让 local_setup.* 只加载自己，
    # 不去 chain 编译时焊死的父链
    COLCON_CURRENT_PREFIX="$_p_prefix"
    export COLCON_CURRENT_PREFIX
    # shellcheck disable=SC1090
    . "$_p_file"
    unset COLCON_CURRENT_PREFIX

    _gps_log "[env] + ${_p_name}  <- ${_p_prefix}"
    return 0
}

# --- 体检模式 ---------------------------------------------------------------
if [ "$_GPS_CHECK_ONLY" -eq 1 ]; then
    printf '=== 环境体检（shell: %s）===\n' "$_GPS_SHELL"
    printf 'ROS 发行版: %s ' "$_GPS_ROS_DISTRO"
    if [ -d "$_GPS_ROS_DISTRO" ]; then printf '存在\n'; else printf '✗ 缺失\n'; fi
    printf '\n'
    printf '%-20s %-12s %s\n' "名称" "状态" "路径"
    printf '%-20s %-12s %s\n' "----" "----" "----"
    for _item in "${_GPS_OVERLAYS[@]}"; do
        _name="${_item%%|*}"
        _path="${_item##*|}"
        if [ ! -d "$_path" ]; then
            _status="✗ 缺目录"
        else
            case "$_GPS_SHELL" in
                zsh) _probe="$_path/local_setup.zsh" ;;
                *)   _probe="$_path/local_setup.bash" ;;
            esac
            if [ -f "$_probe" ] || [ -f "$_path/local_setup.sh" ]; then
                _status="可加载"
            else
                _status="✗ 未编译"
            fi
            unset _probe
        fi
        printf '%-20s %-12s %s\n' "$_name" "$_status" "$_path"
    done
    unset _item _name _path _status

    printf '\n--- 当前终端实测 ---\n'
    printf '  ROS_DISTRO    = %s\n' "${ROS_DISTRO:-<未设置>}"
    printf '  AMENT 条目    = %s\n' "$(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | grep -c .)"
    printf '  ros2 可执行   = %s\n' "$(command -v ros2 2>/dev/null || echo '✗ 找不到')"
    printf '  重复 fast_lio = %s 个\n' "$(printf '%s' "${AMENT_PREFIX_PATH:-}" | tr ':' '\n' | grep -c '/home/w/fastlio/install')"
    printf '\n提示：--check 只做静态检查。要看真实加载效果用 scripts/rosenv\n'
    unset -f _gps_log _gps_warn
    unset _GPS_QUIET _GPS_CHECK_ONLY _GPS_SHELL
    return 0 2>/dev/null || exit 0
fi

# ============================================================================
# 正式激活
# ============================================================================
_gps_log "[env] 激活 gps_ws 环境（shell: $_GPS_SHELL）..."

_failed=0

# 1) ROS 底层。必须按 shell 选对应的 setup，否则 zsh 下静默失败
case "$_GPS_SHELL" in
    zsh) _ros_setup="$_GPS_ROS_DISTRO/setup.zsh" ;;
    *)   _ros_setup="$_GPS_ROS_DISTRO/setup.bash" ;;
esac
if [ -f "$_ros_setup" ]; then
    # shellcheck disable=SC1090
    . "$_ros_setup"
    _gps_log "[env] + ros          <- $_GPS_ROS_DISTRO ($(basename "$_ros_setup"))"
else
    _gps_warn "[env] ✗ 找不到 $_ros_setup"
    _failed=1
fi
unset _ros_setup

# 2) 各 overlay
for _item in "${_GPS_OVERLAYS[@]}"; do
    _name="${_item%%|*}"
    _path="${_item##*|}"
    _gps_load_prefix "$_path" "$_name" || _failed=1
done
unset _item _name _path

# --- conda 与 ROS 的 Python 冲突检查 ---------------------------------------
_py="$(command -v python3 2>/dev/null)"
case "$_py" in
    *anaconda*|*miniconda*)
        _gps_warn "[env] ⚠ python3 指向 conda（$_py），ros2 可能异常。"
        _gps_warn "      临时解决：conda deactivate 后重新 source 本脚本"
        ;;
esac
unset _py

# --- 收尾 -------------------------------------------------------------------
if [ "$_failed" -eq 0 ]; then
    export _GPS_ENV_ACTIVE=1
    _gps_log "[env] 完成。验证："
    _gps_log "     ros2 pkg prefix fast_lio livox_ros_driver2 slam_bridge gps"
else
    unset _GPS_ENV_ACTIVE
    _gps_warn "[env] 部分工作空间未能加载（见上方 ✗），请先编译它们。"
fi
unset _failed

unset -f _gps_log _gps_warn _gps_load_prefix
unset _GPS_QUIET _GPS_CHECK_ONLY _GPS_SHELL
