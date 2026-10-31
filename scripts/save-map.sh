#!/usr/bin/env bash
# ============================================================================
# save-map.sh —— 保存 FAST_LIO 的先验地图（PCD）
#
# 背景（为什么需要这个脚本）：
#   这份 FAST_LIO 建出来的图只在内存 pcl_wait_pub 里，由 publish_map() 以
#   1Hz 累积。唯一能落盘的是 /map_save 服务（std_srvs/srv/Trigger）→
#   save_to_pcd() → 写参数 map_file_path。
#
#   两个坑：
#     1) laserMapping.cpp 里「退出时保存 PCD/scans.pcd」那段被整段注释掉了，
#        所以 Ctrl-C 不会生成任何文件（上游 README 说会生成，别信）。
#     2) map_file_path 若是相对路径（默认 "./test.pcd"），落点是节点进程的
#        当前目录（ros2 launch 时 = 你敲命令的目录），而且写失败也不报错，
#        服务照样返回 success=True。
#
# 用法：
#     scripts/save-map.sh            # 保存到节点当初启动时的 map_file_path
#     scripts/save-map.sh --dry-run  # 只看会存到哪，不写
#
# 前提：FAST_LIO 正在运行，且 config 里 map_en: true（否则没累积点）
# ============================================================================
set -uo pipefail

HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ros2 >/dev/null 2>&1; then
    # env.sh 会 source /opt/ros/humble/setup.bash，里面用了未定义变量，
    # 所以先临时关掉 -u，否则 set -u 会把 ROS 自己的脚本搞崩。
    set +u
    # shellcheck disable=SC1091
    . "$HERE/env.sh" --quiet || { printf '加载 ROS 环境失败\n' >&2; exit 1; }
    set -u
fi

NODE="${FASTLIO_NODE:-/laser_mapping}"
SERVICE="${MAP_SAVE_SERVICE:-/map_save}"
DRY_RUN=0

for arg in "$@"; do
    case "$arg" in
        --dry-run|-n) DRY_RUN=1 ;;
        -h|--help)    sed -n '2,25p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *)            printf '未知参数：%s（看 --help）\n' "$arg" >&2; exit 2 ;;
    esac
done

die()  { printf '\033[31m[保存失败]\033[0m %s\n' "$*" >&2; exit 1; }
ok()   { printf '\033[32m[OK]\033[0m %s\n' "$*"; }
warn() { printf '\033[33m[警告]\033[0m %s\n' "$*" >&2; }
info() { printf '[save-map] %s\n' "$*"; }

# --- 1) 节点在不在 ----------------------------------------------------------
if ! ros2 node list 2>/dev/null | grep -qx -- "$NODE"; then
    die "找不到节点 $NODE。先起 FAST_LIO：ros2 launch fast_lio mapping.launch.py ..."
fi

# --- 2) 取启动时的 map_file_path（运行时改参数没用，C++ 里的变量不会重读） --
cur_path="$(ros2 param get "$NODE" map_file_path 2>/dev/null | sed -n 's/^String value is: //p')"
[ -n "$cur_path" ] || die "读不到 $NODE 的参数 map_file_path"

case "$cur_path" in
    /*) : ;;
    *)  warn "map_file_path 是相对路径：$cur_path"
        warn "   落点是“节点进程的当前目录”，不是 ~/.ros，也不是 FAST_LIO 的 PCD/"
        warn "   下次起 FAST_LIO 时用绝对路径："
        warn "   ros2 launch fast_lio mapping.launch.py config_path:=$HERE/../config config_file:=fastlio_mid360.yaml"
        ;;
esac

info "目标文件：$cur_path"

if [ "$DRY_RUN" -eq 1 ]; then
    info "dry-run：不调用服务"
    exit 0
fi

# --- 3) 调服务 --------------------------------------------------------------
info "调用 $SERVICE ..."
resp="$(ros2 service call "$SERVICE" std_srvs/srv/Trigger 2>&1)" \
    || die "服务调用异常：$resp"
printf '%s' "$resp" | grep -q 'success=True' \
    || { printf '%s\n' "$resp" >&2; die "服务返回 success=False"; }

# --- 4) 校验文件（服务本身不校验；写失败也回 success=True） ----------------
[ -f "$cur_path" ] || die "服务说成功，但文件不存在：$cur_path"

size=$(stat -c %s -- "$cur_path")
points=$(sed -n 's/^POINTS //p' -- "$cur_path" | head -1)
: "${points:=?}"

ok "已保存：$cur_path"
printf '      大小 %s 字节，点数 %s\n' "$size" "$points"

if [ "$points" != "?" ] && [ "$points" -lt 1000 ]; then
    warn "点数偏少。检查 map_en 是否为 true、建图时雷达是否真收到点"
    warn "（RViz 里 /Laser_map 有东西吗？）"
fi
