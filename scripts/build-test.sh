#!/usr/bin/env bash
# ============================================================================
# build-test.sh —— 单独构建 TestChassis 自测程序
#
# 为什么需要它：
#   colcon build --packages-select gps 会连带编译 gps_node.cpp，
#   目前 map/map.h 里 GraphEdge 未定义导致整个包失败，
#   连带 test_chassis 也出不来。这个脚本用**独立 cmake 目录**
#   只编 test_chassis 这一个 target，完全绕开 gps_node。
#
# 用法：
#   scripts/build-test.sh               # 构建（Debug，跑一遍自测）
#   scripts/build-test.sh --run         # 构建后运行自测
#   scripts/build-test.sh --release     # Release 构建
#   scripts/build-test.sh --clean       # 删掉独立构建目录重来
#   scripts/build-test.sh --install     # 装到 install/gps/lib/gps/
#   scripts/build-test.sh --clean --release --run
#
#   # 把参数透传给 test_chassis 本身（-- 之后的部分原样传递，隐含 --run）
#   scripts/build-test.sh -- --live /dev/ttyUSB0
#   scripts/build-test.sh -- --live /dev/ttyUSB0 50 5   # 摇杆量 50，发 5 秒
#
# 退出码（和 test_chassis 一致，方便脚本判断）：
#   0  自测（及实发）全部成功
#   1  自测有失败项
#   2  实发阶段失败：串口设备不存在 / 无权限 / 打不开 / 写失败
#
# 产物：
#   .build-test/test_chassis
#
# 注意：这个脚本和 colcon 的 build/ 完全隔离，互不干扰。
#       GraphEdge 修好后仍然建议用 colcon build，本脚本仅用于快速迭代。
# ============================================================================
set -euo pipefail

HERE="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
WS="$(dirname "$HERE")"
PKG="$WS/src/gps"
BUILD_DIR="$WS/.build-test"
TARGET="test_chassis"

# ---------------------------------------------------------------- 参数
DO_RUN=0
DO_CLEAN=0
DO_INSTALL=0
BUILD_TYPE=Debug
EXTRA_ARGS=()          # "--" 之后的参数，原样交给 test_chassis

while [ "$#" -gt 0 ]; do
    arg="$1"
    shift
    case "$arg" in
        --)
            # 剩余全部透传给二进制（典型：-- --live /dev/ttyUSB0）
            EXTRA_ARGS=("$@")
            break
            ;;
        --run|-r)     DO_RUN=1 ;;
        --clean|-c)   DO_CLEAN=1 ;;
        --install|-i) DO_INSTALL=1 ;;
        --release)    BUILD_TYPE=Release ;;
        --debug)      BUILD_TYPE=Debug ;;
        -h|--help)
            # 打印文件头注释，直到第一行非注释（不写死行号，改注释不会错位）
            awk 'NR==1{next} /^#/ { sub(/^# ?/, ""); print; next } { exit }' "$0"
            exit 0
            ;;
        *)
            echo "未知参数: $arg（用 --help 看用法）" >&2
            echo "提示：要传参数给 test_chassis，用 -- 分隔，例如：" >&2
            echo "      $0 --run -- --live /dev/ttyUSB0" >&2
            exit 2
            ;;
    esac
done

# 带了透传参数就意味着要跑程序
if [ "${#EXTRA_ARGS[@]}" -gt 0 ]; then
    DO_RUN=1
fi

# ---------------------------------------------------------------- 环境
# 独立进程里 source，不影响你当前终端
if [ -z "${ROS_DISTRO:-}" ] || [ ! -d "/opt/ros/${ROS_DISTRO:-}" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        # ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等），
        # 和 set -u 冲突，source 期间临时关掉
        set +u
        # shellcheck disable=SC1091
        . /opt/ros/humble/setup.bash
        set -u
    else
        echo "✗ 找不到 /opt/ros/humble/setup.bash" >&2
        exit 1
    fi
fi

if ! command -v cmake >/dev/null 2>&1; then
    echo "✗ 没装 cmake" >&2
    exit 1
fi

JOBS="$(getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)"

# ---------------------------------------------------------------- 清理
if [ "$DO_CLEAN" -eq 1 ]; then
    if [ -d "$BUILD_DIR" ]; then
        echo "→ 清理 $BUILD_DIR"
        rm -rf "$BUILD_DIR"
    fi
fi

# ---------------------------------------------------------------- 配置
INSTALL_PREFIX=""
if [ "$DO_INSTALL" -eq 1 ]; then
    INSTALL_PREFIX="$WS/install/gps"
fi

if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "→ 配置（$BUILD_TYPE）→ $BUILD_DIR"
    # BUILD_TESTING=OFF：跳过 ament_lint_auto，那些只跟代码风格有关，
    #                     还会额外拖一堆测试依赖进来
    cmake -S "$PKG" -B "$BUILD_DIR" \
          -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
          -DBUILD_TESTING=OFF \
          ${INSTALL_PREFIX:+-DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"} \
          >/dev/null
    echo "  ✓ 配置完成"
else
    echo "→ 复用已有配置：$BUILD_DIR"
    # 保证缓存里的 BUILD_TYPE 和安装前缀和本次参数一致
    cmake -S "$PKG" -B "$BUILD_DIR" \
          -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
          -DBUILD_TESTING=OFF \
          ${INSTALL_PREFIX:+-DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX"} \
          >/dev/null
fi

# ---------------------------------------------------------------- 构建
echo "→ 构建 target: $TARGET（-j$JOBS）"
cmake --build "$BUILD_DIR" --target "$TARGET" -j"$JOBS"

BIN="$BUILD_DIR/$TARGET"
if [ ! -x "$BIN" ]; then
    echo "✗ 构建完成但找不到可执行文件: $BIN" >&2
    exit 1
fi
echo "  ✓ $BIN"

# ---------------------------------------------------------------- 安装
# 注意：不能用 cmake --install，它会尝试安装所有 target（包括没编译的
#       gps_node），直接报 "No such file or directory"。这里只拷这一个文件。
if [ "$DO_INSTALL" -eq 1 ]; then
    DEST="$WS/install/gps/lib/gps"
    echo "→ 安装到 $DEST/"
    mkdir -p "$DEST"
    cp -f "$BIN" "$DEST/$TARGET"
    chmod +x "$DEST/$TARGET"
    echo "  ✓ $DEST/$TARGET"
fi

# ---------------------------------------------------------------- 运行
if [ "$DO_RUN" -eq 1 ]; then
    if [ "${#EXTRA_ARGS[@]}" -gt 0 ]; then
        echo "→ 运行（透传参数：${EXTRA_ARGS[*]}）"
    else
        echo "→ 运行自测"
    fi
    echo "------------------------------------------------------------"
    set +e
    # ${arr[@]+...} 防止老 bash 在 set -u 下把空数组当未定义变量
    "$BIN" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
    rc=$?
    set -e
    echo "------------------------------------------------------------"
    # 退出码和 test_chassis 约定一致：0 成功 / 1 自测失败 / 2 实发失败
    case "$rc" in
        0) echo "✓ 运行成功" ;;
        1) echo "✗ 自测有失败项（见上）" >&2 ;;
        2) echo "✗ 实发失败：串口设备不存在 / 无权限 / 打不开 / 写失败（见上）" >&2 ;;
        *) echo "✗ 退出码 $rc（见上）" >&2 ;;
    esac
    exit "$rc"
fi

echo "✓ 完成。运行： $BIN"
echo "  带参数运行： $BIN --live /dev/ttyUSB0 [摇杆量] [秒数]"
