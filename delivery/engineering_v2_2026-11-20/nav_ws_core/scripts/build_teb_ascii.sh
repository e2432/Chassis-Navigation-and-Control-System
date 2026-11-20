#!/usr/bin/env bash
# 在 ASCII 路径下构建官方 teb_local_planner —— 规避中文路径与 libg2o 的两个环境限制。
#
# 为什么需要这个脚本（都是实测踩出来的，详见 docs/9-1工程执行台账.md §I）：
#   1) 工作区路径含中文（…/桌面/…）时，rosidl 生成接口会把中文前缀截断，报
#      `list index: 1 out of range` / `Target dependency '/Chassis-…/ObstacleState.idl' does not exist`。
#      符号链接救不了（colcon 会 realpath 回中文路径），所以必须用 ASCII 镜像工作区构建。
#   2) `libg2o` 只有 apt 包（ros-humble-libg2o），而本机 sudo 需要密码。
#      用 `apt-get download` + `dpkg -x` 解包到本地前缀即可，**不需要 root**。
#   3) `FindG2O.cmake` 会把 include 目录定位到 `…/include/g2o` 本身，而代码里写的是
#      `#include <g2o/config.h>`，需要父目录 → 构建时显式给 -DG2O_INCLUDE_DIR。
#   4) `costmap_converter` 的 `ros2` 分支用了 `cv_bridge/cv_bridge.hpp`（cv_bridge v4 路径），
#      Humble 只有 `cv_bridge/cv_bridge.h`。只影响动态障碍转换器（TEB 默认用不到），
#      脚本会自动把那 6 个源文件从构建里排除。
#
# 用法：
#   scripts/build_teb_ascii.sh              # 构建到 ~/teb_build_ws，依赖解包到 ~/teb_prebuilt
#   source ~/teb_prebuilt/env.sh            # 之后启动 Nav2 前必须 source（提供 libg2o 的 .so）
#   source ~/teb_build_ws/install/setup.bash
#
# 版本（2026-11-15 定稿，见 docs/9-1参数定稿_2026-11-15.md）：
#   teb_local_planner  rst-tu-dortmund/teb_local_planner  branch humble-devel  (v0.9.1)
#   costmap_converter  rst-tu-dortmund/costmap_converter  branch ros2

set -euo pipefail

PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/../src" && pwd)"
WS="${TEB_ASCII_WS:-$HOME/teb_build_ws}"
PRE="${TEB_PREFIX:-$HOME/teb_prebuilt}"
TEB_BRANCH="${TEB_BRANCH:-humble-devel}"
CC_BRANCH="${CC_BRANCH:-ros2}"

echo "[1/5] 检查路径"
for p in "$WS" "$PRE"; do
    if [[ "$p" =~ [^[:ascii:]] ]]; then
        echo "[错误] 构建目录不能含非 ASCII 字符：$p" >&2; exit 1
    fi
done
mkdir -p "$WS/src" "$PRE"

echo "[2/5] 取官方源码（GitHub 主站可能不通，用 codeload 下 tarball）"
fetch() {  # $1=owner/repo  $2=branch  $3=解包后的期望目录名
    local name="${1##*/}"
    if [[ -d "$WS/src/$name-$2" ]]; then echo "  已存在 $name-$2"; return; fi
    local tmp; tmp="$(mktemp -d)"
    curl -sSL -o "$tmp/src.tgz" "https://codeload.github.com/$1/tar.gz/refs/heads/$2"
    tar xzf "$tmp/src.tgz" -C "$WS/src"
    rm -rf "$tmp"
    echo "  取到 $name-$2"
}
fetch rst-tu-dortmund/teb_local_planner "$TEB_BRANCH"
fetch rst-tu-dortmund/costmap_converter "$CC_BRANCH"

echo "[3/5] 解包 libg2o（无需 root）"
if [[ ! -f "$PRE/opt/ros/humble/lib/cmake/g2o/g2oConfig.cmake" ]]; then
    ( cd "$WS" && apt-get download ros-humble-libg2o )
    dpkg -x "$WS"/ros-humble-libg2o_*.deb "$PRE"
    rm -f "$WS"/ros-humble-libg2o_*.deb
fi
# 生成一个 env.sh，便于启动时 source（libg2o 的 .so 不在系统库路径里）
cat > "$PRE/env.sh" <<EOF
# 由 build_teb_ascii.sh 生成：把解包出来的 libg2o 加入运行时库路径
export LD_LIBRARY_PATH="$PRE/opt/ros/humble/lib/x86_64-linux-gnu:$PRE/opt/ros/humble/lib:\${LD_LIBRARY_PATH:-}"
EOF

echo "[4/5] Humble 兼容补丁：costmap_converter 排除 cv_bridge v4 专用的动态障碍转换器"
CC_CMAKE="$WS/src/costmap_converter-$CC_BRANCH/costmap_converter/CMakeLists.txt"
if [[ -f "$CC_CMAKE" ]] && ! grep -q "HUMBLE-PATCH" "$CC_CMAKE"; then
    python3 - "$CC_CMAKE" <<'PY'
import re, sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text(encoding="utf-8")
drop = ["costmap_to_dynamic_obstacles.cpp", "background_subtractor.cpp", "blob_detector.cpp",
        "Ctracker.cpp", "Kalman.cpp", "HungarianAlg.cpp"]
out = []
for ln in s.splitlines(True):
    if any(d in ln for d in drop) and not ln.lstrip().startswith("#"):
        out.append("  # [HUMBLE-PATCH] 排除：该分支依赖 cv_bridge/cv_bridge.hpp（cv_bridge v4），"
                   "Humble 只有 .h；TEB 默认只用 CostmapToPolygonsDBSMCCH\n")
        out.append("  # " + ln)
    else:
        out.append(ln)
p.write_text("".join(out), encoding="utf-8")
xml = p.parent / "costmap_converter_plugins.xml"
if xml.is_file():
    t = xml.read_text(encoding="utf-8")
    xml.write_text(re.sub(r"\s*<class[^>]*CostmapToDynamicObstacles[^>]*>.*?</class>", "", t, flags=re.S),
                   encoding="utf-8")
print("  已打补丁:", p)
PY
fi

echo "[5/5] colcon build"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO || true
set +u; source /opt/ros/humble/setup.bash; set -u
export CMAKE_PREFIX_PATH="$PRE/opt/ros/humble:$PRE/usr:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="$PRE/opt/ros/humble:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PRE/opt/ros/humble/lib/x86_64-linux-gnu:$PRE/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"
cd "$WS"
colcon build --packages-select teb_msgs costmap_converter teb_local_planner \
    --cmake-args -DG2O_INCLUDE_DIR="$PRE/opt/ros/humble/include" \
                 -DCMAKE_BUILD_RPATH="$PRE/opt/ros/humble/lib/x86_64-linux-gnu"

echo
echo "[完成] 启动 Nav2 前请先执行："
echo "  source $PRE/env.sh"
echo "  source $WS/install/setup.bash"
echo "自检：ros2 pkg prefix teb_local_planner && ros2 interface list | grep teb_msgs"
