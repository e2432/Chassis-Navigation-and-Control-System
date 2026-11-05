#!/usr/bin/env bash
# 依赖检查/安装。默认只检查（不动系统），加 --install 才真正 apt 安装。
#
#   ./tools/engineering/setup_dependencies.sh            # 检查缺失项
#   ./tools/engineering/setup_dependencies.sh --install  # 安装（需要 sudo）
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

DO_INSTALL=0
[ "${1:-}" = "--install" ] && DO_INSTALL=1

# 系统 ROS 2 依赖（Humble，Ubuntu 22.04 jammy）
APT_PACKAGES=(
  ros-humble-desktop
  ros-humble-navigation2
  ros-humble-nav2-bringup
  ros-humble-nav2-mppi-controller
  ros-humble-slam-toolbox
  ros-humble-pointcloud-to-laserscan
  ros-humble-tf2-tools
  ros-humble-teleop-twist-keyboard
  ros-humble-rmw-fastrtps-cpp
  python3-colcon-common-extensions
  python3-rosdep
)

# 可选依赖：缺失不阻塞构建，但会影响后续任务
#   liburdfdom-tools 提供 check_urdf，E05 校验 URDF 时需要
OPTIONAL_APT_PACKAGES=(liburdfdom-tools)

echo "== 检查 apt 依赖 =="
missing=()
for pkg in "${APT_PACKAGES[@]}"; do
  status=$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)
  if [ "$status" = "install ok installed" ]; then
    ver=$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null)
    printf '  [OK]      %-42s %s\n' "$pkg" "$ver"
  else
    printf '  [MISSING] %s\n' "$pkg"
    missing+=("$pkg")
  fi
done

echo
echo "== 检查 ROS 2 与构建环境 =="
if [ -f /opt/ros/humble/setup.bash ]; then
  echo "  [OK]      /opt/ros/humble 存在"
else
  echo "  [MISSING] /opt/ros/humble 不存在"
fi
echo "  python3 = $(command -v python3) ($(python3 -V 2>&1))"
case "$(command -v python3)" in
  /usr/bin/*) echo "  [OK]      当前 python3 是系统 Python，可直接构建" ;;
  *)          echo "  [WARN]    当前 python3 不是系统 Python（conda？）；构建请用 tools/engineering/build_workspace.sh" ;;
esac

echo
echo "== 检查 python 构建依赖（catkin_pkg 等）=="
if /usr/bin/python3 -c "import catkin_pkg" 2>/dev/null; then
  origin=$(dpkg -S "$(/usr/bin/python3 -c 'import catkin_pkg, os; print(os.path.dirname(catkin_pkg.__file__))')" 2>/dev/null | head -1 | cut -d: -f1)
  echo "  [OK]      catkin_pkg 可导入${origin:+（包来源：$origin）}${origin:-（系统 Python 可导入，注意它可能来自 pip 而非 apt）}"
else
  echo "  [MISSING] catkin_pkg 不可导入 —— ament_package_xml 会失败，影响所有 ament_cmake 包"
  missing+=(python3-catkin-pkg)
fi

echo
echo "== 可选依赖（不阻塞构建）=="
for pkg in "${OPTIONAL_APT_PACKAGES[@]}"; do
  status=$(dpkg-query -W -f='${Status}' "$pkg" 2>/dev/null || true)
  if [ "$status" = "install ok installed" ]; then
    printf '  [OK]      %s\n' "$pkg"
  else
    printf '  [OPTIONAL] %s（E05 校验 URDF 时需要；缺它时用 python urdf 解析替代）\n' "$pkg"
  fi
done

if [ ${#missing[@]} -gt 0 ]; then
  echo
  echo "缺失 ${#missing[@]} 个必需包。安装："
  echo "  sudo apt update && sudo apt install -y ${missing[*]}"
  if [ "$DO_INSTALL" = "1" ]; then
    sudo apt update && sudo apt install -y "${missing[@]}"
  fi
  exit 1
fi

echo
echo "== 全部依赖已满足 =="
echo "下一步：./tools/engineering/build_workspace.sh"
