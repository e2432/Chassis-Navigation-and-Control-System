# 依赖说明（E03 产出）

本文记录**构建与运行**本工作空间所需的一切，以及本机上实际验证过的版本。
环境版本冻结见 `docs/engineering/environment.md`。

## 1. 一条命令检查

```bash
./tools/engineering/setup_dependencies.sh          # 只检查，不动系统
sudo ./tools/engineering/setup_dependencies.sh --install   # 缺什么装什么
```

## 2. 必需依赖（本机实测已装）

| 依赖 | 本机版本 | 用途 |
|---|---|---|
| Ubuntu 22.04.5 LTS | — | 系统底座 |
| ROS 2 Humble（`/opt/ros/humble`） | 387 个 `ros-humble-*` 包 | 全部 ROS 2 包构建与运行 |
| `ros-humble-desktop` | 0.10.0-1jammy | rclpy/rclcpp/RViz2 |
| `ros-humble-navigation2` + `nav2-bringup` | 1.1.20-1jammy | E11/E17 导航与基线 |
| `ros-humble-nav2-mppi-controller` | 1.1.20-1jammy | MPPI 基线（E17） |
| `ros-humble-slam-toolbox` | 2.6.10-1jammy | 既有包的 SLAM 模式（可选支线） |
| `ros-humble-pointcloud-to-laserscan` | 2.0.1-3jammy | 既有包的 `/livox/lidar → /scan` |
| `ros-humble-tf2-tools` | 0.25.20-2jammy | `tf2_echo` 核对 TF |
| `ros-humble-teleop-twist-keyboard` | 2.4.1-1jammy | 早期手动速度自检（必须经安全层） |
| `ros-humble-rmw-fastrtps-cpp` | 6.2.10-1jammy | DDS（两侧必须一致） |
| `python3-colcon-common-extensions` | 0.3.0-100 | `colcon build` |
| `python3-rosdep` | 0.26.0-1 | `rosdep install`（可选路线） |
| `catkin_pkg`（系统 Python 可导入） | 由 pip 提供于 `/usr/local/lib/python3.10/dist-packages` | `ament_package_xml` 必需 |
| gcc / cmake / git | 11.4.0 / 3.22.1 / 2.34.1 | 构建与版本管理 |

可选（不影响构建）：`liburdfdom-tools`（提供 `check_urdf`，E05 校验 URDF 用）。

仿真侧依赖见 `docs/engineering/environment.md`：Isaac Sim 5.1（conda `env_isaaclab`，Python 3.11）
**不是** colcon 的依赖，两者环境必须分开。

## 3. 两个必须知道的坑（都是实测）

### 坑 1：conda base 在 PATH 里会让 `colcon build` 失败

现象：

```text
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error at .../ament_package_xml.cmake:95 (message)
  execute_process(${CONDA_ROOT}/bin/python3 ... package_xml_2_cmake.py ...)
```

原因：`ament_package_xml` 用 PATH 里的 `python3`，本机 PATH 第一顺位是 conda base 的
Python 3.13，它没有 `catkin_pkg`。

处理：**构建一律用** `./tools/engineering/build_workspace.sh`（内部把 PATH 收敛到系统路径）。
不要直接用裸 `colcon build`，也不要把 `pip install catkin_pkg` 装进 conda base 来“修好”——
那会让构建悄悄依赖错误的解释器。

### 坑 2：`package.xml` 的邮箱必须是合法格式

`<maintainer email="engineering@todo">` 会被 colcon 判为 `Invalid email`，
导致整个包不被识别（表现为 `colcon build` 里包“消失”）。新包请用 `xxx@example.com` 形式。

## 4. rosdep 路线（可选）

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

本机尚未执行 `rosdep init/update`（需要网络与 sudo）。在能上网的机器上首次执行：

```bash
sudo rosdep init && rosdep update
```

## 5. 构建与验证

```bash
./tools/engineering/build_workspace.sh            # 全量构建（8 个包）
./tools/engineering/acceptance_check.sh           # 完整验收（资产 + 依赖 + 构建 + 接口）
source install/setup.bash               # 之后才能用 ros2 launch / python3 -m
```
