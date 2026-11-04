# E06 全向速度输入检查（motion check）

- 完成日期：2026-10-31 ｜ 对应任务：E06（W2）
- 底盘模型：AGL-CHS-03-F01（八边形车体 + 四个 45° 全向轮）
- **E05 最终产出物固定为 `assets/scenes/e05_min_scene.usd`**（含底盘/地面/墙/箱体/灯光），
  URDF/xacro 是它的源，`results/w2/e05/` 是它的验收证据
- 证据目录：`results/w2/e06/run_kinematic/`（主验收）与 `results/w2/e06/run_physics/`（接触对照）
- 复算方式：

```bash
cd ${NAV_WS}
./tools/engineering/e06_run_all.sh results/w2/e06/run_kinematic kinematic 45   # 主验收（理想运动学）
./tools/engineering/e06_run_all.sh results/w2/e06/run_physics  physics  45     # 接触驱动对照
```

## 1. 结论

| 检查项 | kinematic（早期接口模型） | physics（接触驱动对照） |
|---|---|---|
| 前进 +0.1 m/s | **PASS** 0.297 m / 3 s | 0.082 m（指令的 27%），串扰 0.058 m |
| 后退 −0.1 m/s | **PASS** −0.297 m | −0.092 m，串扰 0.092 m |
| 左移 +0.1 m/s（`vy+`） | **PASS** +0.297 m | +0.089 m（30%），串扰 −0.088 m |
| 右移 −0.1 m/s（`vy−`） | **PASS** −0.297 m | −0.076 m，串扰 0.071 m |
| 逆时针 +0.3 rad/s | **PASS** +0.895 rad | **PASS** +0.799 rad（指令的 89%），位置漂移 0 |
| 顺时针 −0.3 rad/s | **PASS** −0.895 rad | **PASS** −0.798 rad |
| 组合运动（vx+, vy+, ω+） | **PASS** 位移 (0.238, 0.226) m，偏航 +0.597 rad | FAIL 位移 (0.040, −0.021) m |
| 零指令停车 | **PASS** 残余 0.0000 m/s | **PASS** 残余 0.0000 m/s |
| 指令超时停车（0.5 s） | **PASS** 停发后 0.65~2.0 s 内最大速度 0.0000 m/s | **PASS** 同上 |

合计：**kinematic 9/9 PASS**，physics 4/9（旋转与停车通过，平移不达标）。

**关键判断（必须写进报告，不要含糊）**：

1. E06 的验收是在**理想运动学模型**上完成的：指令 → 45° 轮逆解 → 位姿积分。
   该模型**不涉及轮地接触/摩擦/打滑**，因此**不能**当作“已完成轮地接触动力学验证”
   （路线书 E06 允许此阶段这么做，但要求明确标记 —— 本文即标记）。
2. 用**圆柱轮**做接触驱动时：旋转能跟到 89%，但平移只有指令的 ~30% 且串扰接近主轴量级。
   原因是真实全向轮靠**周向滚子**实现横向滑移，圆柱轮没有滚子，侧向被摩擦锁死、只能拖拽。
   这是一个**物理建模缺口**，不是接口 bug。
3. 结论用于后续任务：
   - 基准实验（E12/E13/E21）若需要接触真实的运动，必须给轮子加滚子几何或各向异性摩擦；
   - 现有 W3/W4 的定位/导航链路用 kinematic 模型即可，但报告须注明“运动学推进”，
     不能声称“接触动力学已验证”。

## 2. 接口与实现

### 2.1 速度输入契约（与 `docs/接口契约.md` 第 3 节一致）

| 分量 | 方向 | 取值 |
|---|---|---|
| `linear.x` | 前（机头） | ±2.0 m/s（PDF 空载 0–2 m/s） |
| `linear.y` | 左 | ±2.0 m/s |
| `angular.z` | 逆时针（右手） | ±1.0 rad/s（E06 测试取值；真实上限待 E06/E23 用轮速反推确认） |
| `linear.z`、`angular.x/y` | 必须为 0 | 超限值被丢弃，不参与运动 |

- 超时：**0.5 s** 未收到新指令 → 速度归零；
- 活性判定：`/cmd_vel` 数值可能重复，无法靠“值变化”判断是否还在发，因此测试用独立心跳
  `/e06/heartbeat`（`Twist.angular.x` 承载递增序号）。生产实现里这个活性应改为 ROS 2 侧的
  CAN/串口帧超时，心跳话题只是**测试脚手架**。

### 2.2 45° 全向轮逆解

轮心位置 `r_i = (±c, ±c)`，`c = 0.23546 m`；轮轴水平方位角 `φ = 45° / −45° / 135° / 225°`。
正角速度时轮心前进方向 `u_i = (sin φ, −cos φ)`（由 `ω×r` 推导，见脚本注释），于是：

```text
v_i(point) = (vx − ω·r_iy,  vy + ω·r_ix)
ω_wheel,i = v_i · u_i / R          R = 0.105 m
```

自检（脚本启动时打印）：`cmd → 轮速 → 最小二乘反解` 能还原指令，
纯 ω 时四轮同速、纯 vx 时对角轮反号 —— 与“45° 对角布局”一致。

示例（`cmd=(0.1, 0, 0)`）：`fl +0.673, fr −0.673, rl +0.673, rr −0.673 rad/s`
示例（`cmd=(0, 0, 0.3)`）：四轮同为 `−0.951 rad/s`

### 2.3 两种驱动模式（脚本 `tools/engineering/e06_omni_drive.py`）

| 模式 | 做法 | 用途 |
|---|---|---|
| `kinematic`（默认） | 关闭重力，按指令积分机体位姿，同时按逆解转动轮子做视觉一致 | E06 接口验收；W3/W4 导航链路先用它 |
| `physics` | 关节配速度驱动（kps=0，kds=1e4，maxEffort=80），靠轮地摩擦推动 | 量化接触现实，暴露滚子缺失问题 |

## 3. 测试序列（仿真时间，外部按 20 Hz 发指令）

| 阶段 | 指令 | 保持 |
|---|---|---|
| settle | 0 | 2 s |
| forward | (0.1, 0, 0) | 3 s |
| backward | (−0.1, 0, 0) | 3 s |
| left | (0, 0.1, 0) | 3 s |
| right | (0, −0.1, 0) | 3 s |
| ccw | (0, 0, 0.3) | 3 s |
| cw | (0, 0, −0.3) | 3 s |
| combined | (0.1, 0.05, 0.2) | 3 s |
| zero_cmd | (0, 0, 0) | 1.5 s |
| cmd_drop | (0.1, 0, 0) 后**停发** | 1.5 s + 静默 2.5 s |

各动作之间插 1 s 零速间隔，便于分离阶段轨迹。

## 4. 判定方法

用**仿真时间窗内的机体坐标系位移**判定（世界系位移按阶段起始 yaw 旋转回机体系）：

- 平移：主轴位移 > 0.15 m、另一轴串扰 < 0.04 m、|Δyaw| < 0.10 rad；
- 旋转：|Δyaw| > 0.45 rad、位置漂移 < 0.12 m；
- 零指令：平均残余速度 < 0.005 m/s 且位移 < 0.01 m；
- 超时：停发后 0.65–2.0 s 窗口内最大速度 < 0.02 m/s，且 `timeout` 标志被置位。

kinematic 实测（`results/w2/e06/run_kinematic/summary.txt`）：

```text
forward_+0.1     Δx +0.297 m   Δy 0.000   Δyaw 0.000   实测 vx 0.099
backward_-0.1    Δx −0.297 m   Δy 0.000   Δyaw 0.000   实测 vx −0.099
left_+0.1        Δx 0.000      Δy +0.297  Δyaw 0.000
right_-0.1       Δx 0.000      Δy −0.297  Δyaw 0.000
ccw_+0.3         Δyaw +0.895 rad   位置漂移 0.000
cw_-0.3          Δyaw −0.895 rad   位置漂移 0.000
combined         Δ(0.238, 0.226) m  Δyaw +0.597 rad
zero_cmd         残余速度 0.0000 m/s
cmd_drop         停发后最大速度 0.0000 m/s，过渡期(0.15~0.6 s) 0.100 m/s -> 0
```

对照 physics 实测（`results/w2/e06/run_physics/summary.txt`）：

```text
forward_+0.1     Δ(0.082, 0.058) m   实测 vx 0.027   （指令的 27%）
left_+0.1        Δ(−0.088, 0.089) m                 （串扰与主轴同量级）
ccw_+0.3         Δyaw +0.799 rad    位置漂移 0.000    （指令的 89%）
combined         Δ(0.040, −0.021) m  Δyaw +0.476 rad
```

## 5. 排查记录（踩坑）

1. **`ROS_DOMAIN_ID` 两侧不一致**：仿真侧没设（默认 0）、外部设 42 → 外部看不到 `/clock`，
   测试直接空跑。已在 `e06_run_all.sh` 统一导出。
2. **时间基准**：`omni.timeline.get_current_time()` 在本用例里只回绕在 0~0.033 s，
   与 `/clock` 不同源；改用 OG 节点 `IsaacReadSimulationTime` 的输出作为唯一时间基准。
3. **OG 属性赋值类型**：`ROS2PublishOdometry.inputs:orientation` 是 `quatd[4]`，
   必须传普通序列（IJKR 顺序），传 `Gf.Quatd` 会报 `list of size 1`。
4. **URDF 导入的关节默认驱动不足以做速度控制**：必须先
   `switch_dof_control_mode("velocity")` + `set_gains(kps=0, kds=1e4)` + `set_max_efforts(80)`，
   否则轮子完全不转（第一轮 physics 测试全是 0）。

## 6. 与算法成员需要对齐的点

- [ ] 全向模型假设：本模型按“4 个 45° 全向轮，轮面沿斜面”建模，是否为算法侧 MPC 采用的模型？
- [ ] `angular.z` 上限（当前测试取 ±1.0 rad/s）、加速度限制（仍未定）；
- [ ] 基准实验用哪套运动模型：kinematic 推进（快、可重复）还是补滚子后的接触驱动（慢、真实）；
- [ ] 若要求接触真实，需在 E12/E13 前决定滚子几何方案与性能预算（4 × N 个滚子体）。

## 7. 未完成 / 边界

- 轮地摩擦、打滑、越障（PDF 10 mm）、爬坡（5–6°）**均未验证**；
- 轮速上限、加速度限制未标定；
- 未与真机对照（本期不含真机迁移）；
- `physics` 模式的轮子仍是圆柱，**不能**作为全向运动学正确性的物理证据。

## 8. 在 Isaac Sim GUI 里看底盘运动

`tools/engineering/e06_omni_drive.py` 是**无头**驱动（自己起一个无窗口实例），所以 GUI 窗口里看不到它。
要在窗口里看运动，用 `tools/engineering/e06_gui_drive.py`（在你已打开的 Isaac Sim 里跑）：

### 8.1 演示模式（不需要 ROS，最快的“看一眼”）

1. GUI 里 `File > Open` 打开 `assets/scenes/e05_min_scene.usd`；
2. `Window > Script Editor` → `File > Open` 选 `tools/engineering/e06_gui_drive.py` → `Run`
   （或把内容粘进 Script Editor 后点 Run）；
3. 脚本会 `timeline.play()` 并按序列自动走：
   前进 → 后退 → 左移 → 右移 → 原地逆时针 → 原地顺时针 → 斜向 → 绕圈 → 停车，循环往复。
   控制台每次切换阶段会打印 `[E06-GUI] 演示: <阶段名>`；
4. 想停：在 Script Editor 里执行 `e06_gui_stop()`（会复位到原点并恢复重力）。

### 8.2 两种跑法

| 方式 | 入口 | 需要 ROS |
|---|---|---|
| 一、演示序列 | `e06_gui_drive(ros=False)`（Run 文件即默认） | 不需要 |
| 二、跟随话题 | `e06_gui_drive(ros=True)`，外部发 `/cmd_vel` | 需要（步骤见 8.3） |

### 8.3 用 ROS 话题开（方式二）的完整步骤

**第 0 步：确认 DDS 域一致（最容易踩的坑）**

GUI 脚本启动时会打印一行 `[E06-GUI] ROS_DOMAIN_ID=...（GUI 进程视角）`。
GUI 进程没设这个变量就是 **0**；外部终端必须**一致**：

```bash
# 若 GUI 打印的是 “<未设置=0>”，外部终端就不要设，或显式设 0：
export ROS_DOMAIN_ID=0
```

**第 1 步：GUI 里启动 ROS 模式**

```python
# Script Editor（先 Run 一次脚本文件，再执行）
e06_gui_drive(ros=True)                 # 默认安全边界 0.9 m
e06_gui_drive(ros=True, safe_radius=2.5)  # 想开远一点
```

**第 2 步：外部终端确认能看到话题**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0          # 与 GUI 一致
ros2 topic list                 # 应看到 /clock /cmd_vel /e06/heartbeat
ros2 topic info /cmd_vel -v     # Subscription count 应为 1（GUI 侧）
```

**第 3 步：发指令**

```bash
# 前进 0.1 m/s
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 左移（全向）
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 原地逆时针转
ros2 topic pub --rate 20 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

`Ctrl-C` 后底盘会**继续保持最后一次速度**（因为只发 `/cmd_vel` 时不判超时）。
要停车：再发一条零速度，或在 Script Editor 执行 `e06_gui_stop()`。

**第 4 步（可选）：带心跳，验证 0.5 s 超时停车**

```bash
# 现成脚本：内部同时发 /cmd_vel 与 /e06/heartbeat，跑完整的十段序列
/usr/bin/python3 ${NAV_WS}/tools/engineering/e06_motion_test.py --out /tmp/e06_gui
```

跑完后 `Ctrl-C` 即可；停发心跳 0.5 s 内底盘自动停。

### 8.4 GUI 脚本的几个行为说明

| 行为 | 说明 |
|---|---|
| 时间基准 | 取 OG `IsaacReadSimulationTime` 与 `timeline` 时间的较大者；bridge 不可用时自动退回 timeline |
| 重力 | kinematic 模式会**临时关闭重力**（不做轮地支撑），`e06_gui_stop()` 恢复原值 |
| 轮子 | 按逆解同步转动（视觉一致），但推进方式仍是**理想运动学**，不是接触驱动 |
| 重复运行 | 脚本开头会自动 `e06_gui_stop()`，避免叠加多个 update 订阅 |
| 打印 | 控制台出现 `[E06-GUI] cb#1 sim_t=...` 表示驱动回调已在跑 |
| 指令有效期 | 只发 `/cmd_vel` 时指令持续有效；同时发 `/e06/heartbeat` 才会 0.5 s 超时停车（见 8.5） |
| 重复调用 | 先演示后 `ros=True` 是安全的：会先删掉同名旧图再重建 |

### 8.5 已做的验证

在无头实例里加载同一份脚本跑 360 帧（`/tmp/test_gui_drive.py` 的做法）：

```text
[E06-GUI] articulation 已初始化 (DOF=4)
frame  60  x=0.148  (前进 0.15 m/s × 1 s)
frame 120  x=0.298  (前进 2 s，位移 0.298 m ≈ 0.15×2)
[E06-GUI] 演示: 后退 vx=-0.15
frame 240  x=0.148  (后退中)
```

即脚本的“回调 → 逆解 → 写位姿”链路在 GUI 环境里同样成立；GUI 与无头的差别只有是否开窗口。

### 8.6 GUI 脚本的三个已知坑（2026-11-01 修复）

1. **`OmniGraphError: Failed to wrap graph in node`**
   先跑过演示模式（已建 `/E06GuiDrive`），再执行 `e06_gui_drive(ros=True)` 会在同一路径重复建图。
   修复：建图前先 `RemovePrim` 旧图，失败时清理并重试一次。重复调用现在是安全的。

2. **ROS 模式下指令 0.5 s 后莫名归零**
   原先用“值变化”判断发布方是否还活着；而 `ros2 topic pub` 反复发**相同值**，
   于是被判超时、指令归零（实测位移只有 0.050 m = 0.1 m/s × 0.5 s）。
   桥接侧无法区分“值没变但仍在发”和“已停发”，所以改为：

   | 外部行为 | GUI 脚本表现 |
   |---|---|
   | 只发 `/cmd_vel`（不发心跳） | **指令持续有效**，不发就一直在走（方便 `ros2 topic pub` 调试） |
   | 同时发 `/e06/heartbeat`（`Twist.angular.x` = 递增序号） | 心跳停 0.5 s → **自动停车** |

   实测（无头环境、RTF≈1）：
   - 无心跳、发 3 s 后停发：位移 **2.466 m**（指令一直有效，25 s 都在走）
   - 有心跳、发 2 s 后停发：位移 **0.278 m**（≈ 0.1 m/s × 2.6 s，即 2 s + 0.5 s 超时）
   - 两次都 `dy=0.000, dyaw=0.000`，纯前进无侧移

   生产环境里这个“活性”应该由 ROS 2 侧的底盘驱动节点做（收到 CAN/串口帧即刷新超时），
   GUI 脚本的心跳话题只是调试脚手架；超时停车的正式验收见第 1 节（headless 驱动 9/9 PASS）。

3. **`PhysX error: setDriveTarget() only supports target angle in range [-2Pi, 2Pi]`**
   现象：跑一段时间后（实测 sim_t≈729 s）突然报这条，接着
   `PhysX has reported too many errors, simulation has been stopped`，仿真自己停了。
   **与碰撞无关**（用户已确认没撞到东西）。

   原因：轮子是 **continuous（连续）关节**，而驱动里用 `joint_q += ω·dt` 一直累加转角，
   跑久了必然越过 ±2π；PhysX 的 `setDriveTarget` 对 revolute 关节只接受 ±2π 以内的目标角，
   于是每帧报一次错，累积到阈值就停机。

   修复：把累加后的角度**回卷到 [-π, π)**：

   ```python
   joint_q = (joint_q + math.pi) % (2.0 * math.pi) - math.pi
   ```

   轮子是圆柱、回转对称，回卷在视觉上完全看不出来。同样的问题在无头驱动
   `e06_omni_drive.py` 里也存在（只不过短时测试没跑够久），一并修了。

   修复验证（无头、原地旋转 100 s ≈ 轮子 30 圈）：
   `setDriveTarget` 报错 **0** 次、`too many errors` **0** 次，
   `joint_q` 始终在 [-1.75, +π] 区间内（回卷生效）。

   顺带修掉一个告警：`Articulation needs to be initialized` —— 之前在建驱动时就读了
   `dof_names`，改成 initialize 之后再读并校正关节顺序。

### 8.7 运动学模式的“隐形墙”（安全边界）

运动学模式直接写位姿、不理会碰撞，所以**开进箱体/墙不会产生物理阻挡**，
反而会让 PhysX 看到位姿嵌进静态物体。为此外加了安全边界（默认 0.9 m，障碍在 ≥1.3 m）：

| 模式 | 越界行为 |
|---|---|
| 演示（`ros=False`） | 复位到原点（保证循环干净） |
| ROS（`ros=True`） | **限位**：停止越界方向的速度，相当于隐形墙（不会把车瞬移走） |

实测（ROS 模式持续前进 25 s，目的地原本是 x=1.5 m 的箱体）：
位置停在 **x=0.898 m** 不再前进，`clamps=1427`，PhysX 报错 **0**。
