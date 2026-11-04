# robot_description

**状态：W2/E05「最小底盘模型」已完成**（URDF + Isaac Sim 导入验收）。
详细参数、假设清单与验收结论见 `docs/engineering/robot_model.md`。

## 职责

- 机器人几何、质量/惯量、关节轴、轮径、传感器外参、导航 footprint 的唯一来源；
- 供 `robot_state_publisher`、RViz、Nav2 footprint、安全层外缘计算共同使用。

## 当前内容

| 路径 | 说明 |
|---|---|
| `urdf/agl_chs_03_f01.urdf.xacro` | 最小底盘模型：AGL-CHS-03-F01，参数化（xacro arg 可覆盖），惯量解析计算 |
| `meshes/agl_chassis_octagon.stl` | 按三视图生成的八边形车体网格（由 `tools/engineering/gen_chassis_mesh.py` 生成，凸体） |
| `config/robot_geometry.yaml` | 参数契约：PDF 实测值 + 显式标注的假设 + 上一代参数（仅对照） |
| `meshes/` | 空（模型用基本体建模，不依赖网格；上半身/外观网格待整机提供） |

平面形状按 PDF 第 3 页三视图：**旋转 45° 的方形 + 四角截平**的八边形，
车体 643 × 656 mm，四个 45° 斜面各装一个全向轮（轮面沿斜面、轮轴水平 45°），
跨轮距离 726 mm（图纸 726 尺寸线与轮外缘相切），X/Y 外接包络 662 mm。

模型包含 10 个 link：`base_footprint → base_link → {4 轮, 2 雷达, IMU, 上装安装板帧}`，
4 个连续关节（`base_wheel_{fl,fr,rl,rr}_joint`，与既有控制桥命名一致）。

## 校验与验收

```bash
source /opt/ros/humble/setup.bash
xacro src/robot_description/urdf/agl_chs_03_f01.urdf.xacro > /tmp/chassis.urdf
check_urdf /tmp/chassis.urdf

# Isaac Sim 导入 + 静态稳定性 + 落体 + USD 回读（headless）
./tools/engineering/e01_run_sim_startup.sh results/w2/e05 tools/engineering/e05_import_check.py results/w2/e05
```

最近一次结果（2026-10-31，`results/w2/e05/import_check.txt`）：
跨轮 726.0 mm、外接包络 661.8 mm、base_link z=0.1050 m、底盘离地 0.0250 m、
水平漂移 0、倾角 0°、落体后回到 0.1050 m，**14 项检查全 PASS**。

## 还没有的东西（不要假装有）

- **整机 footprint**：当前只覆盖底盘 0.726 × 0.726，上半身/外伸部件未计入；
- **真实尺寸与安装位姿**：轮宽、轮心间距、雷达/IMU 位姿均为假设（见 `docs/engineering/robot_model.md` 5.1）；
- **外观网格**：没有 STL/DAE，只有基本体；
- **接触/摩擦参数**：URDF 不表达，需在 Isaac Sim 侧用 PhysicsMaterial 配置（E06）；
- **上半身**：升降/双臂/头部不在本期最小模型范围内。
