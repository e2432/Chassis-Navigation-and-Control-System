# scenes/

可直接用 Isaac Sim 打开的场景文件（`File > Open` 后按 **Play**）。

| 文件 | 内容 | 生成方式 |
|---|---|---|
| `e05_min_scene.usd` | E05 最小场景：底盘 + 地面(50 m) + 一面墙(+x 3 m) + 一个箱体(+x 1.5 m) + 平行光/环境光 + 物理场景 | `./tools/engineering/e01_run_sim_startup.sh results/w2/e05 tools/engineering/make_min_scene.py` |

说明：

- 场景用**相对引用**指向 `../robot_a_chassis_agl_chs03_f01.usd`，因此整个 `assets/` 目录必须整体保留；
- 箱体是 20 kg 刚体，可以被推动；墙是静态碰撞体；
- 预览截图：`results/w2/e05/scene_preview.png`（headless RTX 渲染）。
