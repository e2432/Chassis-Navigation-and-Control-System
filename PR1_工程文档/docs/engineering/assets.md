# 工程资产分发与许可边界

本仓库仅分发自建底盘 USD、其 `assets/configuration/` 四个组合层、自建 STL 和五类场景 USD。
保留 USD 原始相对目录；`assets/robot_a_chassis_agl_chs03_f01.usd` 不能移到 `scenes/`，否则场景引用会失效。
运行 `./tools/engineering/verify_assets.sh` 校验 `assets/MANIFEST.sha256`。

既有 `assets/robot_a_isaac/`、`assets/reference/`、`scene_robot_a_nav.usd`、`scene_simulation.usd`
许可未确认，不随本次交付分发。需要旧链路时向原资产持有人申请授权，再在本地按原目录放置；
默认 M1 链路使用自建底盘与 `e05_min_scene.usd`，不依赖这些资产。
`robot_a_sim_control` 及 Isaac 扩展同样不在本次最小交付中，`legacy:=true` 需要自行取得并安装。

本次资产清单只包含实际交付文件；不复用工程私仓含第三方文件的旧哈希清单。
底盘的质量、轮宽、轮心及传感器安装位置仍有假设，详见 [模型记录](robot_model.md)。
