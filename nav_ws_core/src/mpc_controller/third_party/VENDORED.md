# Vendored OSQP（W5 D4）

| 项 | 值 |
|---|---|
| 上游 | https://github.com/osqp/osqp |
| tag | `v0.6.3` |
| commit | `0dd00a578cf1c2691c5c379965d504c75bf6cfad` |
| 嵌入求解器 | qdldl `7d16b70a10a152682204d745d814b6eb63dc5cd2`（`v0.1.3-40-g7d16b70`） |
| Python 对照 | conda `pro` 的 `osqp 0.6.7.post3` 是 **Python 包版本号**，不等于 C 源码版本；W5 D4 以本目录 C 源码为准 |

## 说明

- 仅保留构建所需内容：`include/`、`src/`、`lin_sys/`、`configure/`、`examples/`、LICENSE、NOTICE。
  上游 `tests/`、`docs/`、`site/`、CI 配置已裁掉。
- qdldl 子模块内容已直接展开，不保留 git submodule 元数据。
- 构建方式：`add_subdirectory(third_party/osqp EXCLUDE_FROM_ALL)`，只链接静态 `osqpstatic`；
  不安装 OSQP 自身，避免污染 `mpc_controller` 安装前缀。
- 固定构建选项：`UNITTESTS=OFF`、`PRINTING=OFF`、`PROFILING=ON`、`CTRLC=OFF`、
  `ENABLE_MKL_PARDISO=OFF`、`DLONG=ON`（64 位索引，默认）。
- 上游 CMake 会在源码树生成 `include/osqp_configure.h` 与 qdldl 的
  `include/qdldl_types.h`，两者已加入仓库根 `.gitignore`，不提交。
- 许可证：Apache-2.0（见 `osqp/LICENSE`；qdldl 见其目录 LICENSE）。
