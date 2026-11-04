#!/usr/bin/env python3
"""E01 探针：验证 Isaac Sim 5.1 能以 headless 方式启动空场景，并核对 ROS 2 Bridge 扩展。

只做读取与启动验证，不保存任何 USD，不修改用户配置。
输出：结构化行 `[E01] key=value`，供 env 文档引用。
"""

import os
import sys
import time

print("[E01] py_executable=" + sys.executable, flush=True)
print("[E01] pid=%d" % os.getpid(), flush=True)

t0 = time.time()
from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True, "width": 640, "height": 480})
print("[E01] simapp_ready_wall_s=%.1f" % (time.time() - t0), flush=True)

import carb  # noqa: E402
import omni.kit.app  # noqa: E402

app = omni.kit.app.get_app()
print("[E01] kit_version=%s" % app.get_kit_version(), flush=True)
print("[E01] app_version=%s" % str(app.get_app_version()), flush=True)

# 空场景：不做任何 USD 加载，保持 default stage（默认空白场景）
import omni.usd  # noqa: E402

stage = omni.usd.get_context().get_stage()
print("[E01] default_stage_url=%s" % (stage.GetRootLayer().identifier if stage else "none"), flush=True)
print("[E01] default_stage_prim_count=%d" % (len(stage.GetRootLayer().ExportToString().splitlines()) if stage else -1), flush=True)

# RTX / GPU 可用性
settings = carb.settings.get_settings()
print("[E01] rtx_default_renderer=%s" % settings.get("/app/renderer/resolution/width"), flush=True)
gpu = os.environ.get("CUDA_VISIBLE_DEVICES", "<unset>")
print("[E01] CUDA_VISIBLE_DEVICES=%s" % gpu, flush=True)

# 启用 ROS 2 Bridge 扩展（Isaac Sim 5.1 内扩展名）
ext_mgr = app.get_extension_manager()
ext_name = "isaacsim.ros2.bridge"
print("[E01] bridge_enabled_before=%s" % ext_mgr.is_extension_enabled(ext_name), flush=True)
ext_mgr.set_extension_enabled_immediate(ext_name, True)
sim_app.update()
print("[E01] bridge_enabled_after=%s" % ext_mgr.is_extension_enabled(ext_name), flush=True)
try:
    ids = []
    try:
        ids = ext_mgr.get_enabled_extensions()  # 不同 Kit 版本接口不同，失败就退回读 toml
    except Exception:
        try:
            ids = ext_mgr.get_enabled_extension_ids()
        except Exception:
            ids = []
    print("[E01] bridge_ext_id=%s" % [str(i) for i in ids if ext_name in str(i)], flush=True)
except Exception as exc:  # pragma: no cover
    print("[E01] bridge_ext_id=FAIL %r" % (exc,), flush=True)

# 扩展版本以磁盘上的 extension.toml 为准，不依赖 Kit 内部 API
try:
    import glob
    import re

    site = os.path.join(sys.prefix, "lib", "python3.11", "site-packages")
    cands = glob.glob(os.path.join(site, "isaacsim", "exts", "isaacsim.ros2.bridge", "config", "extension.toml"))
    if cands:
        txt = open(cands[0]).read()
        m = re.search(r'^version\s*=\s*"([^"]+)"', txt, re.MULTILINE)
        print("[E01] bridge_ext_version=%s (from %s)" % (m.group(1) if m else "?", cands[0]), flush=True)
    else:
        print("[E01] bridge_ext_version=NOT_FOUND", flush=True)
except Exception as exc:  # pragma: no cover
    print("[E01] bridge_ext_version=FAIL %r" % (exc,), flush=True)
print("[E01] bridge_ros_distro_setting=%s" % settings.get("/exts/isaacsim.ros2.bridge/ros_distro"), flush=True)
print("[E01] bridge_publish_without_verification=%s"
      % settings.get("/exts/isaacsim.ros2.bridge/publish_without_verification"), flush=True)

# 仿真内置 rclpy（bridge 自带的轻量 ROS 2 Humble 库）
try:
    rclpy_candidates = [p for p in sys.path if p.endswith("humble/rclpy")]
    print("[E01] bundled_rclpy_on_syspath=%s" % rclpy_candidates, flush=True)
except Exception as exc:  # pragma: no cover
    print("[E01] bundled_rclpy_on_syspath=FAIL %r" % (exc,), flush=True)

try:
    import rclpy  # noqa: E402
    from rclpy.utilities import get_rmw_implementation_identifier

    print("[E01] sim_rclpy_import=OK path=%s" % rclpy.__file__, flush=True)
    print("[E01] sim_rmw_implementation=%s" % get_rmw_implementation_identifier(), flush=True)
except Exception as exc:  # pragma: no cover - 仅诊断用
    print("[E01] sim_rclpy_import=FAIL %s" % (str(exc).splitlines()[0] if str(exc) else repr(exc)),
          flush=True)

# 动作图节点清单：以扩展自带的节点文档为静态依据；运行时可用性由 E02 的动作图创建证明
try:
    import glob

    site = os.path.join(sys.prefix, "lib", "python3.11", "site-packages")
    ogn_docs = glob.glob(os.path.join(site, "isaacsim", "exts", "isaacsim.ros2.bridge", "docs", "ogn", "*.rst"))
    names = sorted(os.path.basename(f)[3:-4] for f in ogn_docs)
    print("[E01] bridge_ogn_node_count=%d" % len(names), flush=True)
    print("[E01] bridge_ogn_nodes=%s" % ",".join(names), flush=True)
except Exception as exc:  # pragma: no cover
    print("[E01] bridge_ogn_nodes=FAIL %r" % (exc,), flush=True)

print("[E01] probe_done_wall_s=%.1f" % (time.time() - t0), flush=True)

sim_app.close()
print("[E01] closed_cleanly", flush=True)
