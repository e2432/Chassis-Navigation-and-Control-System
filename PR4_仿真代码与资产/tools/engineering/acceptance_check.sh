#!/usr/bin/env bash
# 合并前验收（E03/E04 的机器可执行版本）。
#
#   ./tools/engineering/acceptance_check.sh            # 全量：资产 + 构建 + 接口自检
#   ./tools/engineering/acceptance_check.sh --no-build # 跳过构建（已构建过时用）
#
# 任何一项失败都会以非 0 退出，供 Git 工作流在合并前卡住。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

DO_BUILD=1
[ "${1:-}" = "--no-build" ] && DO_BUILD=0

PASS=0
FAIL=0
declare -a RESULTS=()

record() { # record <PASS|FAIL> <name> [detail]
    if [ "$1" = "PASS" ]; then PASS=$((PASS+1)); else FAIL=$((FAIL+1)); fi
    RESULTS+=("[$1] $2${3:+ — $3}")
    printf '[%s] %s%s\n' "$1" "$2" "${3:+ — $3}"
}

echo "== 1. 资产完整性 =="
if ./tools/engineering/verify_assets.sh > /tmp/nav_ws_assets.log 2>&1; then
    record PASS "assets/MANIFEST.sha256" "$(grep -o '全部 [0-9]* 个文件' /tmp/nav_ws_assets.log)"
else
    record FAIL "assets/MANIFEST.sha256" "见 /tmp/nav_ws_assets.log"
fi

echo
echo "== 2. 依赖 =="
if ./tools/engineering/setup_dependencies.sh > /tmp/nav_ws_deps.log 2>&1; then
    record PASS "依赖检查" "必需依赖齐全"
else
    record FAIL "依赖检查" "见 /tmp/nav_ws_deps.log"
fi

echo
echo "== 3. 构建 =="
if [ "$DO_BUILD" = "1" ]; then
    if ./tools/engineering/build_workspace.sh > /tmp/nav_ws_build.log 2>&1; then
        record PASS "colcon build --symlink-install" "$(grep -o '[0-9]* packages finished' /tmp/nav_ws_build.log | head -1)"
    else
        record FAIL "colcon build --symlink-install" "见 /tmp/nav_ws_build.log"
    fi
else
    record PASS "colcon build" "已跳过（--no-build）"
fi

echo
echo "== 4. 包可见性与接口自检（系统 python3 + 已 source 的安装空间）=="
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONPATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_DISTRO 2>/dev/null || true
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

EXPECTED_PKGS=(robot_description robot_sim robot_bringup robot_benchmark)
# 注意：不要写成 `ros2 pkg list | grep -qx "$pkg"`。
# 在 `set -o pipefail` 下，grep -q 匹配到就立刻退出，ros2 收到 SIGPIPE（退出码 141），
# 整个管道被判失败 → 随机误报“包缺失”。这里先把列表落盘再逐行比对（只跑一次 ros2，也更快）。
ros2 pkg list > /tmp/nav_ws_pkglist.txt 2>/dev/null
missing=()
for pkg in "${EXPECTED_PKGS[@]}"; do
    grep -qx "$pkg" /tmp/nav_ws_pkglist.txt || missing+=("$pkg")
done
if [ ${#missing[@]} -eq 0 ]; then
    record PASS "四个交付包可见" "${EXPECTED_PKGS[*]}"
else
    record FAIL "四个交付包可见" "缺失: ${missing[*]}"
fi

if python3 - <<'PY' > /tmp/nav_ws_contract.log 2>&1
import robot_benchmark as b
assert len(b.METRIC_KEYS) == 7, b.METRIC_KEYS
assert b.RunStatus.SUCCESS.value == "success"
sc = b.Scenario(
    scenario_id="static_001", category="static", asset_version="2026-10-23",
    map_version="v0", start_pose=b.contract.Pose2D(0, 0, 0), goal_pose=b.contract.Pose2D(1, 1, 0),
    seed=1, task_timeout_s=60.0, goal_tolerance_m=0.2, goal_tolerance_yaw_rad=0.2,
)
res = b.RunResult(run_id="r1", scenario_id="static_001", controller="dwb", seed=1,
                  status=b.RunStatus.SUCCESS, reason="goal_reached")
assert '"status": "success"' in res.to_json()
print("contract ok")
PY
then
    record PASS "契约数据结构可用" "七项指标键 + Scenario/RunResult 序列化"
else
    record FAIL "契约数据结构可用" "见 /tmp/nav_ws_contract.log"
fi

if python3 -m robot_benchmark.run --scenario static_001 --controller dwb --seed 1 > /tmp/nav_ws_run.log 2>&1; then
    record FAIL "未实现接口必须显式失败" "run 竟然返回 0（不应在没有实现时成功）"
else
    code=$?
    if [ "$code" = "2" ]; then
        record PASS "未实现接口必须显式失败" "run 退出码 2 + 明确未实现提示"
    else
        record FAIL "未实现接口必须显式失败" "run 退出码 $code（期望 2）"
    fi
fi

if ros2 launch robot_bringup simulation.launch.py --show-args > /tmp/nav_ws_launch.log 2>&1; then
    record PASS "顶层 launch 可解析" "use_sim_time / controller / use_rviz"
else
    record FAIL "顶层 launch 可解析" "见 /tmp/nav_ws_launch.log"
fi

echo
echo "== 5. E08 传感器接入 =="
if [ -f docs/engineering/sensors.md ] && grep -q "实测频率" docs/engineering/sensors.md && grep -q "近场盲区\|盲区" docs/engineering/sensors.md; then
    record PASS "docs/engineering/sensors.md" "含实测频率、丢帧结论、已知限制"
else
    record FAIL "docs/engineering/sensors.md" "缺实测频率或已知限制章节"
fi

SENSORS_CFG=src/robot_sim/config/sensors.yaml
if [ -f "$SENSORS_CFG" ] && grep -q "^sensors:" "$SENSORS_CFG" && \
   grep -q "measured: [0-9]" "$SENSORS_CFG" && \
   grep -q "frame_id: lidar_front_link" "$SENSORS_CFG" && \
   grep -q "frame_id: imu_link" "$SENSORS_CFG"; then
    record PASS "传感器配置" "$SENSORS_CFG（含实测值 + 帧名与 URDF 一致）"
else
    record FAIL "传感器配置" "$SENSORS_CFG 缺 measured 实测值或帧名不对"
fi

if [ -f tools/engineering/e08_gui_sensors.py ] && grep -q "headless-test" tools/engineering/e08_gui_sensors.py && \
   grep -q "e08_gui_start" tools/engineering/e08_gui_sensors.py; then
    record PASS "GUI 验证入口" "tools/engineering/e08_gui_sensors.py（含 --headless-test 冒烟）"
else
    record FAIL "GUI 验证入口" "tools/engineering/e08_gui_sensors.py 缺失或没有冒烟入口"
fi

E08_DIR=results/w2/e08
missing=""
for f in bag_analysis.txt cloud_frames.csv imu_frames.csv sample_record.txt \
         cloud_topview.png cloud_frames.png imu_series.png rviz_points.png; do
    [ -f "$E08_DIR/$f" ] || missing="$missing $f"
done
if [ -z "$missing" ]; then
    record PASS "E08 证据文件" "$(ls "$E08_DIR" | wc -l) 项（含 RViz 截图与 bag 分析）"
else
    record FAIL "E08 证据文件缺失" "$missing"
fi

# 频率自洽：bag 分析里的"重复倍数"必须是 1.00×（= 每条消息都是新数据），
# 雷达实测频率必须落在 9~11 Hz（说明既不重复发布也不掉帧）。
if [ -f "$E08_DIR/bag_analysis.txt" ]; then
    line=$(grep -aE '^/(points_raw|lidar/points)' "$E08_DIR/bag_analysis.txt" | head -1)
    hz=$(echo "$line" | awk '{print $11}')
    dup=$(echo "$line" | grep -o '重复倍数 *[0-9.]*×' | grep -o '[0-9.]*×' | tr -d '×')
    if [ -n "$dup" ] && awk -v d="$dup" 'BEGIN {exit !(d == "1.00")}' && \
       awk -v h="$hz" 'BEGIN {exit !(h > 9.0 && h < 11.0)}'; then
        record PASS "点云频率自洽" "实测 ${hz} Hz / 重复倍数 ${dup}×（非重复发布、无掉帧）"
    else
        record FAIL "点云频率自洽" "hz=${hz:-?} dup=${dup:-?}（期望 9~11 Hz 且 1.00×）"
    fi
else
    record FAIL "点云频率自洽" "缺 $E08_DIR/bag_analysis.txt"
fi

echo
echo "== 6. E09/E10 TF 树与统一启动 =="
if [ -f docs/engineering/tf_tree.md ] && grep -q "互斥" docs/engineering/tf_tree.md && grep -q "重置" docs/engineering/tf_tree.md; then
    record PASS "docs/engineering/tf_tree.md" "父子帧/发布者/参考系/重置行为/互斥表齐备"
else
    record FAIL "docs/engineering/tf_tree.md" "缺互斥或重置章节"
fi
if [ -f src/robot_sim/config/tf_tree.yaml ] && grep -q "mutual_exclusion" src/robot_sim/config/tf_tree.yaml; then
    record PASS "TF 树配置" "src/robot_sim/config/tf_tree.yaml（含互斥规则）"
else
    record FAIL "TF 树配置" "缺 src/robot_sim/config/tf_tree.yaml"
fi
if ros2 pkg executables robot_sim 2>/dev/null | grep -q truth_odom_node && \
   ros2 pkg executables robot_sim 2>/dev/null | grep -q map_odom_static_node; then
    record PASS "TF/里程计节点可执行" "truth_odom_node.py + map_odom_static_node.py"
else
    record FAIL "TF/里程计节点可执行" "ros2 pkg executables robot_sim 里找不到"
fi
W3_ok=1
for f in results/w3/e09/node_unit_test.txt results/w3/e09/integration/verify_summary.txt \
         results/w3/e10/m1_check.txt results/w3/e10/launch_command.txt \
         results/w3/e10/rviz_e10.png; do
    [ -f "$f" ] || { W3_ok=0; echo "  缺失: $f"; }
done
if [ $W3_ok = 1 ]; then
    record PASS "W3 证据（E09/E10）" "节点单测 + 集成验证 + M1 检查 + RViz 截图"
else
    record FAIL "W3 证据（E09/E10）" "见上"
fi
if grep -q "结果：PASS" results/w3/e09/node_unit_test.txt 2>/dev/null && \
   grep -q "结果：PASS" results/w3/e09/integration/verify_summary.txt 2>/dev/null && \
   grep -q "结果：PASS" results/w3/e10/m1_check.txt 2>/dev/null; then
    odom_hz=$(awk '/^\/odom: .*→/ {for (i=1;i<=NF;i++) if ($i ~ /Hz/) {print $(i-1); exit}}' \
              results/w3/e09/integration/verify_summary.txt)
    record PASS "E09/E10 验证结论" "三份报告均 PASS（/odom 实测 ${odom_hz:-?} Hz）"
else
    record FAIL "E09/E10 验证结论" "有报告不是 PASS"
fi
if ros2 launch robot_bringup simulation.launch.py --show-args 2>/dev/null | grep -q "'demo'"; then
    record PASS "E10 一条命令可解析" "simulation.launch.py 有 sim/rviz/demo/truth/mode 参数"
else
    record FAIL "E10 一条命令可解析" "见 simulation.launch.py"
fi

echo
echo "== 7. E14 五类场景与实例清单 =="
SC_DIR=scenarios
missing=""
for k in static_avoid dynamic_pedestrian narrow_passage dead_end mixed; do
    [ -f "$SC_DIR/$k.yaml" ] || missing="$missing $k.yaml"
    [ -f "$SC_DIR/instances/${k}_instances.yaml" ] || missing="$missing instances/${k}_instances.yaml"
done
[ -f "$SC_DIR/test_list.yaml" ] || missing="$missing test_list.yaml"
if [ -z "$missing" ]; then
    total=$(grep -c "scenario_id:" "$SC_DIR"/instances/*_instances.yaml | awk -F: '{s+=$2} END {print s}')
    record PASS "五类场景配置 + 冻结实例清单" "5 份模板 / ${total} 个实例"
else
    record FAIL "五类场景配置缺失" "$missing"
fi
if python3 -m robot_benchmark.gen_scenarios --out "$SC_DIR" --check \
        > /tmp/nav_ws_scen_check.log 2>&1; then
    record PASS "场景实例校验（确定性+有效性）" "$(tail -1 /tmp/nav_ws_scen_check.log)"
else
    record FAIL "场景实例校验" "见 /tmp/nav_ws_scen_check.log"
fi
if python3 -m pytest src/robot_benchmark/test/test_scenario_gen.py -q > /tmp/nav_ws_scen_test.log 2>&1; then
    record PASS "场景生成器单元测试" "$(tail -1 /tmp/nav_ws_scen_test.log)"
else
    record FAIL "场景生成器单元测试" "见 /tmp/nav_ws_scen_test.log"
fi

echo
echo "== 8. 文档要点 =="
docs_ok=1
for f in docs/engineering/environment.md docs/engineering/bridge_check.md docs/接口契约.md docs/指标定义.md \
         docs/engineering/package_roles.md docs/engineering/assets.md docs/engineering/dependencies.md docs/engineering/records_w1_w3.md \
         docs/engineering/robot_model.md docs/engineering/records_w1_w3.md docs/engineering/motion_check.md docs/engineering/sensors.md \
         docs/engineering/tf_tree.md docs/engineering/records_w1_w3.md; do
    if [ -f "$f" ]; then :; else docs_ok=0; echo "  缺失: $f"; fi
done
if [ $docs_ok = 1 ]; then
    record PASS "W1/W2 文档齐全" "$(ls docs/*.md docs/adr/*.md | wc -l) 份"
else
    record FAIL "W1 文档齐全" "见上"
fi

if grep -q "待确认\|待算法成员确认" docs/接口契约_工程侧对齐记录.md 2>/dev/null && \
   grep -q "确认记录" docs/接口契约_工程侧对齐记录.md 2>/dev/null; then
    record PASS "契约含待确认与签认区" "E04 要求互审记录"
else
    record FAIL "契约含待确认与签认区" "docs/接口契约.md"
fi

echo
echo "================ 验收结果 ================"
for line in "${RESULTS[@]}"; do echo "$line"; done
echo "------------------------------------------"
echo "PASS=$PASS FAIL=$FAIL"
[ "$FAIL" -eq 0 ] || exit 1
