#!/usr/bin/env bash
# 校验仓库内仿真资产的完整性（assets/MANIFEST.sha256）。
# 用途：确认“另一台机器拿到的是同一份资产”，避免几何/场景悄悄漂移。
set -uo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

MANIFEST="assets/MANIFEST.sha256"
if [ ! -f "$MANIFEST" ]; then
    echo "[verify_assets] 缺少 $MANIFEST" >&2
    exit 1
fi

expected=$(wc -l < "$MANIFEST")
echo "[verify_assets] 清单条目：$expected"

if sha256sum -c "$MANIFEST" --quiet 2>&1; then
    echo "[verify_assets] PASS：全部 $expected 个文件校验一致"
    exit 0
else
    echo "[verify_assets] FAIL：存在缺失或被修改的资产（见上面的 sha256sum 输出）" >&2
    exit 1
fi
