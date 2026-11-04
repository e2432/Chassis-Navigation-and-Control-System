#!/usr/bin/env python3
"""按三视图生成 AGL-CHS-03-F01 车体的八边形棱柱网格（二进制 STL）。

车体平面形状（依据 PDF 第 3 页三视图）：
  "旋转 45° 的方形 + 四角切平" —— 4 个 45° 斜面（装全向轮）+ 4 个被截平的
  轴向面（前/后/左/右）；
  包络：X = 643 mm，Y = 656 mm（W643 / L656）；
  斜面的法向距离 d 由"含轮整体包络 726 × 726"反解（见 docs/engineering/robot_model.md）。

棱柱是凸体，因此 Isaac Sim 用凸包做碰撞体与真实形状完全一致。

用法:
  python3 tools/engineering/gen_chassis_mesh.py                    # 生成到 robot_description/meshes/
  python3 tools/engineering/gen_chassis_mesh.py --out /tmp/x.stl   # 指定输出
"""

import argparse
import struct

# ---- 平面参数（米）----
HALF_X = 0.643 / 2.0          # a = 0.3215   PDF: W643
HALF_Y = 0.656 / 2.0          # b = 0.328    PDF: L656
FACE_DIST = 0.290             # d：45° 斜面到中心的法向距离（按轮内侧面留 13 mm 支架间隙取整）
BODY_HEIGHT = 0.226           # 251（离地总高）− 25（离地间隙）
Z_BOTTOM = -0.080             # 相对 base_link(=轮轴平面, 离地 0.105)：底盘底面 = 0.025 − 0.105


def octagon(face_dist=None):
    """逆时针顶点：(E 面)→(NE 斜面)→(N 面)→(NW 斜面)→(W 面)→(SW 斜面)→(S 面)→(SE 斜面)"""
    face_dist = FACE_DIST if face_dist is None else face_dist
    k = face_dist * 2 ** 0.5          # 斜面直线 x+y=k 的常数项
    h_e = k - HALF_X                  # E/W 面的半高
    h_n = k - HALF_Y                  # N/S 面的半宽
    return [
        (HALF_X, -h_e),
        (HALF_X, h_e),                # E 面
        (h_n, HALF_Y),                # NE 斜面
        (-h_n, HALF_Y),               # N 面
        (-HALF_X, h_e),               # NW 斜面
        (-HALF_X, -h_e),              # W 面
        (-h_n, -HALF_Y),              # SW 斜面
        (h_n, -HALF_Y),               # S 面
    ]


def write_stl(path, verts, body_height=None):
    body_height = BODY_HEIGHT if body_height is None else body_height
    z0, z1 = Z_BOTTOM, Z_BOTTOM + body_height
    tris = []
    n = len(verts)
    for i in range(n):                       # 顶面 / 底面（扇形三角化）
        j = (i + 1) % n
        tris.append(((0, 0, z1), (verts[i][0], verts[i][1], z1), (verts[j][0], verts[j][1], z1)))
        tris.append(((0, 0, z0), (verts[j][0], verts[j][1], z0), (verts[i][0], verts[i][1], z0)))
    for i in range(n):                       # 侧壁
        j = (i + 1) % n
        a = (verts[i][0], verts[i][1])
        b = (verts[j][0], verts[j][1])
        tris.append(((a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1)))
        tris.append(((a[0], a[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)))
    with open(path, "wb") as f:
        f.write(b"AGL-CHS-03-F01 chassis octagon prism".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for t in tris:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))
    return len(tris)


def main():
    global FACE_DIST
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="src/robot_description/meshes/agl_chassis_octagon.stl")
    ap.add_argument("--chamfer", type=float, default=FACE_DIST,
                    help="45° 斜面的法向距离（米），改这个会同时改变 726 包络")
    args = ap.parse_args()
    FACE_DIST = args.chamfer
    verts = octagon(args.chamfer)
    tris = write_stl(args.out, verts)
    print("写出 %s：%d 个三角形" % (args.out, tris))
    print("八边形顶点（mm）:", [(round(x * 1000, 1), round(y * 1000, 1)) for x, y in verts])
    print("包络 X=%.1f Y=%.1f mm；高度 %.1f mm（z %.1f → %.1f mm）"
          % ((max(v[0] for v in verts) - min(v[0] for v in verts)) * 1000,
             (max(v[1] for v in verts) - min(v[1] for v in verts)) * 1000,
             BODY_HEIGHT * 1000, Z_BOTTOM * 1000, (Z_BOTTOM + BODY_HEIGHT) * 1000))


if __name__ == "__main__":
    main()
