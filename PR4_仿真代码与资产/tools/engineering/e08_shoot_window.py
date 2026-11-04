#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""抓取指定标题的 X11 窗口为 PNG（无 ImageMagick/Xvfb 时的替代路径）。

用 xwd（XGetImage，不受 GNOME 截图门户限制）截屏幕，再从 XWD 里按窗口几何裁剪，
避免把桌面其它内容写进仓库。只保留目标窗口区域。

用法: e08_shoot_window.py <窗口标题关键词> <输出PNG>
"""
import os
import re
import struct
import subprocess
import sys

from PIL import Image


def find_window(keyword):
    out = subprocess.run(["xwininfo", "-root", "-tree"], capture_output=True, text=True).stdout
    best = None
    for line in out.splitlines():
        if keyword not in line or "has no name" in line:
            continue
        m = re.search(r"(0x[0-9a-f]+) .*?(\d+)x(\d+)\+(-?\d+)\+(-?\d+)", line)
        if not m:
            continue
        wid, w, h, x, y = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
        if w < 400 or h < 300:
            continue
        if best is None or w * h > best[0] * best[1]:
            best = (w, h, x, y, wid)
    return best


def read_xwd(path):
    """解析 xwd（TrueColor，24/32bpp），返回 PIL RGB 图像。

    注意 bytes_per_line 有 4 字节对齐填充，不能直接用 width*bpp。
    byte_order=0（LSBFirst）+ bits_per_pixel=24 时每像素在文件里是 B,G,R。
    """
    import numpy as np

    with open(path, "rb") as f:
        data = f.read()
    hdr = struct.unpack(">25I", data[:100])
    (header_size, _version, _pixmap_format, _depth, width, height, _xoffset, byte_order,
     _bitmap_unit, _bitmap_bit_order, _bitmap_pad, bits_per_pixel, bytes_per_line,
     _visual_class, _rm, _gm, _bm, _bits_per_rgb, _cmap_entries, ncolors) = hdr[:20]
    off = header_size + ncolors * 12
    arr = np.frombuffer(data, dtype=np.uint8, count=height * bytes_per_line,
                        offset=off).reshape(height, bytes_per_line)
    if bits_per_pixel == 32:
        px = arr[:, :width * 4].reshape(height, width, 4)[:, :, :3]
    else:
        px = arr[:, :width * 3].reshape(height, width, 3)
    if byte_order == 0:          # LSBFirst -> 内存里是 B,G,R
        px = px[:, :, ::-1]
    return Image.fromarray(np.ascontiguousarray(px))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    keyword, out_png = sys.argv[1], sys.argv[2]
    geo = find_window(keyword)
    if not geo:
        print("未找到窗口: %s" % keyword)
        return 1
    w, h, x, y, wid = geo
    print("窗口 %s: %dx%d+%d+%d id=%s" % (keyword, w, h, x, y, wid))
    tmp = "/tmp/e08_win.xwd"
    disp = os.environ.get("DISPLAY", ":1")
    # 优先按窗口 id 抓：某些驱动/合成器下抓 root 会 X_GetImage BadMatch
    ok = False
    for cmd in (["xwd", "-id", wid, "-silent", "-display", disp],
                ["xwd", "-root", "-silent", "-display", disp]):
        try:
            with open(tmp, "wb") as fh:
                subprocess.run(cmd, stdout=fh, stderr=subprocess.DEVNULL, check=True)
            ok = True
            break
        except Exception:
            continue
    if not ok:
        print("xwd 抓取失败（DISPLAY=%s）" % disp)
        return 3
    im = read_xwd(tmp)
    if im.width == w and im.height == h:
        im.save(out_png)
    else:
        im.crop((max(0, x), max(0, y), min(im.width, x + w), min(im.height, y + h))).save(out_png)
    os.remove(tmp)
    print("已写出 %s" % out_png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
