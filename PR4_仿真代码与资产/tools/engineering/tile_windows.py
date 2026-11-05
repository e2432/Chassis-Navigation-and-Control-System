#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 Isaac Sim 与 RViz 两个窗口摆成左右平铺（不依赖 wmctrl/xdotool）。

本机没有 wmctrl / xdotool，GNOME 又禁用截图门户，所以直接用 libX11 的
XMoveResizeWindow 挪窗口（用 ctypes，不需要装 python-xlib）。

用法:
  python3 tools/engineering/tile_windows.py                 # Isaac 左半屏、RViz 右半屏
  python3 tools/engineering/tile_windows.py --list          # 只列出匹配到的窗口
"""
import argparse
import ctypes
import ctypes.util
import re
import subprocess
import sys


def windows(display):
    out = subprocess.run(["xwininfo", "-root", "-tree", "-display", display],
                         capture_output=True, text=True).stdout
    res = []
    for line in out.splitlines():
        m = re.search(r"(0x[0-9a-f]+) \"([^\"]*)\":.*?(\d+)x(\d+)\+(-?\d+)\+(-?\d+)"
                      r"(?:\s+\+(-?\d+)\+(-?\d+))?", line)
        if not m:
            continue
        wid, title = int(m.group(1), 16), m.group(2)
        w, h = int(m.group(3)), int(m.group(4))
        absx = int(m.group(7)) if m.group(7) else int(m.group(5))
        absy = int(m.group(8)) if m.group(8) else int(m.group(6))
        if w < 200 or h < 200:
            continue
        res.append((wid, title, w, h, absx, absy))
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--display", default=":1")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    wins = windows(a.display)
    isaac = next((w for w in wins if "Isaac Sim" in w[1]), None)
    rviz = next((w for w in wins if "RViz" in w[1]), None)
    if a.list:
        for w in wins:
            print("0x%x  %-45s %dx%d @ (%d,%d)" % w)
        return 0
    if not isaac and not rviz:
        print("没找到 Isaac Sim / RViz 窗口（先启动 launch）")
        return 1

    lib = ctypes.CDLL(ctypes.util.find_library("X11"))
    # 必须显式声明 argtypes，否则 ctypes 会把指针当 int 截断（本机实测会段错误）
    lib.XOpenDisplay.restype = ctypes.c_void_p
    lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    lib.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                                      ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    lib.XMoveWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_int]
    lib.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    lib.XFlush.argtypes = [ctypes.c_void_p]
    lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    lib.XDisplayWidth.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.XDisplayWidth.restype = ctypes.c_int
    lib.XDisplayHeight.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.XDisplayHeight.restype = ctypes.c_int
    d = lib.XOpenDisplay(a.display.encode())
    if not d:
        print("打不开显示 %s" % a.display)
        return 1
    # 屏幕尺寸
    scr_w, scr_h = lib.XDisplayWidth(d, 0), lib.XDisplayHeight(d, 0)
    h = int(scr_h * 0.62)

    moves = []
    if isaac:
        moves.append(("Isaac Sim", isaac[0], 0, 0, scr_w // 2, h))
    if rviz:
        moves.append(("RViz", rviz[0], scr_w // 2, 0, scr_w // 2, h))
    for name, wid, x, y, w, hh in moves:
        lib.XMoveResizeWindow(d, ctypes.c_ulong(wid), x, y, w, hh)
        lib.XRaiseWindow(d, ctypes.c_ulong(wid))
        print("已摆放 %s → %dx%d at (%d,%d)" % (name, w, hh, x, y))
    lib.XFlush(d)
    lib.XCloseDisplay(d)
    return 0


if __name__ == "__main__":
    sys.exit(main())
