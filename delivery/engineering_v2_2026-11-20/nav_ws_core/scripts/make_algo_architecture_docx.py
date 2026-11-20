#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成《导航基线算法原理与测试架构》Word 文档（默认输出到桌面）。

用法：
    python3 nav_ws_core/scripts/make_algo_architecture_docx.py [输出路径]
"""
from __future__ import annotations

import sys

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_LINE_SPACING
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

ACCENT = RGBColor(0x1F, 0x3B, 0x73)      # 标题深蓝
GREY = RGBColor(0x59, 0x59, 0x59)
HEAD_FILL = "D9E2F3"
CODE_FILL = "F2F2F2"
MONO = "Consolas"
EA_BODY = "等线"
EA_HEAD = "等线"


# --------------------------------------------------------------------------- #
# 基础排版工具
# --------------------------------------------------------------------------- #
def set_run_fonts(run, ascii_font: str, ea_font: str) -> None:
    run.font.name = ascii_font
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    rfonts.set(qn("w:ascii"), ascii_font)
    rfonts.set(qn("w:hAnsi"), ascii_font)
    rfonts.set(qn("w:eastAsia"), ea_font)


def shade(element, fill: str) -> None:
    """给段落或单元格加底色。"""
    pr = element.get_or_add_tcPr() if hasattr(element, "get_or_add_tcPr") else element.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    pr.append(shd)


def body(doc, text: str, *, size: float = 10.5, bold: str = "", space_after: float = 4.0,
         indent_cm: float = 0.0, ea: str = EA_BODY, align=None):
    """正文段落；bold 里的子串会被加粗（简单支持单次加粗）。"""
    p = doc.add_paragraph()
    pf = p.paragraph_format
    pf.space_before = Pt(0)
    pf.space_after = Pt(space_after)
    pf.line_spacing_rule = WD_LINE_SPACING.SINGLE
    if indent_cm:
        pf.left_indent = Cm(indent_cm)
    if align is not None:
        p.alignment = align
    if bold and bold in text:
        head, tail = text.split(bold, 1)
        r = p.add_run(head)
        set_run_fonts(r, "Calibri", ea)
        r.font.size = Pt(size)
        rb = p.add_run(bold)
        set_run_fonts(rb, "Calibri", ea)
        rb.font.size = Pt(size)
        rb.bold = True
        r2 = p.add_run(tail)
        set_run_fonts(r2, "Calibri", ea)
        r2.font.size = Pt(size)
    else:
        r = p.add_run(text)
        set_run_fonts(r, "Calibri", ea)
        r.font.size = Pt(size)
    return p


def bullets(doc, items, *, size: float = 10.5, indent_cm: float = 0.6) -> None:
    for it in items:
        p = doc.add_paragraph(style="List Bullet")
        p.paragraph_format.space_after = Pt(2)
        p.paragraph_format.left_indent = Cm(indent_cm + 0.4)
        p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
        if isinstance(it, tuple):      # (加粗前缀, 正文)
            a, b = it
            r = p.add_run(a)
            set_run_fonts(r, "Calibri", EA_BODY)
            r.font.size = Pt(size)
            r.bold = True
            r2 = p.add_run(b)
            set_run_fonts(r2, "Calibri", EA_BODY)
            r2.font.size = Pt(size)
        else:
            r = p.add_run(it)
            set_run_fonts(r, "Calibri", EA_BODY)
            r.font.size = Pt(size)


def code_block(doc, text: str, *, size: float = 7.6, fill: str = CODE_FILL) -> None:
    """等宽代码/框图块：整块底色 + 细边框。"""
    for line in text.rstrip("\n").split("\n"):
        p = doc.add_paragraph()
        pf = p.paragraph_format
        pf.space_before = Pt(0)
        pf.space_after = Pt(0)
        pf.line_spacing = 1.0
        pf.left_indent = Cm(0.2)
        shade(p._p, fill)
        r = p.add_run(line if line else " ")
        set_run_fonts(r, MONO, MONO)
        r.font.size = Pt(size)


def heading(doc, text: str, level: int, *, page_break: bool = False) -> None:
    h = doc.add_heading(level=level)
    if page_break:
        h.paragraph_format.page_break_before = True
    h.paragraph_format.space_before = Pt(12 if level == 1 else 8)
    h.paragraph_format.space_after = Pt(4)
    r = h.add_run(text)
    set_run_fonts(r, "Calibri", EA_HEAD)
    r.font.size = Pt(15 if level == 1 else (12.5 if level == 2 else 11))
    r.font.color.rgb = ACCENT
    r.bold = True


def table(doc, rows, *, widths=None, font_size: float = 9.5, header: bool = True,
          align_center_from: int = 1) -> None:
    t = doc.add_table(rows=len(rows), cols=len(rows[0]))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = True
    for i, row in enumerate(rows):
        trpr = t.rows[i]._tr.get_or_add_trPr()
        trpr.append(OxmlElement("w:cantSplit"))
        if i == 0 and header:
            trpr.append(OxmlElement("w:tblHeader"))     # 跨页时重复表头
        for j, val in enumerate(row):
            cell = t.cell(i, j)
            cell.text = ""
            p = cell.paragraphs[0]
            p.paragraph_format.space_before = Pt(1)
            p.paragraph_format.space_after = Pt(1)
            p.paragraph_format.line_spacing_rule = WD_LINE_SPACING.SINGLE
            if i == 0 and header:
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            elif j >= align_center_from:
                p.alignment = WD_ALIGN_PARAGRAPH.CENTER
            r = p.add_run(str(val))
            set_run_fonts(r, "Calibri", EA_BODY)
            r.font.size = Pt(font_size)
            if i == 0 and header:
                r.bold = True
                shade(cell._tc, HEAD_FILL)
            if widths:
                cell.width = Cm(widths[j])
    # 表后留一点空隙
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def add_page_number_footer(section, text: str) -> None:
    p = section.footer.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = p.add_run(text + "　第 ")
    set_run_fonts(r, "Calibri", EA_BODY)
    r.font.size = Pt(8)
    r.font.color.rgb = GREY
    fld = OxmlElement("w:fldSimple")
    fld.set(qn("w:instr"), "PAGE")
    run = OxmlElement("w:r")
    rpr = OxmlElement("w:rPr")
    sz = OxmlElement("w:sz")
    sz.set(qn("w:val"), "16")
    rpr.append(sz)
    run.append(rpr)
    fld.append(run)
    p._p.append(fld)
    r2 = p.add_run(" 页")
    set_run_fonts(r2, "Calibri", EA_BODY)
    r2.font.size = Pt(8)
    r2.font.color.rgb = GREY


# --------------------------------------------------------------------------- #
# 文档内容
# --------------------------------------------------------------------------- #
DIAGRAM = """\
+--------------------------------------------------------------------------+
| (1) 场景/实例层  scenarios/*.yaml 模板 + scenario_gen.py                  |
|   5 场景 x 20 run = 100 条冻结实例 instances.yaml                         |
|   每条含: goal(x,y,yaw) / 静态圆障碍 / 墙体 / 契约行人(x_cross,speed,t_meet) |
|           / time_limit_s (9-3 冻结: 60 s, mixed 75 s)                      |
+-----------------------------------+--------------------------------------+
                                    | scenes.py 烘成 .usda + .pgm/.yaml
+-----------------------------------v--------------------------------------+
| (2) 仿真层  Isaac Sim 5.1 + isaac_sim_bridge.py (kinematic 驱动)          |
|   <- /cmd_vel 订阅        -> /clock 60Hz /odom(真值) /tf /lidar/points    |
|   行人按契约公式推进 y = dir*speed*(t - t_meet)，不是物理刚体              |
+-----------------------------------+--------------------------------------+
+-----------------------------------v--------------------------------------+
| (3) 执行层  Nav2 单栈, 三个 controller 插件同时注册                       |
|   map_server -- SmacPlanner2D 全局规划 (5 Hz, 40x40 m @0.05)              |
|   局部代价地图 6x6 m 滚动 @0.05 (120x120), 10 Hz:                        |
|     static -> voxel -> ped_corridor(道口栏杆) -> inflation                |
|   controller_server (20 Hz)  -- FollowPath 的 controller_id 选基线        |
|        | /cmd_vel_raw                                                    |
|        v  flat_speed_limiter (契约限速/限加速度)                           |
|        | /cmd_vel_limited                                                |
|        v  safety_supervisor (odom 过期/急停 -> 0)  -> /cmd_vel -> 仿真     |
+-----------------------------------+--------------------------------------+
+-----------------------------------v--------------------------------------+
| (4) 采集层  run_300.sh -> run_workset.py -> runner.py -> nav2_driver.py   |
|   一场景一次 Isaac, 三基线共用同栈; 每个 episode:                         |
|     /sim/reset -> /sim/episode_start -> 等代价地图稳定                    |
|     -> ComputePathToPose (穿致命格的路径不收) -> FollowPath               |
|     -> 60 Hz 记录(pose, cmd); 超时/挂钟双保险                             |
|   产出 run.json + trajectory.npz; summarize.py -> summary.md/json        |
+-----------------------------------+--------------------------------------+
+-----------------------------------v--------------------------------------+
| (5) 判分层  judge.py (与控制栈完全解耦)                                   |
|   八边形 footprint 逐姿态求"外缘到几何"最小间距:                          |
|     静态圆障碍 / 墙体 / 按契约公式算出的行人圆(r=0.3)                     |
|   success = 无碰撞 AND 到达(<=0.25 m/0.25 rad) AND 不超时                 |
|   失败分类: collision(<=0) / timeout / other(卡死等)                      |
|   指标: 到达时间 / 路径长 / 平滑度(TV) / 最小间距 / 急停次数               |
+-----------------------------------+--------------------------------------+
+-----------------------------------v--------------------------------------+
| (6) 结果层  9-1 冻结快照 -> 9-2 正式 300 run -> 9-3 时限冻结              |
|   T_s = max(T_min,s , 1.5 * max_j t~(s,j))  只用成功 run                  |
|   A/B: 走廊/道口栏杆口径 vs 雷达口径; 上传包 = 代码+配置+清单+文档+汇总    |
+--------------------------------------------------------------------------+"""


def build(path: str) -> None:
    doc = Document()

    # ---- 页面与默认样式 ----
    sec = doc.sections[0]
    sec.page_width, sec.page_height = Cm(21.0), Cm(29.7)
    sec.left_margin = sec.right_margin = Cm(2.0)
    sec.top_margin = sec.bottom_margin = Cm(2.0)
    add_page_number_footer(sec, "导航基线算法原理与测试架构")

    normal = doc.styles["Normal"]
    normal.font.name = "Calibri"
    normal.font.size = Pt(10.5)
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), EA_BODY)

    # ---- 封面标题块 ----
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run("导航基线算法原理与测试架构")
    set_run_fonts(r, "Calibri", EA_HEAD)
    r.font.size = Pt(22)
    r.bold = True
    r.font.color.rgb = ACCENT

    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_after = Pt(10)
    r = p.add_run("全向底盘导航基线横向对比：DWA / MPPI / TEB")
    set_run_fonts(r, "Calibri", EA_BODY)
    r.font.size = Pt(12)
    r.font.color.rgb = GREY

    table(doc, [
        ["项目", "内容"],
        ["文档范围", "三种局部控制器原理 + 整套测试架构 + 本次正式成绩口径"],
        ["对应交付", "9-1 参数定稿、9-2 正式成绩（300 run）、9-3 行人动态障碍与时限冻结"],
        ["仿真/系统", "Isaac Sim 5.1.0（kinematic 驱动）+ ROS 2 Humble + Nav2"],
        ["底盘/契约", "全向（vx, vy, wz）；平面速度 ≤0.6 m/s、|wz| ≤1.5 rad/s、"
                    "加速度 1.0/1.0/2.0 m/s²、控制 20 Hz、到达 0.25 m / 0.25 rad"],
        ["平台", "RTX 5070 Laptop（8 GB）、单机单卡、每次一个 Isaac 实例"],
        ["日期", "2026-11-18"],
    ], widths=[3.0, 14.0], font_size=9.5)

    body(doc, "一句话概括：三种控制器吃同一张代价地图、同一条全局路径、同一套限速与判分，"
              "所以性能差异只来自“如何在动作空间里搜解”。架构的全部设计目的就是保证这一点。",
         size=10.5)

    # ======================= 第一部分 =======================
    heading(doc, "一、三种局部控制器原理", 1)
    body(doc, "三者任务完全相同：给定一条全局路径 + 局部代价地图，每 20 Hz 输出一个 (vx, vy, wz)，"
              "并受同一份契约约束。差别在于“怎么在动作空间里找这个解”。")

    # ---- DWA ----
    heading(doc, "1.1　DWA / DWB —— 采样打分", 2)
    body(doc, "实现：nav2 的 dwb_core::DWBLocalPlanner。原理是把速度空间离散成网格，前向仿真出若干条候选轨迹，"
              "再用一组代价项打分取最优。")
    code_block(doc, """\
速度采样 (vx, vy, vtheta) 各 20 档，最多 20^3 = 8000 条候选（按加速度窗口裁剪）
    -> 每条用上一位姿前推 sim_time = 1.7 s（粒度 0.05 m / 0.025 rad）
    -> 轨迹逐点查局部代价地图：是否碰撞 / 离障碍多近
    -> 6 个 critic 线性加权求和：
         RotateToGoal(16) + BaseObstacle(0.5) + GoalAlign(24)
         + PathAlign(32) + PathDist(32) + GoalDist(24)
    => 最优点 -> 按加速度/速度限幅 -> /cmd_vel_raw""", size=8.6)
    body(doc, "本质：贪心一步预测（receding horizon = 1 个 sim_time），无模型求解，纯“枚举 + 打分”。")
    body(doc, "优点：", bold="优点：")
    bullets(doc, [
        "完全无梯度，能直接吃代价地图这种不连续代价；实现简单、行为可解释。",
        "在走廊/门洞这类“动作空间小、代价形状单一”的场景很稳（本次 static_avoid 90%、mixed 90%）。",
    ])
    body(doc, "缺点：", bold="缺点：")
    bullets(doc, [
        "采样分辨率与代价形状强耦合：BaseObstacle.scale 从 0.02 调到 0.5 才真正起避障作用。",
        "局部极小：被两三个障碍夹住时要么原地微转、要么平移恒 0（本次 mixed 502/506、"
        "narrow_passage 5 例 other 即此类）。",
        "契约不鼓励倒车，进入死角后只能等 patience 中止。",
        "3 维速度栅格在全向底盘上组合爆炸，只能靠剪枝（short_circuit_trajectory_evaluation）。",
    ])

    # ---- MPPI ----
    heading(doc, "1.2　MPPI —— 随机采样 + 路径积分加权", 2)
    body(doc, "实现：nav2_mppi_controller::MPPIController。把局部控制问题写成随机最优控制（SOC），"
              "用蒙特卡洛采样近似“最优控制序列的更新律”。")
    code_block(doc, """\
名义控制序列 U（56 步 x 0.05 s = 2.8 s 时域）
    -> 加高斯噪声 sigma_vx = 0.2, sigma_vy = 0.2, sigma_wz = 0.4
         -> batch_size = 1000 条候选轨迹
    -> 每条算总代价 S(tau) = sum critics（Constraint / Cost(consider_footprint)
         / Goal / GoalAngle / PathAlign / PathFollow / PathAngle ...），gamma=0.015 折扣
    -> 权重 w_i = exp(-S_i / lambda)，lambda = temperature = 0.3
         核心：不是选最好的一条，而是按 exp(-cost/lambda) 加权平均所有候选
    => 更新名义序列（1 次迭代），取第一步 -> /cmd_vel_raw""", size=8.6)
    body(doc, "本质：无梯度的随机最优控制，用 softmax 加权代替“取 argmin”，因此对代价函数的"
              "不连续性、不可微性完全免疫（footprint 碰撞、代价量化、速度约束都能直接写成代价）。")
    body(doc, "优点：", bold="优点：")
    bullets(doc, [
        "加权平均天然平滑，平滑度指标最好（本次 6.55~9.95；DWA 为 18~32）。",
        "1000 条候选覆盖 2.8 s 时域，比 DWA 的 1.7 s 单步“看得更远”，绕障更从容"
        "（本次总成功率最高 90%，mixed 95%）。",
        "全向底盘上可直接采样 (vx, vy, wz) 三维，不必离散成栅格。",
    ])
    body(doc, "缺点：", bold="缺点：")
    bullets(doc, [
        "采样方差导致偶发放弃：代价普遍偏高时权重退化，出现过只走 3 m 就报无可行控制（早退）。",
        "对 temperature 与 gamma 敏感：lambda 偏大则动作软绵、偏小则退化成贪心抖动。",
        "算力集中在批处理上，单帧开销比 DWA 高。",
    ])

    # ---- TEB ----
    heading(doc, "1.3　TEB —— 轨迹优化（稀疏非线性最小二乘）", 2)
    body(doc, "实现：teb_local_planner。不再是“选轨迹”，而是解一个稀疏非线性最小二乘问题："
              "把一条“带时间信息的弹性带”（pose 序列 + 相邻位姿时间差）作为决策变量，"
              "用加权残差把带子拉向目标与全局路径、推开障碍、满足运动学。")
    code_block(doc, """\
min sum_k sigma_k^2 * || r_k(s, dT) ||^2
s = {s_0 .. s_n}：带子上的位姿（max_samples 500，期望位姿间隔 dt_ref = 0.3 s）
残差项 obstacle(weight 100, min_obstacle_dist 0.30, inflation_dist 0.45)
       optimaltime(1.0) / viapoint(1.0) / kinematics / acc & vel limits / goal
求解  g2o 稀疏 Levenberg-Marquardt，5 内迭代 x 4 外迭代 = 20 次
    => 取 s_1 方向上的控制量（按 dT 换算速度）-> /cmd_vel_raw""", size=8.6)
    body(doc, "本质：基于梯度的轨迹优化；残差可微，可用二阶方法高效收敛，并显式处理运动学不可行性"
              "（weight_kinematics_*；全向底盘把非完整约束权重置 0）。")
    body(doc, "优点：", bold="优点：")
    bullets(doc, [
        "轨迹整体最优而非贪心 + 打分，窄通道里能把整条带子压成可通形状（本次窄道 16/20 最高）。",
        "支持拓扑多类解（homotopy class），遇障会尝试“从左边/右边”两种同伦类，DWA/MPPI 没有。",
        "平滑度与时间最优性好，到达中位时间最短（18.3~20.7 s）。",
    ])
    body(doc, "缺点：", bold="缺点：")
    bullets(doc, [
        "梯度法怕局部极小；初始带子落在错误同伦类就出不来。",
        "对障碍安全距离极敏感：min_obstacle_dist = 0.30 对半径 0.43~0.53 m 的圆障碍余量偏小，"
        "本次 3 次 collision 全是 TEB 擦到静态障碍（穿透 ≤2.8 cm）。",
        "残差规模大、参数（权重 / dt_ref / 迭代次数）耦合强，调参成本最高。",
    ])

    # ---- 对照表 ----
    heading(doc, "1.4　三者横向对照（含本次 300 run 实测）", 2, page_break=True)
    table(doc, [
        ["维度", "DWA / DWB", "MPPI", "TEB"],
        ["求解范式", "采样 + 线性打分", "采样 + exp(-cost/λ) 加权", "稀疏非线性最小二乘 (g2o)"],
        ["需要梯度", "否", "否", "是"],
        ["预测时域", "1.7 s 单步", "2.8 s（56×0.05）", "变长弹性带"],
        ["候选/变量规模", "≤8000 条采样", "1000 条采样", "≤500 位姿"],
        ["总成功率（300 run）", "83.0%", "90.0%（最高）", "85.0%"],
        ["动态场景成功率", "82.5%", "97.5%（最高）", "85.0%"],
        ["平滑度（越小越好）", "15~32", "6.6~9.9（最好）", "12~49"],
        ["到达中位时间", "19.4~33.8 s", "21.8~34.6 s", "18.3~32.9 s（最快）"],
        ["典型失败模式", "局部极小、磨不到公差超时", "偶发早退", "静态障碍擦碰、窄道同伦类"],
    ], widths=[3.4, 4.4, 4.4, 4.6], font_size=9.0)
    body(doc, "一句话：DWA 是“枚举 + 打分”的稳健派，MPPI 是“随机采样的软最小值”派，"
              "TEB 是“整条轨迹一起优化的梯度”派。", bold="DWA 是")

    # ======================= 第二部分 =======================
    heading(doc, "二、整套测试架构", 1, page_break=True)
    body(doc, "整体分六层：场景/实例 → 仿真 → 执行 → 采集 → 判分 → 结果。"
              "核心原则是判分只看轨迹与契约几何，不看代价地图，因此控制器无法利用“内部信息”取巧。")

    heading(doc, "2.1　架构总览", 2)
    code_block(doc, DIAGRAM, size=7.2)

    heading(doc, "2.2　① 场景与实例层：可复现的“考题”", 2)
    bullets(doc, [
        ("五个场景：", "static_avoid（静态障碍绕行）、dynamic_pedestrian（行人横穿）、"
                     "mixed（障碍 + 行人）、narrow_passage（窄门）、dead_end（U 型死路）。"),
        ("同一 seed 必然同一几何：", "random.Random(seed) 全程固定；每条实例还带一个随机世界位姿"
                                 "（x, y, yaw 取 8 个方向之一），把场景整体旋转平移，防止对世界坐标过拟合。"),
        ("行人是契约而非刚体：", "x = x_cross、y = dir·speed·(t − t_meet)，匀速直线、不停、不钳位；"
                             "判分与仿真因此完全解耦，任何时刻都能解析算出人在哪。"),
        ("生成器自带可达性校验：", "目标/障碍不重叠、路径在地图内可通行，200 次抽样内不合格就重抽，"
                              "保证 100 条实例都有解。"),
    ])

    heading(doc, "2.3　② 仿真层：最小可信闭环", 2)
    bullets(doc, [
        ("kinematic 驱动：", "位姿直接按 /cmd_vel 积分，不做动力学与接触求解，去掉轮胎打滑、质量/摩擦等"
                         "与算法无关的噪声，使对比只反映规划控制算法差异。"),
        ("桥接进程：", "同时提供 /clock（60 Hz 仿真时钟）、/odom（真值）、/tf（map→odom 静态 + "
                    "odom→base_link）、/lidar/points、/imu，以及 /sim/reset、/sim/episode_start 两个考题开关。"),
        ("行人推进：", "按契约公式逐帧修改 USD prim 位姿（曾出现“终端坐标在变、Isaac 窗口里不动”，"
                   "根因是 prim 路径写死成 /World/ScenarioFrame/Pedestrian_%02d 而该节点为空）。"),
    ])

    heading(doc, "2.4　③ 执行层：一份契约，三级速度链", 2)
    bullets(doc, [
        ("单栈多控制器：", "三个 controller 插件注册在同一个 controller_server 内，FollowPath 请求里的 "
                        "controller_id 决定谁上。好处是换基线不换栈；代价是 server 级参数（频率、"
                        "failure_tolerance、两个 checker）只能有一份，launch 里做了唯一来源 + 一致性校验。"),
        ("三级速度链：", "/cmd_vel_raw → flat_speed_limiter（强制平面限速/限加速度）→ "
                    "/cmd_vel_limited → safety_supervisor（odom 过期或急停请求清零）→ /cmd_vel → 仿真。"
                    "控制器只管算法，契约由执行层保证。"),
        ("代价地图口径：", "全局 40×40 m（2 Hz），局部 6×6 m 滚动（10 Hz，120×120）；局部挂自研 "
                      "PedCorridorLayer（把行人横穿走廊写成“道口栏杆”），全局 voxel 层只 marking 不 clearing。"),
        ("全局规划器共用：", "三者共用 SmacPlanner2D，差异只在局部控制，这正是基线对比要的隔离。"),
    ])

    heading(doc, "2.5　④ 采集层：300 个 episode 如何变成可复现数据", 2)
    bullets(doc, [
        ("一场景一次 Isaac、三基线同栈：", "换基线不重启仿真，墙钟从“3 小时量级”降到数十分钟量级；"
                                     "换种子必须重启（换种子 = 换几何）。"),
        ("固定 episode 流程：", "/sim/reset 复位并校验真回到起点 → /sim/episode_start 打行人相位锚点 → "
                          "等全局代价地图标记稳定（≥静态占用一半且 2.5 s 内变化 <2%）→ "
                          "ComputePathToPose（穿致命格的路径一律不采用）→ FollowPath → 60 Hz 记录。"),
        ("双时钟保护：", "仿真时钟超 time_limit_s 中止、挂钟超 4×limit+60 s 也中止，避免 Isaac 卡住时无限等待。"),
        ("可续跑 + 补跑：", "已有 run.json 的 (场景, 种子, 基线) 直接跳过；“栈未就绪”等基础设施失败进补跑队列，"
                     "最多两轮。本次 3 次启动失败全部补回，无缺号。"),
        ("不变式检查：", "起跑前校验 /cmd_vel、/clock、/odom、两个 action status 各恰好 1 个发布者，"
                   "避免残留进程导致两个栈同时跑。"),
    ])

    heading(doc, "2.6　⑤ 判分层：与控制栈完全解耦", 2)
    bullets(doc, [
        ("几何：", "八边形 footprint（0.643 × 0.656 m）逐姿态计算外缘到几何的最近距离，几何包括静态圆障碍、"
                 "墙体，以及按契约公式即时算出的行人圆（r = 0.3 m）。"),
        ("到达与失败分类：", "进入 0.25 m / 0.25 rad 圈即记到达；失败分 collision（最小间距 ≤0）、"
                       "timeout（超时限）、other（其余，主要为卡死）。失败绝不静默丢弃，"
                       "每份 summary.md 都带逐条异常清单。"),
        ("平滑度：", "控制量重采样到 20 Hz 后算总变差 Σ(‖Δv‖/v_max + |Δω|/ω_max)，即 MPPI 明显占优的那一列。"),
    ])

    heading(doc, "2.7　⑥ 结果层：冻结、复核与交付", 2)
    bullets(doc, [
        ("9-1 冻结：", "采集前冻结 12 项 sha256（四份控制器参数、共享代价地图、launch、栏杆层、采集器、"
                    "判分、实例表、TEB 动态库），采集后逐项复核——本次 12 一致 / 0 漂移。"),
        ("9-2 正式成绩：", "本次 258/300 = 86.0%，dynamic_stress（dynamic_pedestrian + mixed）"
                     "106/120 = 88.3%；并给出与上一版同种子的逐条翻转（改善 17 / 变差 15）。"),
        ("9-3 时限冻结：", "T_s = max(T_min,s , 1.5·max_j t̃(s,j))，只用成功 run；本次全量复核 0 行改写，"
                     "时限仍为 60/60/60/60/75 s。"),
        ("A/B 口径：", "NAV2_COSTMAP_YAML=nav2_common_lidar.yaml 切到“地图只含静态障碍、纯靠雷达避人”；"
                    "mixed 5 种子 × 3 基线仅 3/15、4 次撞人；道口栏杆口径 14/15、0 次撞人。"),
        ("交付包：", "MANIFEST.txt 逐文件 sha256 + 自动生成 README（成绩表/指标/复现命令/口径提醒），"
                  "并强制校验包内无本机绝对路径、无历史版本文件。"),
    ])

    # ======================= 第三部分 =======================
    heading(doc, "三、本次正式 300 run 的关键数字", 1, page_break=True)
    heading(doc, "3.1　成功率（3 基线 × 5 场景 × 20 种子）", 2)
    table(doc, [
        ["场景", "DWA", "MPPI", "TEB"],
        ["static_avoid", "18/20 = 90%", "19/20 = 95%", "18/20 = 90%"],
        ["dynamic_pedestrian", "15/20 = 75%", "20/20 = 100%", "20/20 = 100%"],
        ["narrow_passage", "15/20 = 75%", "15/20 = 75%", "16/20 = 80%"],
        ["dead_end", "17/20 = 85%", "17/20 = 85%", "17/20 = 85%"],
        ["mixed", "18/20 = 90%", "19/20 = 95%", "14/20 = 70%"],
        ["合计", "83/100 = 83.0%", "90/100 = 90.0%", "85/100 = 85.0%"],
    ], widths=[4.6, 4.1, 4.1, 4.1], font_size=9.5)
    body(doc, "全体 258/300 = 86.0%；dynamic_stress 106/120 = 88.3%。")

    heading(doc, "3.2　行人安全（本版核心目标）", 2)
    body(doc, "只用契约行人几何重算 120 条动态 run 的行人最小间距：")
    table(doc, [
        ["版本", "行人最小间距 最坏", "中位", "负值（撞到人）条数"],
        ["上一版 v2（2026-11-16）", "−0.299 m", "0.952 m", "3 / 120"],
        ["本版 v3（2026-11-18）", "+0.622 m", "0.926 m", "0 / 120"],
    ], widths=[5.0, 4.2, 3.0, 4.8], font_size=9.5)
    body(doc, "本次 3 次 collision 逐条核对最近几何，全部是 TEB 擦到静态障碍（穿透 ≤2.8 cm），与行人无关："
              "teb/mixed/502（−0.014 m，障碍 (4.25, −1.09) r=0.52）、teb/mixed/516（−0.028 m，"
              "障碍 (4.60, 1.29) r=0.43）、teb/mixed/520（−0.007 m，障碍 (4.62, 0.84) r=0.53）。"
              "即 300 run 内无一次撞人。")

    heading(doc, "3.3　如实记录的两处代价", 2)
    bullets(doc, [
        ("DWA 在 dynamic_pedestrian 有 5 例超时：", "判分到达约 30 s（进入 0.25 m 圈），但控制器内部目标公差"
                                            "收紧到 0.18 m，DWA 磨不到就持续微调至 60 s 上限。收紧公差是为了"
                                            "修“控制器说到了、评测说没到”的 0.258 m 假到位。"),
        ("TEB 在 mixed 掉到 14/20：", "3 次静态障碍擦碰 + 2 次卡死；与栏杆无关，属 TEB 在 "
                                 "min_obstacle_dist = 0.30 下贴障过近的已知弱点。"),
    ])

    # ======================= 附录 =======================
    heading(doc, "附录　复现命令", 1)
    code_block(doc, """\
# 1) 环境
source nav_ws_core/scripts/env.sh
export ISAAC_PYTHON=$HOME/miniconda3/envs/env_isaaclab/bin/python3
source ~/teb_prebuilt/env.sh && source ~/teb_build_ws/install/setup.bash
bash nav_ws_core/scripts/build.sh          # 含自研代价地图层 ped_corridor_layer

# 2) 全量 300 run（可续跑：已有 run.json 的 (场景,种子,基线) 会跳过）
bash nav_ws_core/scripts/run_300.sh 9-2_official_v3

# 3) 只跑 mixed（走廊口径 / 雷达口径 A/B）
bash nav_ws_core/scripts/run_mixed.sh mixed_v3
bash nav_ws_core/scripts/run_mixed.sh mixed_v3_lidar nav2_common_lidar.yaml

# 4) 在 Isaac GUI 里逐基线看动态避障（一条命令一个基线）
bash nav_ws_core/scripts/demo_gui.sh mixed_01 dwa      # 换成 mppi / teb 验证另外两个

# 5) 9-3 时限复核（dry-run，不改 instances.yaml）
python3 nav_ws_core/scripts/freeze_time_limits.py --workset out/9-2_official_v3""", size=8.4)

    doc.save(path)
    print("已写出：%s" % path)


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "/home/wxw/桌面/导航基线算法原理与测试架构.docx"
    build(out)
