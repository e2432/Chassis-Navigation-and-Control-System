"""Contract v1.2 scenario table; path-frame geometry and frozen world placements."""
from __future__ import annotations
from functools import lru_cache
import heapq
import math
import os
from pathlib import Path
import random
SCENARIO_KEYS = ('static_avoid', 'dynamic_pedestrian', 'narrow_passage', 'dead_end', 'mixed')
SEED_BASE = {key: (i + 1) * 100 for (i, key) in enumerate(SCENARIO_KEYS)}
GENERATOR_VERSION = '2.0.0'
REQUIRED_FIELDS = frozenset(('scenario', 'scenario_index', 'run', 'seed', 'path_length_m', 'time_limit_s', 'goal', 'obstacles', 'pedestrians', 'walls'))
ROBOT_RADIUS_M = 0.35
FOOTPRINT_ENVELOPE_M = 0.662
GRID_M = 0.1
WORLD_YAWS_DEG = (0, 45, 90, 135, 180, 225, 270, 315)

def default_manifest_path():
    bases = [Path(os.environ['NAV_WS']).resolve()] if os.environ.get('NAV_WS') else []
    bases += [Path(__file__).resolve().parents[3], Path.cwd()]
    for base in bases:
        for parent in (base, *base.parents):
            candidate = parent / 'docs/scenarios/instances.yaml'
            if candidate.is_file():
                return candidate
    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'share/robot_benchmark/scenarios/instances.yaml'
        if candidate.is_file():
            return candidate
    return Path(__file__).resolve().parents[4] / 'docs/scenarios/instances.yaml'

def default_templates_path():
    for parent in Path(__file__).resolve().parents:
        for candidate in (parent / 'scenarios', parent / 'share/robot_benchmark/scenarios'):
            if all(((candidate / (key + '.yaml')).is_file() for key in SCENARIO_KEYS)):
                return candidate
    return Path(__file__).resolve().parents[3] / 'scenarios'

def pedestrian_position(pedestrian, t):
    """No start-time gate, stopping bound, clamping or direction reversal."""
    return (pedestrian['x_cross'], pedestrian['dir'] * pedestrian['speed'] * (t - pedestrian['t_meet']))

def transform_pose(x, y, yaw, world_pose):
    angle = world_pose['yaw']
    (c, s) = (math.cos(angle), math.sin(angle))
    return (world_pose['x'] + c * x - s * y, world_pose['y'] + s * x + c * y, yaw + angle)

def _blocked(x, y, obstacles, walls, margin):
    return any((math.hypot(x - o[0], y - o[1]) <= o[2] + margin for o in obstacles)) or any((w[0] - margin <= x <= w[2] + margin and w[1] - margin <= y <= w[3] + margin for w in walls))

@lru_cache(maxsize=1024)
def _reachable(goal, obstacles, walls):
    xs = [0.0, goal[0], *(v for w in walls for v in (w[0], w[2])), *(o[0] for o in obstacles)]
    ys = [0.0, goal[1], *(v for w in walls for v in (w[1], w[3])), *(o[1] for o in obstacles)]
    (x0, x1) = (math.floor(min(xs) - 2), math.ceil(max(xs) + 2))
    (y0, y1) = (math.floor(min(ys) - 3), math.ceil(max(ys) + 3))
    (nx, ny) = (round((x1 - x0) / GRID_M), round((y1 - y0) / GRID_M))

    def cell(x, y):
        return (round((x - x0) / GRID_M), round((y - y0) / GRID_M))

    @lru_cache(maxsize=None)
    def free(ix, iy):
        return 0 <= ix <= nx and 0 <= iy <= ny and (not _blocked(x0 + ix * GRID_M, y0 + iy * GRID_M, obstacles, walls, ROBOT_RADIUS_M))
    (start, end) = (cell(0, 0), cell(*goal))
    if not free(*start) or not free(*end):
        return False
    (heap, costs) = ([(0.0, start)], {start: 0.0})
    while heap:
        (_, current) = heapq.heappop(heap)
        if current == end:
            return True
        for (dx, dy) in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            nxt = (current[0] + dx, current[1] + dy)
            if not free(*nxt):
                continue
            if dx and dy and (not (free(current[0] + dx, current[1]) and free(current[0], current[1] + dy))):
                continue
            cost = costs[current] + math.hypot(dx, dy)
            if cost < costs.get(nxt, math.inf):
                costs[nxt] = cost
                heapq.heappush(heap, (cost + math.hypot(nxt[0] - end[0], nxt[1] - end[1]), nxt))
    return False

def _finite(value):
    return isinstance(value, (int, float)) and (not isinstance(value, bool)) and math.isfinite(value)

def validate_instance(inst, check_reachability=True):
    if not isinstance(inst, dict) or set(inst) != REQUIRED_FIELDS:
        return ['实例字段必须恰为契约的十个字段']
    key = inst['scenario']
    run = inst['run']
    bad = []
    if key not in SCENARIO_KEYS:
        return ['未知场景键']
    if type(run) is not int or not 1 <= run <= 20:
        return ['run 必须为 1..20']
    index = SCENARIO_KEYS.index(key) + 1
    if type(inst['scenario_index']) is not int or type(inst['seed']) is not int or inst['scenario_index'] != index or (inst['seed'] != index * 100 + run):
        bad.append('scenario_index/seed 不符合固定编号规则')
    goal = inst['goal']
    if not isinstance(goal, dict) or set(goal) != {'x', 'y', 'yaw'} or (not all((_finite(v) for v in goal.values()))):
        return bad + ['goal 必须包含有限数值 x/y/yaw']
    if not _finite(inst['path_length_m']) or not math.isclose(inst['path_length_m'], math.hypot(goal['x'], goal['y']), abs_tol=1e-06):
        bad.append('path_length_m 与起终点直线长度不同')
    if not math.isclose(goal['yaw'], math.atan2(goal['y'], goal['x']), abs_tol=1e-06):
        bad.append('goal.yaw 必须为起终点连线方向')
    if not _finite(inst['time_limit_s']) or inst['time_limit_s'] <= 0:
        bad.append('time_limit_s 必须为正数')
    for name in ('obstacles', 'pedestrians', 'walls'):
        if not isinstance(inst[name], list):
            return bad + [name + ' 必须为列表']
    for o in inst['obstacles']:
        if not isinstance(o, dict) or set(o) != {'x', 'y', 'r'} or (not all((_finite(v) for v in o.values()))) or (o['r'] <= 0):
            bad.append('静态圆障碍字段应为 x/y/r，半径为正')
    if len(inst['pedestrians']) > 2:
        bad.append('基准行人数不得超过 2')
    for p in inst['pedestrians']:
        if not isinstance(p, dict) or set(p) != {'x_cross', 'speed', 't_meet', 'dir', 'r'} or (not all((_finite(v) for v in p.values()))) or (p['speed'] <= 0) or (p['r'] <= 0) or (type(p['dir']) is not int) or (p['dir'] not in (-1, 1)) or (p['t_meet'] < 0):
            bad.append('行人应符合 x_cross/speed/t_meet/dir/r，speed>0，dir=±1')
    if len(inst['walls']) > 3:
        bad.append('轴对齐墙体不得超过 3 块')
    for w in inst['walls']:
        if not isinstance(w, dict) or set(w) != {'x0', 'y0', 'x1', 'y1'} or (not all((_finite(v) for v in w.values()))) or (w['x0'] >= w['x1']) or (w['y0'] >= w['y1']):
            bad.append('墙体边界必须满足 x0<x1，y0<y1')
    if bad:
        return bad
    circles = tuple(((o['x'], o['y'], o['r']) for o in inst['obstacles']))
    walls = tuple(((w['x0'], w['y0'], w['x1'], w['y1']) for w in inst['walls']))
    for (label, (x, y)) in (('起点', (0, 0)), ('目标', (goal['x'], goal['y']))):
        if _blocked(x, y, circles, walls, ROBOT_RADIUS_M):
            bad.append(label + '与膨胀后的静态几何相交')
    if not bad and check_reachability and (not _reachable((goal['x'], goal['y']), circles, walls)):
        bad.append('膨胀 0.35 m 后起点到目标无 A* 通路')
    return bad

def _static(rng, tpl):
    length = rng.uniform(*tpl['corridor_length_m'])
    count = rng.randint(*tpl['obstacle_count'])
    circles = []
    for _ in range(count):
        radius = rng.uniform(*tpl['obstacle_radius_m'])
        x = rng.uniform(tpl['obstacle_x_range_m'][0], length - tpl['obstacle_x_range_m'][1])
        y = rng.uniform(-tpl['lateral_offset_m'], tpl['lateral_offset_m'])
        x += rng.uniform(-0.3, 0.3)
        y += rng.uniform(-0.3, 0.3)
        if abs(y) <= tpl['corridor_half_width_m'] - radius:
            circles.append({'x': round(x, 3), 'y': round(y, 3), 'r': round(radius, 3)})
    return (round(length, 3), circles)

def _pedestrians(rng, tpl, length=None):
    length = rng.uniform(*(tpl['corridor_length_m'] if length is None else (length, length)))
    pedestrians = []
    for _ in range(rng.randint(*tpl['pedestrian_count'])):
        speed = rng.uniform(*tpl['pedestrian_speed_mps'])
        cross = length / 2 + rng.uniform(-tpl['cross_x_jitter_m'], tpl['cross_x_jitter_m'])
        direction = 1 if rng.random() < 0.5 else -1
        meet = tpl['encounter_time_s'] + rng.uniform(-tpl['encounter_jitter_s'], tpl['encounter_jitter_s'])
        pedestrians.append({'x_cross': round(cross, 3), 'speed': round(speed, 3), 't_meet': round(meet, 3), 'dir': direction, 'r': tpl['pedestrian_radius_m']})
    return (round(length, 3), pedestrians)

def _box(cx, cy, width, height):
    return {'x0': round(cx - width / 2, 3), 'y0': round(cy - height / 2, 3), 'x1': round(cx + width / 2, 3), 'y1': round(cy + height / 2, 3)}

def _narrow(rng, tpl, run):
    cx = rng.uniform(*tpl['center_x_m'])
    cy = rng.uniform(*tpl['center_y_m'])
    width = rng.uniform(*tpl['net_width_m'])
    length = rng.uniform(*tpl['passage_length_m'])
    thickness = rng.uniform(*tpl['wall_thickness_m'])
    kind = 'guide_corridor' if run % 2 else 'gate_corridor'
    if kind == 'gate_corridor':
        thickness += 1.1
    walls = [_box(cx, cy + side * (width / 2 + thickness / 2), length, thickness) for side in (-1, 1)]
    goal = round(max(rng.uniform(*tpl['goal_distance_m']), cx + length / 2 + 2.0), 3)
    return (goal, walls, {'kind': kind, 'center_x': round(cx, 3), 'center_y': round(cy, 3), 'net_width_m': round(walls[1]['y0'] - walls[0]['y1'], 3), 'length_m': round(walls[0]['x1'] - walls[0]['x0'], 3)})

def _dead_end(rng, tpl, run):
    opening = ('west', 'east', 'south', 'north')[(run - 1) % 4]
    cx = rng.uniform(*tpl['center_x_m'])
    cy = rng.uniform(*tpl['center_y_m'])
    (mouth, depth, thickness) = (rng.uniform(*tpl[k]) for k in ('mouth_width_m', 'depth_m', 'wall_thickness_m'))
    local = [_box(depth / 2, side * (mouth / 2 + thickness / 2), depth, thickness) for side in (-1, 1)]
    local.append(_box(depth + thickness / 2, 0, thickness, mouth + 2 * thickness))
    turns = {'west': 0, 'south': 1, 'east': 2, 'north': 3}[opening]
    walls = []
    for w in local:
        points = []
        for (x, y) in ((w['x0'], w['y0']), (w['x0'], w['y1']), (w['x1'], w['y0']), (w['x1'], w['y1'])):
            x -= depth / 2
            for _ in range(turns):
                (x, y) = (-y, x)
            points.append((x + cx, y + cy))
        walls.append({'x0': round(min((p[0] for p in points)), 3), 'y0': round(min((p[1] for p in points)), 3), 'x1': round(max((p[0] for p in points)), 3), 'y1': round(max((p[1] for p in points)), 3)})
    goal = round(max(rng.uniform(*tpl['goal_distance_m']), max((w['x1'] for w in walls)) + 2.0), 3)
    return (goal, walls, {'kind': 'u_trap', 'opening': opening, 'center_x': round(cx, 3), 'center_y': round(cy, 3), 'mouth_width_m': round(mouth, 3), 'depth_m': round(depth, 3)})

def generate_instance(scenario, run, tpl):
    index = SCENARIO_KEYS.index(scenario) + 1
    seed = index * 100 + run
    rng = random.Random(seed)
    for _ in range(200):
        (circles, pedestrians, walls, layout) = ([], [], [], {})
        if scenario == 'static_avoid':
            (length, circles) = _static(rng, tpl)
        elif scenario == 'dynamic_pedestrian':
            (length, pedestrians) = _pedestrians(rng, tpl)
        elif scenario == 'mixed':
            raw_length = rng.uniform(*tpl['corridor_length_m'])
            (length, circles) = _static(rng, dict(tpl, corridor_length_m=(raw_length, raw_length)))
            (_, pedestrians) = _pedestrians(rng, tpl, raw_length)
        elif scenario == 'narrow_passage':
            (length, walls, layout) = _narrow(rng, tpl, run)
        elif scenario == 'dead_end':
            (length, walls, layout) = _dead_end(rng, tpl, run)
        else:
            raise ValueError(scenario)
        inst = {'scenario': scenario, 'scenario_index': index, 'run': run, 'seed': seed, 'path_length_m': length, 'time_limit_s': float(tpl['timeout_s']), 'goal': {'x': length, 'y': 0.0, 'yaw': 0.0}, 'obstacles': circles, 'pedestrians': pedestrians, 'walls': walls}
        if not validate_instance(inst):
            return (inst, layout)
    raise RuntimeError(f'{scenario} run={run}: 200 次抽样后仍不可达')

def generate_document(templates):
    (instances, poses, layouts) = ([], {}, {})
    for key in SCENARIO_KEYS:
        for run in range(1, 21):
            (inst, layout) = generate_instance(key, run, templates[key])
            instances.append(inst)
            rng = random.Random(inst['seed'] + 9000000)
            poses[str(inst['seed'])] = {'x': round(rng.uniform(-8, 8), 3), 'y': round(rng.uniform(-8, 8), 3), 'yaw': round(math.radians(WORLD_YAWS_DEG[(run - 1) % 8]), 9)}
            if key not in ('narrow_passage', 'dead_end'):
                poses[str(inst['seed'])] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
            if layout:
                layouts[str(inst['seed'])] = layout
    return {'schema_version': 1, 'generator_version': GENERATOR_VERSION, 'asset_version': '2026-11-06', 'coordinate_frame': 'path', 'instances': instances, 'world_poses': poses, 'layouts': layouts}

def validate_document(document, check_reachability=True):
    if not isinstance(document, dict) or not isinstance(document.get('instances'), list):
        return ['缺少 instances 列表']
    instances = document['instances']
    bad = []
    seen = set()
    if document.get('schema_version', 1) != 1:
        bad.append('schema_version 必须为 1')
    if document.get('coordinate_frame', 'path') != 'path':
        bad.append('实例几何必须在 path frame')
    if len(instances) != 100:
        bad.append('必须为 5×20=100 个实例')
    for (i, inst) in enumerate(instances):
        problems = validate_instance(inst, check_reachability)
        bad.extend((f'实例 {i + 1}: {p}' for p in problems))
        if problems:
            continue
        pair = (inst['scenario'], inst['run'])
        if pair in seen:
            bad.append(f'重复实例 {pair}')
        seen.add(pair)
    if seen != {(key, run) for key in SCENARIO_KEYS for run in range(1, 21)}:
        bad.append('五类场景 run=1..20 未完整覆盖')
    if 'world_poses' in document:
        if not isinstance(document['world_poses'], dict):
            return bad + ['world_poses 必须为字典']
        if set(document['world_poses']) != {str(i['seed']) for i in instances if isinstance(i, dict) and 'seed' in i}:
            bad.append('world_poses 必须覆盖所有实例 seed')
        for pose in document['world_poses'].values():
            if not isinstance(pose, dict) or set(pose) != {'x', 'y', 'yaw'} or (not all((_finite(v) for v in pose.values()))):
                bad.append('world_pose 必须为有限 x/y/yaw')
    return bad

def find_instance(document, scenario, run):
    for inst in document['instances']:
        if inst['scenario'] == scenario and inst['run'] == run:
            return inst
    raise ValueError(f'实例不存在: {scenario} run={run}')
