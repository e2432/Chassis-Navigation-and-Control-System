"""Regression checks for the contract table, real preview motion and spatial diversity."""
from pathlib import Path
from copy import deepcopy
import importlib.util
import json
import math
import pytest
import yaml
from robot_benchmark.gen_scenarios import load_templates, main
from robot_benchmark.scenario_gen import SCENARIO_KEYS, REQUIRED_FIELDS, generate_document, find_instance, pedestrian_position, transform_pose, validate_document, validate_instance
ROOT = Path(__file__).resolve().parents[3]
MANIFEST = ROOT.parent / 'docs/scenarios/instances.yaml'

@pytest.fixture(scope='module')
def document():
    return yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))

def test_contract_fields_counts_and_seeds(document):
    assert validate_document(document) == []
    assert len(document['instances']) == 100
    for (index, key) in enumerate(SCENARIO_KEYS, 1):
        values = [i for i in document['instances'] if i['scenario'] == key]
        assert [i['run'] for i in values] == list(range(1, 21))
        assert [i['seed'] for i in values] == list(range(index * 100 + 1, index * 100 + 21))
        for inst in values:
            assert set(inst) == REQUIRED_FIELDS
            assert inst['goal']['yaw'] == pytest.approx(math.atan2(inst['goal']['y'], inst['goal']['x']))
            assert inst['path_length_m'] == pytest.approx(math.hypot(inst['goal']['x'], inst['goal']['y']))

def test_frozen_table_is_reproducible(document):
    assert generate_document(load_templates(ROOT / 'scenarios')) == document
    assert generate_document(load_templates(ROOT / 'scenarios')) == document

@pytest.mark.parametrize('scenario', SCENARIO_KEYS)
def test_every_layout_is_unique(document, scenario):
    values = [i for i in document['instances'] if i['scenario'] == scenario]
    geometry = [json.dumps({k: i[k] for k in ('goal', 'obstacles', 'pedestrians', 'walls')}, sort_keys=True) for i in values]
    assert len(set(geometry)) == 20

def test_frozen_pedestrians_cross_in_both_directions_without_stopping(document):
    pedestrians = [p for i in document['instances'] for p in i['pedestrians']]
    assert len(pedestrians) == 68
    assert {p['dir'] for p in pedestrians} == {-1, 1}
    for p in pedestrians:
        before = pedestrian_position(p, p['t_meet'] - 1.0)
        at = pedestrian_position(p, p['t_meet'])
        after = pedestrian_position(p, p['t_meet'] + 1.0)
        assert at == (p['x_cross'], 0.0)
        assert before[1] * after[1] < 0
        for t in (0.0, p['t_meet'] - 0.01, p['t_meet'] + 0.01, 60.0, 75.0, 200.0):
            y0 = pedestrian_position(p, t)[1]
            y1 = pedestrian_position(p, t + 0.001)[1]
            assert (y1 - y0) / 0.001 == pytest.approx(p['dir'] * p['speed'], abs=1e-08)

def test_actual_preview_animation_does_not_teleport_or_stop(document, monkeypatch):
    path = ROOT / 'src/robot_sim/robot_sim/scripts/scene_from_scenario.py'
    spec = importlib.util.spec_from_file_location('contract_scene_preview', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    observed = {}
    monkeypatch.setattr(module, 'set_translate', lambda prim, xyz: observed.__setitem__(prim, xyz))
    inst = find_instance(document, 'dynamic_pedestrian', 1)
    p = inst['pedestrians'][0]
    scene = {'peds': [{'prim': 'ped', 'spec': p}]}
    for t in (0.0, p['t_meet'] - 0.01, p['t_meet'], p['t_meet'] + 0.01, 200.0):
        module.animate(scene, t)
        (x, y) = pedestrian_position(p, t)
        assert observed['ped'] == pytest.approx((x, y, 0.85))
    assert abs(pedestrian_position(p, p['t_meet'] + 0.01)[1] - pedestrian_position(p, p['t_meet'] - 0.01)[1]) < 0.03

def test_narrow_positions_dimensions_and_world_headings(document):
    values = [i for i in document['instances'] if i['scenario'] == 'narrow_passage']
    kinds = set()
    centers = set()
    angles = set()
    for i in values:
        assert len(i['walls']) == 2 and (not i['pedestrians'])
        (low, high) = i['walls']
        width = high['y0'] - low['y1']
        assert width >= 1.3
        assert (width - 0.662) / 2 >= 0.3
        layout = document['layouts'][str(i['seed'])]
        assert layout['net_width_m'] == pytest.approx(width, abs=1e-06)
        centers.add((layout['center_x'], layout['center_y']))
        kinds.add(layout['kind'])
        angles.add(document['world_poses'][str(i['seed'])]['yaw'])
    assert len(centers) == 20 and len(angles) == 8
    assert kinds == {'guide_corridor', 'gate_corridor'}
    assert any((document['layouts'][str(i['seed'])]['center_y'] < 0 for i in values))
    assert any((document['layouts'][str(i['seed'])]['center_y'] > 0 for i in values))

def test_dead_end_openings_match_actual_three_wall_geometry(document):
    openings = []
    centers = set()
    angles = set()
    for i in document['instances']:
        if i['scenario'] != 'dead_end':
            continue
        assert len(i['walls']) == 3
        layout = document['layouts'][str(i['seed'])]
        opening = layout['opening']
        cap = i['walls'][2]
        (cx, cy) = (layout['center_x'], layout['center_y'])
        cap_x = (cap['x0'] + cap['x1']) / 2
        cap_y = (cap['y0'] + cap['y1']) / 2
        if opening == 'west':
            assert cap_x > cx
        elif opening == 'east':
            assert cap_x < cx
        elif opening == 'south':
            assert cap_y > cy
        elif opening == 'north':
            assert cap_y < cy
        else:
            pytest.fail(opening)
        assert i['goal']['x'] >= max((w['x1'] for w in i['walls'])) + 1.99
        assert layout['mouth_width_m'] >= 1.3
        centers.add((cx, cy))
        openings.append(opening)
        angles.add(document['world_poses'][str(i['seed'])]['yaw'])
    assert len(centers) == 20 and len(angles) == 8
    assert {name: openings.count(name) for name in set(openings)} == dict.fromkeys(('west', 'east', 'south', 'north'), 5)

def test_world_transform_moves_all_path_frame_content_rigidly():
    pose = {'x': 3.0, 'y': -2.0, 'yaw': math.pi / 2}
    assert transform_pose(0.0, 0.0, 0.0, pose) == pytest.approx((3.0, -2.0, math.pi / 2))
    assert transform_pose(10.0, 0.0, 0.0, pose) == pytest.approx((3.0, 8.0, math.pi / 2))
    a = transform_pose(1.0, 2.0, 0.0, pose)
    b = transform_pose(4.0, 6.0, 0.0, pose)
    assert math.hypot(a[0] - b[0], a[1] - b[1]) == pytest.approx(5.0)

@pytest.mark.parametrize('case', ('legacy', 'direction', 'speed', 'wall', 'length', 'blocked_goal'))
def test_rejects_invalid_instance_fields_and_geometry(document, case):
    inst = deepcopy(find_instance(document, 'dynamic_pedestrian', 1))
    if case == 'legacy':
        inst['dynamic'] = []
    if case == 'direction':
        inst['pedestrians'][0]['dir'] = 0
    if case == 'speed':
        inst['pedestrians'][0]['speed'] = -1.0
    if case == 'wall':
        inst['walls'] = [{'x0': 5.0, 'y0': 1.0, 'x1': 4.0, 'y1': 2.0}]
    if case == 'length':
        inst['path_length_m'] += 1.0
    if case == 'blocked_goal':
        inst['obstacles'] = [{'x': inst['goal']['x'], 'y': 0.0, 'r': 1.0}]
    assert validate_instance(inst)

def test_rejects_disconnected_scene(document):
    inst = deepcopy(find_instance(document, 'static_avoid', 1))
    inst['obstacles'] = [{'x': inst['goal']['x'] + 1.2 * math.cos(k * math.pi / 6), 'y': 1.2 * math.sin(k * math.pi / 6), 'r': 0.3} for k in range(12)]
    assert any(('A*' in error for error in validate_instance(inst)))

def test_rejects_duplicate_or_missing_runs(document):
    duplicate = deepcopy(document)
    duplicate['instances'][-1] = deepcopy(duplicate['instances'][0])
    assert validate_document(duplicate, False)
    missing = deepcopy(document)
    missing['instances'].pop()
    assert validate_document(missing, False)

def test_check_cli_is_read_only_and_detects_bad_table(document, tmp_path):
    data = tmp_path / 'instances.yaml'
    data.write_text(yaml.safe_dump(document), encoding='utf-8')
    before = data.read_bytes()
    assert main(['--out', str(data), '--templates', str(ROOT / 'scenarios'), '--check']) == 0
    assert data.read_bytes() == before
    bad = deepcopy(document)
    bad['instances'][0]['time_limit_s'] += 1
    data.write_text(yaml.safe_dump(bad), encoding='utf-8')
    assert main(['--out', str(data), '--templates', str(ROOT / 'scenarios'), '--check']) == 2


def test_installed_share_defaults(document, tmp_path, monkeypatch):
    import robot_benchmark.scenario_gen as module
    prefix = tmp_path / "install"
    package_file = prefix / "lib/python3.10/site-packages/robot_benchmark/scenario_gen.py"
    package_file.parent.mkdir(parents=True)
    package_file.touch()
    share = prefix / "share/robot_benchmark/scenarios"
    share.mkdir(parents=True)
    (share / "instances.yaml").write_text(yaml.safe_dump(document), encoding="utf-8")
    for key in SCENARIO_KEYS:
        (share / (key + ".yaml")).write_text("scenario: " + key, encoding="utf-8")
    monkeypatch.setattr(module, "__file__", str(package_file))
    monkeypatch.delenv("NAV_WS", raising=False)
    monkeypatch.chdir(tmp_path)
    assert module.default_manifest_path() == share / "instances.yaml"
    assert module.default_templates_path() == share


def test_bridge_kinematic_state_retains_loaded_world_pose():
    import ast
    from types import SimpleNamespace
    import numpy as np
    source = ROOT / "src/robot_sim/robot_sim/scripts/isaac_sim_bridge.py"
    wanted = {"initial_position", "initial_orientation", "iqw", "iqx", "iqy", "iqz", "initial_yaw", "state"}
    assignments = []
    for node in ast.parse(source.read_text(encoding="utf-8")).body:
        if isinstance(node, ast.Assign):
            names = {child.id for target in node.targets for child in ast.walk(target) if isinstance(child, ast.Name)}
            if names & wanted:
                assignments.append(node)
    namespace = {"math": math, "np": np,
                 "art": SimpleNamespace(get_world_poses=lambda: (np.array([[3., -2., 0.]]), np.array([[math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)]])))}
    exec(compile(ast.Module(body=assignments, type_ignores=[]), str(source), "exec"), namespace)
    assert namespace["state"]["pose"] == pytest.approx((3., -2., math.pi/2))


def test_generate_cli_produces_identical_frozen_bytes(tmp_path):
    output = tmp_path / "generated.yaml"
    args = ["--out", str(output), "--templates", str(ROOT / "scenarios")]
    assert main(args) == 0
    first = output.read_bytes()
    assert main(args) == 0
    assert output.read_bytes() == first == MANIFEST.read_bytes()
