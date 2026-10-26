"""Real retained failure frames are regression fixtures, not copied assertions."""
import importlib.util
from pathlib import Path
import json


def test_all_three_recorded_flat_road_false_vehicles_remain_rejected():
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('ground_examples',root/'scripts/audit_r10_ground_examples.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);module.main()
    rows=json.loads((module.OUT/'diagnostics/known_ground_vehicle_filter.json').read_text(encoding='utf-8'))
    assert len(rows)==3 and all(not r['kept'] for r in rows)


def test_duplicate_observation_timestamps_do_not_create_confirmation():
    from src.perception.body_approach import BodyApproachMonitor
    from test_body_approach_r10 import candidate
    m=BodyApproachMonitor()
    for _ in range(15):m.observe([candidate()],0.,[10,0,1])
    assert not m.confirmed
