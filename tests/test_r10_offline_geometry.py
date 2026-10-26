import importlib.util
from pathlib import Path


def test_actual_xyz_vertex_log_yields_correct_xy_polygon():
    root=Path(__file__).resolve().parents[1]
    spec=importlib.util.spec_from_file_location('r10_evaluator',root/'scripts/evaluate_r10.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    points=[[x,y,z] for x in [-2,2] for y in [-1,1] for z in [0,2]]
    poly=module.polygon(dict(bbox_world_vertices_xyz=points))
    assert len(poly)==4
    assert module.polygon_clearance(poly,[[7,-1],[9,-1],[9,1],[7,1]])==5.
