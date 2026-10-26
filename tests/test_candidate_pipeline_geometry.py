from __future__ import annotations

import math

import numpy as np

from pathlib import Path
from types import SimpleNamespace

from src.perception.candidate_pipeline import CandidatePipeline, PipelineConfig, project_world_point, transform_matrix
from src.perception.task_spec import TaskSpec


def test_fixed_head_car_cannot_confirm_a_requested_van() -> None:
    pipeline = CandidatePipeline(PipelineConfig(model_path=Path('best.pt'), detector_backend='custom',
        task_spec=TaskSpec.legacy_red_van(),enable_attribute_proposals=False,temporal_hits=2),
        model=SimpleNamespace(names={0:'van',1:'car',2:'truck',3:'bus'}))
    image=np.zeros((600,800,3),dtype=np.uint8); image[:,:,2]=255
    result=SimpleNamespace(names={0:'van',1:'car'}, boxes=SimpleNamespace(
        xyxy=np.array([[200,200,350,350]],dtype=float),conf=np.array([.99]),cls=np.array([1])))
    for frame in range(3):
        candidates=pipeline.process(frame,frame*.1,image,np.full((600,800),20.),
            {'location':[0.,0.,30.],'rotation':[-90.,0.,0.]},result)
        assert candidates[0]['category_match_score']==0.
        assert not candidates[0]['temporal_confirmed']


def test_nadir_projection_places_north_east_point_correctly() -> None:
    transform = {"location": [-21.414405822753906, -56.06676483154297, 58.5], "rotation": [-90.0, 0.0, 0.0]}
    u, v, depth = project_world_point((-39.260032653808594, -41.02096939086914, 1.547958356142044), transform, 800, 600, 100.0)
    assert math.isclose(u, 488.67, abs_tol=0.05)
    assert math.isclose(v, 405.17, abs_tol=0.05)
    assert depth > 0


def test_transform_matrix_is_rigid() -> None:
    matrix = transform_matrix({"location": [2.0, -3.0, 7.0], "rotation": [-90.0, 17.0, 4.0]})
    rotation = matrix[:3, :3]
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-10)
    assert np.allclose(matrix[:3, 3], [2.0, -3.0, 7.0])


def test_candidate_source_label_tracks_configured_detector() -> None:
    pipeline = CandidatePipeline(
        PipelineConfig(model_path=Path("models/yolo26n.pt")),
        model=object(),
    )
    assert pipeline.detector_proposal_source == "yolo26n_coco"
