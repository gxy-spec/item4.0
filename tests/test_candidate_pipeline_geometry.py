from __future__ import annotations

import math

import numpy as np

from src.perception.candidate_pipeline import project_world_point, transform_matrix


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
