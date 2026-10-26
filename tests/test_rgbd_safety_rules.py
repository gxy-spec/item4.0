from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for folder in (ROOT / "scripts", ROOT / "src"):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from run_stage1_dynamic import (
    classify_safety_hazards,
    oriented_box_clearance_xy,
    route_lateral_offset_m,
    route_progress_m,
    route_with_overtake_offset,
)
from safety.rgbd_hazard_perception import HazardDetectorConfig, RGBDHazardPerception, pose_matrix,person_depth_support


def test_pedestrian_preempts_overtake_candidate() -> None:
    decision = classify_safety_hazards(
        [
            {"kind": "vehicle", "forward_m": 12.0, "lateral_m": 0.1, "track_confirmed": True, "world_speed_mps": 0.0},
            {
                "kind": "person", "forward_m": 18.0, "lateral_m": 0.2,
                "track_confirmed": True, "confidence": 0.25,
                "bbox_height_px": 42.0, "bbox_width_px": 18.0,
            },
        ]
    )
    assert decision["forced_stop_reason"] == "pedestrian"
    assert "slow_vehicle_candidate" not in decision


def test_static_vehicle_becomes_overtake_candidate_only_after_tracking() -> None:
    unconfirmed = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 10.0, "lateral_m": 0.0, "track_confirmed": False, "confidence": 0.5}]
    )
    confirmed = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 10.0, "lateral_m": 0.0, "track_confirmed": True, "confidence": 0.5, "world_speed_mps": 0.1}]
    )
    assert unconfirmed["forced_stop_reason"] == ""
    assert "slow_vehicle_candidate" not in unconfirmed
    assert confirmed["slow_vehicle_candidate"]["forward_m"] == 10.0


def test_parked_vehicle_in_adjacent_lane_is_not_ego_lead() -> None:
    decision = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 5.3, "lateral_m": 2.0,
          "track_confirmed": True, "confidence": 0.8,
          "world_speed_mps": 0.0, "lateral_speed_mps": 0.0}]
    )
    assert decision["forced_stop_reason"] == ""
    assert "slow_vehicle_candidate" not in decision


def test_stopped_vehicle_in_ego_lane_remains_lead() -> None:
    decision = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 5.3, "lateral_m": 1.5,
          "track_confirmed": True, "confidence": 0.8,
          "world_speed_mps": 0.0, "lateral_speed_mps": 0.0}]
    )
    assert decision["slow_vehicle_candidate"]["forward_m"] == 5.3


def test_vehicle_crossing_from_adjacent_lane_still_stops() -> None:
    decision = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 5.3, "lateral_m": 4.4,
          "track_confirmed": True, "confidence": 0.8,
          "world_speed_mps": 1.2, "lateral_speed_mps": 1.2,
          "track_age_s": 1.0, "lateral_displacement_m": 1.2}]
    )
    assert decision["forced_stop_reason"] == "crossing_vehicle"


def test_crossing_vehicle_is_stop_not_overtake() -> None:
    decision = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 14.0, "lateral_m": 0.5, "track_confirmed": True, "confidence": 0.5,
          "world_speed_mps": 1.2, "lateral_speed_mps": 1.2,
          "track_age_s": 1.0, "lateral_displacement_m": 1.2}]
    )
    assert decision["forced_stop_reason"] == "crossing_vehicle"


def test_one_frame_track_jitter_is_not_crossing_vehicle() -> None:
    decision = classify_safety_hazards(
        [{"kind": "vehicle", "forward_m": 5.3, "lateral_m": 2.2,
          "track_confirmed": True, "confidence": 0.8,
          "world_speed_mps": 8.0, "lateral_speed_mps": 2.0,
          "track_age_s": 0.2, "lateral_displacement_m": 0.1}]
    )
    assert decision["forced_stop_reason"] == ""


def test_unconfirmed_person_does_not_trigger_stop() -> None:
    decision = classify_safety_hazards(
        [{
            "kind": "person", "forward_m": 5.0, "lateral_m": 0.0,
            "track_confirmed": False, "confidence": 0.8,
            "bbox_height_px": 190.0, "bbox_width_px": 75.0,
        }]
    )
    assert decision["forced_stop_reason"] == ""


def test_person_outside_forward_corridor_does_not_trigger_stop() -> None:
    decision = classify_safety_hazards(
        [{
            "kind": "person", "forward_m": 10.0, "lateral_m": 5.2,
            "track_confirmed": True, "confidence": 0.8,
            "bbox_height_px": 50.0, "bbox_width_px": 18.0,
        }]
    )
    assert decision["forced_stop_reason"] == ""


def test_stationary_pedestrian_on_far_curb_does_not_hold_ugv() -> None:
    decision = classify_safety_hazards(
        [{
            "kind": "person", "forward_m": 8.0, "lateral_m": -3.5,
            "track_confirmed": True, "confidence": 0.8,
            "bbox_height_px": 50.0, "bbox_width_px": 18.0,
            "lateral_speed_mps": 0.0,
        }]
    )
    assert decision["forced_stop_reason"] == ""


def test_stationary_false_person_label_on_parked_car_does_not_hold_ugv() -> None:
    decision = classify_safety_hazards(
        [{"kind": "person", "forward_m": 12.0, "lateral_m": 2.05,
          "track_confirmed": True, "confidence": 0.8,
          "bbox_height_px": 50.0, "bbox_width_px": 18.0,
          "lateral_speed_mps": 0.0}]
    )
    assert decision["forced_stop_reason"] == ""


def test_moving_person_entering_crosswalk_triggers_stop() -> None:
    decision = classify_safety_hazards(
        [{
            "kind": "person", "forward_m": 8.0, "lateral_m": 3.5,
            "track_confirmed": True, "confidence": 0.8,
            "bbox_height_px": 50.0, "bbox_width_px": 18.0,
            "lateral_speed_mps": 1.2,
        }]
    )
    assert decision["forced_stop_reason"] == "pedestrian"


def test_finished_crossing_history_does_not_prevent_recovery() -> None:
    person = dict(kind='person', forward_m=18., lateral_m=-3.5,
                  track_confirmed=True, confidence=.8, bbox_height_px=40.,
                  bbox_width_px=10., lateral_displacement_m=7.,
                  lateral_speed_mps=.02, signed_lateral_velocity_mps=-.02)
    assert classify_safety_hazards([person])['forced_stop_reason'] == ''
    person.update(lateral_speed_mps=1., signed_lateral_velocity_mps=-1.)
    assert classify_safety_hazards([person])['forced_stop_reason'] == ''
    person['signed_lateral_velocity_mps'] = 1.
    assert classify_safety_hazards([person])['forced_stop_reason'] == 'pedestrian'
    person.update(lateral_m=-.5, lateral_speed_mps=0., signed_lateral_velocity_mps=0.)
    assert classify_safety_hazards([person])['forced_stop_reason'] == 'pedestrian'


def test_curbside_yield_precedes_front_camera_blind_spot_and_releases_after_crossing():
    person=dict(kind='person',forward_m=7.5,lateral_m=3.5,track_confirmed=True,
        confidence=.9,bbox_height_px=96.,bbox_width_px=26.,world_speed_mps=0.,
        lateral_speed_mps=0.,signed_lateral_velocity_mps=0.)
    assert classify_safety_hazards([person],pedestrian_stop_forward_m=8.)['forced_stop_reason']=='pedestrian'
    person.update(lateral_m=-3.5,crossed_lane_and_clear=True)
    assert classify_safety_hazards([person],pedestrian_stop_forward_m=8.)['forced_stop_reason']==''
    person.update(signed_lateral_velocity_mps=1.,lateral_speed_mps=1.,world_speed_mps=1.)
    assert classify_safety_hazards([person],pedestrian_stop_forward_m=8.)['forced_stop_reason']=='pedestrian'
    person.update(lateral_m=3.5,crossed_lane_and_clear=False,signed_lateral_velocity_mps=0.,lateral_speed_mps=0.)
    assert classify_safety_hazards([person],pedestrian_stop_forward_m=8.)['forced_stop_reason']==''


def test_rgbd_track_remembers_observed_crossing_clearance():
    detector=RGBDHazardPerception(HazardDetectorConfig(Path('unused')),model=object())
    for i,lateral in enumerate([3.5,0.,-3.5,-3.5]):
        batch=[dict(kind='person',world_xyz=[10.,lateral,0.],lateral_m=lateral)]
        detector._update_tracks(batch,i*.2,{'yaw':0.})
    assert batch[0]['crossed_lane_and_clear']


def test_person_depth_support_rejects_flat_shadow_but_keeps_clipped_upright_body():
    h,w=600,800;fx=335.;yy=np.arange(h)[:,None];plane=np.broadcast_to(2.*fx/np.maximum(1,yy-299.5),(h,w)).copy()
    camera=np.eye(4);camera[2,3]=2.
    accepted,stats=person_depth_support(plane,[340,497,415,600],camera,0.,fx)
    assert not accepted and stats['vertical_span_m']<.01
    upright=np.full((h,w),1.5)
    accepted,_=person_depth_support(upright,[340,400,415,600],camera,0.,fx)
    assert accepted


def test_overtake_route_returns_smoothly_to_original_path() -> None:
    route = [[float(index), 0.0, 0.0] for index in range(41)]
    shifted = route_with_overtake_offset(route, start_s=5.0, end_s=35.0, lateral_offset_m=3.5, ramp_m=7.0)
    assert shifted[0] == route[0]
    assert shifted[-1] == route[-1]
    assert shifted[20][1] == 3.5
    assert 0.0 < shifted[6][1] < 3.5
    assert 0.0 < shifted[34][1] < 3.5


def test_route_lateral_offset_uses_current_position() -> None:
    route = [[0.0, float(index), 0.0] for index in range(11)]
    assert abs(route_lateral_offset_m(route, [-2.5, 5.0, 0.0]) - 2.5) < 1e-6


def test_route_progress_cursor_advances_past_initial_search_window() -> None:
    route = [[0.0, float(index), 0.0] for index in range(60)]
    progress, index = route_progress_m(route, [0.0, 52.0, 0.0], cursor_index=30)
    assert index == 52
    assert progress == 52.0


def test_tracker_identifies_lateral_world_motion() -> None:
    detector = RGBDHazardPerception(HazardDetectorConfig(Path("unused")), model=object())
    detections = []
    for index, timestamp in enumerate((0.0, 0.2, 0.4)):
        row = {"kind": "vehicle", "world_xyz": [float(index) * 0.3, 0.0, 0.0]}
        batch = [row]
        detector._update_tracks(batch, timestamp, {"yaw": -90.0})
        detections.append(batch[0])
    assert detections[-1]["track_confirmed"] is True
    assert detections[-1]["lateral_speed_mps"] > 0.6


def test_track_velocity_smoothing_rejects_single_frame_depth_jitter() -> None:
    detector = RGBDHazardPerception(HazardDetectorConfig(Path("unused")), model=object())
    for timestamp, x in ((0.0, 0.0), (0.1, 0.15), (0.2, 0.0)):
        batch = [{"kind": "vehicle", "world_xyz": [x, 0.0, 0.0]}]
        detector._update_tracks(batch, timestamp, {"yaw": -90.0})
    assert batch[0]["world_speed_mps"] < 0.8


def test_pose_matrix_keeps_ego_translation() -> None:
    matrix = pose_matrix({"x": 3.0, "y": -2.0, "z": 1.0, "yaw": 0.0})
    assert np.allclose(matrix[:3, 3], [3.0, -2.0, 1.0])


def test_oriented_vehicle_boxes_report_parallel_side_clearance_not_circle_overlap() -> None:
    ugv = {"x": -45.0137, "y": -26.5939, "yaw": -86.04, "bbox_extent_xyz": [2.396, 1.082, 0.744]}
    obstacle = {"x": -41.7727, "y": -26.0939, "yaw": -89.57, "bbox_extent_xyz": [2.091, 0.997, 0.693]}
    assert 1.0 < oriented_box_clearance_xy(ugv, obstacle) < 1.3


def test_oriented_vehicle_boxes_report_negative_clearance_when_projected_boxes_overlap() -> None:
    first = {"x": 0.0, "y": 0.0, "yaw": -90.0, "bbox_extent_xyz": [2.0, 1.0, 0.7]}
    second = {"x": 1.5, "y": 0.0, "yaw": -90.0, "bbox_extent_xyz": [2.0, 1.0, 0.7]}
    assert oriented_box_clearance_xy(first, second) < 0.0
