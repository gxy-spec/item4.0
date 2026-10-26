from copy import deepcopy
import pytest
from src.evaluation.acceptance_readiness import REQUIRED_EVIDENCE, readiness_errors, route_budget_error


def complete_evidence():
    result={key:{'passed':True,'evidence_path':f'{key}.json'} for key in REQUIRED_EVIDENCE}
    result.update(scope_matches_requested_baseline=True,rgb_visibility_is_independent_of_instance_depth_proxy=True)
    return result


def test_failed_calibration_cannot_be_promoted_to_formal_by_fallback_threshold():
    cal={'constraints_satisfied':False,'chosen':{'detector_threshold':.03,'color_threshold':.1}}
    assert any('diagnostic fallback' in e for e in readiness_errors(cal,complete_evidence()))


@pytest.mark.parametrize('missing',REQUIRED_EVIDENCE)
def test_missing_preflight_evidence_blocks_formal_batch(missing):
    evidence=complete_evidence();evidence.pop(missing)
    assert any(missing in e for e in readiness_errors({'constraints_satisfied':True},evidence))


def test_geometry_only_visibility_cannot_admit_rgb_task():
    evidence=complete_evidence();evidence['rgb_visibility_is_independent_of_instance_depth_proxy']=False
    assert readiness_errors({'constraints_satisfied':True},evidence)


def test_one_scene_is_not_four_scene_coverage():
    evidence=complete_evidence();evidence['scope_matches_requested_baseline']=False
    assert readiness_errors({'constraints_satisfied':True},evidence)


def test_recorded_red_van_replan_is_impossible_even_at_maximum_speed():
    assert route_budget_error(448.83486826873303,190-48.35000072047114,2.0)


def test_necessary_budget_check_does_not_reject_short_initial_route():
    assert route_budget_error(113.146,190,2.0) is None


def test_complete_traceable_admission_has_no_missing_evidence():
    assert readiness_errors({'constraints_satisfied':True},complete_evidence())==[]
