"""Fail-closed experiment admission, separate from task acceptance thresholds."""
from __future__ import annotations
from math import isfinite


REQUIRED_EVIDENCE = ('baseline_alignment', 'rgb_visibility_review', 'annotation_review',
                     'paired_model_comparison', 'positive_smoke', 'negative_smoke', 'route_budget_review')


def readiness_errors(calibration: dict, evidence: dict) -> list[str]:
    errors=[]
    if calibration.get('constraints_satisfied') is not True:
        errors.append('Calibration has no accepted operating point; diagnostic fallback is not a formal threshold.')
    for key in REQUIRED_EVIDENCE:
        record=evidence.get(key)
        if not isinstance(record,dict) or record.get('passed') is not True or not record.get('evidence_path'):
            errors.append(f'Missing passed, traceable admission evidence: {key}')
    if evidence.get('scope_matches_requested_baseline') is not True:
        errors.append('Scene/task coverage is not aligned to the requested baseline.')
    if evidence.get('rgb_visibility_is_independent_of_instance_depth_proxy') is not True:
        errors.append('RGB visibility cannot be admitted using only instance pixels or a depth proxy.')
    return errors


def route_budget_error(route_length_m: float, remaining_seconds: float, maximum_speed_mps: float) -> str | None:
    values=(route_length_m,remaining_seconds,maximum_speed_mps)
    if not all(isfinite(v) for v in values) or min(values)<0:
        return 'Invalid route/time/speed evidence.'
    ideal_distance=remaining_seconds*maximum_speed_mps
    if route_length_m>ideal_distance:
        return f'Route {route_length_m:.2f}m exceeds ideal remaining travel budget {ideal_distance:.2f}m.'
    # Passing this necessary condition alone does NOT prove practical feasibility.
    return None
