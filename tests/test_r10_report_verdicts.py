import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('r10_report_normalizer',Path(__file__).resolve().parents[1]/'scripts/finalize_r10_business_evaluation.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_new_body_success_does_not_inherit_old_reference_failure_label():
    raw=dict(expected_target_present=True,S3_found_and_safely_approached=True,collisions=0,semantic_verdict='FAIL',reasons=[])
    r=module.normalize(raw,dict(group='C'))
    assert r['semantic_verdict']=='PASS' and r['task_success'] is True
    assert r['inherited_fields_archive']['semantic_verdict']=='FAIL' and raw['semantic_verdict']=='FAIL'


def test_correct_rejection_and_nonresponsive_control_do_not_fail_positive_goal():
    raw=dict(expected_target_present=False,S3_found_and_safely_approached=False,negative_rejection_pass=True,sent_messages=0,
             collisions=0,reasons=['no_verified_near_ground_confirmation'])
    r=module.normalize(raw,dict(group='C'));assert r['semantic_verdict']=='PASS' and not r['reasons'] and r['task_success'] is None
    control={**raw,'expected_target_present':True}
    r=module.normalize(control,dict(group='A'))
    assert r['semantic_verdict']=='CONTROL_NO_RESPONSE_REQUIRED' and not r['reasons'] and r['task_success'] is None


def test_collision_cannot_be_hidden_by_positive_or_negative_flags():
    raw=dict(expected_target_present=True,S3_found_and_safely_approached=True,collisions=1,reasons=['collision'])
    assert module.normalize(raw,dict(group='C'))['semantic_verdict']=='FAIL'
