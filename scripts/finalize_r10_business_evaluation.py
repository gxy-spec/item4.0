"""Consistent report schema from frozen R10 raw evaluations; no rescoring.

Preserves raw R10 evaluation. Old inherited centre-distance verdicts and
non-applicable positive-task reasons are archived, not called the new verdict.
"""
from pathlib import Path
import sys,json,argparse
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest


def normalize(raw,item):
    result=dict(raw)
    present=raw.get('expected_target_present')
    control=item.get('group')=='A'
    collisions=int(raw.get('collisions',0) or 0)
    passed=bool(raw.get('S3_found_and_safely_approached')) and not collisions
    negative=bool(raw.get('negative_rejection_pass') and not raw.get('sent_messages') and not collisions)
    legacy_fields=['semantic_verdict','reasons','local_confirmation_observed','local_confirmation_correct','arrival','maximum_hold_seconds','final_scope','localization_xy_m']
    result['inherited_fields_archive']={k:raw.get(k) for k in legacy_fields}
    for k in legacy_fields:
        result.pop(k,None)
    if control:
        result.update(semantic_verdict='CONTROL_NO_RESPONSE_REQUIRED',task_success=None,task_success_applicable=False,
            reasons=['collision'] if collisions else [],business_verdict='巡检对照；不要求响应目标')
    elif present is False:
        result.update(semantic_verdict='PASS' if negative else 'FAIL',task_success=None,task_success_applicable=False,
            reasons=[] if negative else ['negative_case_false_response_or_collision'],business_verdict='负例拒绝通过' if negative else '负例拒绝未通过')
    else:
        result.update(semantic_verdict='PASS' if passed else 'FAIL',task_success=passed,task_success_applicable=True,
            reasons=raw.get('reasons',[]),business_verdict='找到并安全接近通过' if passed else '找到并安全接近未通过')
    result['report_schema']='R10-business-report-v2'
    result['metric_scope']='S1/S2/S3 separate; reference3..7m and inherited surface/centre fields are historical diagnostics only'
    result['position_metric_note']='Inherited localization_xy_m measures observed point to actor reference. A visible surface point and an actor reference are different anchors; do not treat that offset as body-gap error or a certified same-anchor localisation error.'
    result['stopping_rule']='temporary stopped confirmation on drivable footprint; junction annotated, not real-road parking legality'
    result['final_scope']='Targeted repair verification only; full36 paired/model-superiority comparison deferred. Truth used for independent post-run audit.'
    return result


def finalize(item):
    run=Path(item['run_dir']);source=run/'r10_evaluation.json'
    if not source.exists():return None
    raw=json.loads(source.read_text(encoding='utf-8'));result=normalize(raw,item)
    result['source_evaluation_sha256']=digest(source)
    result['report_normalizer_sha256']=digest(Path(__file__))
    save(run/'r10_final_evaluation.json',result)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('--cohort',choices=['acceptance','diagnostics','all'],default='all');args=p.parse_args()
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));revision=scope['active_validation_revision'];count=0
    cohorts=['acceptance','diagnostics'] if args.cohort=='all' else [args.cohort]
    for cohort in cohorts:
        path=OUT/f'{cohort}_development_{revision}_progress.json'
        if not path.exists():continue
        for item in json.loads(path.read_text(encoding='utf-8')).values():
            if item.get('report_complete') and finalize(item) is not None:count+=1
    print('REPORT SCHEMA NORMALIZED; raw evaluations preserved:',count)


if __name__=='__main__':main()
