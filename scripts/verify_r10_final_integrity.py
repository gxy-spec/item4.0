"""Verify frozen sources/models, run identities, original config and R9 preservation."""
from pathlib import Path
import json,sys,collections
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest,rows


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));rev=scope['active_validation_revision']
    manifest=json.loads(Path(scope['runtime_manifest_path']).read_text(encoding='utf-8'));errors=[];checks=[];cache={}
    def check(path,expected,label):
        p=Path(path);key=str(p.resolve())
        if key not in cache:cache[key]=digest(p) if p.is_file() else None
        ok=cache[key]==expected;checks.append(dict(path=str(p),kind=label,passed=ok))
        if not ok:errors.append(checks[-1])
    for name,h in manifest['files'].items():check(ROOT/name,h,'active_R10_frozen_source')
    for name,h in manifest['artifacts'].items():check(name,h,'frozen_model_or_release')
    old=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')
    r9=json.loads((old/'frozen_runtime_formal_r9/manifest.json').read_text(encoding='utf-8'))
    for name,h in r9['files'].items():check(Path('E:/Research/CityInspection_GOC')/name,h,'original_R9_source')
    for name,h in r9['artifacts'].items():check(name,h,'original_R9_artifact')
    oldaudit=json.loads((OUT/'analysis/R9_72_retraceability_summary.json').read_text(encoding='utf-8'))
    check(old/'formal_progress.json',oldaudit['source_sha256'],'R9_formal_result_index_unchanged')
    records=[]
    for cohort in ['safety','acceptance','diagnostics']:
        p=OUT/f'{cohort}_development_{rev}_progress.json'
        if not p.exists():continue
        for key,item in json.loads(p.read_text(encoding='utf-8')).items():
            if not item.get('report_complete'):continue
            run=Path(item['run_dir']);check(item['config'],item['config_sha256'],'run_config_unchanged')
            source_rev=item.get('development_revision',rev);n=source_rev.removeprefix('validation')
            actual=json.loads((run/'config_snapshot/runtime_source_manifest.json').read_text(encoding='utf-8'))
            frozen=json.loads((OUT/f'frozen_validation_v{n}/manifest.json').read_text(encoding='utf-8'))
            for name,h in actual.items():
                if frozen['files'].get(name)!=h:errors.append(dict(run=str(run),file=name,error='Run source differs from its declared acquisition freeze'))
                check(run/'config_snapshot/runtime_sources'/name,h,'saved_run_source_copy')
            record=dict(cohort=cohort,case=item['case'],variant=item.get('variant'),group=item.get('group'),
                run_dir=str(run),acquisition_revision=source_rev,reused=bool(item.get('evidence_reuse')),
                body_source_files=len(actual),runner_status=item.get('runner_status'))
            records.append(record)
    if len(set(r['run_dir'] for r in records))!=len(records):errors.append(dict(error='A run was counted twice'))
    attempts=rows(OUT/'attempt_history.jsonl')
    formal_attempts=[a for a in attempts if 'formal' in Path(a.get('log','')).parent.name.lower()]
    if formal_attempts or scope.get('full_formal_allowed') is not False:errors.append(dict(error='Reduced scope guard violation'))
    result=dict(passed=not errors,active_revision=rev,completed_unique_runs=len(records),expected_targeted_runs=22,
        counts=dict(collections.Counter(c['kind'] for c in checks)),errors=errors,checks=checks,records=records,
        actual_formal_attempts=len(formal_attempts),scope='Checks listed code/models/configs/result index and saved run-source copies; does not claim a full bitwise hash of all historical sensor data.')
    save(OUT/'analysis/final_integrity_verification.json',result)
    print(json.dumps({k:result[k] for k in ['passed','active_revision','completed_unique_runs','counts','errors','actual_formal_attempts']},ensure_ascii=False))


if __name__=='__main__':main()
