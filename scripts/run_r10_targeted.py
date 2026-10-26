"""Approved targeted cohorts only; stop on failed online/safety acceptance.

The frozen runtime remains separate. Completed task failures are retained, not
restarted or removed. Infrastructure retries are handled by the frozen wrapper.
"""
from pathlib import Path
import json,sys,subprocess,argparse
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest


def main():
    p=argparse.ArgumentParser();p.add_argument('--cohort',choices=['diagnostics','acceptance'],required=True);args=p.parse_args()
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))
    if scope['full_formal_allowed']:raise RuntimeError('This driver is only for the reduced scope')
    index=json.loads((OUT/f'{args.cohort}_index.json').read_text(encoding='utf-8'))
    cases=list(dict.fromkeys(r['case'] for r in index['runs']))
    revision=scope['active_validation_revision']
    if revision not in ['validation1','validation2']:
        receipt=json.loads((OUT/f'{revision}_safety_admission.json').read_text(encoding='utf-8'))
        if not receipt['passed'] or receipt['runtime_manifest_sha256']!=digest(scope['runtime_manifest_path']):
            raise RuntimeError('Unresolved safety or mismatching frozen runtime')
    if args.cohort=='acceptance':
        first='gray_truck' if revision!='validation1' else 'red_van'
        cases=[first]+[c for c in cases if c!=first]
    for case in cases:
        subprocess.run([sys.executable,'-B','-u','-X','utf8',str(ROOT/'scripts/run_r10_development.py'),
            '--cohort',args.cohort,'--case',case,'--revision',revision],cwd=ROOT,check=True)
        progress=json.loads((OUT/f'{args.cohort}_development_{revision}_progress.json').read_text(encoding='utf-8'))
        items=[r for r in progress.values() if r['case']==case]
        for item in items:
            r=json.loads((Path(item['run_dir'])/'r10_evaluation.json').read_text(encoding='utf-8'))
            if r.get('collisions'):raise RuntimeError('Collision regression, retained: '+str(item['run_dir']))
            if args.cohort=='acceptance':
                expected=r['expected_target_present']
                passed=bool(r.get('S3_found_and_safely_approached')) if expected else bool(r.get('negative_rejection_pass') and not r.get('sent_messages'))
                receipt=dict(case=case,passed=passed,result=str(Path(item['run_dir'])/'r10_evaluation.json'),
                    r10_eval_sha256=digest(Path(item['run_dir'])/'r10_evaluation.json'))
                save(OUT/f'online_admission_{revision}/{case}.json',receipt)
                if not passed:raise RuntimeError('Online business acceptance requires diagnosis: '+case)
        print('TARGETED VERIFIED',args.cohort,case,flush=True)
    print('TARGETED COHORT FINISHED',args.cohort,flush=True)


if __name__=='__main__':main()
