"""Bounded development cohort; preserves outcomes and rejects unsafe continuation.

This executes approved diagnostic/safety/online tests only. Formal acquisition
requires an independently written freeze/admission receipt, not runner PASS.
"""
from pathlib import Path
import sys,json,subprocess
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest
from run_recovery_campaign import run_one
from evaluate_r10 import assess


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--cohort',choices=['diagnostics','acceptance','safety'],required=True);p.add_argument('--case');p.add_argument('--revision',default='');args=p.parse_args()
    runs=json.loads((OUT/f'{args.cohort}_index.json').read_text(encoding='utf-8'))['runs']
    if args.case:runs=[r for r in runs if r['case']==args.case]
    tag=args.cohort+'_development'+('_'+args.revision if args.revision else '')
    progress_path=OUT/f'{tag}_progress.json'
    progress=json.loads(progress_path.read_text(encoding='utf-8')) if progress_path.exists() else {}
    for i,item in enumerate(runs):
        key=Path(item['config']).stem
        if progress.get(key,{}).get('report_complete'):continue
        manifest_path=json.loads((OUT/f'{args.cohort}_index.json').read_text(encoding='utf-8')).get('runtime_manifest_path')
        if manifest_path:
            manifest=json.loads(Path(manifest_path).read_text(encoding='utf-8'))
            for name,h in manifest['files'].items():
                if digest(ROOT/name)!=h:raise RuntimeError('Validation source changed after freeze: '+name)
            for name,h in manifest.get('artifacts',{}).items():
                if digest(name)!=h:raise RuntimeError('Validation model changed after freeze: '+name)
        # Every new revision must re-run these real retained failures and all
        # monitor/geometry/navigation safety regressions before acquisition.
        testlog=OUT/f'logs/regression_before_{args.cohort}_{key}.log';testlog.parent.mkdir(exist_ok=True,parents=True)
        with testlog.open('w',encoding='utf-8') as log:
            subprocess.run([sys.executable,'-B','-m','pytest','-q'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        print('START',args.cohort,key,flush=True)
        result=None
        for attempt in range(3):
            result=run_one({**item,'development_revision':args.revision or 'initial'},tag)
            if result.get('report_complete'):break
            print('INFRASTRUCTURE INCOMPLETE, retained attempt',attempt+1,flush=True)
        progress[key]=result;save(progress_path,progress)
        if not result.get('report_complete'):raise RuntimeError('Three infrastructure failures; inspect before any continuation')
        if args.cohort!='safety':
            evaluation=assess(result)
            print('BUSINESS',key,{k:evaluation.get(k) for k in ['S1_correct_discovery','S2_correct_ground_confirmation','S3_found_and_safely_approached','collisions','reasons']},flush=True)
            if evaluation.get('collisions'):raise RuntimeError('Collision regression requires diagnosis')
        elif result.get('runner_status')!='PASS':raise RuntimeError('Safety acceptance failed; retained output')
    print('COHORT COMPLETE, all outcomes retained:',progress_path,flush=True)


if __name__=='__main__':main()
