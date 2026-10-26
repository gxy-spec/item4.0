from pathlib import Path
import json,sys,shutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest


def main():
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--revision',type=int,default=1);args=p.parse_args()
    if args.revision<1:raise ValueError('Positive immutable revision required')
    dest=OUT/f'frozen_validation_v{args.revision}'
    if dest.exists():raise RuntimeError('Do not overwrite frozen validation source')
    old=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')
    baseline=json.loads((old/'frozen_runtime_formal_r9/manifest.json').read_text(encoding='utf-8'))
    names=set(baseline['files'])
    names.update(str(p.relative_to(ROOT)).replace('\\','/') for p in (ROOT/'src').rglob('*.py'))
    names.update(['scripts/run_r10_development.py','scripts/evaluate_r10.py','scripts/build_recovery_replay.py',
                  'scripts/build_synchronized_multiview_replay.py','scripts/run_recovery_campaign.py'])
    for name in names:
        p=ROOT/name;target=dest/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,target)
    manifest=dict(revision=f'R10 targeted validation v{args.revision}',files={n:digest(ROOT/n) for n in sorted(names)},
        artifacts=baseline['artifacts'],scope='22 targeted closed-loop tests; full72 explicitly deferred',
        stopping_rule='drivable-footprint stopped confirmation; annotate junction; not road-parking legality',
        exploratory_planner=('Budgeted received-point observation region is called only when the original route exceeds remaining task time; local safety and independent S3 remain mandatory'
            if args.revision>=5 else 'approach_region.py is offline diagnostic only; NOT called by online runner'))
    save(dest/'manifest.json',manifest)
    for cohort in ['diagnostics','acceptance','safety']:
        p=OUT/f'{cohort}_index.json';index=json.loads(p.read_text(encoding='utf-8'))
        index['runtime_manifest_path']=str(dest/'manifest.json');save(p,index)
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))
    scope.update(active_validation_revision=f'validation{args.revision}',runtime_manifest_path=str(dest/'manifest.json'));save(OUT/'execution_scope.json',scope)
    print('FROZEN',len(names),'source files; models unchanged; full72 disabled',flush=True)


if __name__=='__main__':main()
