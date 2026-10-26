"""Compact status; no full log dumps, process termination or simulator calls."""
from pathlib import Path
import json,sys,psutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT


def read(path):
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,json.JSONDecodeError):return {}


def main():
    status={'active':[],'cohorts':{},'scope':read(OUT/'execution_scope.json')}
    for p in psutil.process_iter(['pid','name','cmdline']):
        try:
            args=p.info['cmdline'] or [];joined=' '.join(args)
            if p.info['name'].lower()=='python.exe' and 'run_stage1_dynamic.py' in joined:
                index=args.index('--config');cfg=Path(args[index+1]);status['active'].append(dict(pid=p.pid,config=str(cfg)))
                import yaml
                c=yaml.safe_load(cfg.read_text(encoding='utf-8'));root=Path(c['output']['root'])
                # Offline audits can touch an older run directory after a new
                # acquisition starts. Run IDs end in immutable UTC timestamps.
                runs=sorted(root.glob('CI_*'),key=lambda p:p.name.rsplit('_',1)[-1])
                if runs:
                    v=read(runs[-1]/'logs/live_progress.json');b=v.get('business',{})
                    status['active'][-1].update(run_dir=str(runs[-1]),sim_elapsed_s=v.get('sim_elapsed_s'),duration_s=v.get('duration_s'),
                        confirmed=b.get('local_confirmation'),online_completion_claim=b.get('sensor_safe_approach'),
                        last_estimated_gap_m=(b.get('observations') or [{}])[-1].get('estimated_body_clearance_m'))
        except (psutil.NoSuchProcess,psutil.AccessDenied,ValueError,OSError):pass
    for cohort in ['diagnostics','acceptance','safety']:
        revision=status['scope'].get('active_validation_revision')
        tag=cohort+'_development'+('_'+revision if revision else '')
        progress=read(OUT/f'{tag}_progress.json');results=[]
        for key,item in progress.items():
            r=read(Path(item['run_dir'])/'r10_evaluation.json') if item.get('run_dir') else {}
            results.append(dict(case=key,complete=item.get('report_complete'),runner=item.get('runner_status'),
                S3=r.get('S3_found_and_safely_approached'),collisions=r.get('collisions'),
                reasons=r.get('reasons'),run_dir=item.get('run_dir')))
        status['cohorts'][cohort]=results
    print(json.dumps(status,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
