"""One compact read-only snapshot for the active targeted validation."""
from pathlib import Path
import json,sys,os,psutil,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save


def read(p):
    try:return json.loads(Path(p).read_text(encoding='utf-8'))
    except (OSError,json.JSONDecodeError):return {}


def main():
    scope=read(OUT/'execution_scope.json');rev=scope.get('active_validation_revision','validation1')
    status=dict(revision=rev,completed=0,planned=22,safety_pass=0,positive_pass=0,negative_pass=0,
                diagnostics_complete=0,queue_alive=False,active=None,failures=[],awaiting_audit=[],
                diagnostic_task_failures=[],paired_control_audit_pass=None)
    for cohort in ['safety','acceptance','diagnostics']:
        progress=read(OUT/f'{cohort}_development_{rev}_progress.json')
        for key,item in progress.items():
            if not item.get('report_complete'):continue
            if cohort=='safety':
                final=read(Path(item['run_dir'])/'r10_safety_final_evaluation.json')
                passed=bool(final['passed']) if final else item.get('runner_status')=='PASS'
                status['completed']+=1;status['safety_pass']+=passed
                if not passed:status['failures'].append(dict(case=key,reasons=item.get('failed_checks'),run_dir=item['run_dir']))
                continue
            r=read(Path(item['run_dir'])/'r10_evaluation.json')
            if not r:status['awaiting_audit'].append(key);continue
            status['completed']+=1
            if cohort=='diagnostics':
                status['diagnostics_complete']+=1
                if item.get('group')!='A' and r.get('expected_target_present') and not r.get('S3_found_and_safely_approached'):
                    status['diagnostic_task_failures'].append(dict(case=key,reasons=r.get('reasons'),run_dir=item['run_dir']))
                continue
            if r.get('expected_target_present'):
                passed=bool(r.get('S3_found_and_safely_approached'));status['positive_pass']+=passed
            else:
                passed=bool(r.get('negative_rejection_pass') and not r.get('sent_messages'));status['negative_pass']+=passed
            if not passed:status['failures'].append(dict(case=key,reasons=r.get('reasons'),run_dir=item['run_dir']))
    for p in psutil.process_iter(['pid','name','cmdline']):
        try:
            if p.pid==os.getpid() or p.info['name'].lower()!='python.exe':continue
            args=p.info['cmdline'] or [];names=[Path(a).name for a in args if a.endswith('.py')]
            if any(a in names for a in ['run_r10_targeted.py','run_r10_development.py']):status['queue_alive']=True
            if 'run_stage1_dynamic.py' not in names:continue
            cfg=Path(args[args.index('--config')+1]);c=yaml.safe_load(cfg.read_text(encoding='utf-8'))
            root=Path(args[args.index('--output-root')+1]) if '--output-root' in args else Path(c['output']['root'])
            runs=sorted(root.glob('CI_*'),key=lambda p:p.name.rsplit('_',1)[-1])
            v=read(runs[-1]/'logs/live_progress.json') if runs else {};b=v.get('business',{})
            if runs and v.get('sim_elapsed_s') is None:
                actorpath=runs[-1]/'actors/actor_states.jsonl'
                if actorpath.exists():
                    with actorpath.open('rb') as f:
                        first=json.loads(f.readline());f.seek(max(0,actorpath.stat().st_size-24576))
                        tail=f.read().decode('utf-8',errors='ignore').splitlines()
                    for line in reversed(tail):
                        try:last=json.loads(line);v['sim_elapsed_s']=last['timestamp']-first['timestamp'];break
                        except (json.JSONDecodeError,KeyError):pass
            status['active']=dict(case=cfg.stem,sim_elapsed_s=v.get('sim_elapsed_s'),duration_s=c['simulation']['duration_seconds'],
                online_claim=b.get('sensor_safe_approach'),run_dir=str(runs[-1]) if runs else None)
        except (psutil.NoSuchProcess,psutil.AccessDenied,ValueError,OSError):pass
    pair=read(OUT/'analysis/N3_A_pair_audit.json')
    status['paired_control_audit_pass']=pair.get('passed')
    status['all_task_requirements_passed']=bool(status['completed']==status['planned'] and not status['failures']
        and not status['diagnostic_task_failures'] and status['paired_control_audit_pass'] is True)
    save(OUT/'STATUS_CURRENT.json',status);print(json.dumps(status,ensure_ascii=False))


if __name__=='__main__':main()
