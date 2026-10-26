"""Compact read-only status of this long-running experiment."""
import sys,json
from pathlib import Path
from datetime import datetime
import psutil,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT


def read(p):
    try:return json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}
    except json.JSONDecodeError:return {}


def main():
    print('LOCAL TIME',datetime.now().isoformat(timespec='seconds'))
    for cohort,n in [('safety_r9',4),('acceptance_r9',10),('formal',72)]:
        p=read(OUT/f'{cohort}_progress.json');done=[r for r in p.values() if r.get('report_complete')]
        receipts=sum((Path(r['run_dir'])/'preview/recovery_artifacts_verified.json').exists() for r in done if r.get('run_dir'))
        text=f'completed={len(done)}/{n} runner_PASS={sum(r.get("runner_status")=="PASS" for r in done)} artifacts={receipts}'
        if cohort=='acceptance_r9':text+=f' online_task_gates={sum(r.get("online_task_gate_passed",False) for r in done)}/{len(done)}'
        elif cohort=='formal':
            audited=[(r,read(Path(r['run_dir'])/'recovery_evaluation.json')) for r in done if r.get('run_dir')]
            responding=[e for r,e in audited if e and r.get('group') in ['B','C']]
            text+=f' audited={sum(bool(e) for r,e in audited)} B/C_identity={sum(bool(e.get("identity_task_success")) for e in responding)}/{len(responding)} B/C_surface={sum(bool(e.get("sensor_safe_approach_audited")) for e in responding)}/{len(responding)}'
        print(cohort,text)
        for k,r in p.items():
            if not r.get('report_complete'):print(' INCOMPLETE',k,r.get('runner_status'),r.get('error',''))
    for p in psutil.process_iter(['name','cmdline']):
        try:
            cmd=p.info['cmdline'] or []
            if p.info['name'].lower()!='python.exe' or not any(Path(a).name=='run_stage1_dynamic.py' for a in cmd):continue
            path=Path(cmd[cmd.index('--config')+1]);cfg=yaml.safe_load(path.read_text(encoding='utf-8'));root=Path(cmd[cmd.index('--output-root')+1]) if '--output-root' in cmd else Path(cfg['output']['root'])
            files=list(root.glob('*/logs/live_progress.json'));latest=max(files,key=lambda q:q.stat().st_mtime) if files else None;live=read(latest) if latest else {}
            print('ACTIVE',path.stem,'sim_s',live.get('sim_elapsed_s','warming up'),'/',cfg['simulation']['duration_seconds'],
                'local',live.get('business',{}).get('local_confirmation'),'surface_hold',live.get('business',{}).get('sensor_safe_approach'))
        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
    print('FREE_GB', {d:round(psutil.disk_usage(d+':/').free/1e9,1) for d in ['C','D','E']})


if __name__=='__main__':main()
