"""Bounded recovery of infrastructure errors during this experiment only.

Never reruns completed task failures; never runs two simulator owners.
"""
import json,sys,subprocess,time
from pathlib import Path
import psutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def read(p,default):
    try:return json.loads(p.read_text(encoding='utf-8')) if p.exists() else default
    except json.JSONDecodeError:time.sleep(.2);return json.loads(p.read_text(encoding='utf-8'))


def driver_active():
    for p in psutil.process_iter(['name','cmdline']):
        try:
            if p.info['name'].lower()=='python.exe' and any(Path(a).name=='execute_recovery_r9.py' for a in p.info['cmdline'] or []):return True
        except (psutil.NoSuchProcess,psutil.AccessDenied):pass
    return False


def main():
    while driver_active():time.sleep(10)
    # Distinct cases may each encounter a sporadic server startup failure.
    # Bound the entire campaign as well as the stricter3 failures per config.
    for restart in range(24):
        formal=read(OUT/'formal_progress.json',{})
        if len(formal)==72 and all(r.get('report_complete') for r in formal.values()):
            print('ALL72 ACQUIRED; final evidence review/report remains',flush=True);return
        if any((OUT/f'{c}.stop').exists() for c in ['acceptance_r9','formal']):
            raise RuntimeError('Explicit between-episode stop or true acceptance failure; not automatically bypassed')
        frozen=read(OUT/'frozen_runtime_r9/manifest.json',{})
        if any(digest(ROOT/p)!=h for p,h in frozen['files'].items()) or any(digest(p)!=h for p,h in frozen['artifacts'].items()):
            raise RuntimeError('Frozen content mismatch; refusing to restart')
        failures=[]
        for cohort in ['safety_r9','acceptance_r9','formal']:
            failures += [(cohort,k,r) for k,r in read(OUT/f'{cohort}_progress.json',{}).items() if not r.get('report_complete')]
        if not failures:raise RuntimeError('Driver stopped outside a recorded infrastructure failure; inspect logs')
        histories=rows(OUT/'attempt_history.jsonl')
        for cohort,key,r in failures:
            count=sum(a.get('config')==r['config'] and not a.get('report_complete') for a in histories)
            if count>=3:raise RuntimeError('Same infrastructure failure already occurred3 times: '+cohort+'/'+key)
        log=OUT/f'bounded_resume_{restart+1}_{int(time.time())}.log'
        save(OUT/'bounded_resume_latest.json',dict(restart=restart+1,infrastructure_failures=[dict(cohort=c,case=k,previous_run=r.get('run_dir')) for c,k,r in failures],log=str(log),policy='Cold restart with identical frozen config; completed task failures never discarded'))
        print('BOUNDED INFRASTRUCTURE RESUME',restart+1,[(c,k) for c,k,r in failures],flush=True)
        with log.open('w',encoding='utf-8') as f:
            result=subprocess.run([sys.executable,'-u','scripts/execute_recovery_r9.py'],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        if result.returncode==0:return
    raise RuntimeError('Bounded infrastructure restart budget exhausted; all attempts retained')


if __name__=='__main__':main()
