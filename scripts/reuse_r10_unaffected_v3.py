"""Reuse v3 only when the newly changed failure-recovery branch was never entered."""
from pathlib import Path
import json,sys,csv,importlib.util
sys.path.insert(0,str(Path(__file__).resolve().parent))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from recovery_stage import OUT,ROOT,save,digest
from safety.recovery_watchdog import ProgressWatchdog


def main():
    previous=OUT/'frozen_validation_v3';manifest=json.loads((previous/'manifest.json').read_text(encoding='utf-8'))
    changed=[p for p,h in manifest['files'].items() if digest(ROOT/p)!=h]
    permitted=['scripts/run_stage1_dynamic.py','src/safety/recovery_watchdog.py']
    if sorted(changed)!=sorted(permitted):raise RuntimeError('Unexpected source differences: '+str(changed))
    old_main=(previous/'scripts/run_stage1_dynamic.py').read_text(encoding='utf-8')
    new_main=(ROOT/'scripts/run_stage1_dynamic.py').read_text(encoding='utf-8')
    old_call="str(ugv_safety.get('ugv_safety_mode','')),bool(sensor_approach_monitor and sensor_approach_monitor.completed))"
    new_call="str(ugv_safety.get('ugv_safety_mode','')),bool(sensor_approach_monitor and sensor_approach_monitor.completed),\n                safe_to_resume=bool(sensor_observation and sensor_observation.get('state')=='CLEAR'\n                    and float(sensor_observation.get('sensor_age_s',float('inf')))<=.3\n                    and not latest_hazard_decision.get('forced_stop_reason')\n                    and not latest_hazard_decision.get('no_safe_overtake_gap')))"
    old_log="s0.json_dump(run_dirs['logs']/'r10_recovery_watchdog.json',dict(failed=repair_watchdog.failed,events=repair_watchdog.events))"
    new_log="s0.json_dump(run_dirs['logs']/'r10_recovery_watchdog.json',dict(failed=repair_watchdog.failed,\n            ever_failed=repair_watchdog.ever_failed,events=repair_watchdog.events))"
    restored=new_main
    for before,after in [(new_call,old_call),("if recovery_action in {'reassess','resumed'}:","if recovery_action=='reassess':"),(new_log,old_log)]:
        if restored.count(before)!=1:raise RuntimeError('Main changed outside the three identified edits')
        restored=restored.replace(before,after,1)
    if restored!=old_main:raise RuntimeError('Additional controller differences exist')
    spec=importlib.util.spec_from_file_location('old_r10_watchdog',previous/'src/safety/recovery_watchdog.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    receipt=[]
    for cohort in ['safety','acceptance']:
        source=OUT/f'{cohort}_development_validation3_progress.json';progress=json.loads(source.read_text(encoding='utf-8'));reused={}
        for key,item in progress.items():
            run=Path(item['run_dir']);log=json.loads((run/'logs/r10_recovery_watchdog.json').read_text(encoding='utf-8'))
            ever_failed=bool(log['failed'] or any(e['event']=='bounded_blockage_failure' for e in log['events']))
            if ever_failed:
                receipt.append(dict(case=key,reused=False,reason='New recovery branch can change this formerly failed blockage',run_dir=str(run)));continue
            own={int(r['frame']):r for r in csv.DictReader((run/'trajectories/ugv_trajectory.csv').open(encoding='utf-8-sig'))}
            safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
            old=module.ProgressWatchdog();new=ProgressWatchdog();origin=float(safety[0]['timestamp']);equal=True
            for row in safety:
                state=own[int(row['frame'])];xy=[float(state['x']),float(state['y'])];t=float(row['timestamp'])-origin;mode=row['ugv_safety_mode']
                completed=mode.startswith('sensor_confirmed_hold_')
                a=old.update(t,xy,mode,completed);b=new.update(t,xy,mode,completed,safe_to_resume=True)
                equal=equal and a==b and old.failed==new.failed and not new.ever_failed
            if not equal:raise RuntimeError('Watchdog action path differs on '+key)
            copy={**item,'evidence_reuse':dict(acquisition_revision='validation3',validated_revision='validation4',
                criterion='No failed-blockage state was entered; changed branch inactive; old/new watchdog tick outputs match',
                receipt=str(OUT/'validation4_unaffected_reuse_receipt.json'))}
            reused[key]=copy
            receipt.append(dict(case=key,reused=True,watchdog_tick_output_identical=True,ticks=len(safety),
                changed_branch_never_entered=True,run_dir=str(run),watchdog_log_sha256=digest(run/'logs/r10_recovery_watchdog.json')))
        dest=OUT/f'{cohort}_development_validation4_progress.json'
        if dest.exists():raise RuntimeError('Do not overwrite validation4 progress')
        save(dest,reused)
    save(OUT/'validation4_unaffected_reuse_receipt.json',dict(records=receipt,permitted_source_differences=permitted,
        main_text_identical_after_reverting_exact_three_edits=True,
        scope='Targeted repair evidence reuse only; not an assertion that these runs acquired v4 source. Failed blockage cases must be rerun.'))
    print('REUSED',sum(r['reused'] for r in receipt),'unaffected runs; EXCLUDED',sum(not r['reused'] for r in receipt),'affected run')


if __name__=='__main__':main()
