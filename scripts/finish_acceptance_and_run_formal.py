"""Current experiment continuation; not a scheduler or notification job."""
from pathlib import Path
import argparse,json,subprocess,sys,time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *


def read_progress(path):
    for attempt in range(5):
        try:return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        except json.JSONDecodeError:time.sleep(.1)
    raise RuntimeError('Unreadable progress: '+str(path))


def main():
    import psutil
    parser=argparse.ArgumentParser();parser.add_argument('--cohort',default='acceptance_r9');args=parser.parse_args()
    cohort=args.cohort;index=read_progress(OUT/f'{cohort}_index.json');last=-1
    while True:
        progress=read_progress(OUT/f'{cohort}_progress.json');complete=sum(bool(r.get('report_complete')) for r in progress.values())
        if complete!=last:print('ACCEPTANCE COMPLETE',complete,'/',len(index['runs']),flush=True);last=complete
        if complete==len(index['runs']):break
        active=False
        for process in psutil.process_iter(['name','cmdline']):
            try:
                command=' '.join(process.info['cmdline'] or [])
                if process.info['name'].lower()=='python.exe' and 'run_recovery_campaign.py run' in command and '--cohort '+cohort in command:active=True
            except (psutil.NoSuchProcess,psutil.AccessDenied):pass
        if not active:raise RuntimeError('Acceptance stopped before all10 cases; inspect retained failures')
        time.sleep(10)
    subprocess.run([sys.executable,'-u','scripts/analyze_recovery_campaign.py','--cohort',cohort],cwd=ROOT,check=True)
    results=read_progress(OUT/f'analysis/{cohort}_results.json');bycase={r['case']:r for r in results};errors=[]
    for case in index['required_positive_cases']:
        r=bycase[case]
        if r.get('runner_status')!='PASS' or not r.get('identity_task_success') or not r.get('sensor_safe_approach_audited'):errors.append(case+': identity/local/surface approach or infrastructure failed')
    for case in index['negative_cases']:
        r=bycase[case]
        if r.get('runner_status')!='PASS' or not r.get('negative_rejection_pass') or r.get('sent_messages',0):errors.append(case+': false message/commit, collision or infrastructure failed')
    for r in results:
        if r.get('collisions') or r.get('missing_instance_frames'):errors.append(r['case']+': collision/missing exact-frame evidence')
    active_release=read_progress(OUT/'release_active.json')
    safety=read_progress(OUT/f"{active_release.get('safety_cohort','safety_m5')}_progress.json")
    if len(safety)!=4 or any(r.get('runner_status')!='PASS' for r in safety.values()):errors.append('Formal-parameter safety validation failed')
    frozen=read_progress(Path(index['runtime_manifest_path']))
    for name,h in frozen['files'].items():
        if digest(ROOT/name)!=h:errors.append('Frozen source changed: '+name)
    for name,h in frozen['artifacts'].items():
        if digest(name)!=h:errors.append('Frozen weights/calibration changed: '+name)
    admission=dict(passed=not errors,errors=errors,positive_cases=index['required_positive_cases'],negative_cases=index['negative_cases'],
        acceptance_results_sha256=digest(OUT/f'analysis/{cohort}_results.json'),runtime_manifest_sha256=digest(index['runtime_manifest_path']),
        criterion='Correct configured target+local confirmation+fresh optical surface depth3..8m, speed<=0.3m/s for>=2s; zero collisions and negative-case messages/commits. Original centre3..7m separately reported.',
        frontend_diagnostic_sha256=digest(OUT/'frontend_release/summary.json'))
    save(OUT/'formal_admission.json',admission)
    if errors:raise RuntimeError('Formal admission refused: '+str(errors))
    if not (OUT/'final_independent_audit_v1/capture_protocol.json').exists():
        subprocess.run([sys.executable,'-u','scripts/capture_recovery_final_audit.py'],cwd=ROOT,check=True)
    if not (OUT/'final_attribute_audit_v1/manifest.jsonl').exists():
        subprocess.run([sys.executable,'-u','scripts/prepare_recovery_independent_benchmark.py','--capture','final_independent_audit_v1','--dest','final_attribute_audit_v1','--expected','512'],cwd=ROOT,check=True)
    if not (OUT/'final_frontend_audit/summary.json').exists():
        subprocess.run([sys.executable,'-u','scripts/audit_recovery_frontend_holdout.py','--capture','final_independent_audit_v1','--benchmark','final_attribute_audit_v1','--dest','final_frontend_audit'],cwd=ROOT,check=True)
    if not (OUT/'final_attribute_audit_results/summary.json').exists():
        subprocess.run([sys.executable,'-u','scripts/audit_recovery_final_attributes.py','--device','cpu'],cwd=ROOT,check=True)
    save(OUT/'post_freeze_audit_receipt.json',dict(model_unchanged=all(digest(p)==h for p,h in frozen['artifacts'].items()),
        capture_protocol_sha256=digest(OUT/'final_independent_audit_v1/capture_protocol.json'),frontend_summary_sha256=digest(OUT/'final_frontend_audit/summary.json'),
        rule='Final snapshots never used for tuning/selection. Remaining weaknesses reported; frozen full36 not filtered by task outcomes.'))
    if not (OUT/'formal_index.json').exists():subprocess.run([sys.executable,'-u','scripts/prepare_recovery_formal_storage.py'],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'-u','scripts/audit_recovery_design.py'],cwd=ROOT,check=True)
    print('FORMAL ADMISSION PASSED:36 upgraded +36 legacy controls',flush=True)
    from run_recovery_campaign import run
    run(OUT/'formal_index.json','formal',resume=True)
    # The artifact worker uses the frozen assessor once per completed run.
    # Wait for its completion instead of rewriting candidate-audit files while
    # its renderer is reading them, or evaluating the same raw images twice.
    for attempt in range(720):
        progress=read_progress(OUT/'formal_progress.json')
        missing=[k for k,r in progress.items() if not (Path(r['run_dir'])/'preview/recovery_artifacts_verified.json').exists()]
        if not missing:break
        if attempt%6==0:print('WAITING FINAL ARTIFACTS',len(missing),flush=True)
        time.sleep(10)
    else:raise RuntimeError('Artifacts did not complete within2h; acquisition preserved, inspect formal artifact log')
    results=[read_progress(Path(r['run_dir'])/'recovery_evaluation.json') for r in progress.values()]
    save(OUT/'analysis/formal_results.json',results)
    subprocess.run([sys.executable,'-u','scripts/package_recovery_release.py'],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'-u','scripts/plot_recovery_release.py'],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'-u','scripts/build_recovery_failure_appendix.py'],cwd=ROOT,check=True)
    subprocess.run([sys.executable,'-u','scripts/export_recovery_release_bundle.py'],cwd=ROOT,check=True)
    print('ACQUISITION AND AUTOMATIC EVALUATION COMPLETE; visual audit/report still required',flush=True)


if __name__=='__main__':main()
