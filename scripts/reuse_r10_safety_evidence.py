"""Reuse only same active safety code; unused business monitor diffs disclosed."""
from pathlib import Path
import json,sys,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest


def main():
    source=OUT/'safety_development_corridor2_progress.json'
    progress=json.loads(source.read_text(encoding='utf-8'));records=[];errors=[]
    if len(progress)!=4:raise RuntimeError('Four safety regressions not complete yet')
    for key,item in progress.items():
        run=Path(item['run_dir']);cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
        monitor=bool(cfg['dynamic'].get('persistent_target_communication',{}).get('sensor_business_monitor_enabled',False))
        hashes=json.loads((run/'config_snapshot/runtime_source_manifest.json').read_text(encoding='utf-8'))
        changed=[n for n,h in hashes.items() if digest(ROOT/n)!=h]
        permitted=[n for n in changed if n=='src/perception/body_approach.py' and not monitor]
        unsafe=[n for n in changed if n not in permitted]
        if item.get('runner_status')!='PASS' or unsafe:errors.append(dict(case=key,unsafe_diffs=unsafe,runner=item.get('runner_status')))
        records.append(dict(case=key,run_dir=str(run),active_safety_source_identical=not unsafe,
            unused_monitor_changes=permitted,business_monitor_enabled=monitor,runner=item.get('runner_status')))
    receipt=dict(passed=not errors,errors=errors,records=records,rule='Reuse four passing same active safety-controller tests; disclose unused business-monitor-only source changes')
    save(OUT/'safety_reuse_receipt.json',receipt)
    if errors:raise RuntimeError(receipt)
    save(OUT/'safety_development_validation1_progress.json',progress)
    print('REUSED four exact active-safety regressions; unused monitor difference disclosed',flush=True)


if __name__=='__main__':main()
