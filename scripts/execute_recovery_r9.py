"""Sequential continuation of the current authorized experiment."""
import sys,json,yaml,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
from run_recovery_campaign import run


def main():
    if not (OUT/'acceptance_r9_index.json').exists():
        subprocess.run([sys.executable,'-u','scripts/prepare_recovery_acceptance_r9.py'],cwd=ROOT,check=True)
    if not (OUT/'safety_r9_index.json').exists():
        previous=json.loads((OUT/'safety_m5_index.json').read_text(encoding='utf-8'));runs=[]
        for i,item in enumerate(previous['runs']):
            c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
            c['experiment_id']=f'CI_E1_R9S{i+1}';c['output']['root']=str(OUT/f'safety_r9/{item["case"]}')
            p=OUT/f'configs/safety_r9/{item["case"]}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
            runs.append({**item,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
        save(OUT/'safety_r9_index.json',dict(runs=runs,runtime_manifest_path=str(OUT/'frozen_runtime_r9/manifest.json'),
            scope='Same final M5 safety conditions/actors/seeds/horizons and formal4m stop/8m warning; verify R9 common runtime. No target-confirmed braking in these non-target safety tests.'))
    run(OUT/'safety_r9_index.json','safety_r9')
    safety=json.loads((OUT/'safety_r9_progress.json').read_text(encoding='utf-8'))
    if len(safety)!=4 or any(r.get('runner_status')!='PASS' for r in safety.values()):raise RuntimeError('R9 safety validation failed')
    run(OUT/'acceptance_r9_index.json','acceptance_r9',acceptance_stop_on_failure=True)
    subprocess.run([sys.executable,'-u','scripts/finish_acceptance_and_run_formal.py','--cohort','acceptance_r9'],cwd=ROOT,check=True)


if __name__=='__main__':main()
