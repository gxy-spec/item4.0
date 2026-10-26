"""New immutable runtime after the retained R8 surface-stop failure."""
import sys,json,yaml,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
sys.path.insert(0,str(ROOT))
from src.scenario.configuration import validate_schema,validate_online_control_config


def main():
    dest=OUT/'acceptance_r9_index.json'
    if dest.exists():raise RuntimeError('Already frozen')
    old=json.loads((OUT/'acceptance_release_index.json').read_text(encoding='utf-8'));bycase={r['case']:r for r in old['runs']}
    cases=['red_truck','yellow_car','white_bus','gray_truck','red_van','blue_van','black_car','gray_car','hard_negative','no_target'];runs=[]
    for i,case in enumerate(cases):
        item=bycase[case];c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
        c['experiment_id']=f'CI_E7_R9_ACC{i+1}';c['output']['root']=str(OUT/f'acc9/{case}')
        c['recovery_evaluation'].update(cohort='acceptance_r9',control_revision='Fresh local target RGB-D surface braking at4.5m; unchanged completion range3..8m. No weights/threshold/scene change from R8.')
        p=OUT/f'configs/acc9/{case}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
        errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
        if errors:raise RuntimeError((case,errors))
        p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8');runs.append({**item,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
    manifest=json.loads((OUT/'frozen_runtime_release/manifest.json').read_text(encoding='utf-8'))
    for name in manifest['files']:
        target=OUT/'frozen_runtime_r9'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
    manifest['files']={name:digest(ROOT/name) for name in manifest['files']}
    manifest['revision']='R9: RGB-D target surface braking; R8 failed firetruck case retained; model/calibration unchanged.'
    save(OUT/'frozen_runtime_r9/manifest.json',manifest)
    save(dest,dict(runs=runs,required_positive_cases=cases[:8],negative_cases=cases[8:],
        runtime_manifest_path=str(OUT/'frozen_runtime_r9/manifest.json'),scope=old['scope'],criterion=old['criterion'],
        previous_release_index=str(OUT/'acceptance_release_index.json')))
    save(OUT/'release_active.json',dict(acceptance_cohort='acceptance_r9',safety_cohort='safety_r9',runtime_manifest_path=str(OUT/'frozen_runtime_r9/manifest.json'),revision='R9'))
    print('FROZEN R9,10 cases',flush=True)


if __name__=='__main__':main()
