"""Preserve V3 scene protocol; validate proposal, tracking and safety repairs."""
import sys,json,yaml,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.scenario.configuration import validate_schema,validate_online_control_config


def main():
    dest=OUT/'acceptance_v4_index.json'
    if dest.exists():raise RuntimeError('Already frozen')
    old=json.loads((OUT/'acceptance_v3_final_index.json').read_text(encoding='utf-8'))
    bycase={r['case']:r for r in old['runs']};runs=[]
    for i,case in enumerate(['yellow_car','white_bus','red_truck','red_van','blue_van','gray_truck','hard_negative','no_target']):
        item=bycase[case];c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
        c['experiment_id']=f'CI_E7_R4_ACC{i+1}'
        c['output']['root']=str(OUT/f'acc4/{case}')
        c['dynamic']['local_target_search']['auxiliary_detector_path']='models/yolo26x.pt'
        c['dynamic']['ugv_safety']['hazard_detector'].update(model_path='models/yolo26x.pt',image_size=960)
        c['recovery_evaluation']['cohort']='acceptance_v4'
        p=OUT/f'configs/acc4/{case}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
        errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
        if errors:raise RuntimeError((case,errors))
        p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
        runs.append({**item,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
    old.update(runs=runs,changes='Same V3 scenes and seeds; complementary COCO proposals, confirmed-identity geometric continuation, goal-aware approach, current-motion pedestrian recovery; short paths')
    save(dest,old)
    files=['scripts/run_stage1_dynamic.py','src/perception/candidate_pipeline.py',
           'src/perception/vehicle_attributes.py','src/perception/depth_proposal_refinement.py',
           'src/perception/auxiliary_vehicle_proposals.py','src/perception/target_approach.py',
           'src/communication/persistent_target.py','src/navigation/ugv_oracle_planner.py',
           'src/perception/task_spec.py','src/perception/attributes.py','src/safety/rgbd_hazard_perception.py',
           'scripts/run_recovery_campaign.py','scripts/analyze_recovery_campaign.py']
    for name in files:
        target=OUT/'frozen_runtime_v4'/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
    save(OUT/'frozen_runtime_v4/manifest.json',{'files':{name:digest(ROOT/name) for name in files},
        'model_sha256':digest(OUT/'attribute_training/selected/best.pt'),
        'auxiliary_sha256':digest(ROOT/'models/yolo26x.pt')})
    print('FROZEN',len(runs),'acceptance cases')


if __name__=='__main__':main()
