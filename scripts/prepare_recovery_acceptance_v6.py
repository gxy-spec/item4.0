"""Freeze final multi-type, six-color and negative online acceptance before runs."""
import sys,json,yaml,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from src.scenario.configuration import validate_schema,validate_online_control_config


def main():
    if (OUT/'acceptance_v6_index.json').exists():raise RuntimeError('Already frozen')
    old=json.loads((OUT/'acceptance_v3_final_index.json').read_text(encoding='utf-8'));bycase={r['case']:r for r in old['runs']}
    cal=json.loads((OUT/'calibration/deployment_v6.json').read_text(encoding='utf-8'));runs=[]
    cases=['yellow_car','white_bus','gray_truck','red_truck','red_van','blue_van','black_car','gray_car','hard_negative','no_target']
    for i,case in enumerate(cases):
        parent='yellow_car' if case in ['black_car','gray_car'] else case
        item=bycase[parent];c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
        c['experiment_id']=f'CI_E7_R6_ACC{i+1}'
        if case in ['black_car','gray_car']:
            color=case.split('_')[0];blueprint='vehicle.audi.tt' if color=='black' else 'vehicle.tesla.model3'
            c['random_seed']=44201+i;c['task'].update(task_id=f'find_{color}_car',instruction=f'Find a {color} car, confirm it and approach safely.',parsed_goal={'target':{'category':'car','attributes':{'color':color}},'action':'approach'})
            c['actors']['target_vehicle'].update(preferred_blueprints=[blueprint],resolved_blueprint=blueprint,color_rgb='8,8,8' if color=='black' else '110,110,110')
            c['dynamic']['same_category_wrong_color_blueprint']=blueprint
            c['dynamic']['same_category_wrong_color_rgb']='255,0,0';c['dynamic']['comparison_distractor_color_rgb']='255,0,0'
            c['online_acceptance'].update(task_category='car',task_color=color)
            c['recovery_evaluation'].update(task_category='car',task_color=color,acceptance_case=case)
        c['output']['root']=str(OUT/f'acc6/{case}')
        c['dynamic']['local_target_search'].update(attribute_model_path=cal['weights'],
            attribute_type_threshold=cal['type_threshold'],attribute_color_threshold=cal['color_threshold'],
            target_color_ratio_threshold=cal['color_threshold'],auxiliary_detector_path='models/yolo26x.pt')
        c['dynamic']['ugv_safety']['hazard_detector'].update(model_path='models/yolo26x.pt',image_size=960)
        c['recovery_evaluation'].update(cohort='acceptance_v6',final_model_calibration_sha256=digest(OUT/'calibration/deployment_v6.json'))
        p=OUT/f'configs/acc6/{case}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
        errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
        if errors:raise RuntimeError((case,errors))
        p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
        runs.append({**item,'case':case,'seed':c['random_seed'],'exploratory_blueprint_challenge':False,
            'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
    save(OUT/'acceptance_v6_index.json',dict(runs=runs,required_positive_cases=cases[:8],negative_cases=cases[8:],
        scope='Same E1 geometry; 8 positive cases spanning4fine types,6paint colors and6blueprints;2negative controls. All8 positive cases required, including previously difficult gray HGV. Developmental same-map regression, not blind cross-city generalization.',
        criterion='Correct audited identity and local confirmation, fresh RGB-D surface3..8m, stopped<=0.3m/s>=2s, zero collisions, no false commits in negative scenes. Original centre-distance criterion reported separately.'))
    files=['scripts/run_stage1_dynamic.py','src/perception/candidate_pipeline.py','src/perception/vehicle_attributes.py',
        'src/perception/depth_proposal_refinement.py','src/perception/auxiliary_vehicle_proposals.py',
        'src/perception/target_approach.py','src/communication/persistent_target.py','src/navigation/ugv_oracle_planner.py',
        'src/perception/task_spec.py','src/perception/attributes.py','src/safety/rgbd_hazard_perception.py',
        'scripts/run_recovery_campaign.py','scripts/analyze_recovery_campaign.py','scripts/recovery_stage.py',
        'configs/schemas/target_candidate_v1.json','configs/schemas/semantic_target_message_schema.json']
    for name in files:
        dest=OUT/'frozen_runtime_v6'/name;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,dest)
    save(OUT/'frozen_runtime_v6/manifest.json',dict(files={name:digest(ROOT/name) for name in files},
        model_sha256=digest(cal['weights']),model_path=cal['weights'],
        primary_detector_sha256=digest(DETECTOR),auxiliary_detector_sha256=digest(ROOT/'models/yolo26x.pt'),
        data_manifest_sha256=digest(OUT/'attribute_data_v6/manifest.jsonl')))
    print('FROZEN10 final acceptance cases; no exploratory positive exclusions',flush=True)


if __name__=='__main__':main()
