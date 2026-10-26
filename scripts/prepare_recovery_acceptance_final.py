"""Independent multi-type/color online acceptance, using RGB-renderable paints."""
import sys,json,yaml
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from run_recovery_campaign import configure,SOURCE
from src.scenario.configuration import validate_schema,validate_online_control_config

CASES=[('red_van','van','red','vehicle.mercedes.sprinter','255,0,0',True),
       ('blue_van','van','blue','vehicle.mercedes.sprinter','0,0,255',True),
       ('yellow_car','car','yellow','vehicle.tesla.model3','255,210,0',True),
       ('gray_truck','truck','gray','vehicle.carlamotors.european_hgv','110,110,110',True),
       ('white_bus','bus','white','vehicle.mitsubishi.fusorosa','255,255,255',True),
       ('hard_negative','van','red','vehicle.tesla.model3','255,0,0',False),
       ('no_target','van','red','vehicle.tesla.model3','110,110,110',False)]


def main():
    if (OUT/'acceptance_final_index.json').exists():raise RuntimeError('Acceptance configs already frozen')
    cal=json.loads((OUT/'calibration/deployment.json').read_text(encoding='utf-8'))
    item=next(r for r in json.loads(SOURCE.read_text(encoding='utf-8'))['runs'] if r['case']=='E1' and r['group']=='C');index=[]
    for number,(case,category,color,blueprint,rgb,present) in enumerate(CASES):
        c=configure(item,'upgraded',cal,'acceptance_final');c['experiment_id']+=f'_{case.upper()}'
        c['random_seed']=43101+number
        c['task'].update(task_id=f'find_{color}_{category}',instruction=f'Find a {color} {category}, confirm it and approach safely.',language='en',
            parsed_goal={'target':{'category':category,'attributes':{'color':color}},'action':'approach'})
        c['actors']['target_vehicle'].update(preferred_blueprints=[blueprint],resolved_blueprint=blueprint,color_rgb=rgb)
        c['actors']['distractors'].update(same_category_wrong_color=1 if case!='no_target' else 0,
            same_color_wrong_category=0,random_vehicles=0,pedestrians=0)
        d=c['dynamic'];d['same_category_wrong_color_blueprint']='vehicle.mercedes.sprinter' if category=='van' else blueprint
        d['same_category_wrong_color_rgb']='0,0,255' if color!='blue' else '255,0,0'
        d['comparison_distractor_color_rgb']=d['same_category_wrong_color_rgb']
        c['output']['root']=str(OUT/f'acceptance_final/{case}')
        c['online_acceptance'].update(task_category=category,task_color=color,expected_target_present=present)
        c['recovery_evaluation'].update(task_category=category,task_color=color,expected_target_present=present,acceptance_case=case)
        # Infrastructure criteria preserve synchronization and zero collisions;
        # expected lack of messages/traffic is handled explicitly for controls.
        a=c['acceptance'];a.update(minimum_moving_vehicles=0,minimum_moving_pedestrians=0,
            require_message_delivery=present,require_expiry_safe_hold=False)
        p=OUT/f'configs/acceptance_final/{case}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
        errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
        if errors:raise RuntimeError((case,errors))
        p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
        index.append({**item,'variant':'upgraded','case':case,'config':str(p),'config_sha256':digest(p),
            'output_root':c['output']['root'],'expected_target_present':present})
    save(OUT/'acceptance_final_index.json',{'runs':index,'criterion':'perception identity and sensor-relative safe approach separately; all failures retained',
        'scope':'single E1 geometry, 5 positive task combinations spanning all4types, 2 negative controls; not cross-city generalization'})
    print('Prepared',len(index),'independent acceptance episodes')


if __name__=='__main__':main()
