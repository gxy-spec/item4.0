"""Reproducible read-only metric/provenance audit for the four user questions."""
from __future__ import annotations
import csv
import json
from pathlib import Path
import shutil
import yaml
from perception_online_acceptance import OUT, ROOT, save_json, sha256
from package_online_acceptance_results import DELIVERY


def get(data,path):
    for part in path.split('.'):
        if not isinstance(data,dict):return None
        data=data.get(part)
    return data


def main():
    dest=OUT/'analysis/followup_questions';dest.mkdir(exist_ok=True)
    ev=Path('E:/CarlaAirData/CityInspection_GOC/evaluation/perception_town10hd_v2')
    names={'original_yoloe':'zeroshot_test_protocol_v1','adapted_yoloe':'adapted_test_protocol_v1'}
    metrics={};sources=[]
    for name,folder in names.items():
        p=ev/folder/'evaluation_manifest.json';m=json.loads(p.read_text(encoding='utf-8'))
        metrics[name]=m
        shutil.copy2(p,dest/f'{name}_evaluation_manifest.json')
        sources.append({'path':str(p),'sha256':sha256(p)})
    rows=[]
    for key,value in metrics['original_yoloe']['metrics'].items():
        if key=='fitness':continue
        new=metrics['adapted_yoloe']['metrics'][key]
        rows.append({'metric':key,'original':value,'adapted':new,'difference_percentage_points':100*(new-value)})
    with (dest/'同协议检测分割对照.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    benchmark=json.loads((OUT/'analysis/benchmark_ego_excluded_at_frozen_thresholds.json').read_text(encoding='utf-8'))
    task_results={name:{split:benchmark[name][split]['overall'] for split in ['validation','test']} for name in ['original','adapted']}
    save_json(dest/'paired_task_selection_counts.json',task_results)
    old_path=ROOT/'configs/experiments/multiscene_abc_formal_20261010_v2/E1_C_formal.yaml'
    template_path=ROOT/'configs/experiments/persistent_memory_smoke_20261012/E1_C_memory.yaml'
    current_path=OUT/'configs/red_van_42101.yaml'
    configs=[yaml.safe_load(p.read_text(encoding='utf-8')) for p in [old_path,template_path,current_path]]
    paths=['simulation.map','simulation.duration_seconds','simulation.quality','simulation.fixed_delta_seconds',
        'region.region_id','region.boundary','region.target','region.target_vehicle','region.ugv',
        'region.planned_ugv_route','region.uav','sensors.width','sensors.height','sensors.fov_degrees',
        'sensors.streams.uav_rgb','dynamic.uav_altitude_m','dynamic.uav_speed_mps',
        'dynamic.uav_camera_yaw_mode','dynamic.uav_camera_yaw_degrees','dynamic.ugv_target_speed_mps',
        'dynamic.local_target_search','dynamic.persistent_target_communication',
        'actors.target_vehicle','actors.distractors','dynamic.sensor_save_every_n_frames',
        'dynamic.background_vehicle_autopilot','dynamic.pedestrian_motion','random_seed']
    differences=[{'field':p,'old_36_E1_C':get(configs[0],p),'memory_E1_template':get(configs[1],p),
        'current_red_van':get(configs[2],p),'old_equals_current':get(configs[0],p)==get(configs[2],p)} for p in paths]
    save_json(dest/'scene_configuration_provenance.json',{'sources':[str(p) for p in [old_path,template_path,current_path]],
        'comparison':differences,'old_region_exactly_equal_current':configs[0]['region']==configs[2]['region'],
        'old_sensors_exactly_equal_current':configs[0]['sensors']==configs[2]['sensors'],
        'note':'same E1 geometry does not establish continuous RGB target visibility; this is not the complete 36-run paired design'})
    oldindex=json.loads((old_path.parent/'formal_36_run_index.json').read_text(encoding='utf-8'))
    newindex=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
    summary={'old_design':oldindex['paired_design'],'old_run_count':len(oldindex['runs']),
        'current_run_count':len(newindex),'current_regions':sorted({yaml.safe_load(Path(r['config']).read_text(encoding='utf-8'))['region']['region_id'] for r in newindex.values()}),
        'sources':sources,'detection_test_images':1781,'detection_test_instances':3967,
        'primary_comparison_baseline':'original YOLOE-26s-seg, NOT historical YOLO26n/YOLO26x',
        'no_accepted_universal_minimum_ap':'COCO/Ultralytics define metrics; no universal application-independent pass score is specified',
        'no_unbiased_online_model_comparison':'12 online episodes use adapted model only; no original-model paired online success claim is supported'}
    save_json(dest/'audit_summary.json',summary)
    for name in ['同协议检测分割对照.csv','paired_task_selection_counts.json','scene_configuration_provenance.json','audit_summary.json']:
        shutil.copy2(dest/name,DELIVERY/name)
    print(json.dumps({'counts':task_results,'old_region_exactly_equal_current':configs[0]['region']==configs[2]['region'],
        'old_sensors_exactly_equal_current':configs[0]['sensors']==configs[2]['sensors'],
        'differing_fields':[r['field'] for r in differences if not r['old_equals_current']]},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
