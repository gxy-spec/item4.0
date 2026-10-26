"""Run task-1 acceptance without dropping failed episodes or changing thresholds."""
from __future__ import annotations
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import shutil
import sys
import time
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'));sys.path.insert(0,str(ROOT/'scripts'))
from scenario.configuration import resolve_experiment,validate_schema,validate_online_control_config
from run_e7_comparison_batch import restart_simulator
from perception_online_acceptance import OUT,MODELS,save_json,sha256
from src.evaluation.acceptance_readiness import readiness_errors

CASES=[('red_van','van','red','vehicle.mercedes.sprinter','255,0,0',True),
    ('blue_car','car','blue','vehicle.tesla.model3','0,0,255',True),
    ('white_truck','truck','white','vehicle.carlamotors.carlacola','255,255,255',True),
    ('yellow_bus','bus','yellow','vehicle.mitsubishi.fusorosa','255,200,0',True),
    ('hard_negative','van','red','vehicle.tesla.model3','255,0,0',False),
    ('no_target','bus','yellow','vehicle.tesla.model3','100,100,100',False)]


def create_configs():
    if (OUT/'online_index.json').exists():
        raise RuntimeError('This campaign is archived. Do not overwrite its configurations; create a separately versioned campaign.')
    cal=json.loads((OUT/'calibration/adapted.json').read_text())
    threshold=cal['chosen']; configs=[]
    for seed,scene in [(42101,'E1'),(42102,'E1')]:
        template=resolve_experiment(ROOT/f'configs/experiments/persistent_memory_smoke_20261012/{scene}_C_memory.yaml')
        for case,category,color,blueprint,rgb,present in CASES:
            c=deepcopy(template)
            c.pop('_source_config',None);c.pop('_included_configs',None)
            c['experiment_id']=f'CI_E7_T1_ONLINE_{case.upper()}_{scene}_S{seed}'
            c['random_seed']=seed;c['simulation']['duration_seconds']=190.
            c['task']['task_id']=f'find_{color}_{category}'
            c['task']['instruction']=f'Find a {color} {category}, confirm it and approach safely.'
            c['task']['language']='en'
            c['task']['parsed_goal']={'target':{'category':category,'attributes':{'color':color}},'action':'approach'}
            c['actors']['target_vehicle'].update(preferred_blueprints=[blueprint],resolved_blueprint=blueprint,color_rgb=rgb)
            # Preserve RGB-D safety and its tested hazard detector. Foreground
            # vehicles remain static as in the existing target task.
            c['actors']['distractors'].update(same_category_wrong_color=1 if case!='no_target' else 0,
                same_color_wrong_category=1 if present else 0,random_vehicles=0,pedestrians=0)
            d=c['dynamic'];d['render_settle_seconds']=.09
            d['same_category_wrong_color_rgb']='0,0,255' if color!='blue' else '255,0,0'
            d['same_color_wrong_category_rgb']=rgb
            d['same_color_wrong_category_blueprint']='vehicle.mercedes.sprinter' if category=='car' else 'vehicle.tesla.model3'
            if case=='hard_negative':d['same_category_wrong_color_blueprint']='vehicle.mercedes.sprinter'
            d['save_all_samples']=False;d['sensor_save_every_n_frames']=5
            d['local_target_search'].update(enabled=True,synchronized_devices=True,model_path=str(MODELS['adapted']),
                detector_backend='custom',detector_confidence=threshold['detector_threshold'],detector_image_size=960,
                target_color_ratio_threshold=threshold['color_threshold'],enable_attribute_proposals=False,
                temporal_window=5,temporal_hits=3,infer_every_n_sensor_frames=5,device=0)
            d['persistent_target_communication'].update(enabled=True,control_policy='dual',expiry_policy='persistent_target_memory',
                task_conditioned_matching=True,require_cross_device_consensus=False,standoff_m=6.,
                minimum_candidate_world_z_m=.2,maximum_candidate_world_z_m=6.,message_ttl_s=1.)
            for name in ['a2g_spatial_v1.yaml','a2g_transport_v1.yaml']:
                c.update(yaml.safe_load((ROOT/'configs/communication'/name).read_text()))
            c['transport_link']['enabled']=True;c['transport_link']['control_group']='geo'
            c['transport_link']['service']['rate_cap_bit_s']=1e7
            c['transport_link']['wire']['sender_fields']+=['category','target_color','target_color_match']
            for dev in ['uav','ugv']:
                spec=deepcopy(c['sensors']['streams'][dev+'_rgb']);spec['type']='sensor.camera.instance_segmentation'
                c['sensors']['streams'][dev+'_instance']=spec
            a=c['acceptance']
            for key in list(a):
                if key.startswith('minimum_') and isinstance(a[key],(float,int)):a[key]=0
            a.update(minimum_common_frame_ratio=.95,minimum_ugv_rgbd_pair_ratio=.95,minimum_safety_tick_coverage=.95,
                maximum_ugv_collisions=0,maximum_uav_collisions=0,require_a2g_obstruction_transition=False,
                require_candidate_alignment=False,maximum_target_routes_planned=1000,require_message_delivery=present)
            # Legacy runner memory matching is red-target-specific; use the
            # task-conditioned external evaluator as the semantic authority.
            a.pop('maximum_memory_commit_error_m',None)
            c['output'].update(root=str(OUT/f'online/{case}_{seed}'),save_rgb=True,save_depth_raw=False,
                save_depth_float_m=False,save_depth_colour=True,save_ground_truth=True,save_instance_segmentation=True)
            c['online_acceptance']={'case':case,'seed':seed,'expected_target_present':present,
                'task_category':category,'task_color':color,'calibration_feasible':cal['constraints_satisfied'],
                'calibration_sha256':sha256(OUT/'calibration/adapted.json'),
                'runner_pass_means':'infrastructure only; use task1_evaluation.json for semantic/task verdict'}
            p=OUT/f'configs/{case}_{seed}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8');configs.append(p)
    for i,(case,*_) in enumerate(CASES[:2]):
        c=yaml.safe_load((OUT/f'configs/{case}_42101.yaml').read_text(encoding='utf-8'))
        c['experiment_id']+='_'+str(i)+'_SMOKE';c['simulation']['duration_seconds']=12.
        c['output']['root']=str(OUT/f'smoke/{case}')
        p=OUT/f'configs/smoke_{case}.yaml';p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
    save_json(OUT/'configs/index.json',[str(p) for p in configs])
    save_json(OUT/'configs/source_versions.json',{str(p.relative_to(ROOT)):sha256(p) for p in [
        ROOT/'scripts/run_stage1_dynamic.py',ROOT/'src/perception/candidate_pipeline.py',
        ROOT/'src/communication/persistent_target.py',Path(__file__)]})
    print('Prepared',len(configs),'formal + 2 smoke configurations',flush=True)


def run(smoke=False,resume=False,diagnostic_only=False):
    if smoke and (OUT/'analysis/final_acceptance_decision.json').exists():
        raise RuntimeError('Archived campaign: new smoke runs need a separately versioned output directory.')
    if not smoke:
        calibration=json.loads((OUT/'calibration/adapted.json').read_text(encoding='utf-8'))
        evidence_path=OUT/'readiness/admission.json'
        evidence=json.loads(evidence_path.read_text(encoding='utf-8')) if evidence_path.exists() else {}
        errors=readiness_errors(calibration,evidence)
        for key,record in evidence.items():
            if not isinstance(record,dict) or not record.get('passed'):continue
            source=Path(record.get('evidence_path',''))
            if not source.is_absolute():source=OUT/source
            if not source.is_file() or record.get('sha256')!=sha256(source):
                errors.append(f'Admission evidence is missing or has no matching SHA256: {key}')
        if errors and not diagnostic_only:
            raise RuntimeError('Formal batch admission rejected BEFORE simulator start:\n'+'\n'.join(errors))
        if diagnostic_only:
            raise RuntimeError('The existing 20261021 diagnostic campaign is archived. Create a new versioned output directory for further diagnostics.')
    index_path=OUT/('smoke_index.json' if smoke else 'online_index.json')
    index=json.loads(index_path.read_text()) if index_path.exists() else {}
    configs=sorted((OUT/'configs').glob('smoke_*.yaml')) if smoke else [Path(p) for p in json.loads((OUT/'configs/index.json').read_text())]
    for cp in configs:
        config=resolve_experiment(cp)
        errors=validate_schema(config,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(config)
        if errors:raise RuntimeError(f'Preflight config error {cp}: {errors}')
    for num,cp in enumerate(configs,1):
        previous=index.get(cp.stem,{})
        if resume and previous.get('report_complete') and previous.get('runner_status') in {'PASS','FAIL'}:continue
        cfg=yaml.safe_load(cp.read_text(encoding='utf-8'));package=Path(cfg['simulation']['package_root'])
        logpath=OUT/f'logs/{cp.stem}.log';logpath.parent.mkdir(parents=True,exist_ok=True)
        if logpath.exists():
            archived=OUT/f'logs/previous_attempts/{cp.stem}_{datetime.now(timezone.utc).strftime("%H%M%S")}.log'
            archived.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(logpath,archived)
        started=time.monotonic();utc=datetime.now(timezone.utc)
        result={'case':cp.stem,'config':str(cp),'config_sha256':sha256(cp),'started_utc':utc.isoformat(),'log':str(logpath)}
        print(f'[{num}/{len(configs)}] START {cp.stem}',flush=True)
        try:
            restart_simulator(package,cfg['simulation']['map'],cp.stem)
            if not smoke:
                # Do not keep a native CARLA client alive in this orchestrator:
                # restarting its server can abort the client process. The map,
                # start and target geometry are identical in all twelve cases.
                ref=yaml.safe_load((OUT/'configs/red_van_42101.yaml').read_text(encoding='utf-8'))
                geometry=lambda c:(c['simulation']['map'],c['region']['ugv_spawn_point_id'],
                    c['region']['target_transform'],c['region']['planned_ugv_route'],
                    c['dynamic'].get('target_location_override_xyz'))
                if geometry(cfg)!=geometry(ref):raise RuntimeError('Cached geometric preflight does not apply')
                preflight=json.loads((OUT/'preflight/red_van_42101.json').read_text(encoding='utf-8'))
                if cp.stem!='red_van_42101':save_json(OUT/f'preflight/{cp.stem}.json',{
                    **preflight,'geometry_identical_to':'red_van_42101','no_native_client_in_orchestrator':True})
                result['preflight_route_feasible']=preflight['feasible']
            before_run=time.monotonic()
            with logpath.open('w',encoding='utf-8') as log:
                proc=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/run_stage1_dynamic.py'),'--config',str(cp)],
                    cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=2400)
            result['runner_wall_seconds']=time.monotonic()-before_run;result['exit_code']=proc.returncode
            reports=[p for p in Path(cfg['output']['root']).glob('*/validation_report.json') if p.stat().st_mtime>=utc.timestamp()]
            if reports:
                rp=max(reports,key=lambda p:p.stat().st_mtime)
                result.update(run_dir=str(rp.parent),report_complete=True,runner_status=json.loads(rp.read_text()).get('status'))
            else:result.update(report_complete=False,error='No complete runner report; see retained log')
        except Exception as exc:result.update(report_complete=False,error=repr(exc))
        finally:
            subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(package/'CarlaAir.ps1'),'--kill'],
                cwd=package,capture_output=True,timeout=60,check=False)
        result['wall_seconds']=time.monotonic()-started;index[cp.stem]=result;save_json(index_path,index)
        with (OUT/'attempt_history.jsonl').open('a',encoding='utf-8') as history:history.write(json.dumps(result,ensure_ascii=False)+'\n')
        print(f'[{num}/{len(configs)}] RESULT {json.dumps(result,ensure_ascii=False)}',flush=True)
    print('Batch finished; all successes/failures retained:',index_path,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['configs','smoke','formal']);p.add_argument('--resume',action='store_true');p.add_argument('--diagnostic-only',action='store_true');a=p.parse_args()
    if a.action=='configs':create_configs()
    else:run(a.action=='smoke',a.resume,a.diagnostic_only)
