"""Versioned paired perception regressions; retain every completed/failed attempt."""
from copy import deepcopy
from datetime import datetime,timezone
from pathlib import Path
import argparse,json,subprocess,sys,time,yaml,shutil
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from run_e7_comparison_batch import restart_simulator
from src.scenario.configuration import validate_schema,validate_online_control_config

SOURCE=ROOT/'configs/experiments/multiscene_abc_formal_20261010_v2/formal_36_run_index.json'


def configure(item,variant,calibration,cohort='formal'):
    c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
    c['experiment_id']=f'CI_E7_R9_{variant[0].upper()}_{item["case"]}{item["group"]}'
    d=c['dynamic'];search=d['local_target_search'];memory=d['persistent_target_communication']
    hazard_release=json.loads((OUT/'hazard_release.json').read_text(encoding='utf-8'))
    d['ugv_safety']['hazard_detector'].update(**hazard_release['configuration'])
    search.update(detector_image_size=960,infer_every_n_sensor_frames=5,device=0)
    memory.update(expiry_policy='persistent_target_memory',sensor_business_monitor_enabled=True,
        memory_association_radius_m=10.,challenger_window_s=4.,challenger_min_updates=3,
        challenger_min_score_margin=.08,challenger_old_unseen_s=2.,stale_max_distance_m=8.)
    if variant=='upgraded':
        search.update(model_path=str(DETECTOR),detector_backend='custom',detector_confidence=.08,
            attribute_model_path=calibration['weights'],attribute_type_threshold=calibration['type_threshold'],
            attribute_color_threshold=calibration['color_threshold'],target_color_ratio_threshold=calibration['color_threshold'],
            temporal_window=5,temporal_hits=3,enable_attribute_proposals=False,depth_proposal_refinement=True,auxiliary_detector_path='models/yolo26x.pt')
        memory.update(task_conditioned_matching=True,minimum_candidate_world_z_m=.2,maximum_candidate_world_z_m=6.)
    elif variant=='legacy':
        # Preserve the original YOLO26n + HSV + generic vehicle/van-prior
        # semantic decision, while giving both systems the same current
        # memory, safety, inference cadence and simulation conditions.
        search.update(model_path='models/yolo26n.pt',detector_backend='coco',detector_confidence=.08,
            enable_attribute_proposals=True,temporal_window=5,temporal_hits=3)
        memory.update(task_conditioned_matching=False,minimum_candidate_world_z_m=.2,maximum_candidate_world_z_m=6.)
    else:raise ValueError(variant)
    d.update(save_all_samples=False,sensor_save_every_n_frames=5,cpu_threads=4)
    # Exact RGB/instance evidence for each inference sample, same mounts.
    for device in ['uav','ugv']:
        spec=deepcopy(c['sensors']['streams'][device+'_rgb']);spec['type']='sensor.camera.instance_segmentation'
        c['sensors']['streams'][device+'_instance']=spec
    c['output'].update(root=str(OUT/f'{cohort}/{variant}/{item["case"]}/{item["group"]}'),
        save_rgb=True,save_depth_raw=True,save_depth_float_m=False,save_depth_colour=True,
        save_ground_truth=True,save_instance_segmentation=True)
    c['online_acceptance']={'task_category':'van','task_color':'red','expected_target_present':True,'calibration_feasible':True}
    c['recovery_evaluation']={'variant':variant,'cohort':cohort,'source_case':item['case'],
        'source_group':item['group'],'source_config_sha256':digest(item['config']),
        'task_category':'van','task_color':'red','expected_target_present':True,
        'runner_verdict':'infrastructure and historical acceptance checks only; task identity audited separately',
        'sensor_approach':'fresh temporal-confirmed UGV RGB-D target surface depth 3..8m, speed<=0.3m/s, hold>=2s',
        'historical_arrival_secondary':'actual target-centre distance 3..7m, stopped >=2s',
        'known_scope':'same-map regression; original region IDs can share physical parking sites'}
    return c


def configs():
    cal=json.loads((OUT/'calibration/deployment_release.json').read_text(encoding='utf-8'))
    items=json.loads(SOURCE.read_text(encoding='utf-8'))['runs'];index=[]
    if (OUT/'formal_index.json').exists():raise RuntimeError('Already frozen: do not rewrite formal configs')
    for item in items:
        for variant in ['legacy','upgraded']:
            c=configure(item,variant,cal);p=OUT/f'configs/formal/{variant}_{item["case"]}_{item["group"]}.yaml'
            errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
            if errors:raise RuntimeError((p,errors))
            p.parent.mkdir(parents=True,exist_ok=True);p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
            index.append({**item,'variant':variant,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
    save(OUT/'formal_index.json',{'design':'36 upgraded +36 same-condition legacy controls','runs':index,
        'calibration_sha256':digest(OUT/'calibration/deployment_final.json'),'source_index_sha256':digest(SOURCE)})
    print('FROZEN',len(index),'runs',flush=True)


def run_one(item,cohort):
    p=Path(item['config']);c=yaml.safe_load(p.read_text(encoding='utf-8'));package=Path(c['simulation']['package_root'])
    configured_root=Path(c['output']['root']);actual_root=configured_root
    estimated_raw_bytes=int(c['simulation']['duration_seconds']*16_000_000+250_000_000)
    configured_root=configured_root.resolve()
    actual_root=configured_root
    if configured_root.anchor.upper() not in ['D:\\','E:\\']:
        raise RuntimeError('R10 formal data must resolve to D/E, never C: '+str(configured_root))
    # Resolve directory junctions before inspecting the physical drive.
    def free_budget(drive):return shutil.disk_usage(drive).free-5*1024**3
    if free_budget(configured_root.anchor)<estimated_raw_bytes:
        drive=max(['D:/','E:/'],key=free_budget)
        if free_budget(drive)<estimated_raw_bytes:raise RuntimeError('Insufficient complete-run payload +5GiB reserve; simulator not launched')
        actual_root=Path(drive)/'CI10'/cohort/p.stem
    potential=actual_root/(c['experiment_id']+'_20991231T235959Z')/'config_snapshot/runtime_sources/src/perception/auxiliary_vehicle_proposals.py'
    if len(str(potential))>240:raise RuntimeError('Output path would exceed the conservative Windows path budget: '+str(potential))
    label=p.stem;logpath=OUT/f'logs/{cohort}/{label}_{datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")}.log'
    logpath.parent.mkdir(parents=True,exist_ok=True);started=datetime.now(timezone.utc);t=time.monotonic()
    result={**item,'started_utc':started.isoformat(),'log':str(logpath),'config_sha256':digest(p),
        'configured_output_root':str(configured_root),'output_root':str(actual_root),
        'storage_override':actual_root!=configured_root,'estimated_raw_bytes':estimated_raw_bytes,
        'storage_policy':'Only output placement may change for free-space protection. Frozen scene/model/controller config remains unchanged; CLI override recorded.'}
    lease=simulator_lease();lease_acquired=False
    try:
        lease.__enter__();lease_acquired=True
        restart_simulator(package,c['simulation']['map'],label)
        with logpath.open('w',encoding='utf-8') as log:
            command=[sys.executable,'-u',str(ROOT/'scripts/run_stage1_dynamic.py'),'--config',str(p),'--timeout','120']
            if actual_root!=configured_root:command+=['--output-root',str(actual_root)]
            result['command']=command
            process=subprocess.run(command,
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=max(2400,c['simulation']['duration_seconds']*12))
        result['runner_exit_code']=process.returncode
        reports=[x for x in actual_root.glob('*/validation_report.json') if x.stat().st_mtime>=started.timestamp()]
        if reports:
            rp=max(reports,key=lambda x:x.stat().st_mtime);r=json.loads(rp.read_text(encoding='utf-8'))
            result.update(report_complete=r['status'] in ['PASS','FAIL'],report_present=True,runner_status=r['status'],run_dir=str(rp.parent),summary=r.get('summary'),
                failed_checks=[x for x in r['checks'] if not x['passed']])
        else:result.update(report_complete=False,error='No complete report; retained logs and partial artifacts')
    except Exception as exc:result.update(report_complete=False,error=repr(exc))
    finally:
        if lease_acquired:
            subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(package/'CarlaAir.ps1'),'--kill'],
                cwd=package,capture_output=True,timeout=60,check=False)
            lease.__exit__(None,None,None)
    result['wall_seconds']=time.monotonic()-t
    with (OUT/'attempt_history.jsonl').open('a',encoding='utf-8') as f:f.write(json.dumps(result,ensure_ascii=False)+'\n')
    return result


def run(index_path,cohort,resume=True,acceptance_stop_on_failure=False):
    scope_path=OUT/'execution_scope.json'
    if cohort=='formal' and scope_path.exists() and not json.loads(scope_path.read_text(encoding='utf-8')).get('full_formal_allowed',False):
        raise RuntimeError('Full72 formal campaign deferred by user; targeted repair validation only')
    index=json.loads(Path(index_path).read_text(encoding='utf-8'));items=index['runs'];progress_path=OUT/f'{cohort}_progress.json'
    progress=json.loads(progress_path.read_text(encoding='utf-8')) if progress_path.exists() else {}
    for number,item in enumerate(items,1):
        if (OUT/f'{cohort}.stop').exists():
            print('STOP requested between episodes; no active acquisition interrupted',flush=True);return
        key=Path(item['config']).stem
        if resume and progress.get(key,{}).get('report_complete'):continue
        if index.get('runtime_manifest_path'):
            manifest=json.loads(Path(index['runtime_manifest_path']).read_text(encoding='utf-8'))
            for name,expected in manifest['files'].items():
                if digest(ROOT/name)!=expected:raise RuntimeError('Runtime changed after freeze: '+name)
            for name,expected in manifest.get('artifacts',{}).items():
                if digest(name)!=expected:raise RuntimeError('Model/calibration changed after freeze: '+name)
        if item.get('config_sha256') and digest(item['config'])!=item['config_sha256']:raise RuntimeError('Config changed after freeze')
        print(f'START {number}/{len(items)} {key}',flush=True)
        result=run_one(item,cohort);progress[key]=result;save(progress_path,progress)
        print(f'RESULT {number}/{len(items)} {key} complete={result.get("report_complete")} runner={result.get("runner_status")} {result["wall_seconds"]:.1f}s',flush=True)
        if not result.get('report_complete'):raise RuntimeError(f'Infrastructure failure: {key}; diagnose then resume without losing attempts')
        if acceptance_stop_on_failure:
            from analyze_recovery_campaign import assess_episode
            assessment=assess_episode({**result,'case':key,'runner_wall_seconds':result.get('wall_seconds')})
            positive=assessment.get('expected_target_present')
            passed=assessment.get('runner_status')=='PASS' and (bool(assessment.get('identity_task_success') and assessment.get('sensor_safe_approach_audited')) if positive else bool(assessment.get('negative_rejection_pass',False) and assessment.get('sent_messages',0)==0))
            result['online_task_gate_passed']=bool(passed);progress[key]=result;save(progress_path,progress)
            print('ONLINE_TASK_GATE',key,passed,flush=True)
            if not passed:
                save(OUT/f'{cohort}.stop',{'case':key,'reason':'Completed developmental acceptance failed; diagnose before starting another case','assessment':str(Path(result['run_dir'])/'recovery_evaluation.json')})
                return
    print('ALL EPISODES RETAINED',progress_path,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['configs','run']);p.add_argument('--index',type=Path);p.add_argument('--cohort',default='formal');p.add_argument('--acceptance-stop-on-failure',action='store_true');args=p.parse_args()
    if args.action=='configs':configs()
    else:run(args.index or OUT/'formal_index.json',args.cohort,acceptance_stop_on_failure=args.acceptance_stop_on_failure)
