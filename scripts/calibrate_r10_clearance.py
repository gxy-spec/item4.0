"""Controlled static calibration, separate from robot closed-loop results.

Truth sets up scenes and labels errors; the estimator receives depth, image box,
camera mount and known own body only. These virtual poses are not robot routes.
"""
from pathlib import Path
import argparse
import sys,json,math,queue,time,subprocess
import cv2,numpy as np,yaml,carla,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import ROOT,OUT,save,simulator_lease
from run_e7_comparison_batch import restart_simulator
from run_stage0_preview import sensor_blueprint,configure_blueprint,dict_to_transform
from src.perception.body_clearance import rectangle,polygon_clearance,observed_clearance
from src.perception.candidate_pipeline import CandidatePipeline,PipelineConfig,transform_matrix
from src.perception.task_spec import TaskSpec
from src.sensors.depth import carla_depth_to_metres
from export_instance_segmentation_dataset import decode_actor_ids,vehicle_actor_mask

SPECS=[('car','vehicle.tesla.model3'),('van','vehicle.mercedes.sprinter'),
       ('truck','vehicle.carlamotors.european_hgv'),('bus','vehicle.mitsubishi.fusorosa')]


def body(actor):
    xy=np.array([[v.x,v.y] for v in actor.bounding_box.get_world_vertices(actor.get_transform())],dtype=np.float32)
    return cv2.convexHull(xy).reshape(-1,2).tolist()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--validation',action='store_true');args=parser.parse_args()
    torch.set_num_threads(4);cv2.setNumThreads(2)
    dest=OUT/('clearance_validation' if args.validation else 'clearance_calibration');dest.mkdir(parents=True,exist_ok=True)
    if (dest/'summary.json').exists():raise RuntimeError('Calibration output exists; use a new version for repeats')
    config=yaml.safe_load((OUT/'configs/acceptance/upgraded_red_van_C.yaml').read_text(encoding='utf-8'))
    if args.validation:
        alternative=yaml.safe_load((Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')/'configs/formal/upgraded_N1_C.yaml').read_text(encoding='utf-8'))
        config['region']['target_transform']=alternative['region']['target_transform']
    search=config['dynamic']['local_target_search'];spec=config['sensors']['streams']['ugv_depth']
    mount=transform_matrix(dict(location=spec['location_xyz_m'],rotation=spec['rotation_pyr_degrees']))
    pipeline=CandidatePipeline(PipelineConfig(model_path=Path(search['model_path']),detector_backend='custom',
        detector_confidence=.08,detector_image_size=960,device=0,enable_attribute_proposals=False,
        detector_prompts=('van','car','truck','bus'),task_spec=TaskSpec.from_task_config({'parsed_goal':{'target':{'category':'vehicle'},'action':'observe'}}),
        attribute_model_path=Path(search['attribute_model_path']),attribute_type_threshold=.4,attribute_color_threshold=.4,
        depth_proposal_refinement=True,auxiliary_detector_path=ROOT/'models/yolo26x.pt'))
    package=Path(config['simulation']['package_root']);sensors={};queues={};actors=[];records=[]
    with simulator_lease():
        restart_simulator(package,'Town10HD','R10 independent clearance calibration')
        client=carla.Client('127.0.0.1',2000);client.set_timeout(40);world=client.get_world();original=world.get_settings()
        settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;world.apply_settings(settings)
        lib=world.get_blueprint_library()
        try:
            own=world.spawn_actor(configure_blueprint(lib.find('vehicle.tesla.model3'),'calibration_own','0,0,255'),world.get_map().get_spawn_points()[37]);actors.append(own)
            for _ in range(25):world.tick()
            own.set_simulate_physics(False)
            box=own.bounding_box;own_polygon=rectangle([box.location.x,box.location.y],[box.extent.x,box.extent.y],box.rotation.yaw)
            own.set_transform(carla.Transform(carla.Location(-200,-200,40)))
            for mode in ['rgb','depth','instance_segmentation']:
                key='instance' if mode=='instance_segmentation' else mode
                bp=sensor_blueprint(lib,'sensor.camera.'+mode,config['sensors']);bp.set_attribute('sensor_tick','0.0')
                camera=world.spawn_actor(bp,carla.Transform(carla.Location(0,0,70)))
                sensors[key]=camera;queues[key]=queue.Queue();camera.listen(queues[key].put)
            for category,blueprint in SPECS:
                target=world.spawn_actor(configure_blueprint(lib.find(blueprint),'calibration_target','255,0,0'),dict_to_transform(config['region']['target_transform']));actors.append(target)
                for _ in range(30):world.tick()
                target.set_simulate_physics(False);tp=target.get_transform();target_poly=body(target)
                for view in ([20,60,180] if args.validation else [0,45,90]):
                    angle=math.radians(tp.rotation.yaw+view+180)
                    vector=np.array([math.cos(angle),math.sin(angle)])
                    yaw=math.degrees(angle)+180
                    for desired_gap in ([3.5,6.,8.] if args.validation else [2.5,5.,8.5]):
                        lo,hi=0.,30.
                        for _ in range(35):
                            r=(lo+hi)/2;xy=np.array([tp.location.x,tp.location.y])+vector*r
                            poly=rectangle(xy,[box.extent.x,box.extent.y],yaw)
                            if polygon_clearance(poly,target_poly)<desired_gap:lo=r
                            else:hi=r
                        xy=np.array([tp.location.x,tp.location.y])+vector*hi
                        own_pose=dict(location=[*xy,tp.location.z],rotation=[0,yaw,0])
                        matrix=transform_matrix(own_pose)@mount
                        # Mount pitch=-5, own pose has pitch=roll=0.
                        cp=carla.Transform(carla.Location(*map(float,matrix[:3,3])),carla.Rotation(pitch=-5,yaw=yaw))
                        for sensor in sensors.values():sensor.set_transform(cp)
                        packets={k:{} for k in queues}
                        for _ in range(8):
                            world.tick();time.sleep(.07)
                            for key,q in queues.items():
                                while not q.empty():
                                    p=q.get();packets[key][p.frame]=p
                        common=set.intersection(*(set(v) for v in packets.values()))
                        if not common:raise RuntimeError('No synchronized calibration frame')
                        frame=max(common);rp=packets['rgb'][frame];dp=packets['depth'][frame];ip=packets['instance'][frame]
                        rgb=np.frombuffer(rp.raw_data,np.uint8).reshape(600,800,4)[:,:,:3].copy()
                        depth=carla_depth_to_metres(dp.raw_data,800,600)
                        packed=np.frombuffer(ip.raw_data,np.uint8).reshape(600,800,4)[:,:,:3].copy()
                        ids,order=decode_actor_ids(packed,[target.id],semantic_tags=[14,15,16]);mask=vehicle_actor_mask(ids,packed,target.id)
                        ys,xs=np.where(mask);truth_box=None if not len(xs) else [int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]
                        camera_pose=dict(location=[cp.location.x,cp.location.y,cp.location.z],rotation=[cp.rotation.pitch,cp.rotation.yaw,cp.rotation.roll])
                        pipeline.history=[];det=pipeline.infer_images([rgb])[0]
                        candidates=pipeline.process(frame,rp.timestamp,rgb,depth,camera_pose,det)
                        candidate=max(candidates,key=lambda c:CandidatePipeline._iou(c['bbox_xyxy'],truth_box),default=None) if truth_box else None
                        overlap=CandidatePipeline._iou(candidate['bbox_xyxy'],truth_box) if candidate else 0
                        estimate=observed_clearance(depth,candidate['bbox_xyxy'],candidate['depth_m'],mount,own_polygon,100.) if candidate and overlap>=.5 else dict(valid=False,reason='target_not_detected')
                        gap=polygon_clearance(rectangle(xy,[box.extent.x,box.extent.y],yaw),target_poly)
                        stem=f'{category}_view{view}_gap{desired_gap}';folder=dest/'frames';folder.mkdir(exist_ok=True)
                        cv2.imwrite(str(folder/f'{stem}_rgb.png'),rgb);cv2.imwrite(str(folder/f'{stem}_depth.png'),np.frombuffer(dp.raw_data,np.uint8).reshape(600,800,4)[:,:,:3]);cv2.imwrite(str(folder/f'{stem}_instance.png'),packed)
                        error=estimate.get('estimated_body_clearance_m',gap)-gap if estimate.get('valid') else None
                        record=dict(id=stem,category=category,view_degrees=view,desired_gap_m=desired_gap,true_gap_m=gap,estimate=estimate,
                            signed_error_m=error,box_iou=overlap,truth_bbox=truth_box,candidate=candidate,
                            own_virtual_pose=own_pose,own_body_polygon=own_polygon,target_polygon=target_poly,camera_pose=camera_pose,frame=frame)
                        records.append(record);save(dest/'records.json',records)
                        print(stem,'valid=',estimate['valid'],'error=',error,flush=True)
                target.destroy();actors.remove(target)
            valid=[r for r in records if r['signed_error_m'] is not None];errors=np.array([r['signed_error_m'] for r in valid])
            summary=dict(total=len(records),valid=len(valid),coverage=len(valid)/len(records),
                mae_m=float(np.mean(abs(errors))) if len(errors) else None,p95_absolute_error_m=float(np.percentile(abs(errors),95)) if len(errors) else None,
                maximum_absolute_error_m=float(max(abs(errors))) if len(errors) else None,
                maximum_dangerous_overestimate_m=float(max(errors)) if len(errors) else None,
                passed=bool(len(valid)>=24 and max(abs(errors))<=1.),scope='36 controlled static virtual poses, not navigation episodes; invalid cases retained')
            save(dest/'summary.json',summary);print(json.dumps(summary),flush=True)
        finally:
            for s in sensors.values():s.stop();s.destroy()
            for a in actors:
                if a.is_alive:a.destroy()
            world.apply_settings(original)
            subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(package/'CarlaAir.ps1'),'--kill'],cwd=package,capture_output=True,timeout=60)


if __name__=='__main__':main()
