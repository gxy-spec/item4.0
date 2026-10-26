"""Offline RGB/instance attribute capture, never a closed-loop result.

Virtual camera positions diversify observed vehicle appearance. Actor truth is
used only to label offline images; these poses are not claimed as robot motion.
All colors/classes are captured in each train/validation/test location.
"""
from pathlib import Path
import json,math,queue,subprocess,sys,time
import cv2,numpy as np,yaml,carla
from datetime import datetime,timezone
from collections import defaultdict
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from run_e7_comparison_batch import restart_simulator
from run_stage0_preview import sensor_blueprint,configure_blueprint,dict_to_transform
from src.perception.candidate_pipeline import project_world_point
from src.sensors.depth import bgra_to_rgb
from scripts.export_instance_segmentation_dataset import decode_actor_ids,vehicle_actor_mask
from src.perception.attributes import analyze_color

COLORS_RGB={'red':'255,0,0','white':'255,255,255','blue':'0,0,255'}
SPECS=[('van','vehicle.mercedes.sprinter'),('car','vehicle.tesla.model3'),
       ('truck','vehicle.carlamotors.european_hgv'),('bus','vehicle.mitsubishi.fusorosa')]
LOCATIONS=[('E2','train'),('N1','validation')]

def main():
    initialize();dest=OUT/'depth_refinement_capture';dest.mkdir(exist_ok=True)
    manifest=dest/'manifest.jsonl';existing=rows(manifest) if manifest.exists() else []
    sequence_ids=defaultdict(set)
    for r in existing:sequence_ids[r['sequence']].add(r['id'])
    completed={seq for seq,ids in sequence_ids.items() if len(ids)==8}
    attempt=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    template=yaml.safe_load((ROOT/'configs/experiments/multiscene_abc_formal_20261010_v2/E1_C_formal.yaml').read_text(encoding='utf-8'))
    package=Path(template['simulation']['package_root']);restart_simulator(package,'Town10HD','balanced offline attribute capture')
    client=carla.Client('127.0.0.1',2000);client.set_timeout(30.);world=client.get_world();original=world.get_settings()
    settings=world.get_settings();settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;world.apply_settings(settings)
    lib=world.get_blueprint_library();sensors={};queues={};vehicle=None;record_count=0
    try:
        for i in range(4):
            for mode,type_id in [('rgb','sensor.camera.rgb'),('instance','sensor.camera.instance_segmentation'),('depth','sensor.camera.depth')]:
                key=f'{i}_{mode}';bp=sensor_blueprint(lib,type_id,template['sensors']);bp.set_attribute('sensor_tick','0.0')
                camera=world.spawn_actor(bp,carla.Transform(carla.Location(0,0,70)));sensors[key]=camera;queues[key]=queue.Queue();camera.listen(queues[key].put)
        for scene,split in LOCATIONS:
            cfg=yaml.safe_load((ROOT/f'configs/experiments/multiscene_abc_formal_20261010_v2/{scene}_C_formal.yaml').read_text(encoding='utf-8'))
            transform=dict_to_transform(cfg['region']['target_transform'])
            for category,blueprint in SPECS:
                for color,rgb in COLORS_RGB.items():
                    sequence=f'{scene}_{category}_{color}'
                    if sequence in completed:continue
                    bp=configure_blueprint(lib.find(blueprint),'offline_attribute_target',rgb)
                    vehicle=world.try_spawn_actor(bp,transform)
                    if vehicle is None:raise RuntimeError(f'Cannot spawn {sequence}')
                    vehicle.set_simulate_physics(False);world.tick();time.sleep(.12);loc=vehicle.get_location();angle=math.radians(transform.rotation.yaw)
                    if loc.distance(transform.location)>.1:raise RuntimeError("Spawn pose snapshot not synchronized")
                    for bank in range(2):
                        camera_poses=[]
                        for i in range(4):
                            if i<2:
                                dx,dy,z=([(0,0,60),(0,0,85)] if bank==0 else [(4,0,60),(0,4,85)])[i]
                                pose=carla.Transform(carla.Location(loc.x+dx,loc.y+dy,z),carla.Rotation(pitch=-90,yaw=0));device='uav'
                            else:
                                theta=angle+math.radians(([0,180] if bank==0 else [45,225])[i-2]);distance=8. if i==2 else 12.
                                x,y=loc.x+distance*math.cos(theta),loc.y+distance*math.sin(theta)
                                yaw=math.degrees(math.atan2(loc.y-y,loc.x-x));pitch=math.degrees(math.atan2(-0.8,distance))
                                pose=carla.Transform(carla.Location(x,y,loc.z+2.0),carla.Rotation(pitch=pitch,yaw=yaw));device='ugv'
                            camera_poses.append((pose,device))
                            for mode in ['rgb','instance','depth']:sensors[f'{i}_{mode}'].set_transform(pose)
                        packets={k:{} for k in queues}
                        for _ in range(7):
                            world.tick();time.sleep(.10)
                            for key,q in queues.items():
                                while not q.empty():
                                    p=q.get();packets[key][p.frame]=p
                        common=set.intersection(*(set(v) for v in packets.values()))
                        if not common:raise RuntimeError(f'No exact-frame image set: {sequence} bank{bank}')
                        frame=max(common)
                        with manifest.open('a',encoding='utf-8') as stream:
                            for i,(pose,device) in enumerate(camera_poses):
                                ip=packets[f'{i}_instance'][frame];rp=packets[f'{i}_rgb'][frame]
                                bgr=cv2.cvtColor(bgra_to_rgb(rp.raw_data,800,600),cv2.COLOR_RGB2BGR)
                                encoded=np.frombuffer(ip.raw_data,np.uint8).reshape(600,800,4)[:,:,:3].copy()
                                ids,order=decode_actor_ids(encoded,[vehicle.id],semantic_tags=[14,15,16]);mask=vehicle_actor_mask(ids,encoded,vehicle.id)
                                camera={'location':[pose.location.x,pose.location.y,pose.location.z],'rotation':[pose.rotation.pitch,pose.rotation.yaw,pose.rotation.roll]}
                                verts=vehicle.bounding_box.get_world_vertices(vehicle.get_transform())
                                projected=[project_world_point((v.x,v.y,v.z),camera,800,600,100.) for v in verts]
                                points=[(u,v) for u,v,d in projected if d>0]
                                if points:
                                    xx,yy=zip(*points);roi=np.zeros_like(mask);x1,y1,x2,y2=max(0,int(min(xx))-2),max(0,int(min(yy))-2),min(800,int(max(xx))+3),min(600,int(max(yy))+3)
                                    if x2>x1 and y2>y1:roi[y1:y2,x1:x2]=True
                                    mask &= roi
                                else:mask[:]=False
                                stem=f'{sequence}_b{bank}_{device}{i}';folder=dest/attempt/'images';folder.mkdir(parents=True,exist_ok=True)
                                image_path=folder/f'{stem}_rgb.png';ins_path=folder/f'{stem}_instance.png'
                                cv2.imwrite(str(image_path),bgr);cv2.imwrite(str(ins_path),encoded)
                                dp=packets[f'{i}_depth'][frame];depth_path=folder/f'{stem}_depth_raw.png'
                                raw_depth=np.frombuffer(dp.raw_data,np.uint8).reshape(600,800,4)[:,:,:3].copy();cv2.imwrite(str(depth_path),raw_depth)
                                ys,xs=np.where(mask);bbox=None if not len(xs) else [int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]
                                evidence=None if bbox is None else analyze_color(bgr,bbox,mask)
                                record={'id':stem,'sequence':sequence,'scene':cfg['region']['region_id'],'split':split,'device':device,
                                    'category':category,'requested_color':color,'blueprint':blueprint,'actor_id':vehicle.id,
                                    'image':str(image_path),'instance':str(ins_path),'depth_bgr':str(depth_path),'mask_order':order,'bbox':bbox,'mask_pixels':len(xs),
                                    'frame':frame,'timestamp':rp.timestamp,'sensor_pose':camera,'capture_scope':'offline virtual-camera training/evaluation data; not robot execution',
                                    'color_evidence':None if evidence is None else {'dominant':evidence.dominant_color,'confidence':evidence.confidence,'ratios':evidence.ratios}}
                                stream.write(json.dumps(record,ensure_ascii=False)+'\n');record_count+=1
                    vehicle.destroy();vehicle=None;completed.add(sequence)
                    print('CAPTURED',sequence,'total images',record_count,flush=True)
        save(dest/'capture_protocol.json',{'virtual_camera_only':True,'same_frame_rgb_instance':True,'split_by_physical_region':LOCATIONS,
            'source_blueprints':SPECS,'all_requested_colors_per_class':COLORS_RGB,
            'color_labels_must_be_rgb_verified':'requested blueprint paint is not automatically accepted as RGB color truth',
            'source_manifest_sha256':digest(manifest)})
    finally:
        for sensor in sensors.values():sensor.stop();sensor.destroy()
        if vehicle is not None:vehicle.destroy()
        world.apply_settings(original)
        subprocess.run(['powershell.exe','-NoProfile','-ExecutionPolicy','Bypass','-File',str(package/'CarlaAir.ps1'),'--kill'],cwd=package,capture_output=True,timeout=60)

if __name__=='__main__':main()
