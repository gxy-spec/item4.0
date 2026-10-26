"""Isolated foliage visibility diagnosis AFTER formal episodes.

Known target pose and map-object switches are used only for this offline
diagnosis, never for an online task-success claim.
"""
from __future__ import annotations
import json
from pathlib import Path
import queue
import sys
import time
import cv2
import numpy as np
import yaml
import carla
sys.path.insert(0,str(Path(__file__).resolve().parent))
from perception_online_acceptance import OUT,ROOT,save_json
from run_e7_comparison_batch import restart_simulator
from run_stage0_preview import sensor_blueprint,configure_blueprint,dict_to_transform
from src.sensors.depth import bgra_to_rgb,carla_depth_to_metres,colourise_depth
from src.perception.attributes import analyze_color
from src.perception.candidate_pipeline import project_world_point
from export_instance_segmentation_dataset import decode_actor_ids,vehicle_actor_mask


def main():
    index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
    if len(index)!=12 or any(v.get('runner_status') not in ['PASS','FAIL'] for v in index.values()):
        raise RuntimeError('Refusing diagnostic while formal batch is incomplete')
    cfg=yaml.safe_load((OUT/'configs/red_van_42101.yaml').read_text(encoding='utf-8'))
    package=Path(cfg['simulation']['package_root'])
    restart_simulator(package,'Town10HD','isolated_visibility_diagnosis')
    client=carla.Client('127.0.0.1',2000);client.set_timeout(30.)
    world=client.get_world();original=world.get_settings();settings=world.get_settings()
    settings.synchronous_mode=True;settings.fixed_delta_seconds=.05;world.apply_settings(settings)
    library=world.get_blueprint_library();sensors=[];target=None;vegetation=[];buildings=[]
    out=OUT/'observability_diagnostic';out.mkdir(parents=True,exist_ok=True)
    results=[]
    try:
        bp=configure_blueprint(library.find('vehicle.mercedes.sprinter'),'diagnostic_target','255,0,0')
        target=world.try_spawn_actor(bp,dict_to_transform(cfg['region']['target_transform']))
        if target is None:raise RuntimeError('Cannot spawn diagnostic foreground actor')
        target.set_simulate_physics(False)
        transform=carla.Transform(carla.Location(23.35,6.,58.5),carla.Rotation(pitch=-90.,yaw=0.))
        queues={};camera_attributes={}
        for name,type_id in [('rgb','sensor.camera.rgb'),('depth','sensor.camera.depth'),('instance','sensor.camera.instance_segmentation'),('rgb_plain','sensor.camera.rgb')]:
            bp=sensor_blueprint(library,type_id,cfg['sensors']);bp.set_attribute('sensor_tick','0.0')
            if name=='rgb_plain':bp.set_attribute('enable_postprocess_effects','false')
            camera=world.spawn_actor(bp,transform);sensors.append(camera);queues[name]=queue.Queue()
            camera_attributes[name]=dict(camera.attributes)
            camera.listen(queues[name].put)
        vegetation=[o.id for o in world.get_environment_objects(carla.CityObjectLabel.Vegetation)]
        buildings=[o.id for o in world.get_environment_objects(carla.CityObjectLabel.Buildings)]
        if not vegetation:raise RuntimeError('Map vegetation inventory is empty; diagnostic cannot run')
        states=[('normal',True,True),('vegetation_off',False,True),
            ('vegetation_and_buildings_off',False,False),('restored',True,True)]
        for state,enabled,buildings_enabled in states:
            world.enable_environment_objects(set(vegetation),enabled)
            if buildings:world.enable_environment_objects(set(buildings),buildings_enabled)
            packets={name:{} for name in queues}
            for tick in range(24):
                world.tick();time.sleep(.12)
                for name,q in queues.items():
                    while not q.empty():
                        p=q.get();packets[name][p.frame]=p
            common=set.intersection(*(set(p.keys()) for p in packets.values()))
            if not common:raise RuntimeError('No exact-frame three-camera capture')
            frame=max(common);rgb_packet=packets['rgb'][frame];depth_packet=packets['depth'][frame];instance_packet=packets['instance'][frame]
            rgb=bgra_to_rgb(rgb_packet.raw_data,rgb_packet.width,rgb_packet.height)
            bgr=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
            plain_packet=packets['rgb_plain'][frame]
            plain=cv2.cvtColor(bgra_to_rgb(plain_packet.raw_data,plain_packet.width,plain_packet.height),cv2.COLOR_RGB2BGR)
            encoded=np.frombuffer(instance_packet.raw_data,np.uint8).reshape((600,800,4))[:,:,:3].copy()
            depth=carla_depth_to_metres(depth_packet.raw_data,depth_packet.width,depth_packet.height)
            ids,order=decode_actor_ids(encoded,[target.id],semantic_tags=[14,15,16])
            mask=vehicle_actor_mask(ids,encoded,target.id)
            if not mask.any():raise RuntimeError(f'No foreground instance mask in {state}')
            ys,xs=np.where(mask);box=[int(xs.min()),int(ys.min()),int(xs.max())+1,int(ys.max())+1]
            color=analyze_color(bgr,box,mask)
            plain_color=analyze_color(plain,box,mask)
            cv2.imwrite(str(out/f'{state}_rgb.png'),bgr);cv2.imwrite(str(out/f'{state}_instance.png'),encoded)
            cv2.imwrite(str(out/f'{state}_rgb_plain.png'),plain)
            np.save(out/f'{state}_depth_metres.npy',depth)
            colourise_depth(depth).save(out/f'{state}_depth.png')
            np.save(out/f'{state}_target_mask.npy',mask)
            x1,y1,x2,y2=box
            cv2.imwrite(str(out/f'{state}_target_crop.png'),bgr[max(0,y1-6):y2+6,max(0,x1-6):x2+6])
            location=target.get_location();t={'location':[23.35,6.,58.5],'rotation':[-90.,0.,0.]}
            center=project_world_point((location.x,location.y,location.z+target.bounding_box.extent.z),t,800,600,100)
            result={'state':state,'frame':frame,'timestamp':rgb_packet.timestamp,'vegetation_enabled':enabled,'buildings_enabled':buildings_enabled,
                'target_actor_id':target.id,'target_position':[location.x,location.y,location.z],
                'target_type':target.type_id,'requested_color':'red','semantic_pixel_count':int(mask.sum()),
                'instance_byte_order':order,'instance_bbox':box,'projected_center_uv_depth':center,
                'rgb_color_at_instance_mask':{'dominant':color.dominant_color,'red_ratio':color.score('red'),'ratios':color.ratios},
                'rgb_plain_color_at_instance_mask':{'dominant':plain_color.dominant_color,'red_ratio':plain_color.score('red'),'ratios':plain_color.ratios},
                'depth_at_instance_mask':{'median':float(np.median(depth[mask])),'p05':float(np.percentile(depth[mask],5))},
                'camera_attributes':camera_attributes,
                'camera_transforms':{name:{'location':[p[frame].transform.location.x,p[frame].transform.location.y,p[frame].transform.location.z],
                    'rotation':[p[frame].transform.rotation.pitch,p[frame].transform.rotation.yaw,p[frame].transform.rotation.roll]}
                    for name,p in packets.items()},'common_source_frame':True}
            results.append(result);save_json(out/'results.json',{'scope':'offline controlled visibility diagnosis, not task acceptance',
                'vegetation_object_count':len(vegetation),'building_object_count':len(buildings),'captures':results})
            print(state,json.dumps(result,ensure_ascii=False),flush=True)
        panels=[]
        for r in results:
            name=r['state'];im=cv2.imread(str(out/f'{name}_rgb.png'));x1,y1,x2,y2=r['instance_bbox']
            cv2.rectangle(im,(x1,y1),(x2,y2),(0,0,255),2)
            cv2.putText(im,name+' | red box = instance mask bbox',(12,25),0,.6,(0,0,255),1)
            panels.append(im)
        cv2.imwrite(str(out/'foliage_rgb_comparison.jpg'),np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])]))
        print('Diagnosis complete:',out,flush=True)
    finally:
        if vegetation:world.enable_environment_objects(set(vegetation),True)
        if buildings:world.enable_environment_objects(set(buildings),True)
        for s in sensors:
            try:s.stop();s.destroy()
            except RuntimeError:pass
        if target is not None:
            try:target.destroy()
            except RuntimeError:pass
        world.apply_settings(original)

if __name__=='__main__':main()
