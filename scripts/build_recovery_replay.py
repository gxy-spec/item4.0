"""Task-specific replay with exact-frame instance audit, including absent targets.

Reuse the established five-panel renderer. Only offline annotations change;
unknown labels are shown as UNVERIFIED rather than declared errors.
"""
import json
from pathlib import Path
import sys
import cv2
import yaml
import numpy as np
from collections import defaultdict
import build_synchronized_multiview_replay as replay
cv2.setNumThreads(2)

run=Path(sys.argv[sys.argv.index('--run-dir')+1])
config=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
present=config['online_acceptance']['expected_target_present']
audit={}
path=run/'task1_candidate_audit.jsonl'
if path.exists():
    for line in path.read_text(encoding='utf-8').splitlines():
        r=json.loads(line);audit[(r['device'],r['frame'],r['candidate_id'])]=r


def overlay(image,candidates,target_xyz,role,sent,selected,frame):
    result=image.copy()
    for c in candidates:
        if not c.get('bbox_xyxy'):continue
        key=(role,frame,str(c['candidate_id']));a=audit.get(key,{})
        active=key in selected or key in sent
        verified=a.get('matched_actor_id') is not None
        # semantic_correct denotes matched object's TASK identity, not agreement
        # between the detector's category and the object's annotated category.
        # The old overlay conflated these, e.g. a car prediction on a target van.
        type_correct=verified and c.get('category')==a.get('matched_category')
        color_correct=verified and c.get('color')==a.get('matched_color')
        correct=type_correct and color_correct
        if active:
            status='SELECT' if key in selected else 'SENT'
            status+='_ATTR_OK' if correct else '_ATTR_ERR' if verified else '_UNVERIFIED'
        else:status='ATTR_OK' if correct else 'ATTR_ERR' if verified else 'UNVERIFIED'
        color=(40,170,40) if correct else (40,40,220) if verified else (130,130,130)
        x1,y1,x2,y2=map(lambda x:int(round(x)),c['bbox_xyxy'])
        cv2.rectangle(result,(x1,y1),(x2,y2),color,2)
        label=f"{c.get('category')} {c.get('color')} {status}"
        cv2.putText(result,label,(max(0,x1),max(12,y1-5)),0,.38,color,1,cv2.LINE_AA)
        if (active or verified and not correct) and a.get('matched_category'):
            cv2.putText(result,f"GT: {a['matched_category']} {a['matched_color']}",(max(0,x1),min(590,y2+14)),0,.38,color,1,cv2.LINE_AA)
    return result


original_background=replay.draw_route_background
full_states=replay.load_states(run/'actors/actor_states.jsonl')
actor_records=replay.read_jsonl(run/'actors/actor_states.jsonl')
timestamps={int(r['frame']):float(r['timestamp']) for r in actor_records}
first_timestamp=min(timestamps.values())
patrol_record=json.loads((run/'planning/ugv_patrol_route_executed.json').read_text(encoding='utf-8'))
patrol=patrol_record['points_xyz']
route_events=[e for e in replay.read_jsonl(run/'planning/target_route_events.jsonl')
              if e.get('event')=='target_route_planned_from_selected_message']
route_events.sort(key=lambda e:e['timestamp_s'])
message_stage={m['message_id']:m.get('evidence_stage','tentative_candidate') for m in replay.read_jsonl(run/'communication/messages.jsonl')}
original_geometry=replay.geometry
def viewport(cfg,width,height):
    points=list(cfg['region']['boundary']['points'])
    points.extend(xy for actors in full_states.values() for role,xy in actors.items() if role in {'ugv','uav','target'})
    points.extend(p[:2] for p in patrol)
    modified={**cfg,'region':{**cfg['region'],'boundary':{'points':points}}}
    geo=original_geometry(modified,width,height)
    geo['scale']=min((width-120)/(geo['x_max']-geo['x_min']), (height-360)/(geo['y_max']-geo['y_min']))
    geo['center_y']=455.0
    return geo


def dashed_line(image,points,geo,color=(70,70,70),width=6):
    # Constant dash length in pixels, continuous through segment boundaries.
    distance=0.0
    for p,q in zip(points,points[1:]):
        a=np.array(replay.world_to_pixel(*p[:2],geo),dtype=float)
        b=np.array(replay.world_to_pixel(*q[:2],geo),dtype=float)
        length=float(np.linalg.norm(b-a))
        if length<.01:continue
        step=0.0
        while step<length:
            on=(distance+step)%20<12
            remaining=(12 if on else 20)-(distance+step)%20
            end=min(length,step+max(.01,remaining))
            if on:cv2.line(image,tuple(np.rint(a+(b-a)*step/length).astype(int)),tuple(np.rint(a+(b-a)*end/length).astype(int)),color,width,cv2.LINE_AA)
            step=end
        distance+=length


def tracks(image,frame,timestamp,states,geo):
    first_replan=route_events[0]['timestamp_s'] if route_events else float('inf')
    visible_plans=[e for e in route_events if e['timestamp_s']<=timestamp and e.get('route_xyz')]
    for event in visible_plans:
        points=[replay.world_to_pixel(*p[:2],geo) for p in event['route_xyz']]
        color=(100,175,100) if message_stage.get(event.get('message_id'))=='reliable_discovery_claim' else (20,170,210)
        if len(points)>1:cv2.polylines(image,[np.array(points,np.int32)],False,color,2,cv2.LINE_AA)
    for role,color in [('ugv',(205,100,20)),('uav',(165,50,145))]:
        rows=[(f,a[role]) for f,a in sorted(states.items()) if f<=frame and role in a]
        points=[replay.world_to_pixel(*xy,geo) for _,xy in rows]
        if len(points)>1:cv2.polylines(image,[np.array(points,np.int32)],False,color,3,cv2.LINE_AA)
        if role=='ugv':
            after=[replay.world_to_pixel(*xy,geo) for f,xy in rows if timestamps[f]>=first_replan]
            if len(after)>1:cv2.polylines(image,[np.array(after,np.int32)],False,(0,130,235),4,cv2.LINE_AA)
    for n,event in enumerate(route_events):
        if event['timestamp_s']>timestamp:continue
        f=min((f for f,a in states.items() if 'ugv' in a),key=lambda f:abs(timestamps[f]-event['timestamp_s']))
        p=replay.world_to_pixel(*states[f]['ugv'],geo)
        cv2.drawMarker(image,p,(0,100,220),cv2.MARKER_DIAMOND,19,3,cv2.LINE_AA)
        cv2.putText(image,f"R{n+1}",(p[0]+12,p[1]-10),0,.55,(0,80,180),2,cv2.LINE_AA)
    cv2.rectangle(image,(0,0),(910,76),(250,250,249),-1)
    cv2.putText(image,'Dashed dark: original patrol | Blue: actual UGV before replan',(18,25),0,.49,(35,45,55),1,cv2.LINE_AA)
    cv2.putText(image,'Orange: actual UGV after replan | Purple: actual UAV',(18,48),0,.49,(35,45,55),1,cv2.LINE_AA)
    cv2.putText(image,'Red star: configured target' if present else 'TARGET ABSENT | non-target reference actors only',(18,70),0,.46,(35,45,55),1,cv2.LINE_AA)
    cv2.rectangle(image,(0,824),(910,930),(250,250,249),-1)
    eligible=[(n,e) for n,e in enumerate(route_events) if e['timestamp_s']<=timestamp]
    if eligible:
        for row,(n,e) in enumerate(eligible[-2:]):
            text=f"R{n+1} t={e['timestamp_s']-first_timestamp:.2f}s {e['message_id']} | planned length={e['route_length_m']:.1f}m"
            cv2.putText(image,text,(18,849+row*22),0,.47,(30,55,70),1,cv2.LINE_AA)
    else:
        cv2.putText(image,'No message-triggered target replan yet',(18,850),0,.5,(30,55,70),1,cv2.LINE_AA)
    plan_label=('Gold: candidate check | Green: reliable target plan' if all(e.get('route_xyz') for e in route_events)
                else 'Some planned polylines unavailable; orange is executed motion.')
    cv2.putText(image,plan_label,(18,900),0,.44,(45,55,70),1,cv2.LINE_AA)
    cv2.putText(image,f"Original patrol: {patrol_record['route_length_m']:.2f}m | XY units: metres",(18,922),0,.45,(45,55,70),1,cv2.LINE_AA)
    return image


def background(cfg,states,geo):
    # Original background provides grid and target only; traces are added causally.
    statecopy={frame:{k:v for k,v in rows.items() if k=='target' and present} for frame,rows in states.items()}
    image=original_background(cfg,statecopy,geo)
    roads_path=Path('E:/CarlaAirData/CityInspection_GOC/perception_online_acceptance_20261021/analysis/map_road_waypoints.json')
    groups=defaultdict(list)
    if roads_path.exists():
        for point in json.loads(roads_path.read_text(encoding='utf-8')):groups[(point['road'],point['section'],point['lane'])].append(point)
        for points in groups.values():
            points.sort(key=lambda p:p['s'])
            for p,q in zip(points,points[1:]):
                if (p['x']-q['x'])**2+(p['y']-q['y'])**2>100:continue
                cv2.line(image,replay.world_to_pixel(p['x'],p['y'],geo),replay.world_to_pixel(q['x'],q['y'],geo),
                    (210,212,215),max(2,int(p['width']*geo['scale']*.8)),cv2.LINE_AA)
        boundary=[replay.world_to_pixel(*xy,geo) for xy in cfg['region']['boundary']['points']]
        cv2.polylines(image,[np.array(boundary,dtype=np.int32)],True,(145,145,145),2,cv2.LINE_AA)
        if present:
            target=next((a['target'] for a in states.values() if 'target' in a),None)
            if target:cv2.drawMarker(image,replay.world_to_pixel(*target,geo),(30,30,210),cv2.MARKER_STAR,22,3)
    dashed_line(image,patrol,geo)
    cv2.putText(image,'Green: exact planned target route | Orange: executed after replan',(20,1020),0,.42,(40,80,40),1,cv2.LINE_AA)
    for point,name in [(patrol[0],'Patrol start'),(patrol[-1],'Patrol end')]:
        p=replay.world_to_pixel(*point[:2],geo)
        cv2.circle(image,p,7,(60,60,60),2,cv2.LINE_AA)
        cv2.putText(image,name,(p[0]+10,p[1]+20),0,.44,(35,45,55),1,cv2.LINE_AA)
    full=tracks(image.copy(),max(states),float('inf'),states,geo)
    if 'ugv' in states[max(states)]:
        p=replay.world_to_pixel(*states[max(states)]['ugv'],geo)
        cv2.drawMarker(full,p,(0,130,235) if route_events else (205,100,20),cv2.MARKER_SQUARE,15,2,cv2.LINE_AA)
        cv2.putText(full,'UGV final',(p[0]+10,p[1]-10),0,.48,(30,55,70),1,cv2.LINE_AA)
    cv2.imwrite(str(run/'preview/recovery_trajectory_map.png'),full)
    evidence={'renderer_version':7,'original_patrol_source':'planning/ugv_patrol_route_executed.json',
        'original_patrol_length_m':patrol_record['route_length_m'],'original_patrol_points':len(patrol),
        'target_replanning_events':route_events,'replanned_target_polyline_archived':all(bool(e.get('route_xyz')) for e in route_events),
        'orange_line':'actual UGV motion at/after first target_route_planned_from_selected_message; NOT planned target route',
        'timestamp_origin_s':first_timestamp,'video_tracks':'only frames at/before displayed simulation timestamp',
        'simulation_rerun':False}
    (run/'preview/recovery_route_overlay_evidence.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    return image


original_markers=replay.draw_actor_markers
def markers(image,states,geo):
    rows=dict(states)
    if not present and 'target' in rows:
        rows['vehicle_reference']=rows.pop('target')
    original_markers(image,rows,geo)


replay.overlay_candidates=overlay
replay.draw_route_background=background
replay.draw_actor_markers=markers
replay.geometry=viewport
replay.draw_map_for_frame=tracks
original_label=replay.put_label
def label(canvas,text,x,y,scale=.75):
    if text.startswith('GLOBAL TRAJECTORY'):text='ORIGINAL PATROL + EXECUTED TRACKS UP TO CURRENT FRAME'
    if not present and text.startswith('UGV '):text=text.replace('| target ','| reference actor ')
    original_label(canvas,text,x,y,scale)
    if text.startswith(config['experiment_id']):
        cv2.rectangle(canvas,(20,5),(1900,42),(242,245,248),-1)
        goal=config['online_acceptance']
        banner=f"PERCEPTION RECOVERY | {goal['task_color']} {goal['task_category']} | TARGET {'PRESENT' if present else 'ABSENT'} | AUTO LABEL AUDIT: RGB visibility unverified"
        cv2.putText(canvas,banner,(35,31),0,.66,(30,40,50),1,cv2.LINE_AA)
replay.put_label=label
if '--output' not in sys.argv:sys.argv+=['--output',str(run/'preview/recovery_five_view.mp4')]
if __name__=='__main__':
    code=replay.main()
    output=Path(sys.argv[sys.argv.index('--output')+1]);manifest=output.with_suffix('.json')
    data=json.loads(manifest.read_text(encoding='utf-8'))
    data.update(renderer_version=7,route_overlay='original patrol + timestamp-causal actual tracks + archived exact target route polylines and replanning events',label_scope='ATTR_OK/ATTR_ERR compares predicted type AND semantic paint color with associated known actor labels; object task-identity is separate; RGB visibility not independently verified; not task success')
    manifest.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    raise SystemExit(code)
