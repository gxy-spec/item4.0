"""Finalize completed acquisitions serially while later simulations run.

Only immutable, completed runs are read/recompressed. No simulator client,
GPU inference, scheduler or external notifications are used here.
"""
import sys,json,time,subprocess,argparse,hashlib
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
from optimize_recovery_pngs import compact_run
from analyze_recovery_campaign import assess_episode
cv2.setNumThreads(2)


def evidence_board(run,result):
    audit=rows(run/'task1_candidate_audit.jsonl');lookup={(a['device'],a['candidate_id']):a for a in audit}
    events=[]
    commit=next(iter(result.get('commits',[])),None)
    events.append(('FIRST COMMIT',commit.get('evidence') if commit else None))
    monitor=result.get('sensor_business_monitor',{})
    for title,event_name in [('LOCAL CONFIRMATION','sensor_local_confirmation'),('STOPPED APPROACH','sensor_safe_approach_hold_completed')]:
        event=next((e for e in monitor.get('events',[]) if e.get('event')==event_name),None)
        events.append((title,lookup.get(('ugv',event.get('candidate_id'))) if event else None))
    final=next((a for a in reversed(audit) if a['device']=='ugv' and a.get('semantic_correct')),None)
    events.append(('LAST MATCHED LOCAL VIEW',final))
    canvas=np.full((960,1280,3),248,np.uint8)
    for n,(title,a) in enumerate(events):
        x0=(n%2)*640;y0=(n//2)*480;panel=np.full((480,640,3),248,np.uint8)
        cv2.putText(panel,title,(10,24),0,.6,(25,35,50),2,cv2.LINE_AA)
        if a:
            image=cv2.imread(a['image'])
            if image is None:raise RuntimeError('Evidence RGB missing '+a['image'])
            box=a['bbox'];x1,y1,x2,y2=map(int,box)
            correct=a.get('category')==a.get('matched_category') and a.get('color')==a.get('matched_color')
            color=(40,150,40) if correct else (40,60,220)
            full=image.copy();cv2.rectangle(full,(x1,y1),(x2,y2),color,2);panel[42:378,8:456]=cv2.resize(full,(448,336))
            pad=max(4,int(min(x2-x1,y2-y1)*.1));crop=image[max(0,y1-pad):min(600,y2+pad),max(0,x1-pad):min(800,x2+pad)]
            if crop.size:
                scale=min(176/crop.shape[1],280/crop.shape[0]);crop=cv2.resize(crop,(max(1,int(crop.shape[1]*scale)),max(1,int(crop.shape[0]*scale))));panel[54:54+crop.shape[0],460:460+crop.shape[1]]=crop
            texts=[f"{a['device'].upper()} frame {a['frame']} | IoU={a['iou']:.3f}",
                f"PRED {a['category']}/{a['color']} | GT {a.get('matched_category')}/{a.get('matched_color')}",
                f"Configured-target match: {a.get('semantic_correct')} | GT role: {a.get('matched_role')}",
                'GT association/paint labels are offline evaluation only.']
            for j,text in enumerate(texts):cv2.putText(panel,text,(8,399+j*21),0,.43,(30,40,55),1,cv2.LINE_AA)
        else:
            cv2.putText(panel,'No such event recorded',(18,110),0,.65,(90,90,110),1,cv2.LINE_AA)
            cv2.putText(panel,'Absence is retained; no success frame substituted.',(18,145),0,.45,(90,90,110),1,cv2.LINE_AA)
        canvas[y0:y0+480,x0:x0+640]=panel
    path=run/'preview/recovery_key_evidence.jpg';cv2.imwrite(str(path),canvas,[cv2.IMWRITE_JPEG_QUALITY,95]);return path


def verify_video(path):
    cap=cv2.VideoCapture(str(path));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=float(cap.get(cv2.CAP_PROP_FPS));width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    checks=[]
    for index in [0,max(0,count//2),max(0,count-1)]:
        cap.set(cv2.CAP_PROP_POS_FRAMES,index);ok,im=cap.read();checks.append(bool(ok and im is not None and np.std(im)>5))
    cap.release()
    if not count or not all(checks) or (width,height)!=(1920,1080):raise RuntimeError('Video decode/size verification failed '+str(path))
    return dict(frame_count=count,fps=fps,width=width,height=height,first_middle_last_decode_passed=all(checks))


def finalize(item,name):
    run=Path(item['run_dir']);receipt=run/'preview/recovery_artifacts_verified.json'
    if receipt.exists():return False
    compact_run(run)
    evaluation=run/'recovery_evaluation.json'
    result=json.loads(evaluation.read_text(encoding='utf-8')) if evaluation.exists() else assess_episode({**item,'case':name,'runner_wall_seconds':item.get('wall_seconds')})
    if result.get('semantic_verdict')=='NOT_EVALUABLE':raise RuntimeError('Completed acquisition not evaluable '+name)
    board=evidence_board(run,result);log=run/'preview/recovery_render.log'
    with log.open('w',encoding='utf-8') as stream:
        subprocess.run([sys.executable,'-u',str(ROOT/'scripts/build_recovery_replay.py'),'--run-dir',str(run),'--fps','4'],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    video=run/'preview/recovery_five_view.mp4';before=verify_video(video);temporary=video.with_name('recovery_five_view_h264.tmp.mp4')
    with (run/'preview/recovery_h264.log').open('w',encoding='utf-8') as stream:
        subprocess.run([str(OUT/'tools/ffmpeg.exe'),'-hide_banner','-y','-i',str(video),'-c:v','libx264','-preset','fast','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart','-threads','2','-an',str(temporary)],stdout=stream,stderr=subprocess.STDOUT,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
    after=verify_video(temporary)
    if before['frame_count']!=after['frame_count'] or abs(before['fps']-after['fps'])>.001:raise RuntimeError('Transcode altered frame count/FPS')
    temporary.replace(video)
    metadata=json.loads(video.with_suffix('.json').read_text(encoding='utf-8'));metadata.update(codec='H264/yuv420p,faststart',video_sha256=digest(video));save(video.with_suffix('.json'),metadata)
    save(receipt,dict(video=str(video),video_sha256=digest(video),trajectory_map=str(run/'preview/recovery_trajectory_map.png'),evidence_board=str(board),
        verification=after,renderer_sha256=digest(ROOT/'scripts/build_recovery_replay.py'),base_renderer_sha256=digest(ROOT/'scripts/build_synchronized_multiview_replay.py'),
        map_source='Original patrol JSON + exact archived planned routes + actual UGV/UAV trajectories. Video uses only past/current actual trajectory samples.',
        visual_audit='Mechanical decoding/overlay verification; human/assistant RGB support review is separate. Labels compare raw type and semantic paint, not task identity alone.'))
    print('ARTIFACTS VERIFIED',name,after['frame_count'],flush=True)
    return True


def main():
    p=argparse.ArgumentParser();p.add_argument('--cohort',default='formal');p.add_argument('--expected',type=int,default=72);p.add_argument('--watch',action='store_true');a=p.parse_args()
    while True:
        path=OUT/f'{a.cohort}_progress.json'
        try:progress=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
        except json.JSONDecodeError:time.sleep(.2);continue
        completed=[(name,r) for name,r in progress.items() if r.get('report_complete') and r.get('run_dir')]
        for name,item in completed:
            created=finalize(item,name)
            if created and a.cohort=='formal':
                subprocess.run([sys.executable,'-u',str(ROOT/'scripts/package_recovery_release.py')],cwd=ROOT,check=True,
                    creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if not a.watch or len(completed)>=a.expected:break
        time.sleep(10)
    print('ALL REQUESTED ARTIFACTS VERIFIED',a.cohort,len(completed),flush=True)


if __name__=='__main__':main()
