"""Video decode/frame-rate checks and selected contact sheets for manual review."""
from pathlib import Path
import sys,json,argparse
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save
from build_r10_targeted_report import DEST


def main():
    p=argparse.ArgumentParser();p.add_argument('--contact-case',action='append',default=[]);args=p.parse_args()
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision'];records=[]
    for cohort in ['safety','acceptance','diagnostics']:
        index=OUT/f'{cohort}_development_{rev}_progress.json'
        if not index.exists():continue
        for item in json.loads(index.read_text(encoding='utf-8')).values():
            if not item.get('report_complete'):continue
            run=Path(item['run_dir']);video=run/'preview/r10_v3_five_view.mp4';meta=video.with_suffix('.json')
            h264=run/'preview/r10_v3_five_view_h264.mp4'
            if h264.exists() and h264.with_suffix('.json').exists():video=h264;meta=h264.with_suffix('.json')
            if not meta.exists():
                records.append(dict(case=item['case'],run_dir=str(run),passed=False,error='Video not rendered'));continue
            m=json.loads(meta.read_text(encoding='utf-8'));cap=cv2.VideoCapture(str(video))
            n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS);frames=[];decoded=True
            for i in [0,n//2,max(0,n-1)]:
                cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,frame=cap.read();decoded=decoded and ok
                if ok:frames.append(frame)
            passed=bool(decoded and n==m['frame_count'] and abs(fps*m['median_persisted_sample_interval_seconds']-1)<1e-3)
            record=dict(case=item['case'],variant=item.get('variant'),group=item.get('group'),run_dir=str(run),
                passed=passed,decoded_first_middle_last=decoded,video_frame_count=n,metadata_frame_count=m['frame_count'],
                actual_fps=fps,simulated_sample_interval_s=m['median_persisted_sample_interval_seconds'],
                true_time_playback=abs(fps*m['median_persisted_sample_interval_seconds']-1)<1e-3)
            if item['case'] in args.contact_case:
                contact=[]
                for frame,label in zip(frames,['FIRST FRAME','MIDDLE FRAME','LAST FRAME']):
                    thumb=cv2.resize(frame,(960,540));card=np.full((570,960,3),250,np.uint8);card[30:]=thumb
                    cv2.putText(card,label,(15,23),0,.6,(30,30,30),1,cv2.LINE_AA);contact.append(card)
                if len(contact)==3:
                    canvas=np.full((1140,1920,3),255,np.uint8)
                    canvas[:570,:960]=contact[0];canvas[:570,960:]=contact[1];canvas[570:,:960]=contact[2]
                    cv2.putText(canvas,f"{item.get('variant','')} {item['case']} {item.get('group','')} | Review only; 3 sampled frames",(980,650),0,.6,(30,30,30),1,cv2.LINE_AA)
                    cv2.putText(canvas,'Full original patrol, exact-frame camera views and causal plan timing',(980,690),0,.5,(30,30,30),1,cv2.LINE_AA)
                    folder=DEST/'可视化核查';folder.mkdir(exist_ok=True,parents=True)
                    target=folder/f"{item.get('variant','')}_{item['case']}_{item.get('group','')}_三帧核查.png"
                    encoded_ok,encoded=cv2.imencode('.png',canvas)
                    if not encoded_ok:raise RuntimeError('Could not encode review image')
                    encoded.tofile(str(target))
                    if not target.is_file():raise RuntimeError('Missing written review image')
                    record['contact_sheet']=str(target)
            cap.release();records.append(record)
    result=dict(revision=rev,records=records,passed=all(r['passed'] for r in records),
        scope='Automated first/middle/last decoding, frame count and real-time playback checks. Human visual inspection of selected contact sheets is separate.')
    save(OUT/'analysis/media_technical_check.json',result);print(json.dumps(dict(passed=result['passed'],records=len(records),failures=[r for r in records if not r['passed']]),ensure_ascii=False))


if __name__=='__main__':main()
