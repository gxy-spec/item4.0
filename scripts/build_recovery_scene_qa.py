"""Raw scene samples for controls with no commit/completion event.

Also makes full UAV visibility inspectable without relying on target crops.
"""
import sys,json,hashlib,argparse
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save


def build(run):
    dest=run/'preview/recovery_scene_samples.jpg';receipt=dest.with_suffix('.json')
    if dest.exists() and receipt.exists():return dest
    bydevice={d:{int(p.stem):p for p in (run/f'sensors/{d}/rgb').glob('*.png')} for d in ['uav','ugv']}
    frames=sorted(set(bydevice['uav'])&set(bydevice['ugv']))
    if not frames:return None
    selected=[frames[0],frames[len(frames)//2],frames[-1]];canvas=np.full((706,1200,3),248,np.uint8);sources=[]
    for row,d in enumerate(['uav','ugv']):
        for col,f in enumerate(selected):
            p=bydevice[d][f];im=cv2.imread(str(p))
            if im is None:raise RuntimeError('Cannot decode '+str(p))
            x=col*400;y=row*340;cv2.putText(canvas,f'{d.upper()} | frame {f} | '+['START','MIDDLE','END'][col],(x+8,y+24),0,.52,(30,40,50),1,cv2.LINE_AA)
            canvas[y+34:y+334,x:x+400]=cv2.resize(im,(400,300))
            sources.append(dict(device=d,frame=f,image=str(p),decoded_pixels_sha256=hashlib.sha256(im.tobytes()).hexdigest()))
    cv2.putText(canvas,'Raw RGB samples; absence of a commit/completion event is not replaced with a success frame.',(8,699),0,.47,(30,40,50),1,cv2.LINE_AA)
    dest.parent.mkdir(parents=True,exist_ok=True);cv2.imwrite(str(dest),canvas,[cv2.IMWRITE_JPEG_QUALITY,95]);save(receipt,dict(sources=sources,scope='Three time samples, exact paired UAV/UGV frames; illustrative RGB visibility, not exhaustive frame-by-frame visibility certification.'))
    return dest


def main():
    p=argparse.ArgumentParser();p.add_argument('--cohort',default='formal');a=p.parse_args();index=OUT/f'{a.cohort}_progress.json'
    data=json.loads(index.read_text(encoding='utf-8')) if index.exists() else {}
    for r in data.values():
        if r.get('report_complete') and r.get('run_dir'):build(Path(r['run_dir']))
    print('SCENE SAMPLES',a.cohort,flush=True)


if __name__=='__main__':main()
