"""H.264 derivatives with unchanged frames/FPS; preserve source MP4V videos."""
from pathlib import Path
import sys,json,subprocess
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest


def verify(path):
    cap=cv2.VideoCapture(str(path));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
    width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH));height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT));ok=True
    for i in [0,n//2,max(0,n-1)]:
        cap.set(cv2.CAP_PROP_POS_FRAMES,i);decoded,im=cap.read();ok=ok and decoded and im is not None and np.std(im)>5
    cap.release()
    if not ok or n==0 or (width,height)!=(1920,1080):raise RuntimeError('Video decode/size failed: '+str(path))
    return dict(frames=n,fps=fps,width=width,height=height,decoded_first_middle_last=True)


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));rev=scope['active_validation_revision'];runs=[]
    for cohort in ['safety','acceptance','diagnostics']:
        p=OUT/f'{cohort}_development_{rev}_progress.json'
        if p.exists():runs.extend(Path(i['run_dir']) for i in json.loads(p.read_text(encoding='utf-8')).values() if i.get('report_complete'))
    retained=OUT/'analysis/retained_failure_media.json'
    if retained.exists():runs.extend(Path(i['run_dir']) for i in json.loads(retained.read_text(encoding='utf-8')))
    ffmpeg=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2/tools/ffmpeg.exe')
    if not ffmpeg.is_file():raise RuntimeError('Verified existing FFmpeg binary missing')
    toolsha=digest(ffmpeg);records=[]
    for run in dict.fromkeys(runs):
        source=run/'preview/r10_v3_five_view.mp4';srcmeta=source.with_suffix('.json')
        if not srcmeta.exists():continue
        meta=json.loads(srcmeta.read_text(encoding='utf-8'))
        if meta.get('renderer_version')!=11:continue
        target=run/'preview/r10_v3_five_view_h264.mp4';targetmeta=target.with_suffix('.json')
        if target.is_file() and targetmeta.is_file():
            records.append(json.loads(targetmeta.read_text(encoding='utf-8')));continue
        if target.exists():raise RuntimeError('Untracked target exists; do not overwrite '+str(target))
        before=verify(source);temporary=target.with_name('r10_v3_five_view_h264_building.mp4')
        with (run/'preview/r10_v3_h264.log').open('w',encoding='utf-8') as f:
            subprocess.run([str(ffmpeg),'-hide_banner','-y','-i',str(source),'-c:v','libx264','-preset','fast','-crf','18',
                '-pix_fmt','yuv420p','-movflags','+faststart','-threads','2','-an',str(temporary)],
                stdout=f,stderr=subprocess.STDOUT,check=True,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        after=verify(temporary)
        if before['frames']!=after['frames'] or abs(before['fps']-after['fps'])>.001:raise RuntimeError('Transcode altered frame count/FPS')
        temporary.replace(target)
        result={**meta,'output_video':str(target),'codec':'H264/yuv420p,faststart','source_video':str(source),
            'source_video_sha256':digest(source),'video_sha256':digest(target),'ffmpeg_sha256':toolsha,
            'transcode_verification':after,'source_preserved':True}
        save(targetmeta,result);records.append(result);print('H264 VERIFIED',run.name,after['frames'],flush=True)
    save(OUT/'analysis/h264_replay_derivatives.json',dict(records=records,scope='Format derivatives only, source MP4V and simulation data preserved'))
    print('H264 DERIVATIVES',len(records),flush=True)


if __name__=='__main__':main()
