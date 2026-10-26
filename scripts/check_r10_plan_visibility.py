"""Renderer-hook regression: no planned polyline before its logged event."""
from pathlib import Path
import json,sys,runpy,argparse
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest
from build_r10_targeted_report import DEST


def count_plan_color(image,spatial_mask=None):
    roi=image[76:824].astype(int);b,g,r=cv2.split(roi)
    green=(g>145)&(g<205)&(g-b>30)&(g-r>30)
    gold=(b<70)&(g>140)&(g<200)&(r>180)&(r<225)
    selected=green|gold
    if spatial_mask is not None:selected &= spatial_mask[76:824]>0
    return int(np.count_nonzero(selected))


def main():
    p=argparse.ArgumentParser();p.add_argument('--run-dir',type=Path,required=True);a=p.parse_args();run=a.run_dir
    renderer=ROOT/'scripts/build_r10_validation_replay_v3.py';sys.argv=[str(renderer),'--run-dir',str(run)]
    ns=runpy.run_path(str(renderer),run_name='r10_renderer_hook_audit')
    events=ns['route_events'];states=ns['full_states'];times=ns['timestamps']
    if not events:raise RuntimeError('No target plan, no positive plan-timing test')
    first=events[0]['timestamp_s'];origin=min(times.values())
    if first<=origin:raise RuntimeError('No archived pre-plan interval available')
    geo=ns['viewport'](ns['config'],910,930);bg=ns['background'](ns['config'],states,geo)
    images=[];counts=[]
    for t,label in [(first-1e-4,'BEFORE FIRST PLAN EVENT'),(first+1e-4,'AFTER FIRST PLAN EVENT')]:
        frame=max((f for f in times if times[f]<=t),default=min(times))
        image=ns['tracks'](bg.copy(),frame,t,states,geo);counts.append(count_plan_color(image))
        image=cv2.copyMakeBorder(image,35,0,0,0,cv2.BORDER_CONSTANT,value=(250,250,250))
        cv2.putText(image,f'{label} | t={t-origin:.4f}s',(15,24),0,.58,(30,30,30),1,cv2.LINE_AA);images.append(image)
    folder=DEST/'可视化核查';folder.mkdir(exist_ok=True,parents=True)
    target=folder/f'{run.name}_规划出现时序.png';ok,data=cv2.imencode('.png',np.concatenate(images,axis=1))
    if not ok:raise RuntimeError('Image encode failed')
    data.tofile(str(target))
    result=dict(passed=counts[0]==0 and counts[1]>0,run_dir=str(run),
        first_plan_relative_s=first-origin,before_plan_color_pixels=counts[0],after_plan_color_pixels=counts[1],
        renderer_sha256=digest(renderer),review_image=str(target),
        scope='Same renderer hook, 0.1ms before/after logged plan; original patrol remains. Not a global online truth-access audit.')
    save(run/'r10_plan_visibility_audit.json',result);print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
