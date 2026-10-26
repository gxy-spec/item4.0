"""Compare corresponding saved sensor frames before physical divergence."""
from pathlib import Path
import json,sys,math
import numpy as np
import cv2
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save,digest
from build_r10_targeted_report import DEST


def main():
    pair=json.loads((OUT/'analysis/N3_A_pair_audit.json').read_text(encoding='utf-8'))['runs']
    streams={v:list(rows(Path(p)/'sensors/ugv/metadata/frames.jsonl')) for v,p in pair.items()}
    streams={v:[a for a in s if a.get('sensor_files_saved')] for v,s in streams.items()}
    result=[];cards=[]
    for t in [0.,1.,5.,7.,7.25]:
        selected={v:min(s,key=lambda a:abs(a['timestamp']-s[0]['timestamp']-t)) for v,s in streams.items()}
        images={v:cv2.imread(a['rgb_path']) for v,a in selected.items()}
        depths={v:cv2.imread(a['depth_raw_path'],cv2.IMREAD_UNCHANGED) for v,a in selected.items()}
        rgbdiff=np.abs(images['legacy'].astype(float)-images['upgraded'].astype(float))
        depdiff=np.abs(depths['legacy'].astype(float)-depths['upgraded'].astype(float))
        transforms={v:np.array(a['sensor_transform']['location']+a['sensor_transform']['rotation']) for v,a in selected.items()}
        r=dict(relative_s=t,actual_times={v:a['timestamp']-streams[v][0]['timestamp'] for v,a in selected.items()},
            rgb_mean_absolute_channel_difference=float(rgbdiff.mean()),rgb_different_pixel_fraction=float(np.mean(np.any(rgbdiff!=0,axis=2))),
            depth_raw_different_pixel_fraction=float(np.mean(np.any(depdiff!=0,axis=2))),
            sensor_transform_max_absolute_component_difference=float(np.max(np.abs(transforms['legacy']-transforms['upgraded']))),
            source_rgb_paths={v:a['rgb_path'] for v,a in selected.items()},source_rgb_sha256={v:digest(a['rgb_path']) for v,a in selected.items()})
        result.append(r)
        if t==7.25:
            for v in ['legacy','upgraded']:
                im=images[v];canvas=np.full((650,800,3),255,np.uint8);canvas[30:630]=im
                cv2.putText(canvas,f'{v}: t=7.25s BEFORE own XY divergence',(15,22),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,0),1,cv2.LINE_AA)
                cards.append(canvas)
    target=DEST/'可视化核查/N3_分歧前同时间输入对照.png';target.parent.mkdir(parents=True,exist_ok=True)
    cv2.imencode('.png',np.hstack(cards))[1].tofile(str(target))
    save(OUT/'analysis/N3_A_sensor_input_comparison.json',dict(records=result,contact_sheet=str(target),
        scope='Observed RGB/depth/sensor-pose differences at matched simulation times, before own XY divergence. Does not by itself identify renderer, animation, detector or GPU as unique cause.'))
    print(json.dumps([{k:r[k] for k in ['relative_s','rgb_mean_absolute_channel_difference','rgb_different_pixel_fraction','depth_raw_different_pixel_fraction','sensor_transform_max_absolute_component_difference']} for r in result]))


if __name__=='__main__':main()
