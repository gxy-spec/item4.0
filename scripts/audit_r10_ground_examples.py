"""Re-evaluate exact retained known false-vehicle RGB-D frames, no simulation."""
from pathlib import Path
import json,sys,math
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from src.safety.rgbd_hazard_perception import vehicle_depth_support
from src.perception.candidate_pipeline import transform_matrix
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows


def main():
    p=Path('C:/Users/Administrator/Documents/ChatGPT/课题1/实验结果/感知升级与36组复验_20261021_v2/诊断证据/safety_false_vehicle_examples.json')
    result=[]
    for e in json.loads(p.read_text(encoding='utf-8')):
        image=Path(e['source']);run=image.parents[3];frame=int(image.stem)
        meta=next(m for m in rows(run/'sensors/ugv/metadata/frames.jsonl') if m['frame']==frame)
        own=next(a for a in rows(run/'actors/actor_states.jsonl') if a['frame']==frame and a['role']=='ugv')
        bgr=cv2.imread(str(run/'sensors/ugv/depth_raw'/image.name)).astype(np.float32)
        depth=(bgr[:,:,2]+bgr[:,:,1]*256+bgr[:,:,0]*65536)/16777215*1000
        keep,g=vehicle_depth_support(depth,json.loads(e['hazard']['bbox_xyxy']),transform_matrix(meta['sensor_transform']),own['z'],800/(2*math.tan(math.radians(100)/2)))
        result.append(dict(case=e['case'],frame=frame,kept=keep,geometry=g,source=str(image)))
    save(OUT/'diagnostics/known_ground_vehicle_filter.json',result)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
