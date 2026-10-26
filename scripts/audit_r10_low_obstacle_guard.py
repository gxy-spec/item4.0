"""Synthetic calibrated depth regression, explicitly not a physical CARLA trial."""
from pathlib import Path
import json,sys,math
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
sys.path.insert(0,str(Path(__file__).resolve().parent))
from safety.rgbd_obstacle_guard import RGBDObstacleGuard,ObstacleGuardConfig
from safety.rgbd_hazard_perception import vehicle_depth_support
from recovery_stage import OUT,save,digest,ROOT


def main():
    h,w=600,800;f=w/(2*math.tan(math.radians(100)/2));cy=(h-1)/2
    ground=np.broadcast_to(2*f/np.maximum(.001,np.arange(h)[:,None]-cy),(h,w)).copy()
    ground=np.minimum(ground,1000.).astype(np.float32)
    pose=np.eye(4);pose[2,3]=2.
    # A 20cm high, 3m distant front face: top/bottom rays calculated from height.
    top=math.ceil(cy+(2-.20)*f/3);bottom=math.floor(cy+2*f/3)
    low=ground.copy();low[top:bottom,330:470]=3.
    keep,geometry=vehicle_depth_support(low,[330,top,470,bottom],pose,0.,f)
    config=ObstacleGuardConfig(pixel_stride=2)
    clear=RGBDObstacleGuard(config).observe(ground,1,0.,0.,pose,route_vehicle_xy=np.array([[0,0],[15,0]]))
    blocked=RGBDObstacleGuard(config).observe(low,1,0.,0.,pose,route_vehicle_xy=np.array([[0,0],[15,0]]))
    unknown=RGBDObstacleGuard(config).observe(np.full_like(low,np.nan),1,0.,0.,pose)
    result=dict(passed=clear['state']=='CLEAR' and blocked['state']=='STOP' and unknown['state']=='SENSOR_STALE',
        synthetic_obstacle_height_m=.20,synthetic_front_face_distance_m=3.,
        semantic_vehicle_filter_kept=keep,semantic_geometry=geometry,
        independent_depth_guard=dict(flat_ground=clear,low_obstacle=blocked,invalid_depth=unknown),
        scope='Synthetic pinhole depth only; not a CARLA low-object closed-loop experiment, and not a claim about objects below configured 0.12m height',
        source_sha256=digest(ROOT/'src/safety/rgbd_obstacle_guard.py'))
    save(OUT/'analysis/low_obstacle_depth_regression.json',result);print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
