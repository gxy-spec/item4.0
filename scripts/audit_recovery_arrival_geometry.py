"""Explain reference-point versus observed surface gaps, offline only."""
import math,json,sys,csv
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def main():
    progress=json.loads((OUT/'acceptance_r9_progress.json').read_text(encoding='utf-8'));records=[]
    for case,p in progress.items():
        run=Path(p['run_dir']);r=json.loads((run/'recovery_evaluation.json').read_text(encoding='utf-8'))
        if not r.get('sensor_safe_approach_audited'):continue
        c=r['completion_candidate_audit'][0];states={a['role']:a for a in rows(run/'actors/actor_states.jsonl') if a['frame']==c['frame']}
        own=states['ugv'];target=states['target'];meta=next(a for a in rows(run/'sensors/ugv/metadata/frames.jsonl') if a['frame']==c['frame'])
        event=next(e for e in r['sensor_business_monitor']['events'] if e['event']=='sensor_safe_approach_hold_completed')
        f=[math.cos(math.radians(own['yaw'])),math.sin(math.radians(own['yaw']))];cam=meta['sensor_transform']['location'];surface=c['world_xyz']
        def dot(x,y):return (x[0]-y[0])*f[0]+(x[1]-y[1])*f[1]
        origin=[own['x'],own['y']];reference=[target['x'],target['y']]
        offset=dot(cam,origin);surface_to_reference=dot(reference,surface);horizontal_surface=dot(surface,cam)
        estimate=offset+surface_to_reference+3*horizontal_surface/event['depth_m']
        records.append(dict(case=case,actual_reference_xy_distance_m=math.dist(origin,reference),optical_surface_depth_m=event['depth_m'],
            camera_forward_offset_m=offset,surface_to_reference_forward_offset_m=surface_to_reference,
            observed_camera_surface_forward_gap_m=horizontal_surface,
            reconstructed_reference_forward_gap_m=offset+surface_to_reference+horizontal_surface,
            estimated_reference_forward_gap_at_3m_optical_surface_m=estimate,
            historical_upper7m_conflicts_with_3m_surface_in_this_pose=estimate>7,
            interpretation='Pose-local straight-approach estimate, not a universal vehicle-class bound. Actor reference/pivot is the historical target point; not asserted to be geometric body centre.'))
    DELIVERY.mkdir(parents=True,exist_ok=True)
    with (DELIVERY/'停车距离几何核查.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    save(OUT/'analysis/arrival_geometry_audit.json',dict(records=records,
        rule='At each audited completion, decompose actual forward reference gap = own-centre-to-camera offset + camera-to-observed-surface gap + surface-to-target-reference offset. Estimate reference gap for unchanged pose/surface with optical depth3m. Offline actor data never used by live stopping controller.'))
    lines=['# 停车距离口径核查','','本轮旧目标参考点距离3–7米与新鲜RGB-D目标表面深度3–8米分别保留，不能把两者的通过情况混写。旧字段用CARLA actor位置作参考，严格说并不总等于几何车身中心。','',
        '|任务|停车时目标参考点距/m|相机光轴表面深度/m|相机前置偏移/m|表面至目标参考点前向偏移/m|同一姿态表面深度3m时的参考点距估计/m|','|---|---:|---:|---:|---:|---:|']
    for r in records:lines.append(f"|{r['case']}|{r['actual_reference_xy_distance_m']:.3f}|{r['optical_surface_depth_m']:.3f}|{r['camera_forward_offset_m']:.3f}|{r['surface_to_reference_forward_offset_m']:.3f}|{r['estimated_reference_forward_gap_at_3m_optical_surface_m']:.3f}|")
    lines+=['','以红色Sprinter为例，UGV相机位于车辆参考点前方约1.5米，观测的厢式车后表面又位于目标参考点后方约2.9米；因此光轴表面深度3米通常已对应约7.4米参考点距。在该近似正向姿态下，表面至少3米与参考点不超过7米存在冲突。客车/消防车的偏移还会不同，不能用同一参考点阈值代表统一车表面安全间距。','',
        '这不改变旧指标的事实：若严格按原3–7米指标验收，本轮该项仍判未通过。也不能只缩小路线终点容差、逼近目标来让统计通过；跨车型时应先明确采用传感器表面间距、车体净间距还是目标参考点距离，再以一致的物理定义控制和验收。当前R9执行已冻结的表面深度制动，不使用目标真值车长或位置。','',
        '上表只是在已完成姿态附近的前向几何分解与局部估计，不能作为所有角度、所有车辆的通用下界。原始分解和真实帧号来源可由各回合recovery_evaluation.json中的completion_candidate_audit追溯。']
    (DELIVERY/'停车距离口径说明.md').write_text('\n'.join(lines)+'\n',encoding='utf-8');print('ARRIVAL GEOMETRY',len(records),flush=True)


if __name__=='__main__':main()
