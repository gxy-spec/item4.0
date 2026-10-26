"""User-identified W2 review, preserving the earlier finding of no traced truth leak."""
from pathlib import Path
import sys,json,csv,runpy
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,rows
from build_r10_targeted_report import DEST
from check_r10_plan_visibility import count_plan_color


def main():
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision']
    progress=json.loads((OUT/f'diagnostics_development_{rev}_progress.json').read_text(encoding='utf-8'))
    item=next(i for i in progress.values() if i['case']=='W2');run=Path(item['run_dir'])
    e=json.loads((run/'r10_evaluation.json').read_text(encoding='utf-8'))
    event=next(r for r in rows(run/'planning/target_route_events.jsonl') if r['event']=='target_route_planned_from_selected_message')
    saved=[r for r in csv.DictReader((run/'synchronization/frame_index.csv').open(encoding='utf-8-sig')) if int(r.get('sensor_files_saved','1'))]
    origin=float(saved[0]['timestamp']);before=max(i for i,r in enumerate(saved) if float(r['timestamp'])<event['timestamp_s'])
    after=min(i for i,r in enumerate(saved) if float(r['timestamp'])>=event['timestamp_s'])
    video=run/'preview/r10_v3_five_view_h264.mp4'
    if not video.exists():video=run/'preview/r10_v3_five_view.mp4'
    renderer=ROOT/'scripts/build_r10_validation_replay_v3.py'
    sys.argv=[str(renderer),'--run-dir',str(run)]
    ns=runpy.run_path(str(renderer),run_name='w2_actual_video_geometry')
    geo=ns['viewport'](ns['config'],910,930)
    plan_mask=np.zeros((930,910),np.uint8)
    points=np.array([ns['replay'].world_to_pixel(*p[:2],geo) for p in event['route_xyz']],np.int32)
    cv2.polylines(plan_mask,[points],False,255,7)
    cap=cv2.VideoCapture(str(video));snapshots=[];counts=[];records=[]
    for i,label in [(before,'LAST SAVED FRAME BEFORE PLAN'),(after,'FIRST SAVED FRAME AFTER PLAN')]:
        cap.set(cv2.CAP_PROP_POS_FRAMES,i);ok,image=cap.read()
        if not ok:raise RuntimeError('W2 video frame not decodable')
        panel=image[95:1025,25:935].copy();mask=plan_mask.copy()
        # Actual video also has orange vehicle and green pedestrian symbols.
        # Exclude their fixed-size marker neighbourhoods, and inspect only the
        # known plan corridor; global colour counts confound actors with plans.
        for xy in ns['full_states'].get(int(saved[i]['frame']),{}).values():
            cv2.circle(mask,ns['replay'].world_to_pixel(*xy,geo),22,0,-1)
        counts.append(count_plan_color(panel,mask));t=float(saved[i]['timestamp'])-origin
        card=cv2.copyMakeBorder(panel,35,0,0,0,cv2.BORDER_CONSTANT,value=(250,250,250))
        cv2.putText(card,f'{label} | t={t:.2f}s',(15,24),0,.55,(30,30,30),1,cv2.LINE_AA);snapshots.append(card)
        records.append(dict(video_frame=i,source_frame=int(saved[i]['frame']),timestamp_s=t,planned_line_pixels=counts[-1]))
    cap.release();folder=DEST/'可视化核查';folder.mkdir(exist_ok=True,parents=True)
    target=folder/'W2_实际回放规划前后.png';ok,data=cv2.imencode('.png',np.concatenate(snapshots,axis=1))
    if not ok:raise RuntimeError('PNG encode failed')
    data.tofile(str(target))
    previous=run/'r10_actual_video_plan_timing.json'
    archive=run/'r10_actual_video_plan_timing_initial_color_only.json'
    if previous.exists() and not archive.exists():
        old=json.loads(previous.read_text(encoding='utf-8'))
        if old.get('passed') is False:
            old['invalidated_diagnostic']='Global color counts included green pedestrian/orange vehicle symbols; not evidence of an early planned line.'
            save(archive,old)
    result=dict(passed=counts[0]==0 and counts[1]>0,video=str(video),records=records,
        pixel_metric='Gold/green pixels only in 7px planned-route corridor, excluding 22px actor marker neighbourhoods',
        scope='Two adjacent actual encoded video samples around first W2 plan, not all video frames or a global simulator truth-access proof')
    save(run/'r10_actual_video_plan_timing.json',result)
    ground=min(o['timestamp_s'] for o in e['local_stage_observations'] if o['identity_correct'])
    completed=min(a['event']['timestamp_s'] for a in e['completion_audits'] if a['passed'])
    s1=e['S1_claim_counts'];old_doc=Path('C:/Users/Administrator/Documents/ChatGPT/课题1/实验结果/感知升级与36组复验_20261021_v2/目标消息与绿色规划时序核查/W2C核查说明.md')
    text=f'''# W2：早出现的规划、目标真值与本轮修复

此前对用户指定的R9 W2-C视频核查，没有发现规划事件之前提前画绿色线，也没有在被追踪的“源候选→消息→选择→规划”链中发现目标真值代替输入。不能把本轮描述成已经证实并修复了真值泄漏。原记录见[此前核查说明]({old_doc.as_posix()})。

旧回合是UGV在1.75秒用约8×10像素、约90米深度的极小候选发消息，1.80秒规划，4fps回放在2.00秒出现绿色线。眼睛在缩小画面中不易辨认目标；问题在于小候选过早被当成可靠目标，没有把“探索核验”与“可靠发现”讲清楚。

本轮保留原场景、路线、种子和时限，识别权重保持冻结；增加质量分级及近距确认，并把两类规划分色。以下均为仿真时间：

|环节|本轮W2-C|
|---|---:|
|候选核验规划事件|{event['timestamp_s']-origin:.2f}秒，金色候选规划|
|实际视频最后一个规划前保存帧|{records[0]['timestamp_s']:.2f}秒，规划走廊颜色像素{counts[0]}|
|实际视频第一个规划后保存帧|{records[1]['timestamp_s']:.2f}秒，规划走廊颜色像素{counts[1]}|
|UAV首次正确可靠发现|{s1['uav']['first_correct_s']:.2f}秒|
|UGV首次正确可靠发现|{s1['ugv']['first_correct_s']:.2f}秒|
|正确近距地面确认|{ground:.2f}秒|
|两秒停稳完成并通过独立核验|{completed:.2f}秒|

最终真实车体净距{e['final_true_body_clearance_m']:.2f}米，原参考点距离{e['final_target_distance_m']:.2f}米，零碰撞。停稳确认位于路口连接区，按用户批准规则单独标注。

金色候选规划可以提示安全探索，不代表已经可靠发现；S1可靠发现声明、S2近距确认、S3完成分别评价。静态总览包含全回合后续轨迹，视频按当前时间显示，二者不能混同。红色目标位置只作离线标注。

颜色像素核验只在规划走廊内、排除演员图标附近后统计；直接统计全图会把绿色行人点和橙色车辆点误当成规划线。该像素结果只是辅助图像核查，前后实际画面和事件日志共同作为依据。

本轮所有已核查规划均追溯到保存的候选、消息和选择记录；控制函数另有禁止目标/障碍actor真值读取的测试。这些证据有明确覆盖边界，不等于已经对整个仿真进程实施了全局真值隔离。

![实际编码视频的规划前后两帧]({target.as_posix()})
'''
    (DEST/'W2早期规划与真值边界说明.md').write_text(text,encoding='utf-8');print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
