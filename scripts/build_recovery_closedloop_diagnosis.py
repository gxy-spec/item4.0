"""Explain every retained closed-loop outcome without changing frozen results."""
import json,csv,sys,shutil,math
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,DELIVERY,save,digest

def main():
    progress=json.loads((OUT/'formal_progress.json').read_text(encoding='utf-8'))
    records=[];evals={}
    for key,p in progress.items():
        run=Path(p['run_dir']);e=json.loads((run/'recovery_evaluation.json').read_text(encoding='utf-8'));evals[key]=e
        if p['group']=='A':reason='固定巡检控制组，不要求响应目标消息'
        elif e['sensor_safe_approach_audited']:reason='身份、本地观测与表面距离停车均通过；旧参考点到达未通过'
        elif e['unverified_commits']:reason='目标提交无法通过实例身份核验；部分关键帧可见为其他车辆、路障或立面'
        elif not e['correct_target_committed']:reason='未形成已核实正确的目标提交'
        elif not e['correct_target_locally_observed']:reason='正确目标已提交，但未获得地面正确目标观测'
        else:reason='正确目标与地面观测成立，但未完成表面距离停车'
        u=e['by_device'].get('ugv',{})
        records.append(dict(case=key,scene=p['scene'],group=p['group'],variant=p['variant'],runner_status=p['runner_status'],
            identity=e['identity_task_success'],surface=e['sensor_safe_approach_audited'],historical=e['historical_centre_arrival_success'],
            correct_commit=e['correct_target_committed'],correct_local_view=e['correct_target_locally_observed'],
            ugv_eligible_target_frames=u.get('positive_frames'),ugv_raw_selected_frames=u.get('selected'),ugv_raw_correct_frames=u.get('correct'),
            final_control_mode=e['final_control_mode'],mode_tick_counts=json.dumps(e['safety_mode_counts']),
            final_reference_distance_m=e['final_target_distance_m'],collisions=e['collisions'],unverified_commits=e['unverified_commits'],
            explanation=reason,run_dir=str(run)))
    DELIVERY.mkdir(parents=True,exist_ok=True)
    with (DELIVERY/'闭环逐回合归因.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    pair=json.loads((DELIVERY/'配对实际状态核查.json').read_text(encoding='utf-8'))
    qa=OUT/'manual_qa_final';evidence=DELIVERY/'诊断证据';evidence.mkdir(exist_ok=True)
    for name in ['safety_false_vehicle_examples.jpg','safety_false_vehicle_examples.json']:
        shutil.copy2(qa/name,evidence/name)
    local64=next(e for e in evals['upgraded_E3_C']['sensor_business_monitor']['events'] if e['event']=='sensor_local_confirmation')
    blocked=['upgraded_E2_C','upgraded_E3_C','upgraded_W3_C','upgraded_F1_C','upgraded_F3_C']
    lines=['# 完整闭环结果与失败归因','','**结论：模型识别与协同组有改善，但本阶段没有达到“成熟、稳定完成原任务”的预期。**模型在线8个正例、2个负例完成验收；原36组的严格参考点到达仍为0。不能把识别验收、运行器PASS或补充的表面距离停车写成原任务全面通过。','',
        '|指标（各24个B/C响应回合）|原感知包|升级感知包|变化|','|---|---:|---:|---:|',
        '|正确提交且地面正确观测|7/24，29.2%|11/24，45.8%|+4回合，+16.7个百分点|',
        '|正确身份且表面距离3–8米停车保持|7/24，29.2%|10/24，41.7%|+3回合，+12.5个百分点|',
        '|旧目标参考点3–7米到达|0/24|0/24|未改善|','',
        'B组表面停车新旧均3/12；C组从4/12增至7/12。升级模型C组12/12均提交了正确目标，8/12获得地面正确观测，7/12完成表面停车；因此剩余瓶颈已经明确包含安全感知与路径执行。A组12对是固定巡检控制，不混入响应成功率分母。','',
        '对表面停车24对结果逐对比较：3对由失败变通过，0对由通过变失败，其余21对不变；身份指标为4对改善、0对退化。这是本批观测到的改善，不能直接推广为稳定显著优势。仅作诊断的双侧精确配对二项检验分别为p=0.25、p=0.125；部署和连续观测也不是完全独立随机样本。','',
        '## 升级模型14个未完成表面停车的响应回合','',
        '**B组9回合：N3、E1、E2、E3、W1、W3、F1、F2、F3。**其中8回合UGV没有达到当前可评价像素门槛的目标帧；W1仅7帧且被蓝色前景车遮挡，未产生正确稳定候选。固定巡检经过的位置和朝向限制了地面观测，不能靠降低车型门槛或把UAV消息接入B组来宣称改进。保持原B组用于公平对照；后续如研究主动观察，应作为单独策略并固定运动/时间预算。','',
        '**C组5回合：E2、E3、W3、F1、F3。**均已正确提交目标，失败具体如下。','',
        '|回合|末状态|主要模式持续时间/秒|目标参考点末距/m|已有证据与判断|','|---|---|---:|---:|---|']
    for k in blocked:
        e=evals[k];c=e['safety_mode_counts'];mode=max(c,key=c.get);seconds=c[mode]*.05
        if k in ['upgraded_E2_C','upgraded_F1_C','upgraded_F3_C']:
            note='相同帧RGB红框落在路面亮斑/纹理上，安全检测器却持续判为近处car；深度通道CLEAR，但无合法超车间隙规则使UGV持续等待。不是目标车型头误判。'
        elif k=='upgraded_E3_C':note=f'地面确认发生时目标光学深度约{local64["depth_m"]:.2f}米，身份确认不代表接近；主要为深度障碍停车，末状态另有横穿车辆等待。实物路缘/交通岛与转弯路径的冲突仍须逐点几何复核。'
        else:note='深度障碍约3.98米，主要为depth_obstacle_stop；RGB/地图显示拐弯处交通岛/路缘。需核验可行驶走廊与局部规划，不能直接关停深度制动。'
        lines.append(f'|{k}|{e["final_control_mode"]}|{mode}：{seconds:.2f}|{e["final_target_distance_m"]:.2f}|{note}|')
    lines+=['','近处假车辆的独立复核：下图红框来自安全检测日志，与保存的原始UGV RGB严格同帧；三处均不是真实车体。它说明此前四类安全场景通过仍不足以排除路面纹理假阳性。','',
        f'![三回合安全检测器将路面纹理误判为车辆](<{(evidence/"safety_false_vehicle_examples.jpg").as_posix()}>)','',
        '对应CSV字段为no_safe_overtake_gap，地图禁止路口超车。E2/F1/F3的该模式累计分别167.25、412.50、419.60秒。安全逻辑持续运行，问题是安全前端假阳性及其决策约束导致无法前进；零碰撞不能代表导航可用。','',
        '## 原感知包的14个运行检查FAIL','',
        '11回合出现无法通过实例身份核验的目标提交，关键帧包括红色SUV、红白路障、停车楼立面及其他车辆；不把所有未标注候选自动改记为已知身份误检。另3回合E2 A/B/C因零消息送达触发原检查失败。其中A本来不响应消息，其零消息检查与控制组业务要求并不等价；原FAIL照实保留，另列原因，不能据此说仿真器崩溃。全部72回合均完整结束、无碰撞。','',
        '## 配对公平性与剩余时序干扰','',
        '36对的实际UGV/UAV/目标及背景演员初始XY位置差均为0，UAV配对轨迹相同。11对A组UGV逐时间步最大位置差不超过0.18米；**N3 A组中间轨迹差最大41.86米**，但最终位置相近。两侧横穿车/行人等待时长不同：旧版横穿等待43.65秒，新版21.90秒。说明同种子、同初态仍有动态交互/感知采样时序差异，不能声称所有过程逐帧相同或将所有耗时变化单独归因于目标前端。当前保留全部结果，不按偏差筛掉回合；下一轮需要固定危险感知采样相位及背景驱动时序，并补多种子重复。','',
        '## 是否符合本阶段预期，以及下一步如何解决','',
        '|预期|本轮判断|已完成或后续处理|','|---|---|---|',
        '|多车型、多颜色|在限定组合中实现|4类6色，8个在线组合；不等于24种组合或开放品牌全部通过。空中黑色厢式车/货车仍有弱项。|',
        '|检测和属性准确率提升|受控图像审计支持改善|保留YOLOE本体，整套前端提升；不是主检测器AP提升的独立证明。完整全图mAP尚未评价。|',
        '|原36组最终完成程度提高|补充安全接近指标部分改善；严格原口径未达到|保留旧到达失败，先明确车体表面和actor参考点的物理距离关系，不通过改名字消除失败。|',
        '|不因误检改变巡检路线|新模型减少错误提交，但安全前端仍会误停|下一版在车辆危险候选上加入地面高度/体积支撑验证及地面纹理负例；必须同时验证真实低矮车、横穿车和静止车，不能只提高阈值压掉假阳性。|',
        '|收到正确目标消息后可完成导航|仍未稳定实现|局部路径要检查交通岛/路缘和车辆扫掠区域，阻塞时有合法替代路线或重规划；保留路口禁超和深度制动。|',
        '|可以冻结为后续GOC成熟闭环基线|当前不宜|先冻结保存本轮结果；修复使用新版本号，先小规模回归真实安全场景及E2/E3/W3/F1/F3，再决定同条件完整复验，不覆盖本轮失败。|','',
        '上述后续处理是尚未执行并验证的修复方案，不能写成问题已解决。该结论不要求更换车辆检测与分割模型；当前最迫切的是危险车辆体积核验和可行驶局部路径，继续扩大车型网络规模不能解决车辆停在路口的问题。','',
        '全部成功和失败的回合级原因见[闭环逐回合归因.csv](<'+(DELIVERY/'闭环逐回合归因.csv').as_posix()+'>)，实际初态和A组轨迹核查见[配对核查](<'+(DELIVERY/'配对实际状态核查.json').as_posix()+'>)。逐帧原始日志和同帧证据均位于各run_dir。','']
    (DELIVERY/'闭环结果与失败归因.md').write_text('\n'.join(lines),encoding='utf-8')
    save(evidence/'诊断来源.json',dict(scope='Post-run source inspection only; does not change frozen models, control or evaluation.',
        manual_review=str(OUT/'manual_visual_review.json'),source_receipt=str(qa/'safety_false_vehicle_examples.json'),source_sha256=digest(qa/'safety_false_vehicle_examples.json')))
    print('CLOSED LOOP DIAGNOSIS',len(records),flush=True)

if __name__=='__main__':main()
