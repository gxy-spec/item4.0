"""Assemble auditable recovery results, retaining every required case.

No simulator or inference calls. Never infer task success from runner PASS.
Incomplete cohorts produce a progress report, never a final acceptance report.
"""
from pathlib import Path
from collections import Counter
from datetime import datetime, timezone
import argparse, csv, json, math, shutil, sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from recovery_stage import ROOT, OUT, DELIVERY, digest, rows, save
from build_recovery_scene_qa import build as build_scene_samples

CASE_LABELS={'red_truck':'红色消防货车','yellow_car':'黄色轿车','white_bus':'白色客车','gray_truck':'灰色货车',
    'red_van':'红色厢式车','blue_van':'蓝色厢式车','black_car':'黑色轿车','gray_car':'灰色轿车',
    'hard_negative':'属性干扰负例','no_target':'目标缺失负例'}
VARIANT_LABELS={'legacy':'原感知包','upgraded':'升级感知包','current_yoloe_hsv':'YOLOE+HSV'}
IMAGE_VARIANT_LABELS={'legacy':'YOLO26n+HSV（通用查询）','current_yoloe_hsv':'YOLOE+HSV','upgraded':'本轮升级前端'}


def read(path, default=None):
    p=Path(path)
    return json.loads(p.read_text(encoding='utf-8')) if p.exists() else default


def table(path, values):
    if not values:return
    keys=list(dict.fromkeys(k for r in values for k in r))
    with Path(path).open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(values)


def link(path,label):
    return f'[{label}](<{Path(path).as_posix()}>)'


def frac(a,n):return f'{a}/{n}（{100*a/n:.1f}%）' if n else '无可评价样本'
def yes(v):return '是' if v is True else '否' if v is False else '待核验'
def num(v):return '—' if v is None else f'{v:.2f}'


def message_audit(run):
    """Audit the transmitted population separately from raw matching candidates."""
    dest=run/'recovery_message_audit.json'
    cached=read(dest)
    if cached:return cached
    evidence={(r['device'],int(r['frame']),str(r['candidate_id'])):r for r in rows(run/'task1_candidate_audit.jsonl')}
    messages=rows(run/'communication/messages.jsonl');records=[]
    for m in messages:
        a=evidence.get((m.get('source'),int(m.get('source_frame',-1)),str(m.get('candidate_id'))))
        if not a or a.get('matched_actor_id') is None:status='unverified'
        elif not a.get('semantic_correct'):status='wrong_identity'
        elif a.get('localization_error_m') is None:status='unverified'
        elif a['localization_error_m']>5:status='correct_identity_position_gt5m'
        else:status='verified_configured_target'
        records.append(dict(message_id=m['message_id'],device=m.get('source'),source_frame=m.get('source_frame'),candidate_id=m.get('candidate_id'),
            status=status,matched_actor_id=a.get('matched_actor_id') if a else None,matched_role=a.get('matched_role') if a else None,
            localization_error_m=a.get('localization_error_m') if a else None))
    detail=run/'recovery_message_audit.csv';table(detail,records)
    result=dict(count=len(records),counts=dict(Counter(r['status'] for r in records)),
        by_device={d:dict(Counter(r['status'] for r in records if r['device']==d)) for d in ['uav','ugv']},
        definition='Exact source frame/candidate match against known configured target. Unlabelled or IoU<0.5 associations remain unverified; not automatically false positives. XY error separately limited to5m.',
        detail=str(detail),detail_sha256=digest(detail) if detail.exists() else None)
    save(dest,result);return result


def collect(cohort):
    index=read(OUT/f'{cohort}_index.json',{'runs':[]});progress=read(OUT/f'{cohort}_progress.json',{});result=[]
    for spec in index['runs']:
        key=Path(spec['config']).stem;p=progress.get(key,{})
        run=Path(p['run_dir']) if p.get('run_dir') else None
        evaluation=read(run/'recovery_evaluation.json',{}) if run else {}
        r={**spec,**p,**evaluation,'cohort':cohort,'case_key':key,'source_case':spec['case'],
           'evaluated':bool(evaluation),'acquisition_complete':bool(p.get('report_complete'))}
        if run and evaluation:
            r['message_audit']=message_audit(run)
            r['artifact_receipt']=read(run/'preview/recovery_artifacts_verified.json')
            build_scene_samples(run)
        result.append(r)
    return result


def aggregate(z):
    valid=[r for r in z if r['evaluated']]
    fields=['correct_target_committed','correct_target_locally_observed','identity_task_success',
            'sensor_safe_approach_audited','historical_centre_arrival_success']
    d=dict(n=len(z),acquired=sum(r['acquisition_complete'] for r in z),evaluated=len(valid),
           infrastructure_pass=sum(r.get('runner_status')=='PASS' for r in z),
           artifacts_verified=sum(bool(r.get('artifact_receipt')) for r in z),
           collisions=sum(r.get('collisions',0) for r in valid),
           wrong_commits=sum(r.get('wrong_commits',0) for r in valid),
           unverified_commits=sum(r.get('unverified_commits',0) for r in valid),
           position_rejected_commits=sum(r.get('position_rejected_commits',0) for r in valid))
    d.update({k:sum(bool(r.get(k)) for r in valid) for k in fields})
    distances=[r['final_target_distance_m'] for r in valid if r.get('final_target_distance_m') is not None]
    d['mean_final_distance_m']=sum(distances)/len(distances) if distances else None
    return d


def flatten(r):
    events=r.get('sensor_business_monitor',{}).get('events',[])
    approach=next((e for e in events if e['event']=='sensor_safe_approach_hold_completed'),{})
    local=next((e for e in events if e['event']=='sensor_local_confirmation'),{})
    origin=None
    if r.get('run_dir'):
        states=Path(r['run_dir'])/'actors/actor_states.jsonl'
        if states.exists():
            with states.open(encoding='utf-8') as f:
                first=f.readline()
                if first.strip():origin=json.loads(first)['timestamp']
    first_correct=next((c for c in r.get('commits',[]) if c.get('correct')),None)
    values={k:r.get(k) for k in ['cohort','case_key','source_case','scene','variant','group','seed','runner_status',
        'acquisition_complete','evaluated','correct_target_committed','correct_target_locally_observed','identity_task_success',
        'sensor_safe_approach_audited','historical_centre_arrival_success','wrong_commits','unverified_commits',
        'position_rejected_commits','sent_messages','collisions','route_plan_count','final_target_distance_m','final_control_mode','wall_seconds','run_dir']}
    values.update(local_confirmation_sim_s=local.get('timestamp_s'),safe_approach_sim_s=approach.get('timestamp_s'),
        stopped_target_optical_depth_m=approach.get('depth_m'),artifact_verified=bool(r.get('artifact_receipt')),
        historical_reasons=';'.join(r.get('reasons',[])),actual_runner_real_time_factor=r.get('actual_runner_real_time_factor'),
        combined_pipeline_median_ms=r.get('online_latency',{}).get('combined_pipeline_ms',{}).get('median'),
        combined_pipeline_p95_ms=r.get('online_latency',{}).get('combined_pipeline_ms',{}).get('p95'),
        combined_pipeline_budget_ms=r.get('combined_pipeline_budget_ms'),
        first_correct_commit_sim_s=first_correct['event']['timestamp_s']-origin if first_correct and origin is not None else None,
        first_correct_commit_source=first_correct['message'].get('source') if first_correct else None)
    message_counts=r.get('message_audit',{}).get('counts',{})
    values.update({'messages_'+k:(message_counts.get(k,0) if r.get('message_audit') is not None else None)
        for k in ['verified_configured_target','wrong_identity','correct_identity_position_gt5m','unverified']})
    sources=r.get('message_count_by_device',{})
    values.update(logged_messages_ignored_by_policy=(sum(sources.values()) if r.get('policy')=='none' else sources.get('uav',0) if r.get('policy')=='ugv_local' else 0),
        selected_state_events=r.get('summary',{}).get('persistent_target_communication',{}).get('selected_state_count'))
    return values


def write_report(final=False):
    active=read(OUT/'release_active.json',{'acceptance_cohort':'acceptance_release','runtime_manifest_path':str(OUT/'frozen_runtime_release/manifest.json')})
    formal_spec=read(OUT/'formal_index.json',{})
    if formal_spec.get('runtime_manifest_path'):active['runtime_manifest_path']=formal_spec['runtime_manifest_path']
    acceptance_cohort=active['acceptance_cohort']
    DELIVERY.mkdir(parents=True,exist_ok=True);accept=collect(acceptance_cohort);formal=collect('formal')
    all_runs=accept+formal;flat=[flatten(r) for r in all_runs]
    table(DELIVERY/'逐回合结果.csv',flat);save(DELIVERY/'逐回合结果.json',flat)
    perception_rows=[]
    for r in all_runs:
        if not r['evaluated']:continue
        f=r.get('frame_metrics',{})
        perception_rows.append(dict(cohort=r['cohort'],case=r['case_key'],variant=r.get('variant'),group=r.get('group'),
            known_target_visible_frames=f.get('positive_frames'),known_target_absent_frames=f.get('negative_frames'),
            raw_task_candidate_selected_frames=f.get('selected'),raw_candidate_correct_known_target_frames=f.get('correct'),
            raw_candidate_on_known_target_absent_frames=f.get('negative_false_triggers'),
            raw_known_target_fraction_among_selected_frames=f.get('precision'),raw_known_target_frame_recall=f.get('recall'),
            raw_task_candidate_rate_on_absent_frames=f.get('negative_fpr'),
            **{'actual_messages_'+k:r.get('message_audit',{}).get('counts',{}).get(k,0)
               for k in ['verified_configured_target','wrong_identity','correct_identity_position_gt5m','unverified']},
            actual_sent_messages=r.get('sent_messages'),message_audit_detail=r.get('message_audit',{}).get('detail')))
    table(DELIVERY/'感知与通信逐回合指标.csv',perception_rows)
    cohorts={k:aggregate(z) for k,z in [('acceptance_release',accept),('formal',formal)]}
    positives=[r for r in accept if r.get('expected_target_present',True) and r['evaluated']]
    negatives=[r for r in accept if not r.get('expected_target_present',True) and r['evaluated']]
    acceptance_headline=(f"已核验正例中，正确目标及本地确认{sum(bool(r.get('identity_task_success')) for r in positives)}/{len(positives)}，"
        f"表面距离停车{sum(bool(r.get('sensor_safe_approach_audited')) for r in positives)}/{len(positives)}；"
        f"已核验负例中，零误发与拒绝通过{sum(bool(r.get('negative_rejection_pass')) and r.get('sent_messages',0)==0 for r in negatives)}/{len(negatives)}。"
        f"原中心距到达{sum(bool(r.get('historical_centre_arrival_success')) for r in positives)}/{len(positives)}，单独保留。");
    formal_headline=''
    if len(formal)==72 and all(r['evaluated'] for r in formal):
        response={v:[r for r in formal if r.get('variant')==v and r['group'] in ['B','C']] for v in ['legacy','upgraded']}
        score={v:sum(bool(r.get('sensor_safe_approach_audited')) for r in z) for v,z in response.items()}
        identity={v:sum(bool(r.get('identity_task_success')) for r in z) for v,z in response.items()}
        if score['upgraded']>score['legacy']:verdict=f"安全接近完成数增加{score['upgraded']-score['legacy']}个回合。"
        elif score['upgraded']==score['legacy']:verdict='安全接近完成数未增加，不能宣称本轮提升了原任务完成率。'
        else:verdict='安全接近完成数下降，未达到原任务完成率改善的预期；须按失败记录区分原因。'
        formal_headline=(f"原36组中的24个B/C响应回合，新模型正确目标及本地观测{identity['upgraded']}/24、受核验安全接近{score['upgraded']}/24；"
            f"同条件旧感知包分别为{identity['legacy']}/24、{score['legacy']}/24。"+verdict)
        formal_headline+=' **严格原参考点到达新旧均0/24，因此本阶段仅部分达到预期，不能宣称成熟闭环验收通过。**'
    groups=[]
    for variant in ['legacy','upgraded']:
        for group in ['A','B','C']:
            z=[r for r in formal if r.get('variant')==variant and r['group']==group]
            groups.append(dict(variant=variant,group=group,**aggregate(z)))
    table(DELIVERY/'分组汇总.csv',groups)
    paired=[]
    for case in dict.fromkeys(r['source_case'] for r in formal):
        for group in ['A','B','C']:
            pair={r['variant']:r for r in formal if r['source_case']==case and r['group']==group}
            if set(pair)!={'legacy','upgraded'}:continue
            a,b=pair['legacy'],pair['upgraded'];p=dict(case=case,group=group,scene=a['scene'],pair_evaluated=a['evaluated'] and b['evaluated'])
            for k in ['identity_task_success','sensor_safe_approach_audited','historical_centre_arrival_success','final_target_distance_m','collisions']:
                p['legacy_'+k]=a.get(k);p['upgraded_'+k]=b.get(k)
            paired.append(p)
    table(DELIVERY/'36组逐对比较.csv',paired)
    visual=read(OUT/'manual_visual_review.json',{'reviews':{}})
    reviews=visual['reviews'];visual_missing=[];valid_reviews=0
    for r in all_runs:
        if not r['acquisition_complete']:continue
        key=r['cohort']+'/'+r['case_key'];review=reviews.get(key,reviews.get(r['case_key'],{}))
        board=Path(r['run_dir'])/'preview/recovery_key_evidence.jpg'
        scene=Path(r['run_dir'])/'preview/recovery_scene_samples.jpg'
        control_reviewed=bool(r.get('commits')) or (scene.exists() and review.get('scene_sample_sha256')==digest(scene))
        if board.exists() and review.get('evidence_board_sha256')==digest(board) and review.get('rgb_support_reviewed') and control_reviewed:
            valid_reviews+=1
        else:visual_missing.append(key)
    complete=len(accept)==10 and len(formal)==72 and all(r['acquisition_complete'] and r['evaluated'] and r.get('artifact_receipt') for r in all_runs)
    if final and (not complete or visual_missing):raise RuntimeError(f'Final report refused: cohort/artifacts complete={complete}; visual review missing={visual_missing}')
    frozen=read(active['runtime_manifest_path'],{})
    source_bad=[p for p,h in frozen.get('files',{}).items() if digest(ROOT/p)!=h]
    artifact_bad=[p for p,h in frozen.get('artifacts',{}).items() if digest(p)!=h]
    if source_bad or artifact_bad:raise RuntimeError('Frozen content changed: '+str(source_bad+artifact_bad))
    save(DELIVERY/'交付清单.json',dict(generated_utc=datetime.now(timezone.utc).isoformat(),final=final,cohorts=cohorts,
        manual_visual_reviews=valid_reviews,manual_visual_missing=visual_missing,runtime_verified=not(source_bad or artifact_bad),
        raw_data_policy='All RGB-D/instance images, logs and telemetry remain at the recorded run_dir, possibly on C/D/E; no lossy replacement of scientific source images.',
        data_root=str(OUT),model_sha256=frozen.get('model_sha256')))
    lines=['# 感知模型升级与36组闭环复验'+('：最终验收报告' if final else '：运行进度'),'',
        '本报告'+('记录已完成的全部实验，所有成功、任务失败和异常尝试均保留。' if final else '**尚不是最终验收结论**；完整回合和证据未齐前，不把部分通过写作整阶段通过。'),'',
        f"在线验收：{cohorts['acceptance_release']['acquired']}/10回合已运行、{cohorts['acceptance_release']['evaluated']}回合已审计；正式复验：{cohorts['formal']['acquired']}/72回合已运行、{cohorts['formal']['evaluated']}回合已审计。",'',
        '## 先看结论', '',
        f"本轮版本：**{active.get('revision','R8')}**。"+acceptance_headline,'',
        formal_headline,'',
        (f'全部82回合的关键RGB证据已逐回合人工复核（{valid_reviews}/82）；72个正式回合轨迹图另经配对复核。人工复核是关键事件及起中末场景抽查，不代表逐帧看完全部视频。' if final else f'人工关键RGB证据复核：{valid_reviews}/82；其余仍待复核。'),'',
        '保留已有YOLOE检测与分割权重，增加通用车辆候选补充、保守深度框补全和RGB车型/颜色双分支模型。正式模型直接从图像预测4类车型与6种颜色；HSV只保留作诊断，不决定新模型的颜色。', '',
        '本次分别回答三个问题：是否找到指定的正确车辆，是否完成本地确认与车表面距离3–8米的停车保持，以及是否达到原车辆中心距3–7米的到达要求。三项分列；运行器PASS只代表运行检查通过。','',
        '原始JSON中的semantic_verdict、task_success及reasons保留旧中心到达重建口径；其中的FAIL或no_local_confirmation不能直接替代新版独立身份与本地监测结果。最终比较请对应identity_task_success、sensor_safe_approach_audited和historical_centre_arrival_success三个明确字段。','',
        '## 在线多类别验收','',
        '|任务|运行|正确目标及本地确认|表面距离停车通过|原中心距到达|最终中心距/m|碰撞|',
        '|---|---|---|---|---|---:|---:|']
    for r in accept:
        neg=not r.get('expected_target_present',True)
        identity=yes(r.get('negative_rejection_pass'))+'（负例拒绝）' if neg else yes(r.get('identity_task_success'))
        lines.append(f"|{CASE_LABELS.get(r['source_case'],r['source_case'])}|{r.get('runner_status','待运行')}|{identity}|{'不适用' if neg else yes(r.get('sensor_safe_approach_audited'))}|{'不适用' if neg else yes(r.get('historical_centre_arrival_success'))}|{num(r.get('final_target_distance_m'))}|{r.get('collisions','—')}|")
    lines+=['','正例覆盖van/car/truck/bus、red/blue/white/black/gray/yellow；两个负例均指不存在符合任务条件的正确目标车，场景仍可有背景车辆。正式36组保留原红色厢式车任务，以便逐组比较；目标多样性由本表及独立图像审计证明。','',
        '**覆盖4类、6色不等于24种车型×颜色组合全部通过在线验收**；本轮在线验收只验证表中8个正例组合。当前客车资产主车身固定为白色，不把无法渲染的其他客车涂装作为已验证能力。','',
        '## 原36组的同条件复验','',
        '每种感知方案各36回合：四个场景族×三个部署×A/B/C。A为固定巡检不响应目标消息，B仅UGV本地来源可触发导航，C允许空地协同。各组仍运行双端感知与消息记录，因此A可能有已发送/记录消息但不选择目标，B也可能记录不被策略采用的UAV消息；不能把本轮消息数直接解释成通信资源优化收益。A是控制条件，仍完整列出；B/C的24对用于评价响应任务完成率。','',
        '|方案|组|已核验/应运行|身份正确且本地确认|表面距离停车|原中心距到达|平均最终中心距/m|碰撞事件|',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for d in groups:
        lines.append(f"|{VARIANT_LABELS[d['variant']]}|{d['group']}|{d['evaluated']}/{d['n']}|{d['identity_task_success']}|{d['sensor_safe_approach_audited']}|{d['historical_centre_arrival_success']}|{num(d['mean_final_distance_m'])}|{d['collisions']}|")
    lines+=['','新旧对照共用当前目标记忆、RGB-D安全模块、停车监测、960输入尺寸、4Hz任务推理、20Hz同步传感器和记录频率；原几何部署、巡检路线、演员、随机种子与时长保留。旧方案保留YOLO26n+HSV及原厢式车/颜色候选规则。比较反映整套感知前端变化，不能把全部收益归于单独的车型分类头，也不能把它称为完全未改动的历史系统。','']
    if complete:
        lines += ['**完整闭环的主要问题**：新B组仍仅3/12完成；新C组12/12正确提交目标，但仅8/12地面正确观测、7/12安全接近。E2/F1/F3的安全检测器把路面纹理当成近车，导致长期等待；E3/W3还受深度障碍和局部路径限制。零碰撞不等于导航可用。', '',
            '36对实际初始位置一致，但N3 A组中间轨迹最大差41.86米，反映动态交互和采样时序仍有干扰，不能声称全部过程逐帧相同。配对表面停车仅3对改善、0对退化，样本不足以支持稳定显著优势。', '',
            link(DELIVERY/'闭环结果与失败归因.md','完整失败归因、同帧路面假车辆证据及修复安排')+'。当前不宜直接冻结为GOC成熟闭环基线；应先修复安全前端假阳性与可行驶局部路径，再用新版本复验。','']
    for name,title in [('final_frontend_audit','冻结后独立图像审计'),('frontend_release','开发压力集复核（不作为盲测）')]:
        audit=read(OUT/name/'summary.json')
        if not audit:continue
        lines += [f'## {title}','','|方案|目标框IoU≥0.5|框及车型正确|框及颜色正确|框、车型、颜色均正确|', '|---|---:|---:|---:|---:|']
        for v,d in audit['models'].items():
            lines.append(f"|{IMAGE_VARIANT_LABELS.get(v,v)}|{frac(d['box_iou50'],d['known_target_views'])}|{frac(d['exact_type_correct'],d['known_target_views'])}|{frac(d['color_correct'],d['color_n'])}|{frac(d['joint_correct'],d['color_n'])}|")
        u=audit['models']['upgraded']
        lines+=['','以上分母包含未检出的目标；颜色仅评价可见且经受控渲染核实的车漆。裁剪分类指标和完整前端指标分别记录。已知目标之外的街景车辆未穷尽标注，因此这不是全图precision或mAP，也不存在可以据此套用的“领域统一最低合格线”。', '',
            f"新模型空中视角：框检出{frac(u['by_device']['uav']['box_iou50'],u['by_device']['uav']['known_target_views'])}，联合正确{frac(u['by_device']['uav']['joint_correct'],u['by_device']['uav']['color_n'])}；地面视角联合正确{frac(u['by_device']['ugv']['joint_correct'],u['by_device']['ugv']['color_n'])}。具体类别计数见原始审计JSON，避免总体均值掩盖空中货车、客车等弱项。",'',
            '旧COCO检测器没有van类别；本表采用通用vehicle查询，不启用原红车区域候选，属于通用感知能力比较。原任务的厢式车先验与颜色候选在72回合闭环旧方案中保留。','']
        if name=='final_frontend_audit':
            predictions=[r for r in rows(OUT/name/'predictions.jsonl') if r['variant']=='upgraded']
            hard_van=[r for r in predictions if r['device']=='uav' and 'sprinter_black' in r['id']]
            hard_truck=[r for r in predictions if r['device']=='uav' and 'european_hgv_black' in r['id']]
            lines += [f"**不能被总体均值掩盖的弱项**：最终独立审计中，空中黑色Sprinter车型正确{sum(r['exact_type_correct'] for r in hard_van)}/{len(hard_van)}，空中黑色HGV车型正确{sum(r['exact_type_correct'] for r in hard_truck)}/{len(hard_truck)}；这些受测组合的地面视角均正确。存在将深色长车顶误判为轿车的情况，当前模型尚不能无条件推广至全部颜色组合和空中视角。下一轮可仅在训练集补充深色车型、多高度及光照样本，考虑几何辅助，再用新独立数据验证；本轮保持冻结结果，不根据最终测试回填修正。",'']
    lines+=['## 问题、原因与处理','','1. **近距车型误判及空中黑灰色不足**：早期模型会把轿车尾部判为van，且颜色样本覆盖不足。增加真实相机尺度的近距训练数据，选择ResNet18双分支；针对训练地点补充空中颜色样本，仅更新颜色头，冻结车型及骨干权重。独立开发压力集暴露问题后已转为开发数据，另采冻结后最终审计图像，避免把调过模型的集合称为盲测。','',
        '2. **安全停车误检与侧向行人盲区**：过往版本出现把阴影当行人而持续停车，以及在真实4米制动设置下碰撞的情况。补充深度高度支撑、当前运动方向判断与近处路边行人让行记忆。最终相同安全参数下四类安全场景均通过、碰撞为0；此前碰撞和卡住回合仍保留，不能当作全场景安全保证。','',
        '3. **停车控制与原参考点距离分开处理**：R8红色货车已正确识别，但最终车表面深度2.984米，低于原3米下限。R9增加新鲜本地确认表面深度4.5米制动，保持3–8米验收范围不变。几何复核还发现，旧字段用目标actor参考点，未必是几何车身中心；红色Sprinter相机前置约1.5米、可见后表面距目标参考点约2.9米，因此表面至少3米通常已对应约7.4米参考点距离，与旧上限7米有冲突。不能只缩终点容差逼近目标来让统计通过；旧指标仍保留未通过，跨车型的物理验收口径需明确。','',
        '4. **数据与运行基础问题**：纠正客车实际白色主车身标签、相机生成时序、实例ID碰撞、物理停车点重复划分及Windows长路径；引入单一仿真占用锁与回合间停止。当前演员真值只用于离线标注与审计，在线识别和控制不读目标真值。','',
        '## 证据与复现入口','',
        link(DELIVERY/'逐回合结果.csv','逐回合结果（含全部原始路径）')+'；'+link(DELIVERY/'分组汇总.csv','分组汇总')+'；'+link(DELIVERY/'36组逐对比较.csv','36组逐对比较')+'；'+link(DELIVERY/'逐回合证据索引.md','每回合轨迹、关键帧与视频')+'。','',
        link(DELIVERY/'感知专项评估详表.md','分类、颜色及薄弱组详表')+'；'+link(DELIVERY/'失败与修复记录.md','失败与修复记录')+'；'+link(DELIVERY/'停车距离口径说明.md','停车距离几何核查')+'。','',
        link(DELIVERY/'闭环结果与失败归因.md','闭环完成度与失败归因')+'；'+link(DELIVERY/'验收字段口径说明.md','结果字段口径说明')+'；'+link(DELIVERY/'配对实际状态核查.json','36对实际初态与A组轨迹核查')+'；'+link(DELIVERY/'可复现模型与代码/README_复现.md','冻结模型与复现代码包')+'。','',
        link(active['runtime_manifest_path'],'冻结源码与模型SHA256')+'；'+link(OUT/'calibration/deployment_release.json','验证集校准与阈值')+'；'+link(OUT/'attempt_history.jsonl','包含失败的全部运行尝试')+'；'+link(OUT/'frontend_release/summary.json','开发前端审计原始计数')+'。','',
        '模型输入仅RGB裁剪；深度用于定位、框补全和安全。当前阶段还未完成真实LiDAR接入，也未完成三层通信、接收融合或GOC决策实验。它为这些后续实验提供可比较的感知基线；不能把RGB-D结果写成异质多模态贡献已获验证。','',
        '当前语言任务解析支持约定的车型和颜色条件，尚未证明任意开放词汇、品牌型号或复杂空间关系理解。推理按仿真时间4Hz触发，逐回合CSV另列双设备前端耗时中位数/p95及完整运行实时因子；即使前端耗时小于250ms，也不能据此把包含仿真、全频安全处理和落盘的整体系统称为已达到墙钟实时运行。','',
        '## 评判口径','',
        '在线帧级指标只评价已知演员及可见目标，帧之间高度相关；不能把几千连续帧当成几千独立试验。消息身份另按实际发送群体审计，分为已核实正确、已知错误、定位超差及无法核实；无法核实不自动计作已知错误。分组统计以完整回合为单位，提供原始分子、分母，不用检测器置信度代替任务收益。标准precision、recall及mAP定义参考[Ultralytics官方指标说明](https://docs.ultralytics.com/guides/yolo-performance-metrics/)，本报告的已知目标正确率不冒用完整检测mAP名称。','',
        '关键帧证据显示原始RGB和目标裁剪；轨迹图含原巡检路线、收到消息后规划路线与实际UGV/UAV轨迹。视频中的实际轨迹只累积到当前时刻。原图只作逐像素验证的无损PNG压缩及同字节硬链接，视频是派生可视化。','']
    target=DELIVERY/('最终验收报告.md' if final else '实验报告_进行中.md')
    target.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    evidence=['# 逐回合证据索引','','所有原始数据保留在各回合目录。下表不以成功与否筛选。负例或A组无目标事件时，关键事件面板会留空；另有起中末三组双设备原始RGB场景样本。','','|批次/回合|原巡检与实际轨迹|关键RGB|同步视频|原始场景样本|结果审计|','|---|---|---|---|---|---|']
    for r in all_runs:
        if not r.get('run_dir'):continue
        run=Path(r['run_dir']);receipt=r.get('artifact_receipt')
        evidence.append('|'+r['cohort']+'/'+r['case_key']+'|'+('|'.join([link(run/'preview/recovery_trajectory_map.png','轨迹'),link(run/'preview/recovery_key_evidence.jpg','关键帧'),link(run/'preview/recovery_five_view.mp4','视频')]) if receipt else '待生成|待生成|待生成')+'|'+link(run/'preview/recovery_scene_samples.jpg','空地RGB样本')+'|'+link(run/'recovery_evaluation.json','身份/停车/中心距')+'|')
    (DELIVERY/'逐回合证据索引.md').write_text('\n'.join(evidence)+'\n',encoding='utf-8')
    print(json.dumps(dict(report=str(target),cohorts=cohorts,visual_missing=len(visual_missing)),ensure_ascii=False),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--final',action='store_true');a=p.parse_args();write_report(a.final)
