"""Final complete deliverable checklist, issue register and portable review ZIP."""
from __future__ import annotations
from collections import Counter
from datetime import datetime
import json
from pathlib import Path
import subprocess
import shutil
import sys
import zipfile
import yaml
from perception_online_acceptance import OUT,ROOT,MODELS,read_jsonl,save_json,sha256
from analyze_online_acceptance import table,quantiles,csv_rows
from package_online_acceptance_results import DELIVERY


def main():
    index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
    if len(index)!=12 or any(v.get('runner_status') not in ['PASS','FAIL'] for v in index.values()):
        raise RuntimeError('Twelve complete formal reports are required')
    if not (OUT/'observability_diagnostic/results.json').exists():raise RuntimeError('Visibility diagnosis still missing')
    for script in ['audit_task1_target_appearance.py','audit_online_acceptance_integrity.py','package_online_acceptance_results.py']:
        log=OUT/f'final_{Path(script).stem}.log'
        with log.open('w',encoding='utf-8') as f:
            p=subprocess.run([sys.executable,'-u',str(ROOT/'scripts'/script)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        if p.returncode:raise RuntimeError(f'Final generation failed: {log}')
    results=json.loads((OUT/'analysis/online_results.json').read_text(encoding='utf-8'))
    checks={};files=[];message_rows=[];missing_sensor=[];latency_rows=[]
    for r in results:
        run=Path(r['run_dir']);name=r['case']
        audit={(x['device'],x['frame'],x['candidate_id']):x for x in read_jsonl(run/'task1_candidate_audit.jsonl')}
        messages=read_jsonl(run/'communication/messages.jsonl')
        counts=Counter();errors=[]
        for m in messages:
            e=audit.get((m['source'],m['source_frame'],m['candidate_id']))
            if e is None or e['matched_actor_id'] is None:counts['unverified']+=1
            elif e['semantic_correct']:
                counts['automatic_label_match']+=1
                if e.get('localization_error_m') is not None:errors.append(e['localization_error_m'])
            else:counts['automatic_label_mismatch']+=1
            counts['dominant_color_differs_from_requested']+=m['color']!=r['task_color']
        message_rows.append({'case':name,'sent':len(messages),**dict(counts),
            'auto_label_precision':counts['automatic_label_match']/len(messages) if messages else None,
            'matching_xy_error_p95_m':quantiles(errors)['p95'],
            'limitation':'automatic instance-label audit; RGB visibility not exhaustively reviewed'})
        frame_metrics=csv_rows(run/'task1_frame_metrics.csv')
        warning_text=Path(r['log']).read_text(encoding='utf-8',errors='replace')
        for device in ['uav','ugv']:
            values=[float(x['pipeline_wall_ms']) for x in frame_metrics if x['device']==device and x.get('pipeline_wall_ms')]
            q=quantiles(values)
            latency_rows.append({'case':name,'device':device,'all_frames_n':q['n'],'all_frames_median_ms':q['median'],
                'all_frames_p95_ms':q['p95'],'all_frames_max_ms':q['maximum'],
                'first_two_processing_ms':str(values[:2]),'processing_over_250ms':sum(v>250 for v in values),
                'steady_p95_ms':r['online_latency'][device]['pipeline_ms']['p95'],
                'nms_warning_count_in_run_log':warning_text.count('NMS time limit'),
                'scope':'candidate pipeline only; not full sensor-to-control end-to-end latency'})
        required=['validation_report.json','resolved_config.yaml','task1_evaluation.json','task1_frame_metrics.csv',
            'task1_candidate_audit.jsonl','preview/task1_five_view.mp4','preview/task1_five_view.json',
            'preview/task1_video_quality_check.json','preview/task1_trajectory_map.png','preview/task1_replay_poster.jpg']
        checks[name]={p:(run/p).exists() for p in required}
        qa=json.loads((run/'preview/task1_video_quality_check.json').read_text(encoding='utf-8'))
        checks[name]['video_quality_passed']=qa['passed'] and qa['renderer_version']==5
        for p in required:files.append((run/p,f'online/{name}/{p}'))
        for p in ['preview/task1_route_overlay_evidence.json','planning/ugv_patrol_route_executed.json','planning/target_route_events.jsonl']:
            if (run/p).exists():files.append((run/p,f'online/{name}/{p}'))
        for p in run.glob('task1_evaluation_v3_before_commit_split.json'):files.append((p,f'online/{name}/{p.name}'))
        for row in csv_rows(run/'synchronization/frame_index.csv'):
            if int(row.get('sensor_files_saved',1))!=1:continue
            frame=int(row['frame'])
            for device in ['uav','ugv']:
                for folder in ['rgb','depth_colour','instance_segmentation']:
                    p=run/f'sensors/{device}/{folder}/{frame:08d}.png'
                    if not p.is_file():missing_sensor.append(str(p))
    table(OUT/'analysis/message_quality_by_run.csv',message_rows)
    table(OUT/'analysis/latency_cold_start_and_warnings.csv',latency_rows)
    positive=[r for r in results if r['expected_target_present']];negative=[r for r in results if not r['expected_target_present']]
    cal=json.loads((OUT/'calibration/adapted.json').read_text(encoding='utf-8'))
    integrity=json.loads((OUT/'analysis/runtime_integrity_audit.json').read_text(encoding='utf-8'))
    protocol=json.loads((OUT/'protocol.json').read_text(encoding='utf-8'))
    decision={
        'twelve_formal_runs_complete':len(results)==12,
        'joint_threshold_constraints_satisfied':cal['constraints_satisfied'],
        'rgb_visibility_ground_truth_independently_validated':False,
        'all_positive_task_endpoints_passed':len(positive)==8 and all(r['task_success'] for r in positive),
        'all_negative_tasks_rejected':len(negative)==4 and all(r['negative_rejection_pass'] for r in negative),
        'no_wrong_or_unverified_target_commits':sum(r.get('commit_verification_failures',r['wrong_commits']) for r in results)==0,
        'no_recorded_collisions':sum(r['collisions'] for r in results)==0,
        'runtime_integrity_audit_passed':integrity['pass'],
        'model_weight_fingerprints_unchanged':all(sha256(path)==protocol['models'][name]['sha256'] for name,path in MODELS.items()),
        'all_videos_maps_and_metrics_delivered':all(all(c.values()) for c in checks.values()),
        'all_archived_sensor_files_referenced_by_replay_exist':not missing_sensor,
    }
    payload={'checks':decision,'model_acceptance_passed':all(decision.values()),'execution_completed':True,
        'historical_run_classification':'failed diagnostic campaign; not an established perception baseline',
        'qualified_formal_online_runs_after_reaudit':0,
        'data_limits':['RGB visibility of instance labels is not fully valid','formal metric depth was not archived'],
        'positive_passed':sum(r['task_success'] for r in positive),'positive_total':len(positive),
        'negative_passed':sum(r['negative_rejection_pass'] for r in negative),'negative_total':len(negative),
        'wrong_commits':sum(r['wrong_commits'] for r in results),
        'unverified_commits':sum(r.get('unverified_commits',0) for r in results),
        'commit_verification_failures':sum(r.get('commit_verification_failures',r['wrong_commits']) for r in results),
        'collisions':sum(r['collisions'] for r in results),
        'per_run_delivery_checks':checks,'missing_sensor_files':missing_sensor}
    save_json(OUT/'analysis/final_acceptance_decision.json',payload)
    checklist=[
        {'交付项':'整改复核主报告','状态':'已重写并分离附件','结论':'任务1未完成合格验收；失败诊断不冻结为基线','位置':'任务1_在线验收报告.md; 问题1-4_依据与整改明细.md'},
        {'交付项':'在线模型配置与阈值','状态':'已交付','结论':'原校准无可行解；固定回退阈值只用于诊断','位置':'protocol.json; configs/; calibration/'},
        {'交付项':'识别与负例统计','状态':'已交付原口径和语义清洗参考','结论':'RGB可见性尚未独立验证，不能当作可信真实精度','位置':'benchmark/; analysis/semantic_clean_reference.*'},
        {'交付项':'分设备、类别、颜色统计','状态':'已交付','结论':'保留样本数与不可评价项；组合覆盖不足','位置':'analysis/benchmark_ego_excluded_at_frozen_thresholds.json'},
        {'交付项':'定位与实时性','状态':'已交付逐帧耗时、P95与RTF','结论':'定位受标签口径限制；未归档数值深度不能完整复算','位置':'各回合task1_evaluation.json; task1_frame_metrics.csv'},
        {'交付项':'逐回合任务结果','状态':'12/12已交付','结论':f'正例{payload["positive_passed"]}/8；负例拒绝{payload["negative_passed"]}/4；错误提交{payload["wrong_commits"]}；碰撞{payload["collisions"]}','位置':'analysis/online_summary.csv; online_results.json'},
        {'交付项':'五视角回放与完整轨迹','状态':'12/12修订并校验','结论':'原巡检路线＋触发点＋前后实际轨迹；已纠正类别匹配显示；760帧/回合、4fps','位置':'整改后可视化/索引.md; 各回合preview/'},
        {'交付项':'详细分析、问题与解决方法','状态':'已交付','结论':'模型验收未通过，执行与交付完成','位置':'任务1_在线验收报告.md; 问题闭环表.csv'},
    ]
    issues=[
        ['R01','训练／验证存在纯非车辆标签','污染训练和调参','全量审计并在独立版本移除812+269条；默认导出修复','仍需RGB可见性与完整标签审核后才能重训'],
        ['R02','无准入检查导致失败条件下扩大批量','浪费实验并混淆诊断和正式验收','新增入口检查；实际失败配置在启动CARLA前被阻断；141项回归通过','在新版本目录提交完整准入证据后再运行'],
        ['R03','目标对象关联被画成预测类别匹配','car预测落在van目标上也被标绿','12回合视频修订为CLS_OK/CLS_ERR；原始预测与任务结论不变','后续保持对象关联与类别正确分别记录'],
        ['A01','RGB可见性与实例标签不一致','阻断可靠精度评价','已定位并隔离诊断，未完成全数据清洗','联合RGB/数值深度/姿态检查并标注真实可见性，重算验证与测试'],
        ['A02','actor-ID非车辆语义碰撞','污染自动标注','已提供可选语义过滤并新增2项回归','以新目录导出干净数据，RGB可见性仍需独立审计'],
        ['A03','微调后局部类别能力退化','阻断模型基线','已确认，尚未重新训练','清晰车型/视角回归集、类别平衡与标签清理后训练，并使用新独立测试'],
        ['A04','发现偏晚，绕行路线超出任务时间','阻断业务完成','已复现并量化','预检查首次可观测时机、关键路口和剩余路线预算，分开验收感知与规划'],
        ['A05','显式确认/完成业务状态未贯通','影响自动业务验收','本轮按冻结规则从日志重建','接通目标提交、本地确认、到达、完成/超时状态机'],
        ['A06','旧runner负例条件要求目标路线存在','导致假FAIL','外部业务判定已纠正，原检查保留','目标路线存在时才检查其保护，保留全程安全覆盖'],
        ['A07','正式数值深度未归档','限制定位复算','旧数据不可补回；隔离诊断保存数值深度','下轮至少保存推理帧数值深度和各相机实际位姿'],
        ['A08','主导颜色分数不等于目标属性证据','影响消息质量解释','已定位，当前运行版本保持固定','目标颜色比例与主导颜色置信度分字段，按任务属性排序'],
        ['A09','固定红色规则/custom来源/van映射/无线字段','阻断新模型链路','已修复，回归与实际消息来源审计通过','后续保持TaskSpec和消息来源记录统一'],
        ['A10','跨重启native CARLA客户端异常','仿真启动失败','已修复编排器并重跑；保留异常历史','不在编排器中跨重启保留native客户端'],
        ['A11','缺少可关联候选的反证反馈','错误目标可能持续被记忆','已从发送筛选与记忆逻辑确认，未在固定版本中改动','区分候选与已确认目标；为类别/颜色冲突增加验证失败反馈，保留TTL不直接取消已确认目标原则'],
    ]
    table(DELIVERY/'交付验收清单.csv',checklist)
    table(DELIVERY/'问题闭环表.csv',[dict(zip(['编号','问题','影响','本轮处理状态','解决方法'],r)) for r in issues])
    for name in ['message_quality_by_run.csv','latency_cold_start_and_warnings.csv','final_acceptance_decision.json','semantic_clean_reference.csv',
        'semantic_clean_reference.json','runtime_integrity_audit.json','evaluation_protocol_notes.json']:
        shutil.copy2(OUT/'analysis'/name,DELIVERY/name)
    shutil.copy2(OUT/'observability_diagnostic/visibility_roi_audit.png',DELIVERY/'可见性诊断.png')
    shutil.copy2(OUT/'analysis/clear_case_model_comparison.jpg',DELIVERY/'清晰样本分类退化.jpg')
    report=DELIVERY/'任务1_在线验收报告.md'
    appendix='\n**10. 最终交付与消息质量复核**\n\n'
    appendix+=f'最终机器判定：[{(OUT/"analysis/final_acceptance_decision.json").name}](<{(OUT/"analysis/final_acceptance_decision.json").as_posix()}>). 执行完成；通过与否由逐项检查产生，当前未通过。\n\n'
    appendix+='交付验收清单.csv与问题闭环表.csv保存在本报告同目录。消息级自动标签匹配、未证实消息和位置误差另见message_quality_by_run.csv；它们同样受RGB可见性标注限制。\n'
    appendix+='\n**另一个闭环缺口：候选反证。** 当前发送池只保留与指令类别、颜色相符的候选；可关联到同一车辆但类别/颜色不符的地面观测会被过滤，因此接收端未必得到否定先前目标假设的证据。持久记忆又会跨TTL保留最初目标。后续应区分待验证候选与已确认目标，并处理冲突证据；这与“消息过期不直接取消已确认任务目标”的既定原则并不冲突。本轮保持原固定版本，未宣称已修复此缺口。\n'
    appendix+='\n本轮对GOC主线的直接启示是：稳定高置信识别也可能造成错误提交，而较晚收到正确目标信息会使路线成本大幅增加。后续信息价值需要与任务阶段、可靠性和信息时效相联系；这些是待验证设计依据，不能由本轮固定策略实验宣称GOC已经有效。\n'
    appendix+='\n**冷启动、长尾与时钟限制。** latency_cold_start_and_warnings.csv保留全帧P95、最大耗时、前两次耗时、超过250ms的次数及NMS超时警告数，避免稳态P95掩盖长尾。当前测量覆盖检测和候选生成，不是完整传感器到控制端到端时延；仿真会随墙钟计算暂停，0碰撞也不能当作真实延迟下的部署安全验证。后续应预热模型并统一信息可用时间。\n'
    appendix+='\n**自动标签污染的范围。** 在2400张回放图像的4858个actor-ID掩码中，297个掩码包含非车辆语义，其中140个达到80像素门槛且没有车辆语义像素。语义过滤重算修正297张图像，掩码数由4858变为4562；这只修正语义/ID问题，不能替代RGB遮挡可见性审计，也不足以断言原训练集全部出错。\n'
    appendix+='\n**明确误选与未证实提交分开。** 两个蓝色轿车回合均将已标注蓝van当作car提交，属于2次明确误选。yellow_bus_42102另有1次提交与实例标签框的IoU约0.302，未达到既定0.5验证门槛；它属于未证实关联，不能据此认定已选错车辆，也不升级为合格提交。最初v3评估将二者都合并为未通过提交，旧JSON以v3_before_commit_split命名保留；v4区分2次明确误选与1次未证实提交。阈值、正式模型、终点判据和任务失败结论不变。\n'
    diagnostic=DELIVERY/'完整诊断记录.md'
    diagnostic.write_text(diagnostic.read_text(encoding='utf-8')+appendix,encoding='utf-8')
    (OUT/'完整诊断记录.md').write_text(diagnostic.read_text(encoding='utf-8'),encoding='utf-8')
    generated=subprocess.run([sys.executable,str(ROOT/'scripts/build_task1_reaudit_report.py')],cwd=ROOT,check=True)
    model_config=ROOT/'configs/perception/yoloe26s_town10hd_adapted.yaml'
    if model_config.exists():
        original_copy=OUT/'prechange_snapshot/yoloe26s_town10hd_adapted.yaml'
        if not original_copy.exists():shutil.copy2(model_config,original_copy)
        config=yaml.safe_load(model_config.read_text(encoding='utf-8'))
        config['perception_model']['status']='online_evaluated_not_accepted'
        config['perception_model']['task_thresholds']='not_accepted_diagnostic_only'
        config['perception_model']['online_acceptance_report']=str(report)
        config['perception_model']['online_acceptance_root']=str(OUT)
        model_config.write_text(yaml.safe_dump(config,allow_unicode=True,sort_keys=False),encoding='utf-8')
        files.append((model_config,'implementation/yoloe26s_town10hd_adapted.yaml'))
    summary=(f'# 任务1在线验收记录\n\n任务1尚未完成合格在线验收；12回合为失败诊断，不作为已建立的感知基线。'
        f'正例终点{payload["positive_passed"]}/8，负例拒绝{payload["negative_passed"]}/4，'
        f'明确误选{payload["wrong_commits"]}次、未证实提交{payload["unverified_commits"]}次，记录碰撞{payload["collisions"]}次。\n\n'
        '已确认清晰样本类别退化；RGB可见性与实例标签存在不一致。原帧级精度和定位率只能作为探索统计。'
        '优先修复验收标签与可见性，再修模型、发现延迟和任务可达性，复验后再建立正式多模态基线。'
        '本轮未改变已约定GOC主线，也未固定接收端融合方案。'
        '待解决事项还包括候选反证反馈与已确认目标记忆的区分；继续保留TTL不直接取消已确认目标原则。'
        '置信度不等同任务收益、发现时机改变路线代价，是本轮观察支持的后续设计依据，尚不是GOC有效性结论。\n\n'
        f'详细报告：[{report.name}](<{report.as_posix()}>)\n')
    (ROOT/'docs/experiments/task1_online_acceptance_20261021.md').write_text(summary,encoding='utf-8')
    memory=DELIVERY.parents[1]/'课题讨论记录/任务1_在线验收_结论与后续门槛_20261021.md'
    memory.parent.mkdir(parents=True,exist_ok=True);memory.write_text(summary,encoding='utf-8')
    summary+='\n整改复核：全量审计原9163图，独立版本移除812条训练及269条验证纯非车辆标签，测试标签保持不变；RGB可见性和静态车辆标注仍待处理。补做旧四类场景32帧4模型诊断；12个视频补原巡检路线与重规划触发点，纠正类别匹配显示；新增正式准入检查并实测阻断，全套141项工程回归通过。未重训、未新增正式在线回合。\n'
    (ROOT/'docs/experiments/task1_online_acceptance_20261021.md').write_text(summary,encoding='utf-8')
    memory.write_text(summary,encoding='utf-8')
    for p in DELIVERY.rglob('*'):
        if p.is_file() and p.name not in ['审阅包文件清单.json','审阅包校验.json']:files.append((p,f'report/{p.relative_to(DELIVERY).as_posix()}'))
    for folder in ['calibration','configs','preflight','code_snapshot','observability_diagnostic']:
        files.extend((p,f'{folder}/{p.relative_to(OUT/folder).as_posix()}') for p in (OUT/folder).rglob('*') if p.is_file())
    files.extend((p,f'analysis/{p.relative_to(OUT/"analysis").as_posix()}') for p in (OUT/'analysis').rglob('*') if p.is_file())
    files.extend((p,f'reaudit_frozen_baseline/{p.relative_to(OUT/"reaudit_frozen_baseline").as_posix()}') for p in (OUT/'reaudit_frozen_baseline').rglob('*') if p.is_file())
    for p in ['formal_admission_rejection_retest.log','reaudit_regression_tests.log','training_label_reaudit.log','semantic_label_repair.log']:
        files.append((OUT/p,f'reaudit_logs/{p}'))
    files.append((ROOT/'src/evaluation/acceptance_readiness.py','implementation/src/evaluation/acceptance_readiness.py'))
    files.append((ROOT/'tests/test_acceptance_readiness.py','implementation/tests/test_acceptance_readiness.py'))
    files.append((ROOT/'scripts/build_synchronized_multiview_replay.py','implementation/build_synchronized_multiview_replay.py'))
    for name in ['manifest.jsonl','summary.json','test_results.json','adapted_predictions.jsonl','original_predictions.jsonl']:
        files.append((OUT/'benchmark'/name,f'benchmark/{name}'))
    for p in (OUT/'logs').rglob('*.log'):files.append((p,f'logs/{p.relative_to(OUT/"logs").as_posix()}'))
    for name in ['smoke_batch.log','smoke_batch_retry.log','formal_batch.log','formal_batch_resume.log',
        'calibration.log','observability_diagnostic.log','semantic_clean_reference.log','clear_case_comparison.log']:
        files.append((OUT/name,f'logs/{name}'))
    for name in ['protocol.json','online_index.json','attempt_history.jsonl','smoke_index.json','smoke_initial_schema_failure.json','regression_tests.log']:
        files.append((OUT/name,name))
    for p in (ROOT/'scripts').glob('*task1*.py'):files.append((p,f'implementation/{p.name}'))
    for name in ['perception_online_acceptance.py','run_online_acceptance_batch.py','analyze_online_acceptance.py',
        'package_online_acceptance_results.py','audit_acceptance_failures.py','audit_static_vehicle_labels.py',
        'review_acceptance_benchmark.py','recompute_semantic_clean_reference.py','check_instance_label_integrity.py',
        'audit_online_acceptance_integrity.py']:
        files.append((ROOT/'scripts'/name,f'implementation/{name}'))
    files.append((ROOT/'scripts/export_instance_segmentation_dataset.py','implementation/export_instance_segmentation_dataset.py'))
    for name in ['test_instance_dataset_export.py','test_candidate_pipeline_geometry.py','test_persistent_target.py','test_online_acceptance_evaluation.py']:
        files.append((ROOT/'tests'/name,f'implementation/tests/{name}'))
    manifest=[]
    seen=set()
    for path,arc in files:
        if arc in seen or not path.exists():continue
        seen.add(arc);manifest.append({'archive_path':arc,'source_path':str(path),'bytes':path.stat().st_size,'sha256':sha256(path)})
    save_json(DELIVERY/'审阅包文件清单.json',manifest)
    destination=OUT/'任务1_验收审阅包_20261021.zip'
    with zipfile.ZipFile(destination,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=4) as archive:
        for record in manifest:archive.write(record['source_path'],record['archive_path'])
        archive.write(DELIVERY/'审阅包文件清单.json','审阅包文件清单.json')
        archive.writestr('请先阅读.txt','请先阅读report/任务1_在线验收报告.md、交付验收清单.csv及问题闭环表.csv。\n原始RGB/实例/彩色深度源数据保留在E盘实验目录，审阅包包含12个完整回放、轨迹、指标、配置、关键诊断与代码快照。\n帧级精度受RGB可见性标注限制；不能据这些参考数字宣称模型已通过验收。\n')
    with zipfile.ZipFile(destination) as archive:
        bad=archive.testzip()
        if bad:raise RuntimeError(f'ZIP integrity failed: {bad}')
    save_json(DELIVERY/'审阅包校验.json',{'path':str(destination),'bytes':destination.stat().st_size,'sha256':sha256(destination),
        'zip_crc_check':'passed','files':len(manifest)+2,'final_acceptance_passed':payload['model_acceptance_passed']})
    if not all(checks[name][p] for name in checks for p in checks[name]):raise RuntimeError('Incomplete deliverables')
    print('FINAL DELIVERED',json.dumps(payload,ensure_ascii=False)[:1500],flush=True)
    print('REPORT',report,flush=True);print('ZIP',destination,flush=True)

if __name__=='__main__':main()
