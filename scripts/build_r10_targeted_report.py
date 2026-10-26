"""Readable targeted-validation report; never imply full36/72 comparison."""
from pathlib import Path
import sys,json,csv,shutil
from datetime import datetime
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,rows

DEST=Path('C:/Users/Administrator/Documents/ChatGPT/课题1/实验结果/本轮修复定向验收_R10_20261023')


def read(p):
    try:return json.loads(Path(p).read_text(encoding='utf-8'))
    except (OSError,json.JSONDecodeError):return {}


def write_csv(path,records):
    if not records:return
    keys=list(dict.fromkeys(k for r in records for k in r))
    with path.open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys);w.writeheader();w.writerows(records)


def main():
    DEST.mkdir(parents=True,exist_ok=True);scope=read(OUT/'execution_scope.json');revision=scope['active_validation_revision']
    all_results=[];flat=[]
    for cohort in ['safety','acceptance','diagnostics']:
        progress=read(OUT/f'{cohort}_development_{revision}_progress.json')
        for key,item in progress.items():
            run=Path(item['run_dir']);r=read(run/'r10_final_evaluation.json') or read(run/'r10_evaluation.json');monitor=read(run/'ground_truth/evaluation_only/sensor_business_monitor.json')
            filter_log=read(run/'logs/r10_ground_vehicle_filter.json');recovery=read(run/'logs/r10_recovery_watchdog.json')
            safety_final=read(run/'r10_safety_final_evaluation.json')
            audits=[a for a in r.get('completion_audits',[]) if a.get('passed')]
            hold=audits[0] if audits else {}
            measures=hold.get('measurements',[])
            s3_applicable=bool(cohort!='safety' and item.get('group')!='A' and r.get('expected_target_present') is not False)
            if cohort=='safety':verdict='安全回归通过' if safety_final.get('passed',item.get('runner_status')=='PASS') else '安全回归未通过'
            elif not r:verdict='采集结束，独立核验待完成'
            elif item.get('group')=='A':verdict='不响应目标的对照；配对轨迹另审计'
            elif not r.get('expected_target_present',True):verdict='负例拒绝通过' if r.get('negative_rejection_pass') and not r.get('sent_messages') else '负例拒绝未通过'
            else:verdict='找到并安全接近通过' if r.get('S3_found_and_safely_approached') else '任务未完成，保留失败'
            flat.append(dict(suite=cohort,case=item['case'],variant=item.get('variant'),group=item.get('group'),
                acquisition_revision=item.get('development_revision'),reused_evidence=bool(item.get('evidence_reuse')),
                runner=item.get('runner_status'),verdict=verdict,safety_label_reassessment=safety_final.get('label_only_reassessment'),S1=r.get('S1_correct_discovery'),S2=r.get('S2_correct_ground_confirmation'),
                S3=r.get('S3_found_and_safely_approached') if s3_applicable else None,S3_applicable=s3_applicable,
                stopped_confirmation_in_junction=hold.get('stopped_confirmation_in_junction'),
                drivable_hold=hold.get('drivable_footprint_pass'),hold_seconds=hold.get('event',{}).get('hold_seconds'),
                hold_min_true_body_gap_m=min((x['true_body_clearance_m'] for x in measures),default=None),
                hold_max_true_body_gap_m=max((x['true_body_clearance_m'] for x in measures),default=None),
                hold_max_speed_mps=max((x['speed_mps'] for x in measures),default=None),
                true_body_gap_m=r.get('final_true_body_clearance_m'),
                reference_distance_m=r.get('final_target_distance_m'),collisions=r.get('collisions',item.get('summary',{}).get('ugv_collision_count')),
                flat_ground_vehicle_rejections=filter_log.get('rejected'),bounded_blockage_failure=recovery.get('failed'),
                had_blockage_timeout=bool(recovery.get('ever_failed') or any(e.get('event')=='bounded_blockage_failure' for e in recovery.get('events',[]))),
                recovered_after_blockage=any(e.get('event')=='blocked_route_resumed_after_fresh_clear' for e in recovery.get('events',[])),
                reason=';'.join(r.get('reasons',[])),run_dir=str(run),wall_seconds=item.get('wall_seconds')))
            all_results.append(dict(suite=cohort,key=key,item=item,evaluation=r))
    safety=[r for r in flat if r['suite']=='safety'];online=[a for a in all_results if a['suite']=='acceptance'];diagnostics=[r for r in flat if r['suite']=='diagnostics']
    positives=[a for a in online if a['evaluation'].get('expected_target_present') is True]
    negatives=[a for a in online if a['evaluation'].get('expected_target_present') is False]
    pos_ok=sum(bool(a['evaluation'].get('S3_found_and_safely_approached')) for a in positives)
    neg_ok=sum(bool(a['evaluation'].get('negative_rejection_pass') and not a['evaluation'].get('sent_messages')) for a in negatives)
    safe_ok=sum(r['verdict']=='安全回归通过' for r in safety)
    reuse_count=sum(r['reused_evidence'] for r in flat)
    complete=len(flat)==22 and all(a['evaluation'] or a['suite']=='safety' for a in all_results)
    geometry=read(OUT/'geometry_verified/clearance_validation/summary.json')
    save(DEST/'定向验收完整结果.json',all_results);write_csv(DEST/'22回合验证明细.csv',flat)
    attempts=rows(OUT/'attempt_history.jsonl');ledger=[]
    for a in attempts:
        actual_run=Path(a['run_dir']) if a.get('run_dir') else None
        business=(read(actual_run/'r10_final_evaluation.json') or read(actual_run/'r10_evaluation.json')) if actual_run else {}
        ledger.append(dict(phase=Path(a['log']).parent.name if a.get('log') else None,case=a.get('case'),group=a.get('group'),variant=a.get('variant'),
            complete=a.get('report_complete'),runner=a.get('runner_status'),error=a.get('error'),
            S3=business.get('S3_found_and_safely_approached'),business_verdict=business.get('business_verdict'),
            business_reasons=';'.join(business.get('reasons',[])),
            failed_checks=json.dumps(a.get('failed_checks',[]),ensure_ascii=False),run_dir=a.get('run_dir'),log=a.get('log')))
    write_csv(DEST/'全部运行尝试_含失败.csv',ledger)
    lines=['# R10问题修复：定向验收报告','',
        f'更新：{datetime.now().strftime("%Y-%m-%d %H:%M")}。当前为'+('已完成22回合采集与独立评价。' if complete else f'验证进行中，已有{len(flat)}/22回合记录。'),'',
        '**本轮暂不运行完整72回合，因此不报告新的完整36对成功率，也不据此声称新模型总体优势增加。**','',
        f'当前活动版本：{revision}。其中{reuse_count}个回合复用此前采集证据，原采集版本在明细中保留。v3→v4逐tick复核阻塞恢复分支未触发；v4→v5逐次复核规划器返回原路线且终点容差仍为3米。受到修改影响的蓝色厢式车和F1分别重新采集；此前失败保留。','',
        '|验收部分|计划|当前通过|','|---|---:|---:|',
        f'|真实安全场景回归|4|{safe_ok}/4|',f'|正确目标在线查找与接近|8|{pos_ok}/8|',f'|无目标/干扰负例拒绝|2|{neg_ok}/2|',
        f'|原问题场景与时序对照|8|已获得{len(diagnostics)}回合结果，逐项归因|','',
        '## 1. 最重要的距离问题怎么处理','',
        '主指标采用两车车体在水平面上的最小净距：真实净距3–8米，并对正确目标持续近距确认、低速停稳2秒、处于可行驶区域且全回合无碰撞。在线估计保持区间为4–7米；参考点距离单独保留。允许路口连接区内的短暂停稳，额外标注位置，不将其称为现实道路合法停车。','',
        f'四车型的另一地点、视角和距离静态验证共{geometry.get("total",0)}个条件，有效{geometry.get("valid",0)}个；平均绝对测距误差{geometry.get("mae_m",0):.2f}米，最大{geometry.get("maximum_absolute_error_m",0):.2f}米。这是有限条件下的测距验证，不替代动态停车验收。','',
        '|在线正例|真实车体净距/m|原参考点距/m|独立任务验收|','|---|---:|---:|---|']
    for a in positives:
        r=a['evaluation'];gap=r.get('final_true_body_clearance_m');ref=r.get('final_target_distance_m')
        lines.append(f"|{a['item']['case']}|{gap:.2f}"+f"|{ref:.2f}|"+('通过' if r.get('S3_found_and_safely_approached') else '未通过')+'|' if gap is not None and ref is not None else f"|{a['item']['case']}|缺少完整测量|—|未通过|")
    lines+=['','## 2. 修复与剩余问题','',
        '- **路面假车辆：**用真实深度高度支撑排除平面纹理；三个历史误检关键帧均已回归。不通过提高检测阈值或关闭避障取得结果。',
        '- **转弯与换道：**缩短转弯前视，深度检查跟随实际控制路线。开发中确实发现过“检查原车道、控制已换道”的新退化，已保留失败并修正，随后重验四类安全场景。',
        '- **安全验收口径：**最终超车实际完成并停回原路线，但运行器的旧判据要求随后必须出现cruise字样，原始报告因此保留FAIL。新增物理审核核验车体完全越过障碍、回到原路线并停稳2秒；历史两次真正未完成的超车仍不通过，明细同时列出原标签与独立审核。',
        '- **阻塞后恢复：**蓝色厢式车开发回合被真实干扰车挡住，干扰车离开后仍被永久制动锁死；修正为保持安全检查，连续1秒获得新鲜、清空且无强制危险制动的证据后才恢复尝试。该修改不提前移除干扰车、不延长时限，原失败保留。',
        '- **确认与停车：**区分远距候选、可靠发现、近距地面确认和完成；已近距确认的同一可见目标允许严格几何跟踪延续，未知属性不冒充新的正确分类。',
        '- **路线和真值：**回放保留原巡检线，候选规划与可靠目标规划分色，逐帧只显示当时已有事件；真值只作离线身份、距离和画图参照。具体消息到规划链的核查另有证据，不能扩大为全仿真无真值访问的证明。',
        '- **仍须如实报告：**E2开发复测虽排除了假车辆误停，仍因长路线未完成；地图查询的接近区域方案只作诊断，尚未接入在线控制。最终问题场景结果按下面表格保留，不能把运行器PASS当作任务通过。','',
        '|诊断回合|任务结果|真实净距/m|说明|','|---|---|---:|---|']
    for r in diagnostics:
        gap='—' if r['true_body_gap_m'] is None else f"{r['true_body_gap_m']:.2f}"
        lines.append(f"|{r['variant']}_{r['case']}_{r['group']}|{r['verdict']}|{gap}|{r['reason'] or '详见逐回合证据'}|")
    lines+=['','## 3. 怎样看这份结果','',
        '模型权重保持冻结。本轮主要检验测距、确认、安全、导航和评价修复，不将共同控制修复的效果归因于识别网络精度提升。空中深色车型弱项按约定延期，已有独立图像审计不改写。','',
        '完整数值见“22回合验证明细.csv”；所有闭环运行尝试（含失败、重试）见“全部运行尝试_含失败.csv”，静态标定与代码回归开发记录另列。原R9实验、代码与报告保留。只有本报告明确列出的实测条件能支撑本轮结论。','']
    (DEST/'修复定向验收报告.md').write_text('\n'.join(lines),encoding='utf-8')
    save(OUT/'report_status.json',dict(report=str(DEST/'修复定向验收报告.md'),complete=complete,records=len(flat),planned=22,
        safety_pass=safe_ok,online_positive_pass=pos_ok,negative_pass=neg_ok,total_attempts=len(attempts)))
    print(json.dumps(read(OUT/'report_status.json'),ensure_ascii=False))


if __name__=='__main__':main()
