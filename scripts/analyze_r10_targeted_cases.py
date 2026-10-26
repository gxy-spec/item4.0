"""Per-case timing, waiting and remaining-budget diagnosis; no new success scores."""
from pathlib import Path
import sys,json,csv,collections,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,rows
from build_r10_targeted_report import DEST


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8')) if Path(p).exists() else {}


def analyze(item,cohort):
    run=Path(item['run_dir']);e=read(run/'r10_final_evaluation.json') or read(run/'r10_evaluation.json')
    cfg=yaml.safe_load((run/'resolved_config.yaml').read_text(encoding='utf-8'))
    safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
    first=float(safety[0]['timestamp']);delta=float(cfg['simulation']['fixed_delta_seconds'])
    modes=collections.Counter(r['ugv_safety_mode'] for r in safety)
    planpath=run/'planning/target_route_events.jsonl'
    plans=[p for p in rows(planpath) if p['event']=='target_route_planned_from_selected_message'] if planpath.exists() else []
    monitor=read(run/'ground_truth/evaluation_only/sensor_business_monitor.json');w=read(run/'logs/r10_recovery_watchdog.json')
    claims=[a for a in e.get('completion_audits',[]) if a['passed']]
    complete_at=min((a['event']['timestamp_s'] for a in claims),default=None)
    near_at=min((a['timestamp_s'] for a in e.get('local_stage_observations',[]) if a['identity_correct']),default=None)
    wait=sum(1 for r in safety if (complete_at is None or float(r['timestamp'])-first<complete_at)
        and any(word in r['ugv_safety_mode'] for word in ['wait_','obstacle_stop','stale_stop','bounded_blockage']))*delta
    plan_at=plans[0]['timestamp_s']-first if plans else None
    length=plans[0].get('route_length_m') if plans else None
    remaining=cfg['simulation']['duration_seconds']-plan_at if plans else None
    nominal=cfg['dynamic']['ugv_target_speed_mps'];nominal_time=length/nominal if length is not None else None
    cause=''
    if cohort=='safety':cause='原安全检查通过；超车若仅有cruise字段误拒，独立物理审核另列。'
    elif item.get('group')=='A':cause='巡检对照，不要求响应目标；实际初态、采样及轨迹以N3配对审核为准。'
    elif e.get('expected_target_present') is False:
        cause='目标不存在；检查是否错误发消息或响应，而不是要求到达。'
    elif e.get('S3_found_and_safely_approached'):
        cause='正确目标、连续近距确认、车体净距及两秒停稳独立审核通过。'
        if any(a.get('passed') and a.get('stopped_confirmation_in_junction') for a in e.get('completion_audits',[])):
            cause+=' 确认位置在路口连接区，按用户批准的短暂停稳规则评价，已单独标注。'
        if any(x['event']=='blocked_route_resumed_after_fresh_clear' for x in w.get('events',[])):cause+=' 曾遇实体阻挡并超时保护停车，清空证据成立后恢复，阻塞历史保留。'
    elif e.get('collisions'):cause='存在碰撞，安全验收未通过，不能用距离或身份正确抵消。'
    elif w.get('failed'):
        cause='结束时仍处于保护阻塞状态；须结合危险检测、道路几何和视频区分实体阻挡、误停或不可执行路线。'
        if item['case']=='F1' and (run/'r10_saved_depth_route_probe.json').exists():
            cause+=' 本次首条路线已缩短至275.29米，但深度保护在约3.47米处检测到约0.15米高的小面积障碍；4次地图重规划仍经过受阻走廊，未能局部绕行。离线实例标签为植被，只用于原因解释，未输入控制。S1若未达到，表示仅有候选核验规划，不能说已可靠发现。详见F1路线修复补测说明。'
    elif length is not None and nominal_time>remaining:
        cause=f'首条计划长{length:.1f}米，按当前{nominal:.1f}米/秒需约{nominal_time:.1f}秒，当时仅余{remaining:.1f}秒；该计划时间预算不足。此计算不证明不存在别的路线或探索策略。'
    elif not e.get('S1_correct_discovery'):cause='未形成正确可靠发现；不能归因为停车距离。需区分有效视野不足与识别漏检。'
    elif not e.get('S2_correct_ground_confirmation'):cause='已有正确发现，但没有正确近距地面确认；尚未走完整个导航接近过程，不能只放宽到达阈值。'
    else:cause='已经地面确认，但没有通过全部停车保持条件，须逐项检查估计净距、速度、观测连续性及车体道路证据。'
    return dict(cohort=cohort,case=item['case'],variant=item.get('variant'),group=item.get('group'),run_dir=str(run),
        acquisition_revision=item.get('development_revision'),evidence_reuse=item.get('evidence_reuse'),
        S1=e.get('S1_correct_discovery'),S2=e.get('S2_correct_ground_confirmation'),S3=e.get('S3_found_and_safely_approached'),
        S3_applicable=cohort!='safety' and item.get('group')!='A' and e.get('expected_target_present') is not False,
        first_ground_confirmation_s=near_at,completion_s=complete_at,first_plan_s=plan_at,
        first_route_length_m=length,remaining_at_first_plan_s=remaining,nominal_route_time_s=nominal_time,
        waiting_before_completion_s=wait,safety_mode_seconds={k:v*delta for k,v in modes.items()},
        final_true_body_gap_m=e.get('final_true_body_clearance_m'),final_reference_distance_m=e.get('final_target_distance_m'),
        collision_count=e.get('collisions',item.get('summary',{}).get('ugv_collision_count')),
        raw_runner_status=item.get('runner_status'),reason_codes=e.get('reasons'),analysis=cause,
        watchdog=w,ground_filter=read(run/'logs/r10_ground_vehicle_filter.json'))


def main():
    rev=read(OUT/'execution_scope.json')['active_validation_revision'];result=[]
    for cohort in ['safety','acceptance','diagnostics']:
        for item in read(OUT/f'{cohort}_development_{rev}_progress.json').values():
            if item.get('report_complete'):result.append(analyze(item,cohort))
    save(OUT/'analysis/targeted_case_diagnostics.json',dict(revision=rev,records=result))
    lines=['# 逐回合解释','',f'活动版本：{rev}；当前{len(result)}/22个回合。原采集版本与证据复用信息在逐回合JSON/CSV中保留。','',
        'S1=正确可靠发现，S2=正确近距地面确认，S3=找到正确目标并安全接近完成；A组和无目标负例不以S3作为必须成功的任务。等待时间仅统计完成前的危险等待/阻塞状态，不把成功后的停稳保持计作误等待。','']
    for r in result:
        def metric(x):return '不适用' if x is None else f'{x:.2f}'
        def stage(x):return '不适用' if x is None else '通过' if x else '未达到'
        stages='不适用' if not r['S3_applicable'] else ' / '.join(stage(r[x]) for x in ['S1','S2','S3'])
        lines += [f"## {r['cohort']} / {r['variant'] or ''} {r['case']} {r['group'] or ''}",'',r['analysis'],'',
            f"S1/S2/S3：{stages}；最终车体净距：{metric(r['final_true_body_gap_m'])}米；原参考点距：{metric(r['final_reference_distance_m'])}米；碰撞：{r['collision_count']}；完成前等待/阻塞：{r['waiting_before_completion_s']:.2f}秒。",'',
            f"原始运行器：{r['raw_runner_status']}；机器判定原因：{'; '.join(r['reason_codes'] or []) or '无，或该项不适用'}。",'',f"证据目录：[{Path(r['run_dir']).name}]({r['run_dir'].replace(chr(92),'/')})",'']
    DEST.mkdir(exist_ok=True,parents=True);(DEST/'逐回合分析.md').write_text('\n'.join(lines),encoding='utf-8')
    print('ANALYZED',len(result),'completed runs')


if __name__=='__main__':main()
