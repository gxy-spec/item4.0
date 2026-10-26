"""Report the actual supplementary failure/success, never infer from offline path."""
from pathlib import Path
import sys,json,csv,collections,statistics
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,rows,save,digest
from build_r10_targeted_report import DEST


def summarize(run):
    e=json.loads((run/'r10_final_evaluation.json').read_text(encoding='utf-8'))
    plans=[a for a in rows(run/'planning/target_route_events.jsonl') if a['event']=='target_route_planned_from_selected_message']
    safety=list(csv.DictReader((run/'safety/safety_state.csv').open(encoding='utf-8-sig')))
    origin=float(safety[0]['timestamp']);first=plans[0]
    wait=collections.Counter(a['ugv_safety_mode'] for a in safety if any(x in a['ugv_safety_mode'] for x in ['stop','wait','bounded_blockage']))
    budget=rows(run/'planning/r10_route_budget_decisions.jsonl') if (run/'planning/r10_route_budget_decisions.jsonl').exists() else []
    return dict(run_dir=str(run),evaluation_sha256=digest(run/'r10_final_evaluation.json'),
        S1=e['S1_correct_discovery'],S2=e['S2_correct_ground_confirmation'],S3=e['S3_found_and_safely_approached'],
        collisions=e['collisions'],final_true_body_gap_m=e['final_true_body_clearance_m'],
        first_plan_s=first['timestamp_s']-origin,first_plan_length_m=first['route_length_m'],
        wait_seconds={k:n*.05 for k,n in wait.items()},budget_decisions=budget)


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text());rev=scope['active_validation_revision']
    progress=json.loads((OUT/f'diagnostics_development_{rev}_progress.json').read_text());item=progress['upgraded_F1_C']
    if not item.get('report_complete'):raise RuntimeError('Wait for full physical run and independent evaluation')
    run=Path(item['run_dir']);old=Path('D:/CI10/diagnostics/upgraded_F1_C/CI_E7_R10_dia_05_20261023T144507Z')
    before=summarize(old);after=summarize(run)
    probe=json.loads((run/'r10_saved_depth_route_probe.json').read_text());points=probe['records'][0]['support_points']
    save(OUT/'analysis/F1_route_followup.json',dict(before=before,after=after,recorded_depth_probe=probe,
        probe_is_4Hz_offline_and_recomputed_route_not_full_live_history=True,
        offline_label_never_used_by_controller=True))
    lines=['# F1 路线修复补测：实际结果与停住原因','',
        f"补测最终：S1={after['S1']}，S2={after['S2']}，S3={after['S3']}，碰撞={after['collisions']}。**离线较短路线不等于闭环完成；本次按实际结果记账。**",'',
        '|项目|修复前 v4|补测 v5|','|---|---:|---:|',
        f"|首条计划长度|{before['first_plan_length_m']:.2f}米|{after['first_plan_length_m']:.2f}米|",
        f"|收到消息后首条计划时刻|{before['first_plan_s']:.2f}秒|{after['first_plan_s']:.2f}秒|",
        f"|最终真实车体净距|{before['final_true_body_gap_m']:.2f}米|{after['final_true_body_gap_m']:.2f}米|",
        f"|任务完成 S3|{before['S3']}|{after['S3']}|",'',
        '目标、种子、场景、时限、模型和速度控制设置不变。新增方法仅在原计划超过剩余时间预算时，根据已收到的传感器点、自身速度历史和地图，查询可观察并接近目标的区域；局部安全检查继续生效。该比较没有声称两回合的 RGB 逐像素一致。','',
        '补测中，在约(-48.77,-26.00)米位置出现持续停住。保存深度的独立复核显示，前方约3.47米处有3个采样点，距车体参考平面约0.15米，触发原有4米深度停车规则。此时保存图像的补充安全检测回放没有检出车辆或行人；因此不能把本次阻塞再次称作车辆类别误检。离线实例标签为Vegetation，只用于解释，从未输入控制或用于删除障碍。','',
        '当前短处是没有为这种低矮、小面积、未被车辆/行人检测器识别的深度障碍提供可执行的局部绕行路径。重复查询道路地图仍可能回到同一受阻走廊。单帧重算深度路线探针不是完整在线历史复现，实际等待状态及持续时间以保存的 safety_state.csv 和恢复日志为准。','',
        '后续处理应在危险警告阶段，将传感器观测到的占据区域加入局部路径规划，提前验证绕行走廊的车体空间和深度可通行性，并补做低矮障碍、真实车辆、行人和换道回归。当前不通过抬高障碍高度阈值、删除植被或关闭制动来换取任务成功。是否能安全跨越该物体也没有经过验证，不据离线标签擅自放行。','',
        f"[补测原始数据]({str(run).replace(chr(92),'/')})；[修复前保留失败]({str(old).replace(chr(92),'/')})；[原始数值与预算决策](E:/CarlaAirData/CityInspection_GOC/perception_repair_20261023_r10/analysis/F1_route_followup.json)",'']
    (DEST/'F1路线修复补测说明.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(dict(before=before['S3'],after=after['S3'],route_m=[before['first_plan_length_m'],after['first_plan_length_m']]),ensure_ascii=False))


if __name__=='__main__':main()
