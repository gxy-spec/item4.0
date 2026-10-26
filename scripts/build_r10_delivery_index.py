"""Human entry point linking every counted run and every retained attempt."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT
from build_r10_targeted_report import DEST


def read(p):return json.loads(Path(p).read_text(encoding='utf-8')) if Path(p).exists() else {}
def link(label,p):return f'[{label}]({str(p).replace(chr(92),"/")})' if Path(p).exists() else '待生成'
def video(run):
    h=Path(run)/'preview/r10_v3_five_view_h264.mp4'
    return h if h.exists() else Path(run)/'preview/r10_v3_five_view.mp4'


def main():
    scope=read(OUT/'execution_scope.json');rev=scope['active_validation_revision'];all_runs=[]
    for cohort in ['safety','acceptance','diagnostics']:
        for item in read(OUT/f'{cohort}_development_{rev}_progress.json').values():
            if item.get('report_complete'):all_runs.append((cohort,item))
    lines=['# 交付与证据入口','',f'当前有{len(all_runs)}/22个定向验证回合。完整72回合未运行。','',
        '建议先看主报告，再按问题查看分项依据；所有开发失败与运行尝试均保留。','',
        '- '+link('简明主报告',DEST/'修复定向验收报告.md'),
        '- '+link('S1/S2/S3任务成功判据',DEST/'任务成功判据.md'),
        '- '+link('22回合明细（含采集版本、路口标注及停稳段数值）',DEST/'22回合验证明细.csv'),
        '- '+link('逐回合原因解释',DEST/'逐回合分析.md'),
        '- '+link('F1路线修复补测及低矮障碍停住分析',DEST/'F1路线修复补测说明.md'),
        '- '+link('N3严格轨迹配对未通过的原因与边界',DEST/'N3对照一致性说明.md'),
        '- '+link('测距标定与独立验证',DEST/'测距验证说明.md'),
        '- '+link('感知指标、原始分母及冻结审计依据',DEST/'感知指标与依据.md'),
        '- '+link('全部运行尝试，含失败',DEST/'全部运行尝试_含失败.csv'),
        '- '+link('研究主线及后续工作边界',DEST/'课题主线与当前交付边界.md'),'',
        '## 本轮回合的回放与原始证据','',
        '视频保留原巡检虚线，规划与实际轨迹分开显示；金色为候选核验规划，绿色为可靠发现声明触发的规划，橙色为收到消息后的实际UGV轨迹。红色目标位置为离线参照。各帧只画当时已经发生的规划与实际轨迹；静态轨迹图则是全回合概览。','',
        '原始task1_evaluation.json及r10_evaluation.json保留旧参考点指标和历史字段用于追溯；本轮最终任务结论查看下表链接的r10_final_evaluation.json，安全场景查看r10_safety_final_evaluation.json。运行器完成、旧指标与新任务成功三者分别解释。','',
        '|部分/回合|采集版本|结论|回放|轨迹总览|独立评价|原始目录|','|---|---|---|---|---|---|---|']
    for cohort,item in all_runs:
        run=Path(item['run_dir']);evaluation=run/('r10_safety_final_evaluation.json' if cohort=='safety' else 'r10_final_evaluation.json');e=read(evaluation)
        if cohort=='safety':verdict='安全通过' if e.get('passed',item.get('runner_status')=='PASS') else '未通过'
        else:verdict=e.get('business_verdict','待独立评价')
        revtext=item.get('development_revision','')+('（核验复用）' if item.get('evidence_reuse') else '')
        label=f"{cohort}/{item.get('variant','')}_{item['case']}_{item.get('group','')}"
        lines.append('|'+label+'|'+revtext+'|'+verdict+'|'+link('视频',video(run))+'|'+link('总览图',run/'preview/r10_v3_trajectory_map.png')+'|'+link('JSON',evaluation)+'|'+link('数据',run)+'|')
    retained=read(OUT/'analysis/retained_failure_media.json')
    if isinstance(retained,list):
        lines+=['','## 修正前失败与开发尝试','',
            '下面保留原始失败标签，包含后续被独立物理审核识别为评价误拒的超车记录；是否计入本轮22项，以前表为准，不把同一轨迹重复计为新增成功。','',
            '|回合/采集版本|原运行器|原S3|回放|原始目录|','|---|---|---|---|---|']
        for r in retained:
            lines.append(f"|{r['case']}/{r.get('acquisition_revision','')}|{r['raw_runner']}|{r.get('S3')}|"+link('视频',video(r['run_dir']))+'|'+link('数据',r['run_dir'])+'|')
    lines+=['','## 审计与复现','']
    for label,p in [
        ('最终冻结清单',scope['runtime_manifest_path']),
        ('未受修改影响的9回合复用校验',OUT/'validation4_unaffected_reuse_receipt.json'),
        ('时间预算路线策略的逐回合影响与证据复用',OUT/'validation5_unaffected_reuse_receipt.json'),
        ('W2早期规划、实际视频时序及真值边界',DEST/'W2早期规划与真值边界说明.md'),
        ('超车原标签误拒及物理审核说明',OUT/'SAFETY_EVALUATION_CORRECTION.md'),
        ('N3-A新旧实际初态/时序/轨迹配对',OUT/'analysis/N3_A_pair_audit.json'),
        ('源码、模型、配置与原R9保留校验',OUT/'analysis/final_integrity_verification.json'),
        ('最终交付链接及成功保持段完整2秒覆盖核对',OUT/'analysis/final_delivery_check.json'),
        ('视频解码、帧数及播放速度核验',OUT/'analysis/media_technical_check.json'),
        ('消息与规划因果链',OUT/'analysis/targeted_media_and_provenance.json'),
        ('已人工抽查的代表画面及范围',OUT/'analysis/manual_visual_review.json'),
        ('场景、路线、种子、时限等原配置核对',OUT/'analysis/scene_design_preservation.json'),
        ('停车段五类距离及原始分子分母',DEST/'停车保持段_逐观测五类距离.csv'),
        ('停车段每个仿真帧的安全证据',DEST/'停车保持段_逐仿真帧安全证据.csv'),
        ('控制函数禁止目标真值输入测试',OUT/'analysis/controller_truth_boundary_audit.json'),
        ('旧72回合新标准可追溯性审计',OUT/'analysis/R9_72_retraceability_summary.json'),
        ('低障碍及无效深度合成回归，非实景试验',OUT/'analysis/low_obstacle_depth_regression.json'),
        ('最终版本代码回归结果',OUT/('tests_before_validation5.xml' if rev=='validation5' else 'tests_before_validation4.xml')),
        ('独立版本源码目录',ROOT),
    ]:lines.append('- '+link(label,p))
    lines+=['','原始R9实验与模型保持保留。本索引链接到D/E盘原始数据，不复制整批图像到C盘。','']
    DEST.mkdir(exist_ok=True,parents=True);(DEST/'交付与证据索引.md').write_text('\n'.join(lines),encoding='utf-8');print('INDEXED',len(all_runs),'completed runs')


if __name__=='__main__':main()
