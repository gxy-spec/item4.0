"""Separate carried frozen-image evidence from new trajectory-dependent frame metrics."""
from pathlib import Path
import sys,json,csv
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest
from build_r10_targeted_report import DEST


def main():
    old=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')
    source=old/'final_frontend_audit/summary.json';frozen=json.loads(source.read_text(encoding='utf-8'))
    old_manifest=json.loads((old/'frozen_runtime_formal_r9/manifest.json').read_text(encoding='utf-8'))
    relevant=['src/perception/candidate_pipeline.py','src/perception/vehicle_attributes.py','src/perception/depth_proposal_refinement.py','src/perception/auxiliary_vehicle_proposals.py','src/perception/task_spec.py']
    unchanged={p:digest(ROOT/p)==old_manifest['files'].get(p) for p in relevant}
    if not all(unchanged.values()):raise RuntimeError('Image frontend changed; cannot carry old audit unchanged')
    attribute=old/'attribute_training/resnet18_color_v1/best.pt';calibration=old/'calibration/deployment_release.json'
    if digest(attribute)!=frozen['weights_sha256'] or digest(calibration)!=frozen['calibration_sha256']:
        raise RuntimeError('Audited model/threshold mismatch')
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision']
    progress=OUT/f'acceptance_development_{rev}_progress.json';records=[];timings=[]
    if progress.exists():
        for item in json.loads(progress.read_text(encoding='utf-8')).values():
            run=Path(item['run_dir']);p=run/'r10_evaluation.json'
            if not p.exists():continue
            e=json.loads(p.read_text(encoding='utf-8'))
            t=e['online_latency']['combined_pipeline_ms']
            timings.append(dict(case=item['case'],samples=t['n'],combined_median_ms=t['median'],combined_p95_ms=t['p95'],
                combined_maximum_ms=t['maximum'],engineering_budget_ms=e['combined_pipeline_budget_ms'],
                episode_simulated_seconds=item['horizon_seconds'],episode_wall_seconds=item['wall_seconds'],
                simulated_time_divided_by_wall_time=e['actual_runner_real_time_factor']))
            for device,m in {'combined':e['frame_metrics'],**e['by_device']}.items():
                records.append(dict(case=item['case'],device=device,**m,run_dir=str(run)))
    DEST.mkdir(exist_ok=True,parents=True)
    with (DEST/'在线候选选择_全部计数及指标.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    with (DEST/'在线处理耗时与运行时长.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(timings[0]));w.writeheader();w.writerows(timings)
    receipt=dict(carried_frozen_audit=str(source),source_sha256=digest(source),frontend_source_unchanged=unchanged,
        weights_sha256=frozen['weights_sha256'],calibration_sha256=frozen['calibration_sha256'],
        scope='R9 frozen-image audit carried without rerunning/tuning; current online frame metrics reported separately')
    save(OUT/'analysis/perception_audit_reuse_receipt.json',receipt)
    def fraction(n,d):return f'{n}/{d}（{n/d:.2%}）' if d else '不适用'
    lines=['# 感知识别结果：两种证据分开看','',
        '本轮没有重新训练或更换识别模型。当前前端为YOLOE主检测/分割、辅助车辆提议及深度框补全，车型和颜色由已冻结的ResNet18属性分支判断。HSV证据仍可记录供诊断，当前升级模型的最终颜色匹配使用学习到的颜色概率和冻结阈值。','',
        '## 1. 同一图像比较：复用原冻结审计','',
        '下表来自原R9冻结后的独立图像审计，前端代码、属性权重及标定阈值已核验一致；不是本轮新增试验。仅有已知目标标注，背景车辆未全量标注，所以框命中率和条件属性正确率均不能称为全图mAP/AP或穷尽精确率。','',
        '|前端|目标框IoU≥0.5命中|严格车型正确|颜色正确|车型与颜色联合正确|','|---|---:|---:|---:|---:|']
    for name,label in [('legacy','原COCO前端'),('current_yoloe_hsv','中间版YOLOE＋HSV'),('upgraded','当前冻结前端')]:
        a=frozen['models'][name]
        lines.append('|'+label+'|'+fraction(a['box_iou50'],a['known_target_views'])+'|'+fraction(a['exact_type_correct'],a['known_target_views'])+'|'+fraction(a['color_correct'],a['color_n'])+'|'+fraction(a['joint_correct'],a['color_n'])+'|')
    lines+=['','这张表是506个已知目标视角的实际前端审计，不是旧报告中617个真值裁剪的属性审计，也不是开发压力集。真值裁剪表预先给出目标区域，不检验检测框能否找到目标；开发压力集参与过问题定位，不能充当冻结后的独立验收。','',
        'COCO没有van类别，严格四车型识别与旧任务中允许car/truck先验匹配van的口径不同。颜色不可观测图像保留在车型分母中，颜色及联合指标只计有可审计颜色的464帧。样本来自同一地图的受控目标和相关视角，不是跨地图泛化证明。','',
        '## 2. 本轮在线观测：逐设备原始计数','',
        '这是本轮实际轨迹上的任务候选选择统计：每设备每个推理帧按候选分数选一个符合任务类别/颜色条件的候选；与离线可见目标框IoU≥0.5关联。目标实例≥80像素计可见正帧，微小可见目标帧忽略；目标实例完全不存在的帧计负帧。','',
        '精确率=正确选择帧/有候选选择帧；召回率=正确选择帧/可见正帧；负帧误触发率=负帧中错误选择帧/负帧数。它们不是所有物体的检测AP。不同轨迹和成功后驻留时间会改变帧构成，连续帧相关，不能仅凭本表汇总值宣称模型优于另一次不同轨迹的模型。','',
        '|回合|设备|正确选择/有选择|正确选择/正帧|负帧误选/负帧|','|---|---|---:|---:|---:|']
    for r in records:
        if r['device']=='combined':continue
        lines.append('|'+r['case']+'|'+r['device']+'|'+fraction(r['correct'],r['selected'])+'|'+fraction(r['correct'],r['positive_frames'])+'|'+fraction(r['negative_false_triggers'],r['negative_frames'])+'|')
    lines+=['','逐设备/合并所有原始分子、分母及路径见“在线候选选择_全部计数及指标.csv”。S1/S2/S3是另外的连续观测及任务完成评价，不用该表的单帧命中代替。','',
        '## 3. 前端耗时与仿真运行时间','',
        '下表为两设备任务感知前端合计处理耗时，按已有评价规则剔除前两个预热样本；250毫秒是本实验的工程预算。它不包含整套仿真、传感器写盘、安全检测或网络端到端开销。视频及任务完成时刻使用仿真时间，整回合实际墙钟耗时另外列出，不能把它们混作实车实时性结论。','',
        '|回合|样本数|中位数/ms|P95/ms|最大/ms|仿真时长/s|实际运行耗时/s|','|---|---:|---:|---:|---:|---:|---:|']
    for t in timings:
        lines.append(f"|{t['case']}|{t['samples']}|{t['combined_median_ms']:.1f}|{t['combined_p95_ms']:.1f}|{t['combined_maximum_ms']:.1f}|{t['episode_simulated_seconds']:.0f}|{t['episode_wall_seconds']:.1f}|")
    lines+=['','完整耗时、工程预算及仿真/墙钟比例见“在线处理耗时与运行时长.csv”。','']
    (DEST/'感知指标与依据.md').write_text('\n'.join(lines),encoding='utf-8');print('EXPORTED',len(records),'online metric rows; original frozen audit unchanged')


if __name__=='__main__':main()
