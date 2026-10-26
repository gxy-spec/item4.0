"""Short decision report plus separate auditable evidence for the task-1 re-review."""
from __future__ import annotations
import json
from pathlib import Path
import shutil
from perception_online_acceptance import OUT,ROOT,read_jsonl,save_json,sha256
from package_online_acceptance_results import DELIVERY,link


def main():
    data=OUT/'analysis/followup_questions'
    verification=json.loads((data/'semantic_repair_verification.json').read_text(encoding='utf-8'))
    assert verification['passed']
    revised=json.loads((OUT/'analysis/visualization_revision_v5.json').read_text(encoding='utf-8'))
    assert revised['all_12_passed']
    results=json.loads((OUT/'analysis/online_results.json').read_text(encoding='utf-8'))
    provenance=json.loads((data/'scene_configuration_provenance.json').read_text(encoding='utf-8'))
    assert provenance['old_region_exactly_equal_current']
    ev=Path('E:/CarlaAirData/CityInspection_GOC/evaluation/perception_town10hd_v2')
    manifests={n:json.loads((ev/f/'evaluation_manifest.json').read_text(encoding='utf-8')) for n,f in
        [('原始YOLOE','zeroshot_test_protocol_v1'),('升级YOLOE','adapted_test_protocol_v1')]}
    b=json.loads((OUT/'analysis/benchmark_ego_excluded_at_frozen_thresholds.json').read_text(encoding='utf-8'))
    old=b['original']['test']['overall'];new=b['adapted']['test']['overall']
    repaired=json.loads((data/'semantic_label_repair_result.json').read_text(encoding='utf-8'))
    audit=json.loads((data/'training_dataset_semantic_audit.json').read_text(encoding='utf-8'))
    diagnostic=OUT/'reaudit_frozen_baseline'
    paired=json.loads((diagnostic/'paired_diagnostic_summary.json').read_text(encoding='utf-8'))
    for sample in read_jsonl(diagnostic/'samples.jsonl'):
        target=diagnostic/f'source_rgb/{sample["review_id"]}.png';target.parent.mkdir(exist_ok=True)
        if not target.exists():shutil.copy2(sample['image_path'],target)
        assert sha256(target)==sha256(sample['image_path'])
    root_causes=[]
    for r in results:
        run=Path(r['run_dir']);records=read_jsonl(run/'task1_candidate_audit.jsonl')
        first=next((x for x in records if x.get('matched_role')=='target' and x.get('semantic_correct')),None)
        routes=read_jsonl(run/'planning/target_route_events.jsonl')
        first_actor=next(iter(read_jsonl(run/'actors/actor_states.jsonl')))
        start=first_actor['timestamp']
        row={'case':r['case'],'first_task_object_associated_candidate':first,
            'meaning_of_semantic_correct':'associated GT object satisfies task; does not certify the predicted category',
            'routes':routes,'simulation_timestamp_origin_s':start}
        if routes and routes[0].get('route_length_m'):
            elapsed=routes[0]['timestamp_s']-start
            row.update(first_replan_elapsed_s=elapsed,remaining_time_s=190-elapsed,
                ideal_remaining_distance_at_2mps=2*(190-elapsed),planned_route_length_m=routes[0]['route_length_m'],
                ideal_budget_exceeded=routes[0]['route_length_m']>2*(190-elapsed))
        root_causes.append(row)
    save_json(data/'episode_failure_causal_evidence.json',root_causes)
    red=next(r for r in root_causes if r['case']=='red_van_42101')
    visuals=DELIVERY/'整改后可视化';visuals.mkdir(exist_ok=True)
    index=['# 12回合修订可视化','',
        '灰黑虚线：UGV原巡检规划；蓝线：重规划前实际行驶；橙线：首次重规划后实际行驶；紫线：UAV实际轨迹。',
        '菱形R1等标出消息触发重规划的位置，图底给出时间、消息ID和规划长度。视频轨迹只显示当前时刻以前的运动。',
        '**橙线是实际轨迹。旧日志没有保存目标重规划折线，不能把橙线当作完整规划路径。**',
        'CLS_OK/CLS_ERR只表示预测类别与关联自动标签是否相同；颜色为测量结果，RGB可见性仍需独立审核。','']
    for item in sorted(revised['runs'],key=lambda r:r['case']):
        run=Path(item['run_dir']);case=item['case'];map_out=visuals/f'{case}_轨迹图.png';poster_out=visuals/f'{case}_回放截图.jpg'
        shutil.copy2(run/'preview/task1_trajectory_map.png',map_out);shutil.copy2(run/'preview/task1_replay_poster.jpg',poster_out)
        index.append(f'- **{case}**：{link(map_out,"轨迹图")} · {link(poster_out,"回放截图")} · {link(run/"preview/task1_five_view.mp4","完整五视角回放")} · {link(run/"preview/task1_route_overlay_evidence.json","路线证据")}')
    (visuals/'索引.md').write_text('\n'.join(index)+'\n',encoding='utf-8')
    details=['# 问题1—4：数据依据与详细复核','',
        '**本文件提供核对依据；主要结论请先看同目录《任务1_在线验收报告》。本轮没有新增正式在线回合，也没有更换正式权重。**','',
        '## 1. 当前模型究竟改善了什么','',
        '先区分三种“原模型”：旧36组在线实验实际用YOLO26n；较早离线基线另有YOLO26x；多类别微调的直接母模型是原始YOLOE-26s-seg。下表比较的是最后一对，不能偷换成旧系统的端到端对照。','',
        '**严格同协议的检测／分割对照。** 同一测试集1781张、3967个标注实例，类别van/car/truck/bus；输入960、batch=8、conf下限0.001、NMS IoU=0.7、agnostic=false、rect=true。只采用两个test_protocol_v1最终目录。','',
        '|指标|原始YOLOE|升级YOLOE|变化（百分点）|','|---|---:|---:|---:|']
    metric_names=[('检测框mAP50','metrics/mAP50(B)'),('检测框mAP50:95','metrics/mAP50-95(B)'),
        ('检测Precision','metrics/precision(B)'),('检测Recall','metrics/recall(B)'),
        ('实例掩码mAP50','metrics/mAP50(M)'),('实例掩码mAP50:95','metrics/mAP50-95(M)')]
    for name,key in metric_names:
        a=manifests['原始YOLOE']['metrics'][key];z=manifests['升级YOLOE']['metrics'][key]
        details.append(f'|{name}|{100*a:.2f}%|{100*z:.2f}%|+{100*(z-a):.2f}|')
    details +=['',
        'Precision/Recall是当前Ultralytics实现按各模型平滑平均F1最佳点取得的类别均值，二者不一定对应同一置信阈值；manifest未保存该最佳点的具体阈值。它们不是准确率，也不能把mAP50=79.92%解释为“所有目标79.92%识别正确”。AP更适合此处统一协议的主比较。',
        '升级模型各类框AP50：van约69.3%、car78.5%、truck82.1%、bus89.9%；厢式车仍是弱项。','',
        f'数值原件：{link(ev/"zeroshot_test_protocol_v1/evaluation_manifest.json","原始模型评估JSON")}；{link(ev/"adapted_test_protocol_v1/evaluation_manifest.json","升级模型评估JSON")}；{link(data/"同协议检测分割对照.csv","全精度对照CSV")}。',
        '', '**主流指标不存在通用的最低合格分。** COCO规定AP50及IoU=0.50:0.05:0.95等计算口径；IoU=0.5是匹配规则，不是“mAP达到50%就合格”。这些指标规范没有给出跨任务通用的最低验收分。因此，本轮能说域内检测有明显提升，不能说已达到某条行业统一及格线，也不能把本数据集的59.44与COCO榜单直接比较。依据：[COCO官方检测评价规范](https://github.com/cocodataset/cocodataset.github.io/blob/master/dataset/detection-eval.htm)、[Ultralytics指标解释](https://docs.ultralytics.com/guides/yolo-performance-metrics/)。','',
        '**相同测试图像、相同操作点的类别＋颜色选择对照。** 1200张RGB（UAV/UGV各600），每张3个任务查询，共3600个相关任务帧；1078正例、2254负例、268小目标忽略。两模型都固定检测阈值0.03、目标颜色比例0.10。此处比较相同操作点，并未用测试集重新选择阈值。','',
        '|指标／原始计数|原始YOLOE＋HSV|升级YOLOE＋HSV|','|---|---:|---:|',
        f'|选中数／其中正确|{old["selected"]}／{old["correct"]}|{new["selected"]}／{new["correct"]}|',
        f'|选择精确率|199/939=21.19%|528/856=61.68%|',
        '|正例召回率|199/1078=18.46%|528/1078=48.98%|',
        '|负例任务帧误触发率|625/2254=27.73%|187/2254=8.30%|','',
        '计算定义：精确率=正确选择/所有选择；召回率=正确选择/正例任务帧；误触发率=负例中发生选择/负例任务帧。误触发率不是1−精确率。这些是已记录车辆的自动标签参考统计；静态车辆漏标和RGB遮挡仍限制可信度，不能当成经过人工完整标注的总体真实性能。验证集与测试集还存在较大差异：升级模型验证精确率28.71%、召回27.58%，提示区域稳定性不足。',
        f'计数与分组：{link(data/"paired_task_selection_counts.json")}；{link(OUT/"analysis/semantic_clean_reference.json","语义过滤后的补充重算")}。连续帧、同图多查询相关，不能按3600个独立样本计算简单置信区间。','',
        '**补做旧基准上的4模型诊断。** 从旧36组A组保存的原RGB中取N1/E1/W2/F1四类场景、双设备，共32帧；按时间分层并补充一个投影／深度代理认为有可见机会的帧，在模型推理前复核RGB。8帧可辨认目标、2帧不确定、其余22帧目标在视野外或遮挡。F1本次取样没有可辨认正例。','',
        '|模型|可见目标上的通用车辆框命中（IoU≥0.5）|同时有van类别框命中|','|---|---:|---:|']
    for name,label in [('historical_yolo26n','旧YOLO26n'),('historical_yolo26x','YOLO26x'),('original_yoloe','原始YOLOE'),('adapted_yoloe','升级YOLOE')]:
        r=paired['results'][name];v=r['van_category_hits']
        details.append(f'|{label}|{r["generic_vehicle_target_hits"]}/{r["rgb_visible_targets"]}|{str(v)+"/8" if v is not None else "COCO无van类，不作此比较"}|')
    details +=['',
        '四模型输入同为960、conf下限0.03、NMS IoU0.7；此处是受控模型诊断，未复现旧YOLO26n线上800/0.08及其启发式全流水线。可见性由助手逐图视觉复核，未双人标注；匹配沿用原三维投影框。样本是诊断性选取，不能据8/8宣称100%识别率，不能替代AP、四场景完整任务率或泛化实验。结果支持“车辆发现改善，但指定车型仍不稳”。',
        f'原图、取样、逐图标签、预测与明细：{link(diagnostic)}；{link(diagnostic/"paired_diagnostic_details.csv")}。','',
        '## 2. 与四场景36组的关系及遮挡原因','',
        '旧设计是北／东／西／整图4类区域，每类3个部署×A/B/C三策略=36回合；不是36个独立地图。当前12回合是E1中6种任务条件×2种子，模板来自persistent_memory_smoke_20261012/E1_C_memory.yaml，不能代替原四场景配对实验。','',
        '逐项核对发现：E1的整个region配置完全一致，包括UGV原巡检路线、目标位置及UAV航线；动态UAV高度60m、速度3m/s、相机俯视−90°、下挂1.5m、RGB尺寸800×600、FOV100°也一致。传感器配置的主要新增是实例分割流。**本次遮挡不能归因于更改了这些几何参数。**',
        '变化包括检测模型／阈值／推理频率／时序确认、目标类别颜色、干扰对象、随机种子、消息匹配与信道配置；没有在新组合下重新完成沿航线RGB可辨识性和业务闭环准入。旧36组中的hold_after_grace策略已被项目标记为历史配置，不能直接复制进新正式实验；应保留已审核几何，再按已确认的持久记忆策略生成新版本。',
        f'配置差异原件：{link(data/"scene_configuration_provenance.json")}；旧组冻结说明：{link(ROOT/"configs/experiments/multiscene_abc_formal_20261010_v2/README.md")}。','',
        '遮挡的直接证据是同一机位的隔离测试：正常植被时实例掩码751像素、RGB红色比例0；只关闭植被时831像素、红色比例70.36%，深度中位数仅变化约0.14m；RGB后处理关闭也未消除此现象。它说明实例／深度中的车辆存在不能直接等同于RGB可辨识。此为受控诊断，不是正式实验中关闭植被，也不能推断整条航线或所有回合都不可见。',
        f'{link(OUT/"observability_diagnostic/results.json","隔离测试原始数据")}；{link(DELIVERY/"可见性诊断.png","同机位可见性图")}。用户看到的窗口若为旁观相机，还应和保存的真实UAV RGB区分；本轮依据以真实传感器流为准。','',
        '**还需纠正“失败都由遮挡造成”的解释。** red_van_42101首帧就有与目标框重叠的红色车辆候选，但类别被预测为car（置信0.582，IoU0.601），因此未通过van任务筛选。该事实本身不证明13像素宽的图像已足够区分车型，却证明当时并非只有树冠导致模型完全看不到目标。',
        f'该回合第{red["first_replan_elapsed_s"]:.2f}秒开始规划时，路线已长{red["planned_route_length_m"]:.2f}m；剩余时间即使一直以2m/s行驶，也最多{red["ideal_remaining_distance_at_2mps"]:.2f}m。初始位置可达不代表较晚收到消息后仍能在190秒内到达。应同时检查最迟发现时间、当时道路路径和剩余预算。',
        f'逐回合因果证据：{link(data/"episode_failure_causal_evidence.json")}。','',
        '## 3. 颜色究竟由谁判断','',
        '实际链路为YOLOE输出类别／框／预测实例掩码，HSV在掩码内测量目标颜色比例，再结合深度定位。当前权重names只有van/car/truck/bus，没有颜色属性输出头，训练目标也没有颜色属性损失。Qwen候选复核默认关闭，本轮未启用。',
        '历史方案同时举了red van等文本提示例子，又明确写了“掩膜内颜色／属性分析”；两者容易让人误以为颜色也全部由YOLOE判断。应承认方案表述和实际能力边界未讲清，不能解释成用户记错。模块化颜色测量可以作为待验证基线，但它不是已完成的“YOLOE全属性识别”。',
        '若要模型负责颜色，须另验开放词汇复合提示或带颜色输出的属性模型／低频VLM；不能对固定四类微调头直接加一个red van字符串就宣布完成。应在同一可见RGB正负例上比较颜色混淆、类别＋颜色联合选择和时延，验证后再变更默认链路。',
        f'历史实现报告：{link(ROOT/"docs/目标识别感知升级_阶段一实施报告_20261016.md")}；颜色调用：{link(ROOT/"src/perception/candidate_pipeline.py:274")}；属性实现：{link(ROOT/"src/perception/attributes.py")}。','',
        '## 4. 可视化已修复及仍有的记录缺口','',
        '12回合已全部重绘轨迹图、重生成同步五视角视频，并检查首／中／末帧可解码、每回合760帧、4fps、1920×1080。原巡检路线来自实际记录的planning/ugv_patrol_route_executed.json，不是凭观测轨迹反推。消息触发点、消息ID、时间和规划长度一并显示；视频只画当前时刻以前的实际轨迹。',
        '进一步修复了旧覆盖框的解释错误：semantic_correct原字段表示关联的真值对象满足任务，不等于检测器预测类别正确。新版CLS_OK/CLS_ERR直接比较预测类别与关联类别，避免把“car框在目标van上”画成类别匹配。此项只修正显示，不改变模型预测、原始数据或任务判定。',
        '目标重规划日志缺少完整路点折线，因此图中橙线明确标为重规划后的实际运动。当前可检查真实偏航与触发位置，无法忠实恢复未保存的完整规划；后续运行需增加每次规划路点的版本化归档。',
        f'全部图与视频：{link(visuals/"索引.md")}；机器校验：{link(OUT/"analysis/visualization_revision_v5.json")}。','',
        '## 5. 新发现的数据错误与实际修复','',
        '|划分|图像|原标注数|无任何车辆语义的错误掩码|','|---|---:|---:|---:|']
    for split in ['train','validation','test']:
        a=audit['splits'][split];details.append(f'|{split}|{a["images"]}|{a["annotations"]}|{a["masks_without_vehicle_pixels"]}|')
    details +=['',
        '本轮已在独立版本移除1081条纯非车辆标签。原9163张RGB、分组和原数据保留，1781份测试标签逐份验证未变；剩余标注18085条。导出器默认启用车辆语义过滤，旧行为只通过显式legacy参数复现，并增加真实构造的建筑ID冲突回归测试。',
        '**清理不是训练完成。** 1081个文件现在没有已知车辆标签，但静态车辆仍可能漏标，不能自动当成真负样本。语义过滤也不解决树冠下RGB看不到车的问题，所以修订数据明确标记未达到正式训练／验收准入；本轮没有重训，没有声称新模型改善了。812条训练污染可能影响分类，但需清洗后受控重训才能确认因果。',
        f'全量审计：{link(data/"training_dataset_semantic_audit.json")}；修订候选：{link(Path(repaired["target"])/"repair_manifest.json")}；文件级核验：{link(data/"semantic_repair_verification.json")}。',
        f'另核对了本机CARLA的语义枚举：污染像素实际来自Poles（标签6，共28569像素）及Other（标签22，共99216像素），不含Car/Truck/Bus/Train/Motorcycle/Bicycle标签14—19；并非简单把摩托车等其他车辆排除后称作非车辆。{link(data/"contaminating_pixel_semantic_names.json","语义枚举核验")}。','',
        '## 6. 执行流程修复与剩余工作','',
        '批处理入口已增加校准、四场景对齐、独立RGB可见性、标签、配对模型对照、正负例闭环和路线预算的准入检查；证据要求文件及SHA256。已用本轮真实失败配置复测：在启动CARLA前拒绝继续formal。已归档目录禁止重新生成配置与追加smoke，新增实验必须使用新版本目录。',
        '13项准入回归测试与1项导出器默认行为回归测试加入后，全套141项通过；这只说明工程检查通过，不代表模型验收通过。上述修改位于批处理／评价／导出／离线渲染，历史12回合的三个运行核心文件和正式权重保持原版本。',
        f'{link(OUT/"formal_admission_rejection_retest.log","真实阻断复测日志")}；{link(OUT/"reaudit_regression_tests.log","141项测试日志")}。',
        '剩余顺序：先按旧四类已审核几何建立新版准入场景，核验沿线RGB可辨识性与最迟发现的可达预算；再完善可见性／类别／颜色标签并比较感知方案；先打通一个完整正例和对应负例，再恢复四类场景配对验收。持久记忆、真实RGB-D安全边界及课题GOC主线继续沿用。未通过前不推进LiDAR融合和GOC正式对照。']
    details_path=DELIVERY/'问题1-4_依据与整改明细.md';details_path.write_text('\n'.join(details)+'\n',encoding='utf-8')
    summary=['# 任务1在线验收：整改复核汇报','',
        '**结论：任务1尚未完成合格的在线验收。原12回合保留为失败诊断，不能作为已建立的感知基线。本次修复了数据导出、实验准入和可视化问题；模型与完整导航能力仍未通过复验。**','',
        '**主要结果**','',
        '|要回答的问题|可靠结论|','|---|---|',
        '|升级是否有收益？|同一1781图检测测试集：框mAP50由21.08%升至79.92%，mAP50:95由14.65%升至59.44%；对照基线是原始YOLOE。|',
        '|是否已能稳定识别指定车辆？|不能这样认定。旧场景补做的8个可辨认目标中，升级模型车辆框命中8个，但van类别框只命中3个；这是小样本诊断，不是总体准确率。|',
        '|完整任务是否完成？|8个有目标回合均未完成；4个无目标回合正确拒绝。明确误选2次、未证实提交1次；记录碰撞0次。|',
        '|是否有“主流最低合格分”？|COCO等规定指标计算方法，没有跨任务通用的最低mAP。不能把79.92%直接称为行业合格。|','',
        '**这次实验的主要失误**','',
        '1. **对照不完整。** 旧实验是四类场景×3部署×A/B/C=36回合，本轮只用了E1的6任务×2种子；并非在原完整实验组上复验。E1几何、高度及RGB-D相机参数实际沿用，不能把遮挡归因于随意改了高度。',
        '2. **准入失效。** 校准失败、RGB可见性未证实、正例尚未完整跑通时，仍展开了批量长回合。程序PASS混入任务验收表述，报告未突出关键失败。',
        f'3. **失败解释不充分。** 车型误判、数据标签错误与遮挡混在一起。红van回合较早已框到目标却判成car；到第{red["first_replan_elapsed_s"]:.2f}秒重规划时，需要走{red["planned_route_length_m"]:.1f}m，剩余理想行程仅{red["ideal_remaining_distance_at_2mps"]:.1f}m，已无法按时完成。','',
        '**本次实际完成的整改**','',
        '|问题|已完成的处理与验证|还未解决的部分|','|---|---|---|',
        '|数据错误|审计全部9163图，移除独立版本中812条训练、269条验证的纯非车辆标签；9163原图、1781测试标签核对不变；修复导出默认行为。|RGB遮挡与静态车辆漏标仍待清理；未重训。|',
        '|比较口径|恢复旧四类场景原RGB的小样本4模型受控诊断，分开“车辆框命中”和“车型识别”。|尚无旧模型与新模型的完整配对在线成功率。|',
        '|执行流程|新增正式实验准入检查；本轮失败配置在启动仿真前被正确阻断；全套141项测试通过。|需要满足准入的新版本感知与闭环。|',
        '|图与回放|12回合全部补画原巡检路线、触发位置及前后实际运动；修复类别错误框被标成匹配的问题；全视频校验。|旧日志未存完整重规划折线，无法凭空补全。|',
        '|报告|主要结论集中于本页；原始计数、配置差异及逐回合证据另附。|颜色实际仍为YOLOE掩码＋HSV，未实现全由YOLOE判断。|','',
        '**下一步的顺序**','',
        '先完成四类场景的RGB可辨识性和发现后路线预算准入，再修订标签与感知方案；用一个完整正例及对应负例验证后，恢复四类场景配对验收。当前不将这批诊断结果冻结为感知基线。','',
        f'{link(details_path,"查看问题1—4的完整数据依据与原因分析")} · {link(visuals/"索引.md","查看12回合修订图与回放")} · {link(DELIVERY/"完整诊断记录.md","逐回合记录附件")}','',
        f'原始失败报告与旧审阅包分别保存在{link(DELIVERY/"历史归档","历史归档目录")}、{link(OUT/"任务1_验收审阅包_20261021_整改前.zip","整改前审阅包")}，本次未删除失败记录。']
    report=DELIVERY/'任务1_在线验收报告.md';report.write_text('\n'.join(summary)+'\n',encoding='utf-8')
    (OUT/'任务1_在线验收报告.md').write_text(report.read_text(encoding='utf-8'),encoding='utf-8')
    for file in data.iterdir():
        if file.is_file() and file.suffix in ['.json','.csv']:shutil.copy2(file,DELIVERY/file.name)
    save_json(OUT/'analysis/reaudit_completion.json',{'historical_episodes_unchanged':True,'new_formal_online_episodes':0,
        'perception_or_navigation_acceptance_passed':False,'engineering_tests_passed':141,
        'semantic_labels_removed':1081,'all_videos_renderer':5,'old_baseline_diagnostic_images':32,
        'remaining':['RGB-visible complete annotation','repaired model category/color validation','first positive complete task','four-family paired online comparison'],
        'main_report':str(report),'evidence_report':str(details_path)})
    print('REVIEW REPORT',report,'lines',len(summary),flush=True)


if __name__=='__main__':main()
