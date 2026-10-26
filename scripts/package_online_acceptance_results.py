"""Incrementally render every completed run and assemble reviewable deliverables."""
from __future__ import annotations
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import time
import cv2
import numpy as np
import yaml
from perception_online_acceptance import OUT,ROOT,read_jsonl,save_json,sha256,observations,metrics
from analyze_online_acceptance import table

DELIVERY=Path('C:/Users/Administrator/Documents/ChatGPT/课题1/实验结果/任务1_在线验收_20261021')


def pct(x):return '—' if x is None else f'{100*x:.2f}%'
def num(x,n=2):return '—' if x is None else f'{x:.{n}f}'
def link(path,label=None):
    p=Path(path);return f'[{label or p.name}](<{p.as_posix()}>)'


def corrected_benchmark():
    data={}
    for model in ['adapted','original']:
        rows=read_jsonl(OUT/f'benchmark/{model}_predictions.jsonl')
        cal=json.loads((OUT/f'calibration/{model}.json').read_text(encoding='utf-8'))
        a=cal['chosen']['detector_threshold'];b=cal['chosen']['color_threshold']
        data[model]={}
        for split in ['validation','test']:
            obs=observations([r for r in rows if r['split']==split])
            data[model][split]={'overall':metrics(obs,a,b),'by_device':{},'by_class':{},'by_color':{}}
            for field,values,key in [('device',['uav','ugv'],'by_device'),('category',['van','car','truck','bus'],'by_class'),('color',['red','blue','white','black','gray','yellow'],'by_color')]:
                data[model][split][key]={v:metrics([o for o in obs if o[field]==v],a,b) for v in values}
    save_json(OUT/'analysis/benchmark_ego_excluded_at_frozen_thresholds.json',data)
    return data


def video_check(run):
    path=run/'preview/task1_five_view.mp4';manifest_path=path.with_suffix('.json')
    if not path.exists() or not manifest_path.exists():return False
    cap=cv2.VideoCapture(str(path));count=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
    checks=[]
    for fraction in [0,.5,.99]:
        cap.set(cv2.CAP_PROP_POS_FRAMES,int((count-1)*fraction));ok,im=cap.read();checks.append(bool(ok and im is not None and im.shape[:2]==(1080,1920)))
    # Prefer the first committed target as the review poster.
    result=json.loads((run/'task1_evaluation.json').read_text(encoding='utf-8'))
    manifest=json.loads(manifest_path.read_text(encoding='utf-8'))
    frame_index=0
    if result.get('commits'):
        source=result['commits'][0]['message'].get('source_frame',manifest['first_frame'])
        with (run/'synchronization/frame_index.csv').open(encoding='utf-8-sig',newline='') as f:
            saved=[int(row['frame']) for row in csv.DictReader(f) if int(row.get('sensor_files_saved','1'))==1]
        frame_index=min(range(len(saved)),key=lambda i:abs(saved[i]-source))
    cap.set(cv2.CAP_PROP_POS_FRAMES,frame_index);ok,poster=cap.read()
    if ok:cv2.imwrite(str(run/'preview/task1_replay_poster.jpg'),poster)
    cap.release()
    good=all(checks) and count==manifest['frame_count'] and abs(fps-4)<.01
    save_json(run/'preview/task1_video_quality_check.json',{'renderer_version':manifest.get('renderer_version',2),'video':str(path),'frame_count':count,'fps':fps,
        'start_mid_end_decodable':checks,'expected_frame_count':manifest['frame_count'],'passed':good,
        'exact_source_frame_alignment':manifest['frame_alignment'],'poster_frame_index':frame_index})
    return good


def build_report(benchmark):
    DELIVERY.mkdir(parents=True,exist_ok=True)
    resultpath=OUT/'analysis/online_results.json'
    results=json.loads(resultpath.read_text(encoding='utf-8')) if resultpath.exists() else []
    completed=[r for r in results if r.get('evaluation_version')==4]
    final=len(completed)==12
    positive=[r for r in completed if r['expected_target_present']];negative=[r for r in completed if not r['expected_target_present']]
    success=sum(r['task_success'] for r in positive);negative_ok=sum(r['negative_rejection_pass'] for r in negative)
    wrong=sum(r['wrong_commits'] for r in completed);collisions=sum(r['collisions'] for r in completed)
    unverified=sum(r.get('unverified_commits',0) for r in completed)
    p=json.loads((OUT/'protocol.json').read_text(encoding='utf-8'))
    a=benchmark['adapted']['test']['overall'];o=benchmark['original']['test']['overall']
    raw=json.loads((OUT/'benchmark/test_results.json').read_text(encoding='utf-8'))
    annotation=json.loads((OUT/'analysis/benchmark_annotation_review.json').read_text(encoding='utf-8')) if (OUT/'analysis/benchmark_annotation_review.json').exists() else {}
    cal=json.loads((OUT/'calibration/adapted.json').read_text(encoding='utf-8'))
    lines=[
        '# 任务1：已有模型在线验收报告',
        '',f'更新：{datetime.now():%Y-%m-%d %H:%M:%S}；状态：'+('**执行结束，模型验收未通过。**' if final else f'**执行中，已完成 {len(completed)}/12 个正式回合；当前不能判定通过。**'),
        '', '本报告区分模型识别、目标提交、本地确认、实际到达和运行安全。离线 AP、程序 PASS、正确提交目标均不单独等于完整任务成功。',
        '', '**评价限制：部分UAV实例掩码覆盖的位置在RGB中被树冠遮挡；另有actor-ID与非车辆语义混淆。下列自动标注口径的帧级指标是探索统计，不能作为已可信验收的真实识别精度。清晰样本错分类、真实目标提交和任务终点另有证据。最终结论同时包含模型问题与验收数据／可见性问题。**',
        '', '**结果概览**',
        '', '|项目|结果|与预期的关系|','|---|---|---|',
        f'|验证集联合阈值|原始校准未找到满足精确率≥90%、负例任务帧误触发率≤1%的组合|未达到预设目标；阈值0.03/0.10仅作为失败诊断配置|',
        f'|独立回放测试：升级模型|精确率 {pct(a["precision"])}；召回率 {pct(a["recall"])}；负例误触发率 {pct(a["negative_fpr"])}|低于预设门槛；统计限定于已记录车辆、排除执行车辆自身|',
        f'|正例终点条件|{success}/{len(positive)} 个已完成正例通过|'+('见逐回合分析|' if final else '其余回合仍在运行|'),
        f'|无目标回合|{negative_ok}/{len(negative)} 个已完成负例正确拒绝|不能用少量回合的零错误推断长期零误报|',
        f'|明确误选／未证实提交／碰撞|{wrong} 次／{unverified} 次／{collisions} 次|错误与未知分别统计；未证实提交仍不合格|',
        '|代码回归|127项测试通过|包含语义碰撞与错误/未知分型回归，不代表模型验收通过|',
        '', '**1. 本轮实施与边界**',
        '', '使用现有 YOLOE 多类别适配权重，四类为 van/car/truck/bus；颜色仍由实例掩码内的 HSV 证据判断。双端独立保留时序历史，使用 RGB、深度和相机位姿定位。语言条件来自结构化 TaskSpec；本轮没有验证任意自然语言解析，也没有启用 VLM 复核。',
        '', '已修复：通信选择及目标记忆只接受红色目标的问题；custom 模型来源未进入发送白名单的问题；van 类别遗漏；固定四类模型仍沿用 COCO 的 car→van 宽松匹配；类别与目标颜色匹配字段未进入无线载荷；按类别配置干扰车辆。车辆表面高度范围改为本轮显式配置，覆盖较高公交车。旧实验默认行为与原始文件快照保留。',
        '', '正式运行保持 RGB-D 安全优先、目标记忆、固定策略和物理链路名义工况，链路上限10 Mb/s，无注入断链。未用真实 LiDAR，本轮结果不构成异质多模态贡献或 GOC 有效性的证据。',
        '', '**2. 验收协议、样本与标注口径**',
        '', f'协议：{link(OUT/"protocol.json")}；模型与源文件指纹：{link(OUT/"configs/source_versions.json")}。阈值只在验证集选择，正式回合均固定为 detector=0.03、color=0.10，5帧窗口内3次命中，每5个传感帧推理一次。校准不可行的标记一直保留，不能把这组回退阈值当作推荐部署值。',
        '', '回放共2400张图像：验证／测试各1200张、各含600张UAV和600张UGV图像。验证区域为north_1、whole_3，测试区域为east_1、west_2，沿用训练导出的区域分组；没有按相邻帧随机拆分。每张图像配配置目标、同类异色、同色异类三条任务，共7200个“图像—指令”任务帧。可见实例至少80像素，正确选择要求可见实例框IoU≥0.5；1–79像素实例单列，不算确定负例。410张图像没有已记录车辆实例，这不等于画面中绝对没有地图静态车辆。',
        '', '补查了33个地图静态车辆对象，20个在回放中有可匹配图像；保留无检测器参与的裁剪图、静态几何投影关联及颜色复核记录。原始标注、排除执行车辆自身后的统计、静态车辆补充标注的参考统计分别保存。静态关联与颜色复核仍非完整独立人工标注，不能把未匹配候选一律认定为真实误报。评估修正不改变运行模型、阈值或既定精度门槛。',
        '', f'标注资料：{link(OUT/"analysis/static_vehicle_annotation_gallery.jpg")}；{link(OUT/"analysis/static_vehicle_annotations.json")}；{link(OUT/"analysis/benchmark_annotation_review.json")}。',
        '', '正式任务为6种条件×2个种子（42101/42102），均在同一east_1场景、同一几何路线中运行190秒，避免把场景变化混入种子对照：红van、蓝car、白truck、黄bus、只有红car/蓝van的困难负例、只有灰car的无指定目标场景。场景初始化与离线标注可读取真值；在线感知和目标控制使用观测、消息、自身位姿与静态地图。',
        '', '**3. 回放识别结果与解释**',
        '', '|模型／数据|精确率|召回率|负例任务帧误触发率|正确选择／已选择|正例／负例任务帧|',
        '|---|---:|---:|---:|---:|---:|',
    ]
    for model in ['original','adapted']:
        for split in ['validation','test']:
            v=benchmark[model][split]['overall']
            lines.append(f'|{model}／{split}|{pct(v["precision"])}|{pct(v["recall"])}|{pct(v["negative_fpr"])}|{v["correct"]}/{v["selected"]}|{v["positive_task_frames"]}/{v["negative_task_frames"]}|')
    lines+=['', '以上为排除执行车辆自身后的固定阈值探索统计；原始校准表保留。升级模型在此自动标注口径上优于原模型，但RGB可见性异常尚未全面清洗，不能据此宣称整体识别精度或泛化改善已获可信验证。AP50约0.799也是原数据协议下的类别检测结果；这里另含颜色、实例选择和无目标任务帧，不能直接比较。原训练／测试标签同样需按RGB可见性复核。',
        '', '|升级模型测试分组|精确率|召回率|负例误触发率|正例任务帧|','|---|---:|---:|---:|---:|']
    for key in ['by_device','by_class','by_color']:
        for name,v in benchmark['adapted']['test'][key].items():
            lines.append(f'|{name}|{pct(v["precision"])}|{pct(v["recall"])}|{pct(v["negative_fpr"])}|{v["positive_task_frames"]}|')
    lines+=['', '分组中“—”表示没有对应分母，不能视为100%或0%。现有颜色／类别组合覆盖并不完整，例如黄色公交车在本轮回放验证数据中缺少同组合正例；统一阈值不能据此获得组合泛化保证。相关帧也不能当作独立重复实验计算显著性。',
        '', f'阈值折中图：{link(OUT/"analysis/calibration_tradeoff.png")}；类别混淆图：{link(OUT/"analysis/class_confusion.png")}；验证集失败图：{link(OUT/"analysis/validation_failure_audit.jpg")}。',
        '', '**4. 在线逐回合结果**',
        '', '|回合|基础程序|任务终点／负例拒绝|错误提交|碰撞|最终目标距离(m)|主要原因|',
        '|---|---|---|---:|---:|---:|---|']
    for r in completed:
        lines.append(f'|{r["case"]}|{r["runner_status"]}|{r["semantic_verdict"]}|{r["wrong_commits"]}|{r["collisions"]}|{num(r.get("final_target_distance_m"))}|{"; ".join(r["reasons"]) or "满足本轮终点条件"}|')
    if len(completed)<12:lines+=['',f'尚未完成的正式回合：{12-len(completed)}个。未完成不能计入成功或失败率。']
    lines+=['', '正例成功要求：正确提交目标，UGV有与目标记忆相符的连续本地观测，感知目标距离3–7米、速度≤0.3m/s稳定保持2秒，并通过离线真实距离复核，且无错误提交和碰撞。负例要求全回合没有目标记忆提交且无碰撞。本地确认与到达由日志按预先固定的在线可用规则重建，再单独核对真值；当前persistent分支没有接通显式业务COMPLETED状态，终点条件通过不等于已实现完整业务状态机。',
        '', '帧级误触发、时序确认后的消息、初次／切换目标提交是三个统计层级。错误候选可能被连续确认抑制，正确提交也可能因发现过晚而无法完成任务。定位误差按原自动标注匹配候选统计，仍受RGB可见性问题限制，不能宣称已取得可信的全域定位精度。',
        '', '|回合|UAV定位P95(m)|UGV定位P95(m)|双端处理P95(ms)|处理预算(ms)|运行RTF|最终控制状态|',
        '|---|---:|---:|---:|---:|---:|---|']
    for r in completed:
        loc=r['localization_xy_m'];lat=r['online_latency']
        lines.append(f'|{r["case"]}|{num(loc.get("uav",{}).get("p95"))}|{num(loc.get("ugv",{}).get("p95"))}|{num(lat["combined_pipeline_ms"]["p95"])}|250|{num(r.get("actual_runner_real_time_factor"),3)}|{r.get("final_control_mode")}|')
    lines+=['', '时延为实际在线检测及候选生成墙钟耗时，剔除每端前2次冷启动，原始逐帧值另存。250ms来自5帧×0.05秒的采样安排。RTF=190秒仿真／runner墙钟耗时，低于1表示整套仿真未达到墙钟实时；模型处理满足周期不代表整套系统已具备实时部署能力。无线模型仍以模拟时钟服务，sender/receiver处理参数为0，实际计算时间尚未反馈进通信因果时序；这点须在后续正式GOC对照前处理。',
        '', '**5. 每个回合的证据与原因分析**']
    for r in completed:
        run=Path(r['run_dir']);lines+=['',f'**{r["case"]}：{r["semantic_verdict"]}**',
            '',f'发送消息{r["sent_messages"]}条，来源{r["message_count_by_device"]}；目标路线规划{r["route_plan_count"]}次；本地确认={r["local_confirmation_observed"]}、确认正确={r["local_confirmation_correct"]}；最终距离{num(r.get("final_target_distance_m"))}米。']
        routes=read_jsonl(run/'planning/target_route_events.jsonl');plans=[x for x in routes if x.get('event')=='target_route_planned_from_selected_message']
        states=read_jsonl(run/'perception/uav_target_candidates_synchronized.jsonl');start=states[0]['timestamp_s'] if states else 0
        for c in r['commits']:
            e=c['evidence'];source=c['message'].get('source')
            lines+=['',f'目标提交来自{source}，自动提交验证={c.get("verification_status")}；'+(f'观测类别／颜色={e["category"]}/{e["color"]}，匹配标注={e.get("matched_category")}/{e.get("matched_color")}，位置误差={num(e.get("localization_error_m"))}米。' if e else '源观测证据不足，不能认定提交正确。')]
        if plans:
            first=plans[0];t=first['timestamp_s']-start;length=first.get('route_length_m');budget=max(0,190-t)*2
            lines+=['',f'首次规划约在第{t:.1f}秒，路线长{num(length)}米；剩余时间按2m/s计算的理想行程上限约{budget:.1f}米，尚未扣除转弯、减速与避障。'+('该回合在当时状态下已无法按时走完路线，发现时机和时间预算构成明确瓶颈。' if length and length>budget else '路线长度未直接超过理想剩余行程，仍需结合末端安全状态和轨迹判断原因。')]
        lines+=['',f'逐帧／提交证据：{link(run/"task1_evaluation.json")}；{link(run/"task1_frame_metrics.csv")}；{link(run/"preview/task1_trajectory_map.png","完整轨迹图")}；{link(run/"preview/task1_five_view.mp4","带框五视角回放")}；{link(run/"preview/task1_video_quality_check.json","视频校验")}。']
    lines+=['', '**6. 已处理的问题、未解决问题与具体后续方法**',
        '', '|问题|本轮证据／处理|下一步解决方法|', '|---|---|---|',
        '|模型与任务接口不兼容|已修复红色固定规则、custom来源、van映射、严格类别匹配及无线字段，127项回归通过|后续沿用同一TaskSpec和来源记录，新增模态也通过同一接口|',
        '|RGB可见性与实例掩码不一致|部分车辆实例框在UAV RGB中覆盖树冠，不能直接当可辨识车辆正例|复核RGB、数值深度、实例相机位姿及遮挡；先建立可靠可见性标签，再重算识别率和阈值|',
        '|actor-ID碰撞引入非车辆标签|新增车辆语义过滤选项及回归；保留原导出和权重|新数据导出启用 --require-vehicle-semantic，并审计RGB可见性；语义过滤本身不能解决遮挡问题|',
        '|清晰样本上能力退化|蓝Tesla后视图：原模型car 0.907，新模型van 0.781；另有蓝van被新模型判car|检查车型、视角、类别平衡与标签质量；保留原模型的通用类别回归集，不能只看合成数据AP|',
        '|类别与颜色联合选择未达门槛|阈值网格没有满足预设约束的组合，真实回合另有提交级核查|补齐同色异类、同类异色、真实空场景；按场景划分新训练／验证／测试集，再进行针对性训练及颜色属性方法对照；禁止用当前测试集调到通过|',
        '|颜色证据不足／不稳定|GT掩码上的HSV诊断也显示部分实例的设定颜色不明显，遮挡、阴影、车顶与涂装影响属性判断|保留不确定颜色；区分可观测与不可观测属性；通过地面近景或换视角确认。不能要求模型从不可见车身颜色中恢复确定答案|',
        '|标注和语言目标集合不一致|已补查静态车辆，排除执行车辆自身；保留原始及修正统计|建立覆盖地图车辆的独立实例及属性标注，并明确任务区域、目标集合与属性可见性；未知标注单列|',
        '|发现偏晚导致路线成本突增|逐回合列出发现时间、规划长度与剩余理想行程|把首次可观测时机、巡逻关键路口和时间预算共同纳入场景预检查；在独立诊断场景验证及时发现后的接近能力，再评估发现延迟的任务代价|',
        '|安全停车与任务完成混淆|基础PASS和业务终点分开；保留最后安全状态|接通显式“目标提交—本地确认—到达—完成／超时”状态机，安全等待必须有独立状态，不能放宽停距或关闭安全机制换取成功|',
        '|旧runner的负例条件不适用|无目标、无路线时仍要求至少存在一段目标路线保护，导致程序FAIL而负例业务正确拒绝|仅在有目标路线时检查其保护；保留全程RGB-D安全tick覆盖。两种判定分别保存|',
        '|数值深度归档不足|正式回合保存RGB、实例图、彩色深度与候选几何日志，未保存逐帧数值深度|下轮至少保存推理帧数值深度和各相机实际位姿；本轮隔离诊断另存数值深度，不能完整补回正式数据|',
        '|颜色分数语义不完全一致|color_score为主导颜色比例，不一定对应任务颜色|排序和质量估计显式使用目标属性分数，与主导颜色置信度分开；后续新版本复验，保持本轮版本不变|',
        '|仿真客户端跨重启异常|保留启动失败与日志；批处理移除跨重启native客户端，复用离线几何预检查|继续使用无native客户端的编排器，并区分仿真异常、模型失败和任务失败|',
        '|计算时延尚未进入通信时序|本轮测量真实处理时间，但通信处理参数仍为0|下一阶段将编码、推理、排队、传输和接收处理统一到信息可用时间，避免低估信息年龄|',
        '', '**7. 对实验1和课题的贡献、当前结论**',
        '', '本轮把离线模型推进到了真实双端运行、消息传输与目标记忆链路，建立了可复现的失败判据、负例、分设备统计、真实处理时延和逐回合可视化。即使最终未通过，也明确了感知错误、属性可观测性、发现时机、标注覆盖和规划时间预算各自的作用。',
        '', '当前不能把候选模型冻结为正式GOC对照的合格感知版本，也不能宣布多设备、异质多模态感知基线建立完成。优先修复验收数据与可见性判定，再修复已确认的类别混淆、发现延迟与任务可达性并复验。真实LiDAR及融合选型随后按既定阶段推进，不能预设LiDAR必然解决这些问题。“特征融合＋目标状态融合”仍是暂定方案。',
        '', '**8. 完整交付索引**',
        '', f'- {link(OUT/"protocol.json","协议与权重指纹")}；{link(OUT/"configs/index.json","12个正式配置")}；{link(OUT/"calibration/adapted.json","新模型阈值全网格")}；{link(OUT/"calibration/original.json","原始模型阈值全网格")}。',
        f'- {link(OUT/"benchmark/manifest.jsonl","2400张图像清单")}；{link(OUT/"benchmark/adapted_predictions.jsonl","新模型全部预测")}；{link(OUT/"benchmark/original_predictions.jsonl","原始模型全部预测")}。',
        f'- {link(OUT/"analysis/benchmark_ego_excluded_at_frozen_thresholds.json","分设备、类别、颜色结果")}；{link(OUT/"analysis/online_summary.csv","逐回合总表")}；{link(OUT/"analysis/online_results.json","逐回合详细结果")}。',
        f'- {link(OUT/"online_index.json","正式运行索引")}；{link(OUT/"attempt_history.jsonl","重试历史")}；{link(OUT/"smoke_initial_schema_failure.json","首次配置校验失败记录")}；{link(OUT/"regression_tests.log","回归日志")}。',
        f'- {link(OUT/"prechange_snapshot","原始工作区快照")}；{link(OUT/"online","已归档传感器、轨迹、通信与安全数据")}。',
        '', '原始图像与日志保存在E盘。每回合均有评估JSON、逐帧CSV、轨迹图、五视角视频和视频校验；数值深度、可见性标注等不足均明确保留。',
    ]
    lines+=['','**9. 像素、语义与模型错误复核**','',
        f'实例语义完整性：{link(OUT/"analysis/instance_label_integrity.json")}；像素几何对应：{link(OUT/"analysis/foreground_pixel_geometry_alignment.json")}；{link(OUT/"analysis/alignment_yellow_bus_42101.jpg","树冠遮挡示例")}。',
        '',f'清晰样本模型对照：{link(OUT/"analysis/clear_case_model_comparison.jpg")}；{link(OUT/"analysis/clear_case_model_comparison.json")}。两个样本按已发现错误选取，用于诊断，不作为模型总体性能的独立估计。',
        '',f'语义过滤重算：{link(OUT/"analysis/semantic_clean_reference.json")}；{link(OUT/"analysis/semantic_clean_reference.csv")}。原始自动标注、预测及正式阈值均未覆盖。']
    cleanpath=OUT/'analysis/semantic_clean_reference.json'
    if cleanpath.exists():
        clean=json.loads(cleanpath.read_text(encoding='utf-8'));lines+=['',f'语义过滤修正涉及{clean["changes"]["images_with_changed_gt"]}张图像。车辆语义仍不能证明RGB可辨识性，因此重算值继续作为参考。',
            '', '|模型／语义清洗参考|精确率|召回率|负例误触发率|','|---|---:|---:|---:|']
        for model,values in clean['models'].items():
            for split,v in values['splits'].items():lines.append(f'|{model}/{split}|{pct(v["precision"])}|{pct(v["recall"])}|{pct(v["negative_fpr"])}|')
    diagpath=OUT/'observability_diagnostic/results.json'
    if diagpath.exists():
        diag=json.loads(diagpath.read_text(encoding='utf-8'));lines+=['',f'隔离可见性诊断：{link(diagpath)}；{link(OUT/"observability_diagnostic/visibility_roi_audit.png","像素区域对照")}；{link(OUT/"observability_diagnostic/foliage_rgb_comparison.jpg","完整开关对照")}。固定RGB/深度/实例相机，增加关闭RGB后处理的对照，已知目标与场景开关只用于独立诊断，不改变12个正式回合，也不计入在线成功率。',
            '', '|隔离状态|实例掩码像素|RGB掩码内红色比例|深度中位数(m)|','|---|---:|---:|---:|']
        for capture in diag['captures']:
            lines.append(f'|{capture["state"]}|{capture["semantic_pixel_count"]}|{pct(capture["rgb_color_at_instance_mask"]["red_ratio"])}|{num(capture["depth_at_instance_mask"]["median"])}|')
        lines+=['','在该固定视角，正常植被下RGB中的目标区域被树冠遮挡，HSV红色比例为0；关闭植被后红色比例约70.36%，实例掩码仅由751增至831像素（正常时仍保留约90.4%），深度中位数仅变化约0.14米。关闭RGB后处理时相同差异仍存在；相机记录的位姿、FOV与图像尺寸一致，各状态内RGB/深度/实例使用同一帧。此证据说明实例掩码或深度中“有车辆”不能直接等同于RGB中“车辆可辨识”，不能把相关帧直接算作RGB模型漏检或颜色判断错误。结论限于该诊断样本。',
            '', '关闭建筑的补充状态和恢复状态另有渲染变化，完整原图一并保留；关键判断依赖正常植被与仅关闭植被的局部对照，并没有借关闭城市遮挡提高正式任务成功率。']
    else:lines+=['','隔离可见性诊断在12个正式回合结束后执行；当前该项尚未完成。']
    text='\n'.join(lines)+'\n'
    path=DELIVERY/'完整诊断记录.md';path.write_text(text,encoding='utf-8')
    (OUT/'完整诊断记录.md').write_text(text,encoding='utf-8')
    for name in ['online_summary.csv','online_results.json','benchmark_ego_excluded_at_frozen_thresholds.json',
        'benchmark_annotation_review.csv','calibration_tradeoff.png','class_confusion.png','online_task_results.png']:
        source=OUT/'analysis'/name
        if source.exists():shutil.copy2(source,DELIVERY/name)
    save_json(DELIVERY/'交付状态.json',{'formal_completed':len(completed),'planned':12,'execution_finished':final,
        'model_acceptance_passed':False,'positive_passed':success,'positive_completed':len(positive),
        'negative_passed':negative_ok,'negative_completed':len(negative),'wrong_commits':wrong,'unverified_commits':unverified,'collisions':collisions,
        'raw_output_root':str(OUT),'report':str(DELIVERY/'任务1_在线验收报告.md'),'diagnostic_attachment':str(path),
        'qualified_formal_completed':0,'historical_diagnostic_runs':12,'baseline_established':False})
    return final


def main():
    p=argparse.ArgumentParser();p.add_argument('--watch',action='store_true');a=p.parse_args()
    benchmark=corrected_benchmark();seen=set()
    while True:
        try:
            index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8')) if (OUT/'online_index.json').exists() else {}
            changed=False
            for name,item in index.items():
                if not item.get('run_dir') or item.get('runner_status') not in ['PASS','FAIL']:continue
                run=Path(item['run_dir']);key=str(run)
                if key in seen:continue
                log=OUT/f'logs/deliver_{name}.log'
                with log.open('w',encoding='utf-8') as f:
                    code=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/analyze_online_acceptance.py')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
                    if code:raise RuntimeError(f'Analysis failed: {log}')
                    qp=run/'preview/task1_video_quality_check.json'
                    old_qa=json.loads(qp.read_text(encoding='utf-8')) if qp.exists() else {}
                    if not (run/'preview/task1_five_view.mp4').exists() or old_qa.get('renderer_version')!=5:
                        code=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/build_task1_acceptance_replay.py'),'--run-dir',str(run),'--fps','4'],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT).returncode
                        if code:raise RuntimeError(f'Replay failed: {log}')
                if not video_check(run):raise RuntimeError(f'Video QA failed: {run}')
                seen.add(key);changed=True;print('DELIVERED',name,flush=True)
            if changed or not (DELIVERY/'任务1_在线验收报告.md').exists():
                done=build_report(benchmark)
                print('REPORT',len(seen),'/12',flush=True)
                if done and len(seen)==12:break
            if not a.watch:break
        except Exception as exc:
            print('DELIVERY_ERROR',repr(exc),flush=True)
            if not a.watch:raise
        time.sleep(20)

if __name__=='__main__':main()
