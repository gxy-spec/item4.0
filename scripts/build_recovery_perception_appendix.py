"""Detailed final perception denominators and remaining error strata."""
import json,sys,csv
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def rate(a,n):return f'{a}/{n}（{100*a/n:.2f}%）' if n else '无可评价样本'
def link(p,label):return f'[{label}](<{Path(p).as_posix()}>)'


def main():
    root=OUT/'final_frontend_audit';summary=json.loads((root/'summary.json').read_text(encoding='utf-8'))
    pred=rows(root/'predictions.jsonl');up=[r for r in pred if r['variant']=='upgraded']
    data=rows(OUT/'final_attribute_audit_v1/manifest.jsonl');head=json.loads((OUT/'final_attribute_audit_results/summary.json').read_text(encoding='utf-8'))
    lines=['# 最终感知专项评估详表','','以下全部来自模型和阈值冻结后新采的FTEST审计。512幅原始图像；6幅因预先设定的小目标规则排除，保留506个已知车辆视图。另有111个经语义确认的非车辆裁剪，共617个GT裁剪。颜色仅评价464个主涂装色可观察视图。','',
        'FTEST只有一个新目标部署地点；相机以目标为中心取受控顶视、近距及斜侧视角，仍使用同一地图与车辆资产。它衡量指定目标的检测/属性能力，不衡量完整城市搜索覆盖或任意未见城市泛化。原36组闭环另用于实际轨迹回归；T1/T2更难遮挡场景作为开发压力集保留。','',
        '## 相同完整RGB-D图像上的前端比较','','|方案|已知车辆框IoU≥0.5|框IoU≥0.75|框与车型正确|框与颜色正确|框、车型和颜色均正确|','|---|---:|---:|---:|---:|---:|']
    for name,r in summary['models'].items():lines.append(f"|{name}|{rate(r['box_iou50'],r['known_target_views'])}|{rate(r['box_iou75'],r['known_target_views'])}|{rate(r['exact_type_correct'],r['known_target_views'])}|{rate(r['color_correct'],r['color_n'])}|{rate(r['joint_correct'],r['color_n'])}|")
    lines+=['','legacy是通用COCO检测+HSV，没有原红色厢式车任务的颜色区域候选；COCO没有van类别。current_yoloe_hsv是当前YOLOE+HSV；upgraded是保留YOLOE、增加通用候选补充/深度框补全/RGB属性头的完整新前端。这里的收益不能全归因于单一分类头。72回合旧对照保留原任务先验与颜色候选，因此闭环比较和本表的通用感知比较口径不同。','',
        '## 新前端按设备、车型分层','','|分层|目标视图数|框检出|框与车型正确|可评价颜色数|联合正确|','|---|---:|---:|---:|---:|---:|']
    for group in ['by_device','by_type']:
        for name,r in summary['models']['upgraded'][group].items():lines.append(f"|{name}|{r['known_target_views']}|{rate(r['box_iou50'],r['known_target_views'])}|{rate(r['exact_type_correct'],r['known_target_views'])}|{r['color_n']}|{rate(r['joint_correct'],r['color_n'])}|")
    lines+=['','## 新前端按主车身涂装色分层','','|颜色|颜色可评价视图|框与颜色正确|框、车型与颜色均正确|','|---|---:|---:|---:|']
    for color in COLORS:
        z=[r for r in up if r['truth_color']==color];lines.append(f"|{color}|{len(z)}|{rate(sum(r['color_correct'] for r in z),len(z))}|{rate(sum(r['joint_correct'] for r in z),len(z))}|")
    lines+=['','## 车型×颜色交叉分层','','单元格是联合正确数/可评价数。无样本的组合不能视为已通过，尤其本资产库中的客车主车身固定白色。','','|车型|'+'|'.join(COLORS)+'|','|---|'+'|'.join(['---:']*6)+'|']
    for typ in CLASSES:
        values=[]
        for color in COLORS:
            z=[r for r in up if r['truth_type']==typ and r['truth_color']==color];values.append(rate(sum(r['joint_correct'] for r in z),len(z)))
        lines.append('|'+typ+'|'+'|'.join(values)+'|')
    lines+=['','## 分类头自身的条件准确率','','使用GT目标框裁剪，另包含单列的非车辆背景裁剪；这部分排除了检测器漏检影响，不能冒充完整前端或街景mAP。','','|GT类别|裁剪数|按部署阈值判对车型/非车辆类别|','|---|---:|---:|']
    for typ,r in head['by_type'].items():lines.append(f"|{typ}|{r['n']}|{rate(r['deployed_type_correct'],r['n'])}|")
    lines += ['',f"颜色条件准确率{rate(head['deployed_color_correct'],head['color_n'])}，GT裁剪车型与颜色联合正确{rate(head['joint_correct'],head['color_n'])}。主报告采用完整前端联合正确446/464，不能以GT裁剪的较高结果替换。",'',
        '## 剩余错误及处理建议','','最终前端18个错误视图均来自UAV：15个车型错误、3个黑灰颜色混淆。车型错误集中为8个黑色Sprinter、1个灰色Sprinter和6个黑色HGV；地面256个车辆视图在本受控集中均判对车型和颜色。', '',
        '这是明确的颜色×车型×视角薄弱组。错误与深色顶视外观、低分辨率及表征中的颜色/车型耦合有关，但本轮没有单独做因果消融，不能把原因写成已证明。置信度门限也不能保证排除高置信错类。','',
        '下一轮可仅在训练地点补充深色长车顶、多高度/光照样本，重新训练车型分支及必要骨干；验证集按车型×颜色×设备交叉分层，加入最差组约束，必要时引入实际可获取的几何特征。再用新独立数据评价。最终测试FTEST不回填训练，也不据此挑模型来改写本轮结果。', '',
        '在线多设备策略还需区分“目标颜色在该视角不可观察”和“模型漏检/误判”：VW T2白色车顶使42个合格空中视图不具备足够主涂装色响应，这些视图仍计入车型评价，只从颜色分母剔除。增加训练不能凭空恢复不可见车身色，可由地面视角补充。此现象支持多设备视角互补的动机，尚不能证明未接入的LiDAR贡献。','',
        '## 原始依据与逐图审查','',''+link(root/'summary.json','完整前端计数JSON')+'；'+link(root/'predictions.jsonl','每个模型、每幅图像的预测')+'；'+link(OUT/'final_attribute_audit_results/summary.json','分类头计数')+'；'+link(OUT/'final_attribute_audit_v1/summary.json','样本排除与颜色可见性规则')+'。','',
        link(OUT/'final_visibility_review/contact_sheet_index.json','全部512图像及18个剩余错误的联系表索引')+'；'+link(OUT/'final_visibility_review/assistant_review.json','逐图复核记录')+'；'+link(OUT/'final_visibility_review/frontend_errors_1.jpg','全部18个剩余前端错误')+'。','',
        link(OUT/'attribute_training/resnet18_color_v1/model_manifest.json','最终模型训练与验证结果')+'；'+link(OUT/'attribute_training/resnet18_color_v1/frozen_type_proof.json','颜色头微调时车型与骨干不变的逐张量核验')+'；'+link(OUT/'calibration/deployment_release.json','验证集阈值和温度校准')+'；'+link(OUT/'post_freeze_audit_receipt.json','独立审计前后模型不变凭据')+'。']
    DELIVERY.mkdir(parents=True,exist_ok=True);(DELIVERY/'感知专项评估详表.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('PERCEPTION APPENDIX WRITTEN',flush=True)


if __name__=='__main__':main()
