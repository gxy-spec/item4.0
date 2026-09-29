"""Build the 2026-09-24 to 2026-10-02 group-meeting report."""
from pathlib import Path
from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn

ROOT=Path(r"E:\Research\CityInspection_GOC")
FIG=ROOT/"docs"/"figures"/"group_meeting_20260924_20261002"
OUT=ROOT/"docs"/"城市巡检空地协同阶段性组会汇报_20260924_20261002.docx"

def shade(cell, fill):
    tcPr=cell._tc.get_or_add_tcPr(); shd=OxmlElement('w:shd'); shd.set(qn('w:fill'), fill); tcPr.append(shd)
def borders(table, color='D9D9D9'):
    tblPr=table._tbl.tblPr; b=tblPr.first_child_found_in('w:tblBorders')
    if b is None: b=OxmlElement('w:tblBorders'); tblPr.append(b)
    for edge in ('top','left','bottom','right','insideH','insideV'):
        tag='w:'+edge; el=b.find(qn(tag))
        if el is None: el=OxmlElement(tag); b.append(el)
        el.set(qn('w:val'),'single'); el.set(qn('w:sz'),'4'); el.set(qn('w:space'),'0'); el.set(qn('w:color'),color)
def set_cell_text(cell, text, bold=False, color='000000', size=9):
    cell.text=''; p=cell.paragraphs[0]; p.alignment=WD_ALIGN_PARAGRAPH.LEFT
    r=p.add_run(str(text)); r.bold=bold; r.font.size=Pt(size); r.font.color.rgb=RGBColor.from_string(color)
    cell.vertical_alignment=WD_CELL_VERTICAL_ALIGNMENT.CENTER
def table(data, widths=None):
    t=doc.add_table(rows=1, cols=len(data[0])); t.alignment=WD_TABLE_ALIGNMENT.CENTER; t.style='Table Grid'; borders(t)
    for j,h in enumerate(data[0]): set_cell_text(t.rows[0].cells[j],h,True,'FFFFFF',9); shade(t.rows[0].cells[j],'365F91')
    for i,row in enumerate(data[1:]):
        cells=t.add_row().cells
        for j,val in enumerate(row):
            set_cell_text(cells[j],val,size=8.5)
            if i%2==1: shade(cells[j],'F4F7FA')
    if widths:
        for row in t.rows:
            for cell,w in zip(row.cells,widths): cell.width=Inches(w)
    doc.add_paragraph().paragraph_format.space_after=Pt(2)
    return t
def heading(text, level=1):
    p=doc.add_paragraph(style=f'Heading {level}'); p.paragraph_format.space_before=Pt(10); p.paragraph_format.space_after=Pt(5); p.add_run(text); return p
def para(text):
    p=doc.add_paragraph(); p.paragraph_format.line_spacing=1.15; p.paragraph_format.space_after=Pt(5); p.add_run(text); return p
def fig(name, caption, width=6.3):
    p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.paragraph_format.space_before=Pt(4); p.paragraph_format.space_after=Pt(2); p.add_run().add_picture(str(FIG/name), width=Inches(width))
    c=doc.add_paragraph(); c.alignment=WD_ALIGN_PARAGRAPH.CENTER; c.paragraph_format.space_after=Pt(7); r=c.add_run(caption); r.italic=True; r.font.size=Pt(8.5); r.font.color.rgb=RGBColor(80,80,80)

doc=Document(); sec=doc.sections[0]; sec.top_margin=Inches(.72); sec.bottom_margin=Inches(.65); sec.left_margin=Inches(.78); sec.right_margin=Inches(.78)
styles=doc.styles
styles['Normal'].font.name='Noto Sans SC'; styles['Normal']._element.rPr.rFonts.set(qn('w:eastAsia'),'Noto Sans SC'); styles['Normal'].font.size=Pt(10)
for s in ['Title','Heading 1','Heading 2','Heading 3']:
    styles[s].font.name='Noto Sans SC'; styles[s]._element.rPr.rFonts.set(qn('w:eastAsia'),'Noto Sans SC'); styles[s].font.color.rgb=RGBColor(0,0,0)
title_ppr=styles['Title']._element.get_or_add_pPr(); title_bdr=title_ppr.find(qn('w:pBdr'))
if title_bdr is not None: title_ppr.remove(title_bdr)
styles['Title'].font.size=Pt(24); styles['Title'].font.bold=True
styles['Heading 1'].font.size=Pt(15); styles['Heading 1'].font.bold=True
styles['Heading 2'].font.size=Pt(12); styles['Heading 2'].font.bold=True

p=doc.add_paragraph(style='Title'); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.add_run('城市巡检空地协同阶段性组会汇报')
p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.add_run('实验范围 2026年9月24日至10月2日').font.size=Pt(12)
p=doc.add_paragraph(); p.alignment=WD_ALIGN_PARAGRAPH.CENTER; p.add_run('CARLA-Air Town10HD Zone A 结构化指令驱动 UAV–UGV 协同').font.size=Pt(10)
para('本报告汇总当前阶段已经完成并可复核的实验，不把早期失败运行、被替换的配置或未经配对比较的结果当作正式结论。结论重点是：系统已经从单端感知闭环推进到双端候选对齐、持续语义通信和路径影响对照；但车辆细粒度识别、颜色泛化、通信扰动鲁棒性和跨场景泛化仍未完成。')
heading('一 研究问题与实验链条',1)
para('当前任务固定为“寻找一辆红色厢式货车，并前往其所在位置”。UAV沿固定搜索航线获取远距离视野，UGV从启动即巡检并用RGB-D感知进行本地安全控制；候选经过类别、颜色、深度和时间一致性门控后，通信模块只传输目标级状态，而不是连续原始图像。UGV根据新鲜消息规划道路路线，消息超过1秒未更新时撤销旧路线并回到巡检路线。')
para('实验链条按“平台与场景冻结 → S1感知闭环 → RGB-D安全避障 → UGV本地搜索 → UAV/UGV候选对齐 → 持续通信与A/B/C路径对照”推进。S0 Oracle仅作为闭环上界和故障定位工具，不作为最终感知结果。')
heading('二 阶段验收概览',1)
table([
['阶段','核心问题','有效样本','正式结论'],
['S1 感知闭环','UAV候选是否能驱动UGV规划、复核与安全停车','3种子 × 2目标位置 = 6组','6/6通过；错误目标消息0；最终停车距离约5.1–6.6 m'],
['RGB-D 避障','UGV能否只用自身深度完成减速、停车、恢复和失效保护','6类场景','最终有效运行全部通过；碰撞0；在线障碍真值读取0'],
['UGV 本地搜索','UGV从启动巡检并在本地RGB-D中形成目标候选','1组60 s','通过；83.14 m巡检；候选171点；不驱动路线'],
['候选对齐','UAV/UGV候选的帧、时间、坐标是否统一','1组60 s','通过；共同观测96.6%；跨端误差中位3.09 m、P90 3.11 m'],
['E7 A/B/C 对照','UAV信息是否改变路线并产生任务增益','3种子 × A/B/C + 3个B正对照','ABC 9/9通过；C最终距离均值7.28 m，A/B约71.41 m']
],[1.0,2.55,.9,2.4])
fig('fig01_stage_acceptance.png','图1 9月24日至今各阶段有效验收通过率。失败运行不删除，但不计入正式统计分母。')
heading('三 S1 感知闭环与候选生成',1)
para('S1使用YOLO车辆类提议、HSV红色属性、深度反投影和独立时序确认。当前“厢式车”不是专门训练的细粒度分类器，而是通用车辆检测与van-like启发式分数的组合；因此S1证明的是受控条件下目标级候选可以驱动闭环，不等同于已经解决真实场景中的车型和颜色识别。')
para('6组批量运行全部通过，消息定位误差约0.618–1.092 m，UGV均完成近端复核和3–7 m安全停车。候选基线的目标召回率为98.04%，但所有红色候选的精度代理只有20.67%，说明主要瓶颈不是“看不到目标”，而是“红色背景和同色干扰仍会产生过多候选”。')
heading('四 UGV RGB-D 避障验收',1)
para('避障控制在线只使用UGV RGB-D深度反投影得到的前向走廊障碍距离，并将状态划分为CLEAR、CAUTION、STOP和SENSOR_STALE。CARLA参与者位置只在仿真结束后计算碰撞和间距，不进入实时控制。')
fig('fig02_rgbd_safety_states.png','图2 最终有效RGB-D安全场景中的状态持续时间。每个控制tick为50 ms。')
para('静态障碍、横穿车辆、横穿行人、遮挡后横穿和深度失效等最终有效场景均无碰撞。深度失效场景产生81个SENSOR_STALE tick，车辆进入停车保护，深度恢复后重新起步。需要保留的限制是：部分后处理间距采用包围圆近似，不能把负的近似间距直接解释为真实碰撞；严格最小间距仍应改用朝向包围盒或多边形距离。')
heading('五 UGV 本地搜索与 E6 双端候选对齐',1)
para('9月30日的UGV本地搜索中，UGV从启动即沿巡检路线行驶，目标候选仅用于后评估，不驱动路线。正式运行完成83.14 m巡检、600/600 RGB-D配对，产生171个红色车辆候选点，其中120个点距目标真值不超过5 m。这是单目标静态场景下的候选匹配代理指标，不是复杂交通中的总体检测精度。')
para('10月1日的E6验收统一了两端的帧号、CARLA世界坐标XYZ和时间戳格式。UAV对目标的中位定位误差约0.046 m，UGV约2.92 m，跨端候选误差中位3.09 m、P90 3.11 m；共同目标观测帧比例96.6%。结果支持后续消息选择，但也表明UGV近端候选更易受视角、遮挡和深度尺度影响。')
fig('fig03_candidate_alignment_error.png','图3 E6 候选定位和跨端坐标对齐误差。误差线从中位数延伸至P90，红色虚线为5 m验收上限。')
heading('六 E7 持续通信与 A/B/C 路径影响',1)
para('E7采用配对随机种子1001、2001、3001。A组只按预设巡检路线；B组只使用UGV本地候选；C组使用UAV与UGV候选并按任务匹配、深度可靠性、时效性和时间连续性选择状态。单条消息TTL为1 s；没有新鲜消息时，系统撤销消息路线并回到巡检，不继续追踪过期目标。安全避障在三组中始终开启。')
fig('fig04_e7_abc_effect.png','图4 E7 A/B/C配对对照。左图为末时刻目标距离，误差线为3个种子的标准误；右图为偏离预设巡检路线的平均距离。')
para('正式ABC批次9/9组通过。A和B在“目标位于巡检路线之外”条件下的末时刻目标距离分别约71.41 m和71.41 m，说明UGV本地端没有把远处红色候选误认为可行动目标。C组平均末时刻距离约7.28 m，平均比A组减少约64.13 m；三种子均规划了UAV来源的目标路线，UGV碰撞数为0。B本地正对照中目标被UGV视野真正看到后，末时刻距离约7.9 m，证明B并非不能行动，而是只在本地证据足够时行动。')
fig('fig05_e7_seed1001_trajectories.png','图5 E7同一种子（1001）的目标距离轨迹。A/B保持巡检距离平台，C在收到UAV候选后明显接近目标。')
heading('七 结论与当前边界',1)
para('当前最可靠的结论是：在固定Town10HD区域、静止目标、固定航线和受控干扰下，空地双端可以从RGB-D观测生成统一目标状态，经1秒TTL语义通信驱动UGV规划，并在消息失效后安全回到巡检路线。E7对照明确显示，UAV信息在UGV本地不可见时能够产生额外的路线影响和目标接近效果。')
para('当前不能声称的内容包括：开放词汇视觉语言理解已经完成；厢式车细粒度识别已经达到高准确率；颜色分类已具备真实道路泛化能力；动态避障已完全由视觉模型解决；无线丢包、带宽竞争、随机时延和能耗权衡已经验证；C组已经在所有环境下优于B组。')
heading('八 下一步实验建议',1)
table([
['优先级','实验任务','建议指标'],
['1','把van-like启发式替换为专用车辆属性/重识别模型','车辆类别混淆矩阵、颜色Precision/Recall/F1、目标级误发率'],
['2','扩展E7通信对照到延迟、丢包和连续中断','消息新鲜度、成功率、到达时间、路线回退时间、通信字节数'],
['3','扩大目标位置、遮挡、天气和地图组合','均值、标准差、95%置信区间、失败类型分层统计'],
['4','将安全控制从深度规则基线推进到检测+跟踪','碰撞率、TTC、最小净空、误停率、恢复时间'],
['5','建立统一论文图表和数据字典','每张图明确样本数、单位、误差定义、是否使用Oracle真值']
],[.7,3.15,3.0])
heading('附录 A 术语与数据边界',1)
para('Oracle表示仿真器提供的理想真值信息，用于S0闭环上界和实验后评价，不是感知模型。当前在线目标消息标记为oracle=false；但避障验收中的碰撞事件、actor轨迹和最小间距只用于后处理。E7中的“路线偏离”是UGV轨迹到实际执行的预设巡检路线的平面距离，不是路线规划误差。')
heading('附录 B 可复核输出位置',1)
table([
['结果','路径'],
['本报告图表',r'E:\Research\CityInspection_GOC\docs\figures\group_meeting_20260924_20261002'],
['E1 RGB-D避障',r'E:\CarlaAirData\CityInspection_GOC\e4_ugv_sensor_safety\20260929'],
['E5 UGV本地搜索',r'E:\CarlaAirData\CityInspection_GOC\e5_ugv_patrol_search\20260930\formal'],
['E6 候选对齐',r'E:\CarlaAirData\CityInspection_GOC\e6_candidate_alignment\20261001\formal'],
['E7 A/B/C正式批次',r'E:\CarlaAirData\CityInspection_GOC\e7_revised_v2'],
['统一重绘脚本',r'E:\Research\CityInspection_GOC\scripts\generate_group_meeting_figures.py']
],[1.45,5.55])
for section in doc.sections:
    footer=section.footer.paragraphs[0]; footer.alignment=WD_ALIGN_PARAGRAPH.CENTER
    footer.add_run('城市巡检空地协同阶段性组会汇报 | 2026').font.size=Pt(8)
doc.save(OUT)
print(OUT)
