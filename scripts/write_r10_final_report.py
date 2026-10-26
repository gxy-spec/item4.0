"""Outcome-first final document, after technical media/integrity verification."""
from pathlib import Path
from datetime import datetime
import json,sys,csv
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT
from build_r10_targeted_report import DEST


def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def link(label,p):return f'[{label}]({str(p).replace(chr(92),"/")})'


def main():
    media=read(OUT/'analysis/media_technical_check.json');integrity=read(OUT/'analysis/final_integrity_verification.json')
    if not media['passed'] or len(media['records'])!=22 or not integrity['passed'] or integrity['completed_unique_runs']!=22:
        raise RuntimeError('Finish all22 media/integrity checks first')
    result=read(DEST/'定向验收完整结果.json');f1=read(OUT/'analysis/F1_route_followup.json')
    flat=list(csv.DictReader((DEST/'22回合验证明细.csv').open(encoding='utf-8-sig')))
    if len(flat)!=22:raise RuntimeError('Final22 rows required')
    collisions=sum(int(a.get('collisions') or 0) for a in flat)
    attempts=list(csv.DictReader((DEST/'全部运行尝试_含失败.csv').open(encoding='utf-8-sig')))
    diag=[a for a in result if a['suite']=='diagnostics' and a['item']['group']=='C']
    rows=[]
    for a in diag:
        e=a['evaluation'];case=a['item']['case']
        reason={'E2':'首条计划约712米，当时仅余143秒；未能近距确认',
                'F1':'短路线已接入，仍被低矮深度障碍阻塞；缺少局部绕行'}.get(case,'连续地面确认和安全停车审核通过')
        rows.append(f"|{case}|"+('通过' if e['S3_found_and_safely_approached'] else '未完成')+f"|{e['final_true_body_clearance_m']:.2f}米|{reason}|")
    lines=['# R10 定向修复验收：尚未全部通过','',f'更新：{datetime.now():%Y-%m-%d %H:%M}。',
        '', '**22项定向验证配置已经完成采集和独立评价；距离规则、假车辆过滤和阻塞恢复已有实测支持，但E2、F1任务仍失败，N3严格轨迹配对也未通过，当前不能宣布所有问题修复完成。**','',
        '|验证部分|结果|含义|','|---|---|---|',
        '|安全场景|4/4通过|行人、横穿车辆、遮挡、超车；见下方原标签说明|',
        '|在线目标查找|8/8完成|覆盖4车型、6颜色，均找到正确目标并安全接近|',
        '|无目标与干扰负例|2/2通过|两回合均未发送目标消息|',
        '|原问题场景|4/6完成|E3、W2、W3、F3通过；E2、F1未完成|',
        '|N3旧/新对照2回合|不响应目标通过；严格配对未通过|初态和采样一致，后续轨迹仍不同|','',
        f'最终计入的22个回合均无碰撞（记录碰撞合计{collisions}）；这不能代替任务成功，也不是全场景安全保证。超车原运行器因要求返回后必须出现“cruise”字样而给出FAIL；独立物理审核证实已超越障碍、回到原路线并停稳，原FAIL保留，未掩盖历史两次真正失败的超车。','',
        '**你最关心的距离问题**','',
        '最终成功改为：正确目标的连续地面确认，车体处于可行驶区域，真实车体间隙3–8米，速度不超过0.3米/秒且停稳2秒，全回合无碰撞；在线使用传感器估计间隙4–7米，并要求至少6次新鲜观测。路口内短暂停稳按已确认规则允许，单独标注。',
        '', '8个在线正例的最终真实车体间隙为**4.47–4.70米**，原参考点距离却为**9.20–12.71米**。这些回合均通过新的独立审核，却都不能满足旧参考点上限7米。这直接说明旧口径会误拒安全接近，尤其影响长车辆；但不能据此把旧72回合直接改判为成功。',
        '', '独立静态测距验证36/36条件有效，平均绝对误差0.21米、最大0.56米；12个成功停车保持段另有100次观测和492个仿真帧证据。它们是有限条件验证，不是所有视角、天气下的精度保证。'+link('完整判据',DEST/'任务成功判据.md')+'；'+link('测距依据',DEST/'测距验证说明.md')+'。','',
        '**已经有证据支持的修正**','',
        '- 路面假车辆：真实深度高度证据排除平面纹理，3个历史误检关键帧回归通过，独立深度避障继续工作。',
        '- 转弯与换道：缩短前视距离，深度检查跟随实际执行路线；修正过程中出现的新退化也保留，并重新检查安全场景。',
        '- 阻塞后恢复：蓝色厢式车回合被真实干扰车挡住，障碍离开后，系统在连续1秒获得新鲜清空证据后恢复并完成任务，原失败保留。',
        '- 早期规划与回放：保留原巡检虚线；金色为候选核验规划，绿色为可靠发现规划，实际行驶轨迹另画。W2核查未发现已追踪链路中的提前画线或真值坐标替代；早期候选规划不等于可靠发现。'+link('W2时序证据',DEST/'W2早期规划与真值边界说明.md')+'。','',
        '**未完成项与处理方向**','',
        '|场景|任务结果|最终车体间隙|原因|','|---|---|---:|---|',*rows,'',
        'F1的首条路线已从760.04米缩短到275.29米，但实际又在前方约3.47米处检测到约0.15米高的小面积障碍，4次道路地图重规划仍未绕开，最终净距45.09米，S1/S2/S3均未通过。离线标签为植被，仅用于解释，没有送给控制器。下一步需要将实际深度占据加入局部绕行规划，并验证低矮障碍、行人、车辆和换道安全；本轮没有删除障碍或放宽制动换取成功。'+link('F1补测完整分析',DEST/'F1路线修复补测说明.md')+'。','',
        'E2需要改进收到消息前的探索/巡检与时间预算协调；本次查询的另一条约676米路线仍无法在剩余约143秒内完成。不能靠放宽到达距离或事后延长时限解决；也不能因此证明所有其他策略均不可行。','',
        'N3两回合初态、时间步与安全采样帧一致，但UGV轨迹最大相差30.62米。微小RGB差异在位置分歧前已经存在，安全决定先于行人位置出现差异；补充33帧同输入复验通过，严格闭环配对仍失败。后续模型对比需同帧感知审计及多种子统计，不能把该轨迹差异归因于目标模型。'+link('N3证据与边界',DEST/'N3对照一致性说明.md')+'。','',
        '**感知模型与本轮结论的边界**','',
        '模型仍为冻结的YOLOE主检测/分割、辅助车辆提议与深度补全、ResNet18车型/颜色属性分支，本轮未重新训练。在线灰色、黑色车辆的UAV候选精确率分别为117/445（26.29%）、107/288（37.15%），弱项仍在；最终任务成功不能掩盖这些逐帧误选。受控同帧审计、在线候选指标与闭环完成率分开解释。'+link('全部感知计数与依据',DEST/'感知指标与依据.md')+'。','',
        f'验证版本为v5，242项代码回归检查通过，94份配置的场景、路线、种子和时限保持原基准。最终v5证据包中的21回合经逐次分支影响核对复用此前结果，F1另行补测；原采集版本及复用依据保留。这是给定记录输入的决策核对，不保证另跑一次的渲染像素和物理轨迹完全相同。运行尝试台账共{len(attempts)}条，含失败和重试。原R9源码、模型和正式结果索引保留；完整72回合按要求未重跑。','',
        link('22回合明细',DEST/'22回合验证明细.csv')+' · '+link('逐回合解释',DEST/'逐回合分析.md')+' · '+link('所有回放、失败记录与审计入口',DEST/'交付与证据索引.md'),'']
    (DEST/'修复定向验收报告.md').write_text('\n'.join(lines),encoding='utf-8')
    print('FINAL REPORT WRITTEN; unresolved failures explicitly retained; no full72 claim')


if __name__=='__main__':main()
