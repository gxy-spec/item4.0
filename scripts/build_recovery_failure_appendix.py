"""Retain a readable ledger of development/infra failures and their evidence."""
import json,sys,csv
from pathlib import Path
from collections import Counter
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def main():
    all_attempts=rows(OUT/'attempt_history.jsonl');ledger=[]
    for i,a in enumerate(all_attempts,1):
        run=Path(a['run_dir']) if a.get('run_dir') else None
        ev=run/'recovery_evaluation.json' if run else None
        r=json.loads(ev.read_text(encoding='utf-8')) if ev and ev.exists() else {}
        if not a.get('report_complete'):outcome='infrastructure_incomplete'
        elif a.get('runner_status')!='PASS':outcome='runtime_checks_failed'
        elif r:
            if r.get('expected_target_present') is False:outcome='negative_rejected' if r.get('negative_rejection_pass') and r.get('sent_messages',0)==0 else 'negative_false_response'
            elif a.get('group')=='A':outcome='A_nonresponsive_control'
            else:outcome='identity_and_surface_pass' if r.get('identity_task_success') and r.get('sensor_safe_approach_audited') else 'task_incomplete'
        else:outcome='runtime_pass_without_business_audit'
        ledger.append(dict(attempt=i,started_utc=a.get('started_utc'),case=a.get('case'),group=a.get('group'),variant=a.get('variant'),
            outcome=outcome,runner_status=a.get('runner_status'),complete=a.get('report_complete'),
            identity=r.get('identity_task_success'),surface=r.get('sensor_safe_approach_audited'),centre=r.get('historical_centre_arrival_success'),
            collisions=r.get('collisions',a.get('summary',{}).get('ugv_collision_count') if isinstance(a.get('summary'),dict) else None),
            task_reasons=';'.join(r.get('reasons',[])),runner_failed_checks=json.dumps(a.get('failed_checks',[]),ensure_ascii=False),
            error=a.get('error'),config=a.get('config'),run_dir=str(run) if run else None,log=a.get('log')))
    DELIVERY.mkdir(parents=True,exist_ok=True)
    with (DELIVERY/'全部运行尝试_含失败.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(ledger[0]));w.writeheader();w.writerows(ledger)
    lines=['# 失败与修复记录','','所有尝试保留在原始目录和attempt_history.jsonl；运行检查通过不等于任务完成。下表按原因归类，完整逐次记录见“全部运行尝试_含失败.csv”。','',
        '|问题|直接证据与影响|处理|保留的边界|','|---|---|---|---|',
        '|早期近距车型不稳定|同一黄色轿车尾部曾被判van，阻断正确目标确认|补充近距真实相机尺度数据，双分支ResNet18选择；训练点与后续图像审计分开|同地图、有限资产，非任意实车泛化|',
        '|早期空中黑灰颜色不足|新增独立压力集暴露黑色空中识别为0等弱项；原黑色测试仅2例不能证明稳定性|原训练地点新采空中六色数据，只微调颜色头；骨干与车型头逐张量一致性验证|T1/T2因此转为开发压力集，另采冻结后最终审计|',
        '|仿真标签与相机采集错误|相机曾瞄准未同步的原点；实例ID误并非车辆像素；客车请求颜色未改变真实白色主车身|生成后tick及位置断言；语义与投影ROI过滤；六色渲染响应与可见车漆核实|完整背景车辆仍非穷尽标注，不能据此报告完整mAP|',
        '|划分编号重复实际停车点|N1/N2、W1/W2、F3/W3等物理位置存在重用|记录物理位置重叠；同图压力与冻结后新目标部署审计分别报告|原36组保持原样，只作同地图回归|',
        '|行人侧向盲区|M3真实4米制动条件发生29个碰撞事件（同一个行人），不能用更早8米停止的测试覆盖|近处路边行人让行、当前方向与越线历史、短时轨迹记忆|最终四类安全场景是有限场景验证，不是交通安全认证|',
        '|阴影/静止物体误检导致卡停|去除底部截断过滤后，平面阴影低置信person可造成长期停车|深度高度支撑、直立跨度及高置信近侧让行；保留原误检回合|深度地面判断基于当前Town10局部平路约定|',
        '|R8红色货车识别正确但停车失败|最终表面深度2.984米，低于未修改的3米下限；路线终点不保证不同车长的表面间距|R9在新鲜本地确认表面<=4.5米时额外制动；重验后约4.214米停车、保持通过|原车辆中心距约10.03米仍不满足3–7米，单列失败|',
        '|原中心距与表面距离不等价|远端位置估计/终点容差与不同车长，使正确识别停车仍可能中心距>7米|保留中心指标；分别记录身份、本地确认、表面停车与中心到达|不能把新业务停车判据替换成旧指标已通过|',
        '|Windows长路径及停止竞态|旧负例采集完成后报告复制失败，算ERROR；曾在父进程停止时留下新启动子进程|短ID/路径预算检查，单一仿真锁、孤儿进程拒绝、回合间停止|不完整尝试不计作成功或有效任务失败|',
        '|R9灰轿车预热中断|RPC120秒超时、传感器连接被拒绝，尚未产生有效回合|相同冻结配置冷重启；限制同配置最多3次基础设施失败；保留原ERROR|服务器原生失效原因尚无日志可定论，不虚称已修复底层引擎|','',
        '当前逐次分类计数：`'+json.dumps(dict(Counter(r['outcome'] for r in ledger)),ensure_ascii=False)+'`。','',
        '正式36+36内，完成采集的任务失败均继续保留，不重跑筛掉；只有基础设施不完整回合允许相同冻结配置重跑，并保留原尝试记录。所有模型、阈值或控制更改使用新版本，避免在同一正式批次内混用。']
    (DELIVERY/'失败与修复记录.md').write_text('\n'.join(lines)+'\n',encoding='utf-8');print('ATTEMPTS',len(ledger),flush=True)


if __name__=='__main__':main()
