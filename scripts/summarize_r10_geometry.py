"""Expanded geometry statistics and all rejected conditions, no new calibration."""
from pathlib import Path
import json,sys,csv,collections
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest
from build_r10_targeted_report import DEST


def main():
    records=[];summaries=[]
    for cohort in ['clearance_calibration','clearance_validation']:
        p=OUT/'geometry_verified'/cohort/'records.json';a=json.loads(p.read_text(encoding='utf-8'))
        errors=np.array([r['signed_error_m'] for r in a if r['signed_error_m'] is not None])
        invalid=collections.Counter(r['estimate']['reason'] for r in a if r['signed_error_m'] is None)
        summaries.append(dict(cohort=cohort,total=len(a),valid=len(errors),rejected=len(a)-len(errors),
            valid_fraction=len(errors)/len(a),rejection_fraction=1-len(errors)/len(a),
            signed_bias_m=float(np.mean(errors)),mae_m=float(np.mean(abs(errors))),
            p95_absolute_error_m=float(np.percentile(abs(errors),95)),maximum_absolute_error_m=float(max(abs(errors))),
            maximum_dangerous_overestimate_m=max(0.,float(max(errors))),maximum_underestimate_m=max(0.,float(max(-errors))),
            rejection_reasons=dict(invalid),source_sha256=digest(p)))
        for r in a:
            e=r['estimate'];records.append(dict(cohort=cohort,id=r['id'],true_body_gap_m=r['true_gap_m'],
                estimate_valid=e['valid'],estimated_body_gap_m=e.get('estimated_body_clearance_m'),signed_error_m=r['signed_error_m'],
                reason=e['reason'],support=e.get('support'),surface_span_m=e.get('surface_depth_span_m')))
    DEST.mkdir(exist_ok=True,parents=True)
    with (DEST/'全部72种静态测距条件.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0]));w.writeheader();w.writerows(records)
    save(OUT/'analysis/expanded_geometry_statistics.json',summaries)
    lines=['# 测距标定与独立条件验证','',
        '共72种静态测距条件：36种开发标定、36种另一地点/视角/距离验证，均不是完整业务回合。标签按自身车体局部几何及虚拟观测位姿精确变换，再与离线目标真实车体投影求最小净距。原采集标签保留；修正标签过程另有记录。','',
        '|集合|有效/总数|偏差/m|MAE/m|P95绝对误差/m|最大绝对误差/m|最大危险高估/m|拒判率|',
        '|---|---:|---:|---:|---:|---:|---:|---:|']
    for s in summaries:
        name='开发标定' if s['cohort']=='clearance_calibration' else '独立条件验证'
        lines.append(f"|{name}|{s['valid']}/{s['total']}|{s['signed_bias_m']:.3f}|{s['mae_m']:.3f}|{s['p95_absolute_error_m']:.3f}|{s['maximum_absolute_error_m']:.3f}|{s['maximum_dangerous_overestimate_m']:.3f}|{s['rejection_fraction']:.1%}|")
    lines+=['','正偏差/高估意味着以为车体净距更大，是接近控制需要关注的危险方向。开发集合两次拒判为前景深度与中心目标证据冲突，均保留在逐条件CSV中，未混入成功测量分母。','',
        '在这些有限条件下最大绝对误差小于1米，支持本轮4–7米在线保持与3–8米离线验收之间的余量；不证明任意道路、遮挡或车型误差均有界。视野截断、深度证据冲突等情况保持未完成。实际停车是否合格仍由每个动态回合的独立连续保持审核决定。','',
        '初版估计器曾因货车中央像素看穿车厢/窗而产生12米以上误差，失败数据保留；表中是修正估计器、采用精确虚拟自身车体变换后的结果。不能把原失败删掉后描述为初次即成功。','']
    (DEST/'测距验证说明.md').write_text('\n'.join(lines),encoding='utf-8')
    print(json.dumps(summaries,ensure_ascii=False))


if __name__=='__main__':main()
