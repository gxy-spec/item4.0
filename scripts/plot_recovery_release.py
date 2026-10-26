"""Scientific summary figures from audited counts; never synthetic results."""
import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,DELIVERY,save,digest


def main():
    dest=DELIVERY/'figures';dest.mkdir(parents=True,exist_ok=True);receipts=[]
    for folder,label in [('frontend_release','Development stress set'),('final_frontend_audit','Post-freeze independent image audit')]:
        path=OUT/folder/'summary.json'
        if not path.exists():continue
        data=json.loads(path.read_text(encoding='utf-8'))['models'];fig,axs=plt.subplots(1,3,figsize=(13,4.6),sharey=True)
        for ax,dev in zip(axs,['all','uav','ugv']):
            for k,(variant,color) in enumerate([('legacy','#8d99ae'),('current_yoloe_hsv','#df9b36'),('upgraded','#247ba0')]):
                r=data[variant] if dev=='all' else data[variant]['by_device'][dev]
                counts=[(r['box_iou50'],r['known_target_views']),(r['exact_type_correct'],r['known_target_views']),(r['joint_correct'],r['color_n'])]
                x=np.arange(3)+(k-1)*.25;y=[a/n if n else 0 for a,n in counts]
                bars=ax.bar(x,y,.24,color=color,label=variant)
                for bar,(a,n) in zip(bars,counts):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.018,f'{a}/{n}',ha='center',fontsize=7,rotation=65)
            ax.set_xticks(np.arange(3),['Box IoU>=0.5','Box + type','Box + type + color']);ax.set_title(dev.upper());ax.set_ylim(0,1.28);ax.set_yticks(np.arange(0,1.01,.2));ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        axs[0].set_ylabel('Known-target correctness / view')
        handles,labels=axs[0].get_legend_handles_labels();fig.legend(handles,labels,loc='upper center',bbox_to_anchor=(.5,.93),ncol=3,fontsize=8)
        fig.suptitle(label+' (not exhaustive precision/AP)',fontsize=12,y=.995);fig.tight_layout(rect=[0,.08,1,.86])
        footer=('One new target deployment; controlled views on same map/assets, not city-wide AP. ' if folder=='final_frontend_audit' else 'Developmental stress views on same map/assets. ')
        fig.text(.5,.02,footer+'All eligible known targets included; color-unobservable views excluded only from color scores; correlated views.',ha='center',fontsize=7)
        p=dest/(folder+'_comparison.png');fig.savefig(p,dpi=160);plt.close(fig);receipts.append(dict(figure=str(p),source=str(path),source_sha256=digest(path)))
    path=DELIVERY/'逐回合结果.json'
    if path.exists():
        data=[r for r in json.loads(path.read_text(encoding='utf-8')) if r['cohort']=='formal' and r['evaluated']]
        if data:
            fig,axs=plt.subplots(1,3,figsize=(12,4.5),sharey=True)
            for ax,key,title in zip(axs,['identity_task_success','sensor_safe_approach_audited','historical_centre_arrival_success'],['Correct identity + local confirmation','Stopped surface approach3..8m','Historical centre arrival3..7m']):
                for i,(variant,color) in enumerate([('legacy','#8d99ae'),('upgraded','#247ba0')]):
                    z=[[r for r in data if r['variant']==variant and r['group']==g] for g in ['A','B','C']]
                    counts=[sum(bool(r.get(key)) for r in q) for q in z];bars=ax.bar(np.arange(3)+(i-.5)*.3,counts,.29,color=color,label=variant)
                    for b,n,q in zip(bars,counts,z):ax.text(b.get_x()+b.get_width()/2,n+.15,f'{n}/{len(q)}',ha='center',fontsize=9)
                ax.set_xticks(np.arange(3),['A: patrol','B: UGV','C: UAV+UGV']);ax.set_title(title,fontsize=10);ax.set_ylim(0,13);ax.set_yticks(range(0,13,2));ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
            axs[0].legend();axs[0].set_ylabel('Successful episodes /12 per group');fig.suptitle(f'Original36 paired regression: {"complete" if len(data)==72 else "IN PROGRESS"} {len(data)}/72, all outcomes retained');fig.tight_layout(rect=[0,.06,1,.95]);fig.text(.5,.015,'A is the non-responsive control. Incomplete counts are labelled explicitly;0/0 is not a completed comparison.',ha='center',fontsize=8)
            p=dest/'formal_paired_outcomes.png';fig.savefig(p,dpi=160);plt.close(fig);receipts.append(dict(figure=str(p),source=str(path),source_sha256=digest(path)))
    save(dest/'figure_provenance.json',receipts);print('FIGURES',len(receipts),flush=True)


if __name__=='__main__':main()
