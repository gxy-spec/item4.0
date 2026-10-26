"""Same episodes, two physical distance definitions. Not a model comparison."""
from pathlib import Path
import sys,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT
from build_r10_targeted_report import DEST

NAMES={'red_van':'红色厢式车','red_truck':'红色货车','yellow_car':'黄色轿车','white_bus':'白色客车',
       'gray_truck':'灰色货车','blue_van':'蓝色厢式车','black_car':'黑色轿车','gray_car':'灰色轿车'}


def main():
    scope=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'));rev=scope['active_validation_revision']
    path=OUT/f'acceptance_development_{rev}_progress.json'
    if not path.exists():return
    progress=json.loads(path.read_text(encoding='utf-8'));values=[]
    for a in progress.values():
        p=Path(a['run_dir']);source=p/'r10_final_evaluation.json'
        if not source.exists():source=p/'r10_evaluation.json'
        if not source.exists():continue
        r=json.loads(source.read_text(encoding='utf-8'))
        if r.get('expected_target_present') is True:
            values.append((NAMES[a['case']],r.get('final_true_body_clearance_m'),r.get('final_target_distance_m')))
    if not values:return
    plt.rcParams.update({'font.family':'Microsoft YaHei','axes.unicode_minus':False,'font.size':11})
    fig,axes=plt.subplots(1,2,figsize=(10,max(3.4,len(values)*.48+1.4)),sharey=True)
    y=np.arange(len(values));labels=[v[0] for v in values]
    common_limit=max(10,max(v for row in values for v in row[1:] if v is not None)+2)
    for ax,index,title,interval,color in [(axes[0],1,'真实车体净距：本轮主要距离指标',(3,8),'#2875A6'),
                                         (axes[1],2,'目标参考点距离：历史辅助指标',(3,7),'#BD742D')]:
        ax.axvspan(*interval,color='#E0EADD',alpha=.8,zorder=0)
        numbers=[v[index] if v[index] is not None else np.nan for v in values]
        ax.barh(y,numbers,color=color,height=.55)
        for row,x in enumerate(numbers):
            if np.isfinite(x):ax.text(x+.15,row,f'{x:.2f}',va='center',fontsize=10)
        ax.set_title(title,fontsize=12);ax.set_xlim(0,common_limit);ax.set_xlabel('米')
        ax.set_yticks(y,labels);ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
        ax.spines[['top','right']].set_visible(False)
    axes[0].invert_yaxis()
    fig.suptitle('同一次停车可以满足安全净距，却超过旧参考点上限',fontsize=14,y=.995)
    fig.text(.5,.015,'阴影为对应距离区间；主任务还需同时满足身份、近距确认、停稳保持和零碰撞。',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.05,1,.96]);DEST.mkdir(parents=True,exist_ok=True)
    name='停车距离口径对照.png' if len(values)==8 else '停车距离口径对照_进行中.png'
    target=DEST/name;fig.savefig(target,dpi=220,bbox_inches='tight');plt.close(fig);print(target)


if __name__=='__main__':main()
