"""Publication-style ROI evidence from the isolated visibility captures."""
import json
import cv2
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from perception_online_acceptance import OUT

root=OUT/'observability_diagnostic'
data=json.loads((root/'results.json').read_text(encoding='utf-8'))
captures=[next(r for r in data['captures'] if r['state']==s) for s in ['normal','vegetation_off']]
boxes=[r['instance_bbox'] for r in captures]
x1=max(0,min(b[0] for b in boxes)-8);x2=min(800,max(b[2] for b in boxes)+8)
y1=max(0,min(b[1] for b in boxes)-8);y2=min(600,max(b[3] for b in boxes)+8)
fig,axes=plt.subplots(2,4,figsize=(13.5,7.4))
for row,r in enumerate(captures):
    state=r['state'];rgb=cv2.cvtColor(cv2.imread(str(root/f'{state}_rgb.png')),cv2.COLOR_BGR2RGB)
    plain=cv2.cvtColor(cv2.imread(str(root/f'{state}_rgb_plain.png')),cv2.COLOR_BGR2RGB)
    mask=np.load(root/f'{state}_target_mask.npy');depth=np.load(root/f'{state}_depth_metres.npy')
    axes[row,0].imshow(rgb[y1:y2,x1:x2]);axes[row,1].imshow(plain[y1:y2,x1:x2])
    axes[row,2].imshow(mask[y1:y2,x1:x2],cmap='gray',vmin=0,vmax=1)
    image=axes[row,3].imshow(depth[y1:y2,x1:x2],cmap='viridis',vmin=50,vmax=60)
    axes[row,0].set_title(f"RGB | red fraction {r['rgb_color_at_instance_mask']['red_ratio']:.1%}",fontsize=10)
    axes[row,1].set_title(f"RGB postprocess off | {r['rgb_plain_color_at_instance_mask']['red_ratio']:.1%}",fontsize=10)
    axes[row,2].set_title(f"Instance mask | {r['semantic_pixel_count']} pixels",fontsize=10)
    axes[row,3].set_title(f"Depth median | {r['depth_at_instance_mask']['median']:.2f} m",fontsize=10)
    axes[row,0].set_ylabel(('Normal vegetation' if row==0 else 'Vegetation disabled')+f"\nframe {r['frame']}",fontsize=11)
    for ax in axes[row]:ax.set_xticks([]);ax.set_yticks([])
fig.suptitle('Fixed camera and target: RGB visibility differs from automatic instance/depth evidence',fontsize=13)
fig.subplots_adjust(top=.86,bottom=.07,left=.07,wspace=.3,hspace=.35,right=.89)
color_axis=fig.add_axes([.925,.18,.012,.60])
fig.colorbar(image,cax=color_axis,label='Camera depth (m)')
fig.savefig(root/'visibility_roi_audit.png',dpi=180,bbox_inches='tight');plt.close(fig)
print(root/'visibility_roi_audit.png')
