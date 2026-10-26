"""Contact sheets of every final snapshot, plus all final frontend errors.

Human/assistant review does not alter the predeclared evaluation denominator.
"""
import sys,json,cv2,numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def paint_cell(canvas,row,col,r,title):
    x=col*200;y=row*140;im=cv2.imread(r['image']);box=r.get('truth_bbox',r.get('bbox'))
    if box:
        a,b,c,d=map(int,box);crop=im[max(0,b-2):min(600,d+2),max(0,a-2):min(800,c+2)]
    else:crop=im
    scale=min(194/crop.shape[1],106/crop.shape[0]);crop=cv2.resize(crop,(max(1,int(crop.shape[1]*scale)),max(1,int(crop.shape[0]*scale))))
    canvas[y+23:y+23+crop.shape[0],x+3:x+3+crop.shape[1]]=crop
    cv2.putText(canvas,title[:37],(x+3,y+16),0,.34,(25,35,50),1,cv2.LINE_AA)
    cv2.putText(canvas,r['id'].split('_b')[-1][-20:],(x+3,y+138),0,.32,(25,35,50),1,cv2.LINE_AA)


def main():
    capture=rows(OUT/'final_independent_audit_v1/manifest.jsonl');pred=rows(OUT/'final_frontend_audit/predictions.jsonl')
    dest=OUT/'final_visibility_review';dest.mkdir(exist_ok=True);files=[]
    for start in range(0,len(capture),64):
        z=capture[start:start+64];canvas=np.full((1120,1600,3),248,np.uint8)
        for i,r in enumerate(z):paint_cell(canvas,i//8,i%8,r,f"{r['category']}/{r['requested_color']} {r['device']}"+(' NO MASK' if not r['bbox'] else ''))
        p=dest/f'all_views_{start//64+1}.jpg';cv2.imwrite(str(p),canvas,[cv2.IMWRITE_JPEG_QUALITY,95]);files.append(dict(file=str(p),ids=[r['id'] for r in z]))
    errors=[r for r in pred if r['variant']=='upgraded' and (not r['exact_type_correct'] or r['truth_color'] is not None and not r['color_correct'])]
    for start in range(0,len(errors),32):
        z=errors[start:start+32];canvas=np.full((560,1600,3),248,np.uint8)
        for i,r in enumerate(z):paint_cell(canvas,i//8,i%8,r,f"GT {r['truth_type']}/{r['truth_color']} -> {r['predicted_type']}/{r['predicted_color']}")
        p=dest/f'frontend_errors_{start//32+1}.jpg';cv2.imwrite(str(p),canvas,[cv2.IMWRITE_JPEG_QUALITY,95]);files.append(dict(file=str(p),ids=[r['id'] for r in z],all_remaining_upgraded_frontend_errors=True))
    save(dest/'contact_sheet_index.json',dict(total_original_snapshots=len(capture),frontend_error_views=len(errors),files=files,
        scope='Every controlled snapshot and every remaining upgraded frontend type/color error displayed; no denominator changes or retraining from these sheets. Contact crops are derived from offline GT boxes, never runtime inputs.'))
    print('VISIBILITY SHEETS',len(files),'error views',len(errors),flush=True)


if __name__=='__main__':main()
