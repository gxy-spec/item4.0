"""Prepare paired contact sheets for human review; never marks them reviewed."""
import json,sys,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    progress=json.loads((OUT/'formal_progress.json').read_text(encoding='utf-8'))
    reviews=json.loads((OUT/'manual_visual_review.json').read_text(encoding='utf-8'))['reviews']
    dest=OUT/'manual_qa_final';dest.mkdir(exist_ok=True)
    font=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',19)
    small=ImageFont.truetype('C:/Windows/Fonts/arial.ttf',15)
    pairs={};manifest={}
    for k,v in progress.items():pairs.setdefault((v['case'],v['group']),{})[v['variant']]=(k,v)
    for (case,group),vv in pairs.items():
        if all('formal/'+k in reviews for k,v in vv.values()):continue
        canvas=Image.new('RGB',(1600,1140),'white');draw=ImageDraw.Draw(canvas);sources=[]
        for col,variant in enumerate(['legacy','upgraded']):
            k,v=vv[variant];run=Path(v['run_dir']);e=json.loads((run/'recovery_evaluation.json').read_text(encoding='utf-8'))
            draw.text((col*800+5,5),k+' | identity='+str(e.get('identity_task_success'))+' surface='+str(e.get('sensor_safe_approach_audited')),font=font,fill='black')
            hashes={}
            for name,y,size in [('recovery_key_evidence.jpg',35,(800,600)),('recovery_scene_samples.jpg',655,(800,471))]:
                p=run/'preview'/name;im=Image.open(p).convert('RGB');im=im.resize(size,Image.Resampling.LANCZOS);canvas.paste(im,(col*800,y));hashes[name]=digest(p)
            sources.append(dict(key=k,run_dir=str(run),hashes=hashes))
        path=dest/f'{case}_{group}_rgb.jpg';canvas.save(path,quality=94)
        manifest[path.name]=dict(kind='rgb',path=str(path),sources=sources,sheet_sha256=digest(path))
    for case in dict.fromkeys(v['case'] for v in progress.values()):
        canvas=Image.new('RGB',(1560,1650),'white');draw=ImageDraw.Draw(canvas);sources=[]
        for row,group in enumerate(['A','B','C']):
            for col,variant in enumerate(['legacy','upgraded']):
                k,v=pairs[(case,group)][variant];run=Path(v['run_dir']);p=run/'preview/recovery_trajectory_map.png'
                im=Image.open(p).convert('RGB');im.thumbnail((750,510));x=col*780;y=row*550
                canvas.paste(im,(x,y+30));draw.text((x+5,y+5),k,font=font,fill='black')
                sources.append(dict(key=k,run_dir=str(run),map_sha256=digest(p)))
        path=dest/f'{case}_maps.jpg';canvas.save(path,quality=94)
        manifest[path.name]=dict(kind='maps',path=str(path),sources=sources,sheet_sha256=digest(path))
    (dest/'index.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(directory=str(dest),rgb=sum(v['kind']=='rgb' for v in manifest.values()),maps=sum(v['kind']=='maps' for v in manifest.values()))))

if __name__=='__main__':main()
