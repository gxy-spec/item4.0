"""Record only contact sheets explicitly inspected by the assistant."""
import sys,json,hashlib,argparse
from pathlib import Path
from datetime import datetime,timezone
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--sheets',nargs='+',required=True);p.add_argument('--notes',required=True);a=p.parse_args()
    index=json.loads((OUT/'manual_qa_final/index.json').read_text(encoding='utf-8'))
    path=OUT/'manual_visual_review.json';data=json.loads(path.read_text(encoding='utf-8'))
    for name in a.sheets:
        sheet=index[name];assert digest(sheet['path'])==sheet['sheet_sha256']
        for src in sheet['sources']:
            run=Path(src['run_dir']);r=data['reviews'].setdefault('formal/'+src['key'],{})
            r.update(run_dir=str(run),reviewed_utc=datetime.now(timezone.utc).isoformat())
            if sheet['kind']=='rgb':
                for n,h in src['hashes'].items():assert digest(run/'preview'/n)==h
                r.update(evidence_board_sha256=src['hashes']['recovery_key_evidence.jpg'],scene_sample_sha256=src['hashes']['recovery_scene_samples.jpg'],rgb_support_reviewed=True,notes=a.notes)
            else:
                assert digest(run/'preview/recovery_trajectory_map.png')==src['map_sha256']
                r.update(trajectory_map_sha256=src['map_sha256'],trajectory_notes=a.notes)
            r.setdefault('reviewed_contact_sheets',{})[name]=sheet['sheet_sha256']
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print('FORMAL RGB REVIEWED',sum(k.startswith('formal/') and v.get('rgb_support_reviewed',False) for k,v in data['reviews'].items()))

if __name__=='__main__':main()
