from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest

source=ROOT/'scripts/build_r10_validation_replay.py';target=ROOT/'scripts/build_r10_validation_replay_v2.py'
if target.exists():raise RuntimeError('Renderer v2 already exists')
s=source.read_text(encoding='utf-8')
old='    points.extend(p[:2] for p in patrol)'
assert s.count(old)==1;s=s.replace(old,old+"\n    points.extend(p[:2] for event in route_events for p in event.get('route_xyz',[]))")
s=s.replace('r10_validation_five_view.mp4','r10_v2_five_view.mp4').replace('r10_validation_trajectory_map.png','r10_v2_trajectory_map.png')
s=s.replace('r10_validation_route_overlay_evidence.json','r10_v2_route_overlay_evidence.json')
s=s.replace('data.update(renderer_version=7,','data.update(renderer_version=10,renderer_revision="R10 post-run v2; viewport includes every planned route",')
target.write_text(s,encoding='utf-8')
save(OUT/'renderer_revision_v2.json',dict(revision='R10 post-run renderer2',source_sha256=digest(source),
    renderer_path=str(target),renderer_sha256=digest(target),
    changes='Viewport also contains complete planned paths; separate filenames preserve v1. No acquisition/controller changes.'))
print(target)
