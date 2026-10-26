"""Separate post-run renderer revision; no frozen acquisition source is edited."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest

source=OUT/'frozen_validation_v1/scripts/build_recovery_replay.py'
target=ROOT/'scripts/build_r10_validation_replay.py'
if target.exists():raise RuntimeError('Renderer exists; create a new renderer revision')
s=source.read_text(encoding='utf-8')
old="if e.get('event')=='target_route_planned_from_selected_message']"
assert s.count(old)==1;s=s.replace(old,"if e.get('event') in {'target_route_planned_from_selected_message','blocked_route_replanned'}]")
s=s.replace('Red star: configured target','Red star: target (offline reference only)')
s=s.replace('recovery_five_view.mp4','r10_validation_five_view.mp4').replace('recovery_trajectory_map.png','r10_validation_trajectory_map.png')
s=s.replace('recovery_route_overlay_evidence.json','r10_validation_route_overlay_evidence.json')
target.write_text(s,encoding='utf-8')
save(OUT/'renderer_revision.json',dict(revision='R10 post-run renderer1',source_sha256=digest(source),
    renderer_path=str(target),renderer_sha256=digest(target),changes='Include actual bounded-recovery routes, offline target annotation, causal candidate/selected labels and original patrol; no simulation rerun'))
print(target)
