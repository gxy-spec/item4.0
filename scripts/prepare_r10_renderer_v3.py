"""Separate post-run renderer; also supports safety-only cases, no capture change."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import ROOT,OUT,save,digest

source=ROOT/'scripts/build_r10_validation_replay_v2.py';target=ROOT/'scripts/build_r10_validation_replay_v3.py'
if target.exists():raise RuntimeError('Do not overwrite renderer revisions')
s=source.read_text(encoding='utf-8')
s=s.replace("present=config['online_acceptance']['expected_target_present']", "goal=config.get('online_acceptance')\npresent=bool(goal and goal['expected_target_present'])")
s=s.replace("        goal=config['online_acceptance']\n", "        goal=config.get('online_acceptance')\n")
old="        banner=f\"PERCEPTION RECOVERY | {goal['task_color']} {goal['task_category']} | TARGET {'PRESENT' if present else 'ABSENT'} | AUTO LABEL AUDIT: RGB visibility unverified\""
assert old in s
s=s.replace(old,old+" if goal else f\"SAFETY REGRESSION | {config['experiment_id']} | map is an offline annotation\"")
s=s.replace('r10_v2_','r10_v3_').replace('renderer_version=10','renderer_version=11').replace('R10 post-run v2; viewport includes every planned route','R10 post-run v3; complete viewport and safety-case support')
target.write_text(s,encoding='utf-8')
save(OUT/'renderer_revision_v3.json',dict(revision='R10 post-run renderer3',path=str(target),sha256=digest(target),
    parent_sha256=digest(source),scope='Video/PNG only; safety banner supported; runtime and frozen snapshots unchanged'))
print(target)
