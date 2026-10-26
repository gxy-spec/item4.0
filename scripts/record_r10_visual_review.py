"""Record the representative images actually inspected during this session."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest
from build_r10_targeted_report import DEST


def main():
    directory=DEST/'可视化核查'
    checks=[('_overtake__三帧核查.png','First/middle/last: original patrol, executed passing offset and stationary return are visible; no task-target claim.'),
        ('upgraded_blue_van_C_三帧核查.png','First/middle/last: no future plan in first frame, real red vehicle blocker at95s, correct blue target at end. UAV view can include roof occlusion.'),
        ('upgraded_W3_C_三帧核查.png','First/middle/last: full original patrol, time-gated plan and correct target final ground view.'),
        ('upgraded_F3_C_三帧核查.png','First/middle/last: complete long map route, no first-frame plan, final red van ground view.'),
        ('upgraded_F1_C_三帧核查.png','First/middle/last: final failed run remains stationary before target; golden candidate plans and actual orange path are separate; no success inferred from the shorter plan.'),
        ('N3_分歧前同时间输入对照.png','Matched7.25s full RGB images viewed side by side; subtle numerical differences quantified separately, not visible evidence of unique renderer root cause.'),
        ('W2_实际回放规划前后.png','Actual encoded before/after frames were inspected; no pre-event gold plan, post-event candidate line appears. Numeric ROI avoids counting pedestrian/actor markers as routes.')]
    records=[]
    for name,note in checks:
        p=directory/name
        if not p.is_file():raise RuntimeError('Missing inspected artifact '+str(p))
        records.append(dict(path=str(p),sha256=digest(p),review_note=note))
    save(OUT/'analysis/manual_visual_review.json',dict(records=records,
        scope='Selected first/middle/last contact sheets and the actual W2 boundary frames were visually inspected; not a frame-by-frame review of every complete video. Automated decode/provenance checks are separate.'))
    print('RECORDED',len(records),'actually inspected visual artifacts')


if __name__=='__main__':main()
