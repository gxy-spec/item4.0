"""Offline v4 replay revision. Preserve v3, render every formal episode, QA all."""
from concurrent.futures import ThreadPoolExecutor, as_completed
import json
from pathlib import Path
import shutil
import subprocess
import sys
from perception_online_acceptance import OUT, ROOT, save_json
from package_online_acceptance_results import video_check


def revise(item):
    name, record=item
    run=Path(record['run_dir']); preview=run/'preview'
    old=preview/'v3_before_patrol_overlay'; old.mkdir(exist_ok=True)
    for filename in ['task1_five_view.mp4','task1_five_view.json','task1_trajectory_map.png','task1_video_quality_check.json','task1_replay_poster.jpg']:
        source=preview/filename
        if source.exists() and not (old/filename).exists():shutil.copy2(source,old/filename)
    log=OUT/f'logs/replay_v5_{name}.log'
    with log.open('w',encoding='utf-8') as stream:
        completed=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/build_task1_acceptance_replay.py'),
            '--run-dir',str(run),'--fps','4'],cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    if completed.returncode:raise RuntimeError(f'{name}: {log}')
    if not video_check(run):raise RuntimeError(f'Video QA failed: {name}')
    manifest=json.loads((preview/'task1_five_view.json').read_text(encoding='utf-8'))
    if manifest['renderer_version']!=5:raise RuntimeError('Wrong renderer version')
    evidence=json.loads((preview/'task1_route_overlay_evidence.json').read_text(encoding='utf-8'))
    row={'case':name,'passed':True,'run_dir':str(run),'renderer_version':5,
         'frames':manifest['frame_count'],'patrol_length_m':evidence['original_patrol_length_m'],
         'replan_event_count':len(evidence['target_replanning_events']), 'target_planned_polyline_archived':False,
         'map':str(preview/'task1_trajectory_map.png'),'video':str(preview/'task1_five_view.mp4')}
    print('REVISED',name,'QA PASS',flush=True)
    return row


if __name__=='__main__':
    index=json.loads((OUT/'online_index.json').read_text(encoding='utf-8'))
    rows=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        jobs=[pool.submit(revise,item) for item in index.items()]
        for future in as_completed(jobs):rows.append(future.result())
    rows.sort(key=lambda x:x['case'])
    save_json(OUT/'analysis/visualization_revision_v5.json',{'simulation_rerun':False,'all_12_passed':len(rows)==12 and all(r['passed'] for r in rows),'runs':rows})
