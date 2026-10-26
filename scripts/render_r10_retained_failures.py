"""Preserve reviewable media for complete failed/development attempts, outside22."""
from pathlib import Path
import json,sys,csv,subprocess,argparse
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,rows


def main():
    p=argparse.ArgumentParser();p.add_argument('--render',action='store_true');args=p.parse_args();records=[];seen=set()
    for item in rows(OUT/'attempt_history.jsonl'):
        if not item.get('report_complete') or not item.get('run_dir'):continue
        run=Path(item['run_dir'])
        if str(run) in seen:continue
        seen.add(str(run));ep=run/'r10_evaluation.json';e=json.loads(ep.read_text(encoding='utf-8')) if ep.exists() else {}
        report=json.loads((run/'validation_report.json').read_text(encoding='utf-8'))
        if report['status']=='ERROR':continue
        failure=report['status']=='FAIL' or bool(e.get('expected_target_present') and item.get('group')!='A' and not e.get('S3_found_and_safely_approached'))
        if not failure:continue
        if args.render and not (run/'preview/r10_v3_five_view.json').exists():
            framepath=run/'synchronization/frame_index.csv'
            frames=[r for r in csv.DictReader(framepath.open(encoding='utf-8-sig')) if int(r.get('sensor_files_saved','1'))]
            fps=1/float(np.median(np.diff([float(r['timestamp']) for r in frames])))
            log=OUT/f'logs/render_retained_{run.name}.log'
            with log.open('w',encoding='utf-8') as f:
                subprocess.run([sys.executable,'-B','-u','-X','utf8',str(ROOT/'scripts/build_r10_validation_replay_v3.py'),
                    '--run-dir',str(run),'--fps',str(fps)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
            print('RENDERED RETAINED',item['case'],run.name,flush=True)
        records.append(dict(case=item['case'],variant=item.get('variant'),group=item.get('group'),
            acquisition_revision=item.get('development_revision'),raw_runner=report['status'],S3=e.get('S3_found_and_safely_approached'),
            reason=e.get('reasons',item.get('failed_checks')),run_dir=str(run),
            video=str(run/'preview/r10_v3_five_view.mp4'),trajectory_map=str(run/'preview/r10_v3_trajectory_map.png'),
            scope='Retained raw failure/development evidence; may include documented evaluator-only false rejection; not silently counted as a new successful run'))
    save(OUT/'analysis/retained_failure_media.json',records);print('RETAINED MEDIA',len(records),flush=True)


if __name__=='__main__':main()
