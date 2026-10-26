"""Postprocess completed runs only. Never touches simulator/acquisition sources."""
from pathlib import Path
import sys,json,csv,subprocess,argparse
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest
from finalize_r10_business_evaluation import finalize
from audit_r10_route_provenance import audit


def main():
    p=argparse.ArgumentParser();p.add_argument('--render',action='store_true');a=p.parse_args()
    rev=json.loads((OUT/'execution_scope.json').read_text(encoding='utf-8'))['active_validation_revision']
    results=[]
    for cohort in ['safety','acceptance','diagnostics']:
        index=OUT/f'{cohort}_development_{rev}_progress.json'
        if not index.exists():continue
        for item in json.loads(index.read_text(encoding='utf-8')).values():
            if not item.get('report_complete'):continue
            run=Path(item['run_dir']);record=dict(cohort=cohort,case=item['case'],variant=item.get('variant'),group=item.get('group'),run_dir=str(run))
            if cohort!='safety':
                if not (run/'r10_evaluation.json').exists():continue
                finalize(item);provenance=audit(run)
                record.update(route_causality_pass=provenance['passed'],route_chains=provenance['chains'])
            if a.render and not (run/'preview/r10_v3_five_view.json').exists():
                frames=[r for r in csv.DictReader((run/'synchronization/frame_index.csv').open(encoding='utf-8-sig')) if int(r.get('sensor_files_saved','1'))]
                times=[float(r['timestamp']) for r in frames]
                interval=float(np.median(np.diff(times)));fps=1./interval
                log=OUT/f'logs/render_{rev}_{run.name}.log'
                with log.open('w',encoding='utf-8') as f:
                    subprocess.run([sys.executable,'-B','-u','-X','utf8',str(ROOT/'scripts/build_r10_validation_replay_v3.py'),
                        '--run-dir',str(run),'--fps',str(fps)],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
                print('RENDERED',cohort,item['case'],f'fps={fps:.6f}',flush=True)
            video=run/'preview/r10_v3_five_view.mp4';manifest=video.with_suffix('.json')
            h264=run/'preview/r10_v3_five_view_h264.mp4'
            if h264.exists() and h264.with_suffix('.json').exists():video=h264;manifest=h264.with_suffix('.json')
            record.update(video=str(video) if video.exists() else None,trajectory_map=str(run/'preview/r10_v3_trajectory_map.png'))
            if manifest.exists():
                m=json.loads(manifest.read_text(encoding='utf-8'))
                record.update(video_frames=m['frame_count'],playback_seconds=m['duration_seconds'],sample_interval_s=m['median_persisted_sample_interval_seconds'])
            results.append(record)
    save(OUT/'analysis/targeted_media_and_provenance.json',dict(revision=rev,records=results,renderer_sha256=digest(ROOT/'scripts/build_r10_validation_replay_v3.py')))
    print('POSTPROCESSED',len(results),'completed runs',flush=True)


if __name__=='__main__':main()
