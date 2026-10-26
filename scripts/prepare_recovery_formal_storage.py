"""Freeze the original 36 x two models; storage placement does not alter scenes."""
import json,sys,yaml,shutil
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from recovery_stage import *
from run_recovery_campaign import configure,SOURCE
from src.scenario.configuration import validate_schema,validate_online_control_config


def main():
    if (OUT/'formal_index.json').exists():raise RuntimeError('Already frozen')
    cal=json.loads((OUT/'calibration/deployment_release.json').read_text(encoding='utf-8'))
    # Extend the unchanged R9 release boundary to the original perception
    # checkpoint and all local src modules before any paired formal run.
    # Keep the acceptance manifest immutable as its own provenance record.
    base_manifest=OUT/'frozen_runtime_r9/manifest.json'
    formal_manifest=OUT/'frozen_runtime_formal_r9/manifest.json'
    if not formal_manifest.exists():
        frozen=json.loads(base_manifest.read_text(encoding='utf-8'))
        for p in (ROOT/'src').rglob('*.py'):frozen['files'][p.relative_to(ROOT).as_posix()]=digest(p)
        frozen['files']['scripts/run_e7_comparison_batch.py']=digest(ROOT/'scripts/run_e7_comparison_batch.py')
        frozen['files']['scripts/run_stage0_preview.py']=digest(ROOT/'scripts/run_stage0_preview.py')
        frozen['artifacts'][str(ROOT/'models/yolo26n.pt')]=digest(ROOT/'models/yolo26n.pt')
        frozen['base_acceptance_manifest_sha256']=digest(base_manifest)
        frozen['formal_addendum']='No model/controller change. Expand pre-formal hash coverage to legacy checkpoint, launcher helper, preview helper and all local src Python modules.'
        for name in frozen['files']:
            target=formal_manifest.parent/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/name,target)
        save(formal_manifest,frozen)
    source=json.loads(SOURCE.read_text(encoding='utf-8'));runs=[]
    for item in source['runs']:
        for variant in ['legacy','upgraded']:
            c=configure(item,variant,cal)
            if variant=='legacy':
                volume='C' if item['scene'] in ['north','whole'] else 'D'
                if (item['case']=='N1' and item['group']=='A') or (item['scene']=='east' and item['group']=='A'):volume='E'
                if volume=='E':
                    c['output']['root']=str(OUT/f'formal/legacy/{item["case"]}/{item["group"]}')
                else:
                    c['output']['root']=f'{volume}:/CityInspectionData/R8/formal/legacy/{item["case"]}/{item["group"]}'
            p=OUT/f'configs/formal/{variant}_{item["case"]}_{item["group"]}.yaml'
            errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
            if errors:raise RuntimeError((p,errors))
            p.parent.mkdir(parents=True,exist_ok=True);p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
            runs.append({**item,'variant':variant,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
    save(OUT/'formal_index.json',dict(design='Original 12 scene deployments x A/B/C x legacy/upgraded =72; original task remains red van',runs=runs,
        source_index_sha256=digest(SOURCE),calibration_sha256=digest(OUT/'calibration/deployment_release.json'),
        storage='Upgraded on E; legacy north/whole on C and east/west on D. Completed PNGs may be recompressed losslessly with pixel identity verification and per-file manifests.',
        paired_common_changes='Same current persistent memory, RGB-D safety using the frozen hazard_release.json (YOLO26m), confirmed-goal approach monitor, inference cadence, image size, archive cadence. Old/new difference is the complete perception package, not only the type classifier.',
        runtime_manifest_sha256=digest(formal_manifest),runtime_manifest_path=str(formal_manifest)))
    print('FROZEN',len(runs),'formal episodes',flush=True)


if __name__=='__main__':main()
