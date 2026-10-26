"""Check regression scene invariants before launching the frozen 72 episodes."""
from pathlib import Path
from copy import deepcopy
import sys,json,yaml,platform,importlib.metadata
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
from run_recovery_campaign import configure,SOURCE


def differences(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        out=[]
        for key in sorted(set(a)|set(b)):
            child=path+'.'+key if path else key
            if key not in a or key not in b:out.append(child)
            else:out.extend(differences(a[key],b[key],child))
        return out
    return [] if a==b else [path]


def main():
    source=json.loads(SOURCE.read_text(encoding='utf-8'));cal=json.loads((OUT/'calibration/deployment_release.json').read_text(encoding='utf-8'))
    formal=json.loads((OUT/'formal_index.json').read_text(encoding='utf-8')) if (OUT/'formal_index.json').exists() else None
    actual={(r['case'],r['group'],r['variant']):r for r in formal['runs']} if formal else {}
    permitted=['experiment_id','output','online_acceptance','recovery_evaluation','dynamic.local_target_search',
        'dynamic.persistent_target_communication','dynamic.ugv_safety.hazard_detector','dynamic.save_all_samples',
        'dynamic.sensor_save_every_n_frames','dynamic.cpu_threads','sensors.streams.uav_instance','sensors.streams.ugv_instance']
    records=[]
    for s in source['runs']:
        before=yaml.safe_load(Path(s['config']).read_text(encoding='utf-8'));versions={}
        for variant in ['legacy','upgraded']:
            spec=actual.get((s['case'],s['group'],variant));after=yaml.safe_load(Path(spec['config']).read_text(encoding='utf-8')) if spec else configure(s,variant,cal)
            changed=differences(before,after)
            illegal=[p for p in changed if not any(p==q or p.startswith(q+'.') for q in permitted)]
            if illegal:raise RuntimeError((s['case'],s['group'],variant,illegal))
            versions[variant]=after
            records.append(dict(case=s['case'],group=s['group'],variant=variant,source_config_sha256=digest(s['config']),
                actual_config_sha256=digest(spec['config']) if spec else None,changed_paths=changed,unexpected_changes=illegal))
        l,u=versions['legacy'],versions['upgraded']
        for key in ['region','simulation','sensors']:
            if l[key]!=u[key]:raise RuntimeError('Paired difference in '+key)
        # Common control/navigation/safety must remain equal after removing
        # the deliberate perception/matching differences.
        dl,du=deepcopy(l['dynamic']),deepcopy(u['dynamic'])
        dl.pop('local_target_search');du.pop('local_target_search')
        dl['persistent_target_communication'].pop('task_conditioned_matching');du['persistent_target_communication'].pop('task_conditioned_matching')
        if dl!=du:raise RuntimeError('Paired control difference '+str(differences(dl,du)))
    versions={}
    for name in ['torch','torchvision','ultralytics','opencv-python','numpy','carla','psutil','PyYAML','matplotlib']:
        try:versions[name]=importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:versions[name]='not distributed under this package name'
    result=dict(passed=True,checked_configs=len(records),source_index_sha256=digest(SOURCE),formal_configs_frozen=bool(formal),
        invariants='Original region/simulation/actor setup/patrol/sensors retained; only declared common safety/memory/cadence/archive and perception differences permitted. Legacy/upgraded navigation/control/safety equal.',
        permitted_change_prefixes=permitted,records=records,environment=dict(python=sys.version,platform=platform.platform(),packages=versions))
    save(OUT/('formal_design_integrity.json' if formal else 'formal_design_preflight.json'),result)
    print('DESIGN VERIFIED',len(records),'frozen',bool(formal),flush=True)


if __name__=='__main__':main()
