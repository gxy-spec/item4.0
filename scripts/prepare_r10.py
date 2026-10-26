"""New output indexes from exact R9 resolved configs. No old outputs rewritten."""
from pathlib import Path
import sys,json,hashlib,copy
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import ROOT,OUT,DELIVERY,save,digest
sys.path.insert(0,str(ROOT))
from src.scenario.configuration import validate_schema,validate_online_control_config

OLD=Path('E:/CarlaAirData/CityInspection_GOC/perception_recovery_20261021_v2')


def generate():
    DELIVERY.mkdir(parents=True,exist_ok=True)
    formal=json.loads((OLD/'formal_index.json').read_text(encoding='utf-8'))['runs']
    acc=json.loads((OLD/'acceptance_r9_index.json').read_text(encoding='utf-8'))['runs']
    safety=json.loads((OLD/'safety_r9_index.json').read_text(encoding='utf-8'))['runs']
    dev=[x for x in formal if x['variant']=='upgraded' and x['group']=='C' and x['case'] in ['E2','E3','W3','F1','F3','W2']]
    dev += [x for x in formal if x['case']=='N3' and x['group']=='A']
    groups={'diagnostics':dev,'acceptance':acc,'safety':safety,'formal':formal}
    for cohort,items in groups.items():
        dest=OUT/f'{cohort}_index.json'
        if dest.exists():continue
        runs=[]
        for i,item in enumerate(items):
            c=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
            key=f"{item.get('variant','upgraded')}_{item['case']}_{item.get('group','C')}"
            c['experiment_id']=f'CI_E7_R10_{cohort[:3]}_{i+1:02}'
            c['output']['root']=f'D:/CI10/{cohort}/{key}'
            c.setdefault('recovery_evaluation',{}).update(repair_revision='R10',
                physical_success='correct identity + persistent UGV confirmation + true body clearance3..8m + stopped2s + zero collision',
                original_config_path=item['config'],original_config_sha256=digest(item['config']))
            errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')+validate_online_control_config(c)
            if errors:raise RuntimeError((key,errors))
            p=OUT/f'configs/{cohort}/{key}.yaml';p.parent.mkdir(parents=True,exist_ok=True)
            p.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8')
            runs.append({**item,'config':str(p),'config_sha256':digest(p),'output_root':c['output']['root']})
        save(dest,dict(cohort=cohort,runs=runs,scope='R9 geometry/route/seed/horizon/model preserved; common R10 controller/evaluation'))
    manifest=json.loads((OLD/'frozen_runtime_formal_r9/manifest.json').read_text(encoding='utf-8'))
    errors=[]
    for name,h in manifest['files'].items():
        if digest(Path('E:/Research/CityInspection_GOC')/name)!=h:errors.append(name)
    for name,h in manifest.get('artifacts',{}).items():
        if digest(name)!=h:errors.append(name)
    save(OUT/'r9_preservation_check.json',dict(errors=errors,checked=len(manifest['files'])+len(manifest.get('artifacts',{}))))
    if errors:raise RuntimeError(('R9 preservation audit',errors))
    print('PREPARED',[(k,len(v)) for k,v in groups.items()],flush=True)


if __name__=='__main__':generate()
