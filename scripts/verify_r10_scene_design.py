"""Compare current prepared YAML to original R9 YAML, including unrun configs."""
from pathlib import Path
import sys,json,copy,yaml
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save,digest


def clean(value):
    c=copy.deepcopy(value);c.pop('experiment_id',None);c.pop('recovery_evaluation',None)
    c.get('output',{}).pop('root',None);return c


def diff(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        return [d for k in sorted(set(a)|set(b)) for d in diff(a.get(k),b.get(k),path+'.'+str(k))]
    return [] if a==b else [dict(path=path,original=a,current=b)]


def main():
    records=[];errors=[]
    for cohort in ['safety','acceptance','diagnostics','formal']:
        p=OUT/f'{cohort}_index.json';index=json.loads(p.read_text(encoding='utf-8'))
        for item in index['runs']:
            new=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'));m=new['recovery_evaluation'];source=Path(m['original_config_path'])
            old=yaml.safe_load(source.read_text(encoding='utf-8'));differences=diff(clean(old),clean(new))
            sha_ok=digest(source)==m['original_config_sha256'];passed=not differences and sha_ok
            r=dict(cohort=cohort,case=item['case'],variant=item.get('variant'),group=item.get('group'),passed=passed,
                original_config_sha256_unchanged=sha_ok,unexpected_differences=differences,
                original_config=str(source),new_config=item['config'],acquisition_required_for_this_audit=False)
            records.append(r)
            if not passed:errors.append(r)
    result=dict(passed=not errors,configs=len(records),errors=errors,records=records,
        allowed_differences=['experiment_id','output.root','added recovery_evaluation metadata'],
        scope='Configuration-file comparison only. 72 formal configurations remain unrun; this is not a 72-episode experiment.')
    save(OUT/'analysis/scene_design_preservation.json',result);print(json.dumps({k:result[k] for k in ['passed','configs','errors']},ensure_ascii=False))


if __name__=='__main__':main()
