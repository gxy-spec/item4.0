"""Prepare a fresh run config without changing the validated source snapshot."""
from pathlib import Path
import argparse,json,hashlib,copy,sys
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from scenario.configuration import validate_schema

def prepare(source,output_root,assets_root=None,package_root=None):
    c=yaml.safe_load(Path(source).read_text(encoding='utf-8'))
    if c.get('native_lidar',{}).get('enabled',False):raise ValueError('This release is the RGB-D mainline')
    c.pop('native_lidar',None)
    assets=json.loads((ROOT/'configs/releases/mainline_20261026/assets.json').read_text(encoding='utf-8'))['assets']
    used=[]
    def walk(obj):
        if isinstance(obj,dict):
            for k,v in list(obj.items()):
                if k in {'model_path','attribute_model_path','auxiliary_detector_path'} and isinstance(v,str):
                    found=[a for a in assets if v.replace('\\','/') in [s.replace('\\','/') for s in a['config_aliases']]]
                    if len(found)!=1:raise ValueError('Unregistered model path: '+v)
                    a=found[0];p=(Path(assets_root)/a['filename']) if assets_root else Path(a['local_source'])
                    if not p.is_file():raise FileNotFoundError(f"Provide model {a['filename']} via --assets-root")
                    if hashlib.sha256(p.read_bytes()).hexdigest()!=a['sha256']:raise ValueError('Model hash differs: '+str(p))
                    obj[k]=str(p.resolve());used.append(a['filename'])
                else:walk(v)
        elif isinstance(obj,list):
            for v in obj:walk(v)
    walk(c)
    if package_root:c['simulation']['package_root']=str(Path(package_root).resolve())
    c['output']['root']=str(Path(output_root).resolve())
    c['mainline_release']=dict(version='20261026-rgbd-r10-strict',source_config=Path(source).name,
        source_config_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),verified_models=sorted(set(used)),
        scope='Configuration materialization only; no new closed-loop results or LiDAR contribution claimed')
    errors=validate_schema(c,ROOT/'configs/schemas/dynamic_experiment_schema.json')
    if errors:raise ValueError(errors)
    return c

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output-root',type=Path,required=True);p.add_argument('--destination',type=Path,required=True);p.add_argument('--assets-root',type=Path);p.add_argument('--package-root',type=Path);a=p.parse_args()
    if a.destination.resolve()==a.source.resolve():raise ValueError('Preserve the source configuration')
    if a.destination.exists():raise FileExistsError('Destination already exists: '+str(a.destination))
    c=prepare(a.source,a.output_root,a.assets_root,a.package_root);a.destination.parent.mkdir(parents=True,exist_ok=True)
    a.destination.write_text(yaml.safe_dump(c,allow_unicode=True,sort_keys=False),encoding='utf-8');print(a.destination)

if __name__=='__main__':main()
