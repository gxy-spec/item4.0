"""Copy final weights/configs/source for reproducibility without copying raw data."""
from pathlib import Path
import json,sys,shutil,subprocess
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def main():
    active=json.loads((OUT/'release_active.json').read_text(encoding='utf-8'))
    if (OUT/'formal_index.json').exists():active['runtime_manifest_path']=json.loads((OUT/'formal_index.json').read_text(encoding='utf-8'))['runtime_manifest_path']
    frozen=json.loads(Path(active['runtime_manifest_path']).read_text(encoding='utf-8'))
    if any(digest(ROOT/p)!=h for p,h in frozen['files'].items()):raise RuntimeError('Frozen runtime mismatch')
    dest=DELIVERY/'可复现模型与代码';dest.mkdir(parents=True,exist_ok=True);copied={}
    def copy(source,relative):
        source=Path(source);target=dest/relative;target.parent.mkdir(parents=True,exist_ok=True)
        if not target.exists() or digest(target)!=digest(source):shutil.copy2(source,target)
        h=digest(source)
        if digest(target)!=h:raise RuntimeError('Copy checksum failed '+str(target))
        copied[str(relative)]=dict(original_path=str(source),sha256=h,bytes=target.stat().st_size)
    for folder in ['src','scripts','configs']:
        for p in (ROOT/folder).rglob('*'):
            if p.is_file() and p.suffix in ['.py','.yaml','.yml','.json']:
                copy(p,p.relative_to(ROOT))
    weights={DETECTOR:'models/yoloe26s.pt',ROOT/'models/yolo26x.pt':'models/yolo26x.pt',ROOT/'models/yolo26m.pt':'models/yolo26m.pt',
        ROOT/'models/yolo26n.pt':'models/yolo26n.pt',Path(frozen['model_path']):'models/vehicle_attributes.pt'}
    for p,relative in weights.items():copy(p,relative)
    for p in [Path(active['runtime_manifest_path']),OUT/'calibration/deployment_release.json',OUT/'hazard_release.json',OUT/'evaluation_protocol.json']:
        copy(p,Path('provenance')/p.name)
    for name in ['formal_index.json',active['acceptance_cohort']+'_index.json','formal_design_integrity.json','formal_design_preflight.json','engineering_tests_r9.log']:
        p=OUT/name
        if p.exists():copy(p,Path('provenance')/name)
    for cohort in [active['acceptance_cohort'],'formal']:
        index=OUT/f'{cohort}_index.json'
        if index.exists():
            for r in json.loads(index.read_text(encoding='utf-8'))['runs']:copy(r['config'],Path('frozen_experiment_configs')/cohort/Path(r['config']).name)
    env=subprocess.run([sys.executable,'-m','pip','freeze','--all'],capture_output=True,check=True)
    (dest/'environment_freeze.txt').write_bytes(env.stdout)
    save(dest/'bundle_manifest.json',dict(revision=active['revision'],frozen_manifest_original=active['runtime_manifest_path'],files=copied,
        note='Code, weights and exact frozen configs copied. Original raw observations remain at each report run_dir. Configs intentionally preserve original absolute paths; adjust deployment paths only in a new config copy on another machine.'))
    (dest/'README_复现.md').write_text('''# 复现入口

此目录包含完整Python源码、项目配置、最终车型颜色权重、YOLOE主检测器、通用候选检测器、安全检测器和旧模型对照权重；不复制数百GB原始图像。

`bundle_manifest.json`逐文件记录来源与SHA256，`provenance/manifest.json`记录正式冻结边界，`frozen_experiment_configs/`保存逐回合配置。完整原始回合位置见上一级逐回合结果.csv。

原机复验命令和每次实际执行参数保存在原始回合记录中。环境为Windows、CARLA-Air0.1.7、Python3.10及CUDA版PyTorch，实际包版本以environment_freeze.txt为准。

迁移到另一台机器时，在配置副本中修改simulation.package_root、output.root，以及local_target_search的model_path/attribute_model_path/auxiliary_detector_path和安全检测器model_path。models中yoloe26s.pt为保留的主检测分割模型，vehicle_attributes.pt为最终ResNet18双分支；yolo26x.pt用于通用候选补充，yolo26m.pt用于所有对照共用安全感知，yolo26n.pt用于旧方案。

启动一个回合：`python -u scripts/run_stage1_dynamic.py --config <配置副本.yaml> --timeout 120`。需先按项目原流程启动Town10HD仿真服务；同一时间只能有一个采集进程。配置副本的结果不能冒充本报告的原始冻结回合。

在线分类只使用RGB；深度用于定位、框补全与安全。主任务语义可修改为支持的车型/颜色组合，但新组合应独立验收。当前权重不承诺任意车型名称、品牌型号、未知城市或实车泛化。
''',encoding='utf-8')
    print('BUNDLE VERIFIED',len(copied),'files',sum(r['bytes'] for r in copied.values())/1e6,'MB',flush=True)


if __name__=='__main__':main()
