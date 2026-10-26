"""Versioned perception recovery campaign; never overwrite the failed acceptance."""
from pathlib import Path
import hashlib,json,shutil
from contextlib import contextmanager
import os

ROOT=Path(__file__).resolve().parents[1]
OUT=Path('E:/CarlaAirData/CityInspection_GOC/perception_repair_20261023_r10')
DELIVERY=Path('C:/Users/Administrator/Documents/ChatGPT/课题1/实验结果/问题修复与36组复验_R10_20261023')
CLASSES=['van','car','truck','bus']
COLORS=['red','blue','white','black','gray','yellow']
DETECTOR=Path('E:/CarlaAirData/CityInspection_GOC/training/yoloe_perception_town10hd_v2/yoloe26s_multiclass_pe_v1/weights/best.pt')

def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@contextmanager
def simulator_lease():
    """One simulator owner; reject orphan acquisition children after a parent exit."""
    import psutil,msvcrt
    for p in psutil.process_iter(['pid','name','cmdline']):
        try:
            if p.pid==os.getpid() or p.info['name'].lower()!='python.exe':continue
            scripts=[Path(a).name for a in (p.info['cmdline'] or []) if a.endswith('.py')]
            if any(a=='run_stage1_dynamic.py' or a.startswith('capture_') for a in scripts):
                raise RuntimeError('Another simulator acquisition is active: PID '+str(p.pid)+' '+str(scripts))
        except (psutil.NoSuchProcess,psutil.AccessDenied):continue
    OUT.mkdir(parents=True,exist_ok=True)
    handle=(OUT/'simulator.lock').open('a+b');handle.seek(0)
    if handle.read(1)==b'':handle.write(b'0');handle.flush()
    handle.seek(0)
    try:msvcrt.locking(handle.fileno(),msvcrt.LK_NBLCK,1)
    except OSError:
        handle.close();raise RuntimeError('Simulator lease held by another experiment')
    try:yield
    finally:
        handle.seek(0);msvcrt.locking(handle.fileno(),msvcrt.LK_UNLCK,1);handle.close()

def initialize():
    OUT.mkdir(exist_ok=True);DELIVERY.mkdir(parents=True,exist_ok=True)
    if (OUT/'contract.json').exists():return
    originals=['src/perception/candidate_pipeline.py','scripts/run_stage1_dynamic.py',
        'src/communication/persistent_target.py','src/navigation/ugv_oracle_planner.py',
        'configs/schemas/target_candidate_v1.json','configs/schemas/semantic_target_message_schema.json']
    hashes={}
    for name in originals:
        target=OUT/'before_recovery'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,target);hashes[name]=digest(target)
    save(OUT/'contract.json',{
        'scope':'final perception acceptance + complete 4-family x 3 deployments x A/B/C closed-loop regression',
        'baseline':'historical 36 geometry/route/seed/duration retained; current valid persistent-memory and RGB-D safety common to comparisons',
        'detector_preference':'retain current YOLOE segmentation checkpoint unless controlled evidence warrants replacement',
        'attribute_goal':'learned van/car/truck/bus and red/blue/white/black/gray/yellow, RGB only, no online actor metadata',
        'final_goal':'correct target identity, local confirmation, safe arrival reported separately; unknown is not completion',
        'formal_order':'attribute train/calibration/held-out test -> sensor/visibility/positive/negative pilot -> frozen formal cohort',
        'comparison':'same-condition legacy perception control in addition to upgraded 36; historical archives are references only',
        'diversity':'independent multi-category/color acceptance; original 36 task semantics retained for regression comparability',
        'no_hidden_ground_truth':'ground truth used only in scene setup, training labels, offline admission and evaluation',
        'model_training_overlap':'original 36 is regression coverage, not unseen-map generalization; report held-out groups separately',
        'detector_path':str(DETECTOR),'detector_sha256':digest(DETECTOR),'original_source_hashes':hashes})

if __name__=='__main__':initialize();print(OUT)
