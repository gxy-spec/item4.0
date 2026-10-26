"""Foreground continuation of the current data/training stage, never a scheduler."""
import sys,json,time,subprocess
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *


def main():
    import psutil
    protocol=OUT/'nearfield_attribute_capture_v1/capture_protocol.json';last=-1
    while not protocol.exists():
        manifest=protocol.parent/'manifest.jsonl';n=len(rows(manifest)) if manifest.exists() else 0
        if n//384!=last:print('CAPTURE',n,'/3072',flush=True);last=n//384
        active=False
        for p in psutil.process_iter(['name','cmdline']):
            try:
                if p.info['name'].lower()=='python.exe' and any(Path(a).name=='capture_recovery_nearfield.py' for a in p.info['cmdline'] or []):active=True
            except (psutil.NoSuchProcess,psutil.AccessDenied):pass
        if not active:raise RuntimeError('Capture stopped without complete protocol; retain partial images, diagnose')
        time.sleep(10)
    commands=[['scripts/prepare_recovery_nearfield_attributes.py'],
              ['scripts/train_recovery_attributes.py','--architecture','resnet18','--name','resnet18_dual_v2','--manifest',str(OUT/'attribute_data_v5/manifest.jsonl'),'--nearfield-weight','3','--epochs','20'],
              ['scripts/calibrate_recovery_attributes.py','--model-name','resnet18_dual_v2','--deployment','deployment_v5_candidate.json']]
    for command in commands:
        print('RUN',command,flush=True);subprocess.run([sys.executable,'-u',*command],cwd=ROOT,check=True)
    print('CANDIDATE TRAINING COMPLETE; validation comparison and online acceptance still required',flush=True)


if __name__=='__main__':main()
