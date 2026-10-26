"""Process-scoped sleep inhibition for this authorized overnight experiment.

No power-plan/registry/display setting is changed. Windows clears the request
when this process exits; explicit user sleep/shutdown remains under user control.
"""
import ctypes,json,sys,time
from pathlib import Path
import psutil
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,DELIVERY,save


def main():
    fn=ctypes.windll.kernel32.SetThreadExecutionState
    fn.argtypes=[ctypes.c_uint];fn.restype=ctypes.c_uint
    if not fn(0x80000001):raise RuntimeError('Cannot request process-scoped system wakefulness')
    started=time.monotonic();idle_since=None;last=None
    save(OUT/'process_sleep_request.json',dict(active=True,pid=psutil.Process().pid,
        policy='ES_CONTINUOUS|ES_SYSTEM_REQUIRED only; no display/power-plan/registry changes; automatically released at exit. Max24h; after full acquisition allow up to4h for final visual report.'))
    try:
        while time.monotonic()-started<24*3600:
            final=DELIVERY/'交付清单.json'
            try:
                if final.exists() and json.loads(final.read_text(encoding='utf-8')).get('final'):break
            except json.JSONDecodeError:pass
            active=False
            for p in psutil.process_iter(['name','cmdline']):
                try:
                    if p.info['name'].lower()=='python.exe' and any(Path(s).name in ['execute_recovery_r9.py','supervise_recovery_release.py','run_stage1_dynamic.py'] for s in p.info['cmdline'] or []):active=True
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
            if active:idle_since=None
            elif idle_since is None:idle_since=time.monotonic()
            else:
                progress=OUT/'formal_progress.json'
                try:data=json.loads(progress.read_text(encoding='utf-8')) if progress.exists() else {}
                except json.JSONDecodeError:data={}
                acquired=len(data)==72 and all(r.get('report_complete') for r in data.values())
                if time.monotonic()-idle_since>(4*3600 if acquired else 120):break
            if active!=last:print('EXPERIMENT ACTIVE',active,flush=True);last=active
            time.sleep(20)
    finally:
        fn(0x80000000);save(OUT/'process_sleep_request.json',dict(active=False,released=True,pid=psutil.Process().pid))
        print('PROCESS SLEEP REQUEST RELEASED',flush=True)


if __name__=='__main__':main()
