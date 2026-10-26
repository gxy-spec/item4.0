"""Executable target-actor input boundary audit, no simulator connection."""
from pathlib import Path
from types import SimpleNamespace
import json,sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest
from run_stage1_dynamic import apply_safe_route_control


class ForbiddenTruth:
    def __init__(self,payload):
        object.__setattr__(self,'_payload',payload);object.__setattr__(self,'_reads',0)
    def __getattribute__(self,name):
        if name.startswith('_'):return object.__getattribute__(self,name)
        object.__setattr__(self,'_reads',self._reads+1)
        raise AssertionError('Unexpected target/obstacle truth input: '+name)


class OwnVehicle:
    def get_location(self):return SimpleNamespace(x=0.,y=0.,z=0.)
    def get_transform(self):return SimpleNamespace(location=self.get_location(),rotation=SimpleNamespace(yaw=0.))
    def get_velocity(self):return SimpleNamespace(x=0.,y=0.,z=0.)
    def apply_control(self,c):self.command={k:getattr(c,k) for k in ['throttle','brake','steer','hand_brake']}


def main():
    cases=[dict(name='clear_drive',sensor=dict(state='CLEAR',nearest_obstacle_m=30.),hazard={}),
           dict(name='depth_obstacle',sensor=dict(state='STOP',nearest_obstacle_m=3.),hazard={}),
           dict(name='missing_depth',sensor=None,hazard={}),
           dict(name='pedestrian_stop',sensor=dict(state='CLEAR',nearest_obstacle_m=30.),hazard=dict(forced_stop_reason='pedestrian'))]
    results=[]
    for case in cases:
        controls=[];modes=[];reads=[]
        for truth in [dict(x=1.,color='red',extent=2.),dict(x=9999.,color='blue',extent=20.),dict(x=-9999.,color='black',extent=.1)]:
            target=ForbiddenTruth(truth);obstacle=ForbiddenTruth(truth);own=OwnVehicle()
            r=apply_safe_route_control(own,[[float(x),0.,0.] for x in range(0,102,2)],2.,target,[obstacle],
                dict(enabled=True,obstacle_source='ugv_rgbd',target_stop_enabled=False),
                sensor_observation=case['sensor'],hazard_decision=case['hazard'],target_xyz=[100.,0.,0.])
            controls.append(own.command);modes.append(r['ugv_safety_mode']);reads.append(target._reads+obstacle._reads)
        results.append(dict(case=case['name'],truth_variants=3,zero_actor_truth_attribute_reads=not any(reads),
            command_invariant=all(c==controls[0] for c in controls),modes=modes,command=controls[0]))
    result=dict(passed=all(r['zero_actor_truth_attribute_reads'] and r['command_invariant'] for r in results),records=results,
        source_sha256=digest(ROOT/'scripts/run_stage1_dynamic.py'),
        scope='apply_safe_route_control and follow_route tested with three forbidden target/obstacle truth proxies, fixed sensor message and own state. Not an instrumented whole-simulator truth-access proof.')
    save(OUT/'analysis/controller_truth_boundary_audit.json',result);print(json.dumps(result,ensure_ascii=False))


if __name__=='__main__':main()
