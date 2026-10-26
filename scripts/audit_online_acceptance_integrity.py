"""Reproducibility, actual payload/schema, freshness and observation-boundary audit."""
from __future__ import annotations
import argparse
import ast
from collections import Counter
import json
from pathlib import Path
import shutil
import sys
import yaml
from jsonschema import Draft202012Validator
from perception_online_acceptance import OUT,ROOT,read_jsonl,save_json,sha256


def progress():
    p=OUT/'online_index.json';d=json.loads(p.read_text(encoding='utf-8')) if p.exists() else {}
    print(json.dumps({'completed_reports':len(d),'cases':{k:v.get('runner_status') for k,v in d.items()}},ensure_ascii=False),flush=True)
    return d


def audit():
    index=progress();frozen=json.loads((OUT/'configs/source_versions.json').read_text(encoding='utf-8'))
    sources={p:{'frozen_sha256':h,'current_sha256':sha256(ROOT/p),'matches':sha256(ROOT/p)==h} for p,h in frozen.items()}
    runtime_ok=all(v['matches'] for p,v in sources.items() if not p.endswith('run_online_acceptance_batch.py'))
    schema=Draft202012Validator(json.loads((ROOT/'configs/schemas/target_candidate_v1.json').read_text(encoding='utf-8')))
    runs=[]
    for name,item in index.items():
        if item.get('runner_status') not in ['PASS','FAIL']:continue
        run=Path(item['run_dir']);cfg=yaml.safe_load(Path(item['config']).read_text(encoding='utf-8'))
        records=read_jsonl(run/'perception/candidate_records.jsonl');failures=[]
        for r in records:
            errors=list(schema.iter_errors(r))
            if errors:failures.append({'frame':r['frame_id'],'candidate':r['candidate_id'],'errors':[e.message for e in errors]})
        bykey={}
        for device in ['uav','ugv']:
            for row in read_jsonl(run/f'perception/{device}_target_candidates_synchronized.jsonl'):
                for c in row['candidates']:bykey[(device,row['frame_id'],c['candidate_id'])]=c
        messages=read_jsonl(run/'communication/messages.jsonl');origins=[];bad_semantics=[]
        for m in messages:
            c=bykey.get((m['source'],m['source_frame'],m['candidate_id']))
            if c is None or c['world_position_xyz']!=m['world_position_xyz'] or c['timestamp']!=m['source_timestamp_s']:
                origins.append(m['message_id'])
            if m.get('category')!=cfg['online_acceptance']['task_category'] or not m.get('target_color_match') or not m.get('temporal_confirmed'):
                bad_semantics.append(m['message_id'])
        selected=read_jsonl(run/'communication/selected_target_states.jsonl')
        stale=[m['message_id'] for m in selected if not m['source_timestamp_s']<=m['timestamp_s']<m['expires_at_s']]
        safety=cfg['dynamic']['ugv_safety']
        checks={'configuration_hash_matches':sha256(item['config'])==item['config_sha256'],
            'rgbd_safety_enabled':safety['enabled'] and safety['obstacle_source']=='ugv_rgbd' and not safety['target_stop_enabled'],
            'non_oracle_mode':cfg['dynamic']['mode']=='ugv_patrol_with_local_rgbd_search',
            'no_attribute_only_proposals':not cfg['dynamic']['local_target_search']['enable_attribute_proposals'],
            'all_actual_candidate_records_schema_valid':not failures,'all_message_positions_from_exact_source_candidate':not origins,
            'all_sent_messages_task_conditioned_and_temporally_confirmed':not bad_semantics,
            'selected_new_observations_never_expired':not stale}
        runs.append({'case':name,'checks':checks,'candidate_records':len(records),'messages':len(messages),
            'selected_records':len(selected),'schema_errors':failures[:20],'origin_mismatches':origins,'semantic_gate_mismatches':bad_semantics,'stale_new_selections':stale})
    tree=ast.parse((ROOT/'scripts/run_stage1_dynamic.py').read_text(encoding='utf-8'))
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='apply_safe_route_control')
    forbidden=[n.attr for n in ast.walk(function) if isinstance(n,ast.Attribute) and isinstance(n.value,ast.Name) and n.value.id in ['target_actor','obstacles']]
    payload={'runtime_core_sources_unchanged':runtime_ok,'sources':sources,'runs':runs,
        'apply_safe_route_control_dereferences_target_or_obstacle_actor_arguments':forbidden,
        'scope':'configuration, exact candidate/message provenance, executable function structure and actual record audit; counter zeros alone are not treated as proof',
        'pass':runtime_ok and not forbidden and all(all(r['checks'].values()) for r in runs)}
    save_json(OUT/'analysis/runtime_integrity_audit.json',payload)
    for p in ['scripts/run_stage1_dynamic.py','src/perception/candidate_pipeline.py','src/perception/attributes.py',
        'src/perception/task_spec.py','src/communication/persistent_target.py','src/communication/a2g_transport.py',
        'src/navigation/ugv_oracle_planner.py','src/safety/rgbd_obstacle_guard.py']:
        target=OUT/'code_snapshot'/p;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/p,target)
    save_json(OUT/'analysis/evaluation_protocol_notes.json',{
        'original_protocol_sha256':sha256(OUT/'protocol.json'),
        'clarification':'Exclude execution vehicles from business-target ground truth; retain them for all-vehicle detection diagnostics',
        'map_static_vehicles':'supplementary offline annotation audit, incomplete coverage explicitly retained',
        'unchanged':'model weights, online thresholds, precision/FPR goals, scenarios, real payloads and all raw sensor data',
        'original_results_retained':['calibration/*.json','benchmark/test_results.json'],
        'corrected_fixed_threshold_results':'analysis/benchmark_ego_excluded_at_frozen_thresholds.json',
        'orchestration_repair':'no persistent native CARLA client across server restarts; no change to the three runtime core source files'})
    print('INTEGRITY',payload['pass'],'runs',len(runs),'schema records',sum(r['candidate_records'] for r in runs),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--progress',action='store_true');a=p.parse_args()
    if a.progress:progress()
    else:audit()
