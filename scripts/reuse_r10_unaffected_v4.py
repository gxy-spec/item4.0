"""Explicit conditional-branch impact proof; preserve acquisition and prior reuse."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,ROOT,save,digest


def main():
    previous=OUT/'frozen_validation_v4';old=json.loads((previous/'manifest.json').read_text(encoding='utf-8'))
    changed=[p for p,h in old['files'].items() if digest(ROOT/p)!=h]
    if changed!=['scripts/run_stage1_dynamic.py']:raise RuntimeError('Unexpected frozen v4 differences: '+str(changed))
    proof=json.loads((OUT/'validation5_exact_main_edits.json').read_text(encoding='utf-8'))
    restored=(ROOT/'scripts/run_stage1_dynamic.py').read_text(encoding='utf-8')
    for edit in reversed(proof['edits']):
        if restored.count(edit['after'])!=1:raise RuntimeError('Exact patch reversal failed')
        restored=restored.replace(edit['after'],edit['before'],1)
    if restored!=(previous/'scripts/run_stage1_dynamic.py').read_text(encoding='utf-8'):raise RuntimeError('Extra main changes')
    impact=json.loads((OUT/'analysis/proposed_v5_budget_policy_impact.json').read_text(encoding='utf-8'))
    if len(impact['records'])!=22 or impact['source_revision']!='validation4':raise RuntimeError('Screen all 22 v4 cases before reuse')
    bykey={r['key']:r for r in impact['records']};records=[]
    for cohort in ['safety','acceptance','diagnostics']:
        progress=json.loads((OUT/f'{cohort}_development_validation4_progress.json').read_text(encoding='utf-8'));reused={}
        for key,item in progress.items():
            screen=bykey[key];accepted=screen['changes_control']
            if any(not d['unchanged_route'] or d['chosen_end_stop_tolerance_m']!=3. for d in screen['decisions']) and not accepted:
                raise RuntimeError('Unaccepted case changed route or tolerance')
            record=dict(key=key,run_dir=item['run_dir'],reused=not accepted,decisions=len(screen['decisions']),
                original_route_object_unchanged=not accepted,end_stop_tolerance_unchanged=not accepted,
                acquisition_revision=item.get('development_revision'))
            records.append(record)
            if accepted:continue
            previous_reuse=item.get('evidence_reuse')
            copy={**item,'evidence_reuse':dict(acquisition_revision=item.get('development_revision'),validated_revision='validation5',
                criterion='Every recorded initial/recovery plan returns the identical original route and default3m terminal; A/negative/safety without plans never call chooser',
                previous_reuse=previous_reuse,receipt=str(OUT/'validation5_unaffected_reuse_receipt.json'))}
            reused[key]=copy
        dest=OUT/f'{cohort}_development_validation5_progress.json'
        if dest.exists():raise RuntimeError('Do not overwrite validation5 progress')
        save(dest,reused)
    receipt=dict(records=records,exact_main_patch_reversed_to_v4=True,changed_existing_files=changed,
        impact_screen_sha256=digest(OUT/'analysis/proposed_v5_budget_policy_impact.json'),
        scope='Conditional route-policy output equivalence on recorded v4 inputs; not newly acquired v5 runs, not a whole-simulator nondeterminism guarantee.')
    save(OUT/'validation5_unaffected_reuse_receipt.json',receipt)
    admission=json.loads((OUT/'validation4_safety_admission.json').read_text(encoding='utf-8'))
    admission.update(revision='validation5',runtime_manifest_sha256=digest(OUT/'frozen_validation_v5/manifest.json'),
        additional_evidence_reuse=str(OUT/'validation5_unaffected_reuse_receipt.json'))
    save(OUT/'validation5_safety_admission.json',admission)
    print('REUSED',sum(r['reused'] for r in records),'EXCLUDED',[r['key'] for r in records if not r['reused']])


if __name__=='__main__':main()
