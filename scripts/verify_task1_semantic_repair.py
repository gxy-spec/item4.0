"""Verify the actual copied dataset and original provenance after repair."""
from pathlib import Path
import json
import os
from perception_online_acceptance import OUT,read_jsonl,sha256,save_json
from repair_task1_semantic_labels import SOURCE,TARGET

summary=json.loads((TARGET/'repair_manifest.json').read_text(encoding='utf-8'))
records=read_jsonl(TARGET/'manifest.jsonl');same_rgb=0;test_labels_same=0;instances=0;empty=0
for r in records:
    source_image=Path(r['source_image']);new_image=Path(r['image'])
    assert os.path.samefile(source_image,new_image) or sha256(source_image)==sha256(new_image)
    same_rgb+=1
    label=Path(r['label']);lines=label.read_text(encoding='utf-8').splitlines()
    assert len(lines)==len(r['annotations'])
    instances+=len(lines);empty+=not lines
    if r['split']=='test':
        assert sha256(label)==r['source_label_sha256'];test_labels_same+=1
assert sha256(SOURCE/'manifest.jsonl')==summary['source_manifest_sha256']
assert same_rgb==9163 and test_labels_same==1781 and instances==18085 and empty==1081
save_json(OUT/'analysis/followup_questions/semantic_repair_verification.json',{
    'passed':True,'identical_rgb_files':same_rgb,'unchanged_test_label_files':test_labels_same,
    'remaining_annotations':instances,'empty_known_label_frames':empty,'source_manifest_unchanged':True,
    'scope':'file/label integrity only; empty labels do not certify true negative RGB; no model performance or RGB visibility pass'})
print('REPAIR FILE QA PASS: 9163 identical RGB; 1781 unchanged test labels; 18085 remaining annotations; original manifest unchanged',flush=True)
