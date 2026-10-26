"""Remove proven non-vehicle pseudo-labels in a NEW version, preserving all images/splits.

This repairs semantic/ID contamination only. It does not certify RGB visibility,
fill static-vehicle labels, or promote a model to acceptance.
"""
from collections import Counter
from datetime import datetime,timezone
import json
import os
from pathlib import Path
import shutil
import yaml
from perception_online_acceptance import OUT,read_jsonl,save_json,sha256

SOURCE=Path('E:/CarlaAirData/CityInspection_GOC/datasets/perception_instance_town10hd_v2')
TARGET=SOURCE.with_name('perception_instance_town10hd_v2_semantic_repair_20261021')
CLASSES=['van','car','truck','bus']


def main():
    audit=OUT/'analysis/followup_questions/training_dataset_semantic_audit.json'
    if not audit.exists():raise RuntimeError('Dataset audit must precede repair')
    import csv
    with (audit.parent/'training_dataset_nonvehicle_labels.csv').open(encoding='utf-8-sig',newline='') as f:bad=list(csv.DictReader(f))
    if any(int(r['vehicle_pixels'])!=0 for r in bad):raise RuntimeError('Mixed masks require polygon recomputation; do not simply remove label lines')
    bad_pairs={(r['image'],int(r['actor_id'])) for r in bad}
    if TARGET.exists():raise RuntimeError(f'New dataset version already exists; will not overwrite: {TARGET}')
    TARGET.mkdir();counts=Counter();records=[];source_manifest=SOURCE/'manifest.jsonl'
    for row in read_jsonl(source_manifest):
        image=Path(row['image']);label=Path(row['label']);lines=label.read_text(encoding='utf-8').splitlines()
        annotations=row['annotations']
        if len(lines)!=len(annotations):raise RuntimeError(f'Label/manifest correspondence mismatch: {label}')
        for line,actor in zip(lines,annotations):
            if int(line.split()[0])!=CLASSES.index(actor['category']):raise RuntimeError('Category order mismatch')
        selected=[i for i,a in enumerate(annotations) if (str(image),a['actor_id']) not in bad_pairs]
        new_image=TARGET/image.relative_to(SOURCE);new_label=TARGET/label.relative_to(SOURCE)
        new_image.parent.mkdir(parents=True,exist_ok=True);new_label.parent.mkdir(parents=True,exist_ok=True)
        try:os.link(image,new_image);counts['image_hardlinks']+=1
        except OSError:shutil.copy2(image,new_image);counts['image_copies']+=1
        new_label.write_text(''.join(lines[i]+'\n' for i in selected),encoding='utf-8')
        records.append({**row,'image':str(new_image),'label':str(new_label),'annotations':[annotations[i] for i in selected],
            'source_image':str(image),'source_label_sha256':sha256(label),'repair':'removed actor-ID masks with zero vehicle semantic pixels'})
        counts[f'{row["split"]}_images']+=1;counts[f'{row["split"]}_removed_annotations']+=len(annotations)-len(selected)
        counts[f'{row["split"]}_remaining_annotations']+=len(selected);counts[f'{row["split"]}_empty_label_images']+=not selected
        counts['retained_rgb_files']+=1
    if len(records)!=9163 or sum(counts[f'{s}_removed_annotations'] for s in ['train','validation','test'])!=1081:
        raise RuntimeError('Repair counts differ from audited evidence')
    (TARGET/'manifest.jsonl').write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in records),encoding='utf-8')
    config={'path':str(TARGET),'train':'images/train','val':'images/validation','test':'images/test',
            'names':dict(enumerate(CLASSES))}
    (TARGET/'dataset.yaml').write_text(yaml.safe_dump(config,sort_keys=False),encoding='utf-8')
    summary={'created_utc':datetime.now(timezone.utc).isoformat(),'source':str(SOURCE),'source_manifest_sha256':sha256(source_manifest),
        'target':str(TARGET),'counts':dict(counts),'source_dataset_unchanged':True,
        'split_and_test_labels_unchanged':counts['test_removed_annotations']==0,
        'rgb_visibility_certified':False,'static_vehicles_exhaustively_labelled':False,'ready_for_formal_training_or_acceptance':False,
        'remaining':'RGB occlusion and annotation coverage still require separate review; no retraining or online pass claimed'}
    save_json(TARGET/'repair_manifest.json',summary);save_json(audit.parent/'semantic_label_repair_result.json',summary)
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
