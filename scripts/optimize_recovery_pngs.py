"""Lossless storage compaction, only after each acquisition has finished.

Never removes an image. An atomic replacement occurs only after decoded
pixels match exactly. Record encoded and pixel SHA256 for every replacement.
"""
import argparse,hashlib,json,sys,time,os
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import *
cv2.setNumThreads(1)


def compact_one(p):
    before=p.read_bytes();original=cv2.imdecode(np.frombuffer(before,np.uint8),cv2.IMREAD_UNCHANGED)
    level=3 if p.parent.name=='rgb' else 6
    ok,data=cv2.imencode('.png',original,[cv2.IMWRITE_PNG_COMPRESSION,level])
    if not ok:raise RuntimeError(str(p))
    after=data.tobytes()
    if len(after)>=len(before):return None
    restored=cv2.imdecode(data,cv2.IMREAD_UNCHANGED)
    if not np.array_equal(original,restored):raise RuntimeError('Pixel identity failed '+str(p))
    tmp=p.with_suffix('.compact.tmp');tmp.write_bytes(after);os.replace(tmp,p)
    return dict(path=str(p),original_bytes=len(before),compacted_bytes=len(after),
        original_encoded_sha256=hashlib.sha256(before).hexdigest(),compacted_encoded_sha256=hashlib.sha256(after).hexdigest(),
        decoded_pixels_sha256=hashlib.sha256(original.tobytes()).hexdigest(),pixel_identity_verified=True)


def compact_run(run):
    report=run/'lossless_png_compaction.json'
    if report.exists():return
    files=list((run/'sensors').rglob('*.png'));records=[]
    # Completed stationary sequences contain many byte-identical PNGs. Encode
    # one canonical file, then preserve every original filename as a hardlink.
    # This avoids repeatedly compressing the same pixels thousands of times.
    canonical={};duplicates=[];unique=[]
    for p in files:
        signature=hashlib.sha256(p.read_bytes()).hexdigest()
        if signature in canonical:
            duplicates.append((p,canonical[signature],signature,p.stat().st_size))
        else:
            canonical[signature]=p;unique.append(p)
    with ThreadPoolExecutor(max_workers=2) as pool:
        for r in pool.map(compact_one,unique):
            if r:records.append(r)
    replacements={r['path']:r for r in records};links=[];link_errors=[]
    for p,source,original_signature,original_size in duplicates:
        signature=hashlib.sha256(source.read_bytes()).hexdigest();tmp=p.with_suffix('.dedup.tmp')
        try:
            os.link(source,tmp);os.replace(tmp,p)
            links.append(dict(path=str(p),same_volume_hardlink_to=str(source),encoded_sha256=signature,physical_bytes_avoided=p.stat().st_size))
            if str(source) in replacements:
                r=replacements[str(source)]
                records.append({**r,'path':str(p),'original_bytes':original_size,
                    'original_encoded_sha256':original_signature,
                    'identity_proof':'Original bytes identical to verified canonical PNG; replacement is the same NTFS file',
                    'canonical_path':str(source)})
        except OSError as exc:
            link_errors.append(dict(path=str(p),error=str(exc)))
            if tmp.exists():tmp.unlink()
    detail=run/'lossless_png_compaction_files.jsonl'
    detail.write_text(''.join(json.dumps(r)+'\n' for r in records),encoding='utf-8')
    save(report,dict(images_scanned=len(files),images_replaced=len(records),
        saved_bytes=sum(r['original_bytes']-r['compacted_bytes'] for r in records),
        unique_images_encoded=len(unique),method='Original byte-identical PNGs grouped first; canonical RGB level3, depth/instance level6 encodes verified by exact decoded-array equality. Duplicate filenames atomically linked to their verified canonical file. Filenames and pixels preserved.',
        hardlinked_frames=len(links),hardlinks=links,hardlink_errors=link_errors,physical_bytes_avoided_by_hardlinks=sum(r['physical_bytes_avoided'] for r in links),
        detailed_manifest=str(detail),manifest_sha256=digest(detail)))
    print('COMPACTED',run.name,len(records),'savedGB',sum(r['original_bytes']-r['compacted_bytes'] for r in records)/1e9,flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('--cohort',default='formal');p.add_argument('--expected',type=int,default=72);p.add_argument('--watch',action='store_true');a=p.parse_args()
    progress_path=OUT/f'{a.cohort}_progress.json'
    while True:
        progress=json.loads(progress_path.read_text(encoding='utf-8')) if progress_path.exists() else {}
        done=[r for r in progress.values() if r.get('report_complete') and r.get('run_dir')]
        for r in done:compact_run(Path(r['run_dir']))
        if not a.watch or len(done)>=a.expected:break
        time.sleep(15)


if __name__=='__main__':main()
