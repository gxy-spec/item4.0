"""Check final report links, full hold coverage and explicit retained failures."""
from pathlib import Path
import sys,json,re,csv
sys.path.insert(0,str(Path(__file__).resolve().parent))
from recovery_stage import OUT,save
from build_r10_targeted_report import DEST


def main():
    errors=[];links=[]
    for path in DEST.glob('*.md'):
        for target in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            target=target.strip('<>')
            if not re.match(r'^[A-Za-z]:[/\\]',target):continue
            exists=Path(target).exists();links.append(dict(document=str(path),target=target,exists=exists))
            if not exists:errors.append(links[-1])
    result=json.loads((DEST/'定向验收完整结果.json').read_text(encoding='utf-8'));holds=[]
    for item in result:
        for a in item['evaluation'].get('completion_audits',[]):
            if not a.get('passed'):continue
            times=[m['timestamp_s'] for m in a['measurements']];event=a['event']
            full=bool(len(times)>=41 and max(times)-min(times)>=2.-1e-6
                and abs(min(times)-event['hold_start_s'])<=1e-6 and abs(max(times)-event['timestamp_s'])<=1e-6)
            holds.append(dict(case=item['item']['case'],samples=len(times),span_s=max(times)-min(times),full_two_second_coverage=full))
            if not full:errors.append(holds[-1])
    failures=[a['item']['case'] for a in result if a['suite']=='diagnostics' and a['item']['group']=='C'
        and not a['evaluation'].get('S3_found_and_safely_approached')]
    if sorted(failures)!=['E2','F1']:errors.append(dict(error='Unexpected diagnostic failure summary',actual=failures))
    if len(result)!=22:errors.append(dict(error='Missing final cases',actual=len(result)))
    maintext=(DEST/'修复定向验收报告.md').read_text(encoding='utf-8')
    if '尚未全部通过' not in maintext or '26.29%' not in maintext or '30.62' not in maintext:
        errors.append(dict(error='Material failed/weak outcomes absent from main report'))
    data=dict(passed=not errors,errors=errors,checked_links=len(links),links=links,successful_holds=holds,
        diagnostic_failures=failures,final_cases=len(result),
        scope='Artifact consistency and full2-second coverage of passed hold traces; not an additional physical experiment.')
    save(OUT/'analysis/final_delivery_check.json',data)
    print(json.dumps({k:data[k] for k in ['passed','errors','checked_links','successful_holds','diagnostic_failures','final_cases']},ensure_ascii=False))
    if errors:raise RuntimeError('Resolve delivery consistency before delivery')


if __name__=='__main__':main()
