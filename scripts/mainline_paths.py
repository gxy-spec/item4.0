"""Fresh output area for the published RGB-D baseline; history stays read-only."""
from pathlib import Path
import os,json
ROOT=Path(__file__).resolve().parents[1]
OUT=Path(os.environ.get('CITYINSPECTION_MAINLINE_OUTPUT',ROOT/'outputs/mainline_20261026')).resolve()

def save(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')

def rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding='utf-8').splitlines() if line.strip()]
