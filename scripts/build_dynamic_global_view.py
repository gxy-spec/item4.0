#!/usr/bin/env python3
"""Build an interactive and video global view from a completed CI-E1 run."""

from __future__ import annotations

import argparse
import base64
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fragment", type=Path, required=True)
    return parser.parse_args()


def load_run(run_dir: Path) -> tuple[list[dict], dict, list[list[float]]]:
    frames: dict[int, dict] = defaultdict(dict)
    with (run_dir / "actors" / "actor_states.jsonl").open(encoding="utf-8") as handle:
        for line in handle:
            state = json.loads(line)
            frames[int(state["frame"])][state["role"]] = [
                round(float(state["x"]), 3),
                round(float(state["y"]), 3),
            ]
    ordered = []
    first_timestamp = None
    with (run_dir / "actors" / "actor_states.jsonl").open(encoding="utf-8") as handle:
        timestamps = {}
        for line in handle:
            state = json.loads(line)
            timestamps.setdefault(int(state["frame"]), float(state["timestamp"]))
    for frame_id in sorted(frames):
        timestamp = timestamps[frame_id]
        if first_timestamp is None:
            first_timestamp = timestamp
        ordered.append({"f": frame_id, "t": round(timestamp - first_timestamp, 3), "a": frames[frame_id]})
    config = yaml.safe_load((run_dir / "resolved_config.yaml").read_text(encoding="utf-8"))
    route = config["region"]["uav"]["waypoints_xyz"]
    return ordered, config, route


def map_geometry(config: dict) -> dict[str, float]:
    points = config["region"]["boundary"]["points"]
    return {
        "x_min": min(point[0] for point in points) - 8.0,
        "x_max": max(point[0] for point in points) + 8.0,
        "y_min": min(point[1] for point in points) - 8.0,
        "y_max": max(point[1] for point in points) + 8.0,
        "left": 116.0,
        "right": 1461.0,
        "top": 76.0,
        "bottom": 1421.0,
    }


def world_to_pixel(x: float, y: float, geometry: dict[str, float]) -> tuple[int, int]:
    px = geometry["left"] + (x - geometry["x_min"]) / (geometry["x_max"] - geometry["x_min"]) * (geometry["right"] - geometry["left"])
    py = geometry["bottom"] - (y - geometry["y_min"]) / (geometry["y_max"] - geometry["y_min"]) * (geometry["bottom"] - geometry["top"])
    return int(round(px)), int(round(py))


def make_video(run_dir: Path, frames: list[dict], config: dict, route: list[list[float]]) -> Path:
    source = cv2.imread(str(run_dir / "preview" / "trajectory_map.png"))
    if source is None:
        raise FileNotFoundError("trajectory_map.png could not be read")
    geometry = map_geometry(config)
    output = run_dir / "preview" / "dynamic_global_view.mp4"
    width = 960
    height = int(round(source.shape[0] * width / source.shape[1]))
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (width, height))
    scale = width / source.shape[1]
    planned = [world_to_pixel(point[0], point[1], geometry) for point in route]
    sampled = frames[::2]
    for frame in sampled:
        canvas = source.copy()
        for start, end in zip(planned, planned[1:]):
            delta = np.asarray(end, dtype=float) - np.asarray(start, dtype=float)
            length = float(np.linalg.norm(delta))
            count = max(1, int(length // 20))
            for index in range(count):
                if index % 2:
                    continue
                a = tuple(np.rint(np.asarray(start) + delta * index / count).astype(int))
                b = tuple(np.rint(np.asarray(start) + delta * min(index + 1, count) / count).astype(int))
                cv2.line(canvas, a, b, (175, 80, 130), 3, cv2.LINE_AA)
        for role, xy in frame["a"].items():
            position = world_to_pixel(xy[0], xy[1], geometry)
            if role == "uav":
                cv2.drawMarker(canvas, position, (170, 40, 150), cv2.MARKER_TRIANGLE_UP, 22, 4)
            elif role == "ugv":
                cv2.drawMarker(canvas, position, (210, 90, 20), cv2.MARKER_SQUARE, 18, 4)
            elif role == "target":
                cv2.drawMarker(canvas, position, (30, 30, 210), cv2.MARKER_STAR, 22, 4)
            elif role.startswith("vehicle"):
                cv2.circle(canvas, position, 7, (20, 140, 225), -1, cv2.LINE_AA)
            elif role.startswith("pedestrian"):
                cv2.circle(canvas, position, 6, (100, 160, 35), -1, cv2.LINE_AA)
        cv2.rectangle(canvas, (125, 92), (520, 155), (255, 255, 255), -1)
        cv2.putText(canvas, f"CI-E1 global dynamics   t={frame['t']:.1f}s", (145, 132), cv2.FONT_HERSHEY_SIMPLEX, 0.82, (25, 38, 58), 2, cv2.LINE_AA)
        resized = cv2.resize(canvas, (width, height), interpolation=cv2.INTER_AREA)
        writer.write(resized)
    writer.release()
    return output


def make_fragment(fragment: Path, run_dir: Path, frames: list[dict], config: dict, route: list[list[float]]) -> None:
    image_bytes = (run_dir / "preview" / "trajectory_map.png").read_bytes()
    image_data = base64.b64encode(image_bytes).decode("ascii")
    geometry = map_geometry(config)
    payload = json.dumps(frames, ensure_ascii=False, separators=(",", ":"))
    route_payload = json.dumps([[round(point[0], 3), round(point[1], 3)] for point in route], separators=(",", ":"))
    geometry_payload = json.dumps(geometry, separators=(",", ":"))
    html = f"""<div id="ci-e1-global-dynamics">
  <h2>CI-E1 全局动态回放</h2>
  <div class="viz-controls">
    <button class="btn btn-primary" id="ci-play" type="button" aria-pressed="false">播放</button>
    <label class="form-label" for="ci-time">时间 <span id="ci-time-label" class="tabular-nums">0.0 s</span></label>
    <input class="form-range" id="ci-time" type="range" min="0" max="{len(frames) - 1}" value="0" step="1">
    <label class="form-label" for="ci-speed">速度</label>
    <select class="form-select" id="ci-speed"><option value="1">1×</option><option value="2">2×</option><option value="4">4×</option></select>
  </div>
  <div class="ci-map-wrap">
    <img src="data:image/png;base64,{image_data}" alt="Town10HD Zone A道路和完整轨迹底图">
    <svg id="ci-overlay" viewBox="0 0 1481 1512" role="img" aria-label="无人机、地面车、目标车辆、干扰车辆和行人的动态位置">
      <polyline id="ci-plan" class="ci-plan" points=""></polyline>
      <g id="ci-trails"></g><g id="ci-actors"></g>
    </svg>
  </div>
  <div class="viz-row ci-legend" aria-label="图例">
    <span><b class="uav"></b>UAV</span><span><b class="ugv"></b>UGV</span><span><b class="target"></b>目标车</span><span><b class="vehicle"></b>干扰车</span><span><b class="pedestrian"></b>行人</span><span><i></i>规划弓字航线</span>
  </div>
  <p id="ci-status" class="text-small text-muted" aria-live="polite"></p>
</div>
<style>
#ci-e1-global-dynamics{{width:100%;color:var(--foreground)}}
#ci-e1-global-dynamics h2{{margin:0 0 10px;font-weight:500}}
#ci-e1-global-dynamics .viz-controls{{display:grid;grid-template-columns:auto auto 1fr auto auto;align-items:center;gap:10px;margin-bottom:10px}}
#ci-e1-global-dynamics .form-label{{margin:0;white-space:nowrap}}
#ci-e1-global-dynamics .ci-map-wrap{{position:relative;width:100%;overflow:hidden}}
#ci-e1-global-dynamics .ci-map-wrap img{{display:block;width:100%;height:auto}}
#ci-e1-global-dynamics #ci-overlay{{position:absolute;inset:0;width:100%;height:100%}}
#ci-e1-global-dynamics .ci-plan{{fill:none;stroke:var(--purple);stroke-width:4;stroke-dasharray:14 10;opacity:.9}}
#ci-e1-global-dynamics .ci-trail-uav{{fill:none;stroke:var(--purple);stroke-width:7}}
#ci-e1-global-dynamics .ci-trail-ugv{{fill:none;stroke:var(--blue);stroke-width:7}}
#ci-e1-global-dynamics .ci-actor-label{{fill:var(--foreground);font-size:20px;font-weight:500;paint-order:stroke;stroke:var(--background);stroke-width:5px}}
#ci-e1-global-dynamics .ci-legend{{justify-content:center;gap:15px;margin-top:8px;flex-wrap:wrap}}
#ci-e1-global-dynamics .ci-legend span{{display:inline-flex;align-items:center;gap:6px}}
#ci-e1-global-dynamics .ci-legend b{{display:inline-block;width:11px;height:11px;border-radius:50%}}
#ci-e1-global-dynamics .ci-legend .uav{{background:var(--purple)}}
#ci-e1-global-dynamics .ci-legend .ugv{{background:var(--blue);border-radius:2px}}
#ci-e1-global-dynamics .ci-legend .target{{background:var(--red)}}
#ci-e1-global-dynamics .ci-legend .vehicle{{background:var(--orange)}}
#ci-e1-global-dynamics .ci-legend .pedestrian{{background:var(--green)}}
#ci-e1-global-dynamics .ci-legend i{{display:inline-block;width:24px;border-top:3px dashed var(--purple)}}
#ci-e1-global-dynamics #ci-status{{text-align:center;margin:6px 0 0}}
@media(max-width:600px){{#ci-e1-global-dynamics .viz-controls{{grid-template-columns:auto 1fr auto}}#ci-e1-global-dynamics .form-range{{grid-column:1/-1}}}}
</style>
<script>
(() => {{
const root=document.getElementById('ci-e1-global-dynamics');
const frames={payload}; const route={route_payload}; const geo={geometry_payload};
const slider=root.querySelector('#ci-time'), label=root.querySelector('#ci-time-label'), play=root.querySelector('#ci-play'), speed=root.querySelector('#ci-speed');
const actors=root.querySelector('#ci-actors'), trails=root.querySelector('#ci-trails'), plan=root.querySelector('#ci-plan'), status=root.querySelector('#ci-status');
const ns='http://www.w3.org/2000/svg'; let timer=null;
const xy=(p)=>[geo.left+(p[0]-geo.x_min)/(geo.x_max-geo.x_min)*(geo.right-geo.left),geo.bottom-(p[1]-geo.y_min)/(geo.y_max-geo.y_min)*(geo.bottom-geo.top)];
plan.setAttribute('points',route.map(p=>xy(p).join(',')).join(' '));
function el(name,attrs){{const n=document.createElementNS(ns,name);Object.entries(attrs).forEach(([k,v])=>n.setAttribute(k,v));return n}}
function draw(index){{
 const frame=frames[index]; actors.replaceChildren(); trails.replaceChildren();
 for(const role of ['uav','ugv']){{const pts=frames.slice(0,index+1).map(f=>f.a[role]).filter(Boolean).map(p=>xy(p).join(',')).join(' ');trails.appendChild(el('polyline',{{points:pts,class:'ci-trail-'+role}}))}}
 Object.entries(frame.a).forEach(([role,p])=>{{const [x,y]=xy(p);let fill='var(--orange)',shape='circle';if(role==='uav')fill='var(--purple)';else if(role==='ugv'){{fill='var(--blue)';shape='rect'}}else if(role==='target')fill='var(--red)';else if(role.startsWith('pedestrian'))fill='var(--green)';let mark;if(shape==='rect')mark=el('rect',{{x:x-9,y:y-9,width:18,height:18,rx:2,fill}});else mark=el('circle',{{cx:x,cy:y,r:role==='target'?11:7,fill,stroke:'var(--background)','stroke-width':3}});actors.appendChild(mark);if(['uav','ugv','target'].includes(role)){{const text=el('text',{{x:x+13,y:y-12,class:'ci-actor-label'}});text.textContent=role.toUpperCase();actors.appendChild(text)}}}});
 label.textContent=frame.t.toFixed(1)+' s'; slider.value=index; status.textContent=`frame ${{frame.f}} · UAV/UGV/目标/7辆干扰车/6名行人`;
}}
function stop(){{if(timer)clearInterval(timer);timer=null;play.textContent='播放';play.setAttribute('aria-pressed','false')}}
function start(){{stop();play.textContent='暂停';play.setAttribute('aria-pressed','true');timer=setInterval(()=>{{let i=Number(slider.value)+Number(speed.value);if(i>=frames.length){{i=frames.length-1;stop()}}draw(i)}},50)}}
play.addEventListener('click',()=>timer?stop():start());slider.addEventListener('input',()=>{{stop();draw(Number(slider.value))}});speed.addEventListener('change',()=>{{if(timer)start()}});draw(0);
}})();
</script>
"""
    fragment.parent.mkdir(parents=True, exist_ok=True)
    fragment.write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir.resolve()
    frames, config, route = load_run(run_dir)
    video = make_video(run_dir, frames, config, route)
    make_fragment(args.fragment.resolve(), run_dir, frames, config, route)
    total_length = sum(np.linalg.norm(np.asarray(a[:2]) - np.asarray(b[:2])) for a, b in zip(route, route[1:]))
    print(json.dumps({"frames": len(frames), "video": str(video), "fragment": str(args.fragment.resolve()), "planned_route_m": float(total_length), "planned_duration_s": float(total_length / config["dynamic"]["uav_speed_mps"])}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
