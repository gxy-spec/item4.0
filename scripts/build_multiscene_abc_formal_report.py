#!/usr/bin/env python3
"""Create a concise Word report from the 36-run paired-batch analysis."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from docx import Document
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor


BLUE = "1F4E79"
PALE_BLUE = "EAF1F7"
LIGHT_GRAY = "D9E0E7"
BLACK = "000000"
MUTED = "5B6573"
FONT = "Microsoft YaHei"


def set_run(run, size: float | None = None, bold: bool | None = None, color: str = BLACK, italic: bool | None = None) -> None:
    run.font.name = FONT
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), FONT)
    run._element.get_or_add_rPr().rFonts.set(qn("w:ascii"), FONT)
    run._element.get_or_add_rPr().rFonts.set(qn("w:hAnsi"), FONT)
    run.font.color.rgb = RGBColor.from_string(color)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.bold = bold
    if italic is not None:
        run.italic = italic


def set_cell_shading(cell, fill: str) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_margins(cell, top: int = 95, start: int = 105, bottom: int = 95, end: int = 105) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for side, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{side}"))
        if node is None:
            node = OxmlElement(f"w:{side}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def set_table_borders(table) -> None:
    tbl_pr = table._tbl.tblPr
    borders = tbl_pr.first_child_found_in("w:tblBorders")
    if borders is None:
        borders = OxmlElement("w:tblBorders")
        tbl_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        tag = f"w:{edge}"
        element = borders.find(qn(tag))
        if element is None:
            element = OxmlElement(tag)
            borders.append(element)
        element.set(qn("w:val"), "single")
        element.set(qn("w:sz"), "5")
        element.set(qn("w:space"), "0")
        element.set(qn("w:color"), LIGHT_GRAY)


def repeat_table_header(row) -> None:
    tr_pr = row._tr.get_or_add_trPr()
    repeat = OxmlElement("w:tblHeader")
    repeat.set(qn("w:val"), "true")
    tr_pr.append(repeat)


def add_table(doc: Document, headers: list[str], rows: list[list[str]], widths: list[float], font_size: float = 9.2):
    table = doc.add_table(rows=1, cols=len(headers))
    table.autofit = False
    table.alignment = 1
    set_table_borders(table)
    for index, (cell, text) in enumerate(zip(table.rows[0].cells, headers)):
        cell.width = Inches(widths[index])
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        set_cell_shading(cell, BLUE)
        set_cell_margins(cell)
        para = cell.paragraphs[0]
        para.alignment = WD_ALIGN_PARAGRAPH.CENTER
        para.paragraph_format.space_after = Pt(0)
        run = para.add_run(text)
        set_run(run, font_size, True, "FFFFFF")
    repeat_table_header(table.rows[0])
    for row_index, values in enumerate(rows):
        cells = table.add_row().cells
        for col_index, (cell, text) in enumerate(zip(cells, values)):
            cell.width = Inches(widths[col_index])
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            set_cell_margins(cell)
            if row_index % 2 == 1:
                set_cell_shading(cell, PALE_BLUE)
            para = cell.paragraphs[0]
            para.paragraph_format.space_after = Pt(0)
            para.paragraph_format.line_spacing = 1.08
            para.alignment = WD_ALIGN_PARAGRAPH.LEFT if col_index == 0 else WD_ALIGN_PARAGRAPH.CENTER
            run = para.add_run(text)
            set_run(run, font_size, False, BLACK)
    return table


def add_body(doc: Document, text: str, bold_lead: str | None = None) -> None:
    paragraph = doc.add_paragraph()
    paragraph.paragraph_format.space_after = Pt(6)
    paragraph.paragraph_format.line_spacing = 1.14
    if bold_lead and text.startswith(bold_lead):
        first = paragraph.add_run(bold_lead)
        set_run(first, 10.2, True)
        rest = paragraph.add_run(text[len(bold_lead):])
        set_run(rest, 10.2)
    else:
        run = paragraph.add_run(text)
        set_run(run, 10.2)


def add_heading(doc: Document, text: str, level: int = 1) -> None:
    paragraph = doc.add_paragraph(style=f"Heading {level}")
    paragraph.paragraph_format.keep_with_next = True
    paragraph.paragraph_format.space_before = Pt(7 if level == 1 else 4)
    paragraph.paragraph_format.space_after = Pt(4)
    run = paragraph.add_run(text)
    set_run(run, 14 if level == 1 else 11.5, True)


def add_caption(doc: Document, text: str) -> None:
    paragraph = doc.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_before = Pt(2)
    paragraph.paragraph_format.space_after = Pt(7)
    run = paragraph.add_run(text)
    set_run(run, 9, False, MUTED)


def fmt(value: float, digits: int = 1) -> str:
    return f"{value:.{digits}f}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    analysis_dir = args.analysis_dir.resolve()
    output_path = args.output.resolve()
    summary = json.loads((analysis_dir / "analysis_summary.json").read_text(encoding="utf-8"))
    acceptance = summary["acceptance"]
    group_rows = summary["group_summary"]
    effect_rows = {row["contrast"]: row for row in summary["paired_effects"]}
    groups = {row["group"]: row for row in group_rows}

    doc = Document()
    section = doc.sections[0]
    section.page_width = Inches(8.5)
    section.page_height = Inches(11)
    section.top_margin = Inches(0.62)
    section.bottom_margin = Inches(0.62)
    section.left_margin = Inches(0.68)
    section.right_margin = Inches(0.68)

    normal = doc.styles["Normal"]
    normal.font.name = FONT
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
    normal.font.size = Pt(10.2)
    normal.font.color.rgb = RGBColor.from_string(BLACK)
    for style_name in ("Title", "Heading 1", "Heading 2"):
        style = doc.styles[style_name]
        style.font.name = FONT
        style._element.rPr.rFonts.set(qn("w:eastAsia"), FONT)
        style.font.color.rgb = RGBColor.from_string(BLACK)

    title = doc.add_paragraph(style="Title")
    title.paragraph_format.space_after = Pt(2)
    title.paragraph_format.keep_with_next = True
    run = title.add_run("Town10HD 多场景 A/B/C 配对实验报告")
    set_run(run, 20, True)
    subtitle = doc.add_paragraph()
    subtitle.paragraph_format.space_after = Pt(8)
    run = subtitle.add_run("36 回合正式对照实验 | 2026 年 10 月 10 日")
    set_run(run, 10, False, MUTED)

    add_body(
        doc,
        "本报告汇总北区、东区、西区和 Town10HD 整图中的 12 个配对场景，每个场景分别运行 A、B、C 三种策略，共 36 回合。三组共享场景种子、UGV 起点、目标车辆、传感器和仿真时长。技术验收 36/36 通过；任务表现上，C 组平均更接近目标，但并非每个场景都优于 B 组，且多数回合尚未进入 10 m 范围。",
    )

    add_heading(doc, "实验设置", 1)
    add_table(
        doc,
        ["区域", "配对场景", "每回合时长", "每场景样本"],
        [
            ["北区", "N1–N3", "180 s", "3 个种子"],
            ["东区", "E1–E3", "190 s", "3 个种子"],
            ["西区", "W1–W3", "210 s", "3 个种子"],
            ["Town10HD 整图", "F1–F3", "450 s", "3 个种子"],
        ],
        [1.4, 1.45, 1.45, 1.45],
        9.0,
    )
    add_body(doc, "A 组按预设巡检路线运行；B 组使用 UGV 本地候选；C 组融合 UAV 与 UGV 候选并通过消息选择目标。三组均启用 RGB-D 安全避障。通信采用固定 50 ms 延迟和 1 s TTL，没有加入带宽限制、衰落或随机丢包。")

    add_heading(doc, "验收结果", 1)
    add_table(
        doc,
        ["检查项", "观测结果", "判断"],
        [
            ["逐回合验收", "36/36 PASS；失败检查 0 项", "通过"],
            ["同步采集", f"最低公共帧比例 {acceptance['minimum_common_frame_ratio'] * 100:.3f}%", "通过"],
            ["碰撞", f"UGV {acceptance['total_ugv_collisions']} 次；UAV {acceptance['total_uav_collisions']} 次", "通过"],
            ["动态干扰", f"每回合至少 {acceptance['minimum_moving_vehicles']} 辆移动汽车、{acceptance['minimum_moving_pedestrians']} 名移动行人", "通过"],
            ["传感器归档", "36/36 使用紧凑保存；轨迹、元数据和审计记录保留", "通过"],
        ],
        [1.45, 4.35, 0.95],
        8.8,
    )
    add_body(doc, "这些 PASS 表示场景运行、双端同步、数据记录、无碰撞和动态干扰等验收项满足要求；它们不等同于 36 回合都完成了目标接近任务。")

    doc.add_page_break()
    add_heading(doc, "目标接近效果", 1)
    image = analysis_dir / "target_approach_by_scene.png"
    paragraph = doc.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_after = Pt(0)
    paragraph.add_run().add_picture(str(image), width=Inches(7.05))
    add_caption(doc, "图 1. 每个场景内的配对结果。纵轴 = UGV 起始目标距离 − 最终目标距离；正值表示更接近。灰线连接同一场景的三个种子，黑色菱形线表示该场景均值。")
    add_body(doc, "总体均值显示 A、B、C 组分别接近目标 14.8 m、40.7 m 和 56.5 m。按同一场景配对，C 相比 B 平均多接近 15.8 m，中位数多 5.4 m；12 个配对场景中有 9 个 C 优于 B、2 个较差、1 个持平。中位数明显小于均值，说明少数场景贡献了较大的增益。")
    add_table(
        doc,
        ["配对差值", "平均接近增量", "C/B 或 B/A 更优场景数", "中位数"],
        [
            ["B − A", f"{effect_rows['B-A']['mean_target_reduction_delta_m']:+.1f} m", f"{effect_rows['B-A']['cases_with_more_target_approach']}/12", f"{effect_rows['B-A']['median_target_reduction_delta_m']:+.1f} m"],
            ["C − A", f"{effect_rows['C-A']['mean_target_reduction_delta_m']:+.1f} m", f"{effect_rows['C-A']['cases_with_more_target_approach']}/12", f"{effect_rows['C-A']['median_target_reduction_delta_m']:+.1f} m"],
            ["C − B", f"{effect_rows['C-B']['mean_target_reduction_delta_m']:+.1f} m", f"{effect_rows['C-B']['cases_with_more_target_approach']}/12", f"{effect_rows['C-B']['median_target_reduction_delta_m']:+.1f} m"],
        ],
        [1.05, 1.55, 2.3, 1.2],
        8.6,
    )
    add_body(doc, "最终距离 ≤10 m 的回合为 A 组 0/12、B 组 0/12、C 组 1/12。因此当前证据支持“协同信息平均改善了目标接近”，尚不足以声称目标到达任务已稳定完成。")

    doc.add_page_break()
    add_heading(doc, "路线变化和结果解释", 1)
    image = analysis_dir / "patrol_route_departure_by_scene.png"
    paragraph = doc.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.paragraph_format.space_after = Pt(0)
    paragraph.add_run().add_picture(str(image), width=Inches(7.05))
    add_caption(doc, "图 2. 每回合 UGV 轨迹采样点到记录的预设巡检路线的平均平面距离。偏离增大说明实际轨迹离开了巡检线，不代表路径更安全或更优。")
    add_body(doc, "A 组平均偏离预设路线 0.34 m，说明它基本沿巡检路线行驶。B 组平均偏离 19.1 m，C 组为 36.4 m，且 C 组 12 回合都生成了目标路线（其中 6 条由 UAV 来源消息触发、6 条由 UGV 来源消息触发）；B 组生成 8 条，均由 UGV 来源消息触发。轨迹偏离和目标接近增量合起来说明感知消息确实改变了 UGV 行为，但偏离越大本身不是性能越好。")

    add_heading(doc, "各组描述性指标", 1)
    add_table(
        doc,
        ["组别", "平均接近增量", "最终距离均值", "≤10 m", "路线平均偏离", "目标路线数"],
        [
            [f"{g} {GROUP_LABEL}", f"{groups[g]['mean_target_distance_reduction_m']:.1f} m", f"{groups[g]['mean_final_target_distance_m']:.1f} m", f"{groups[g]['final_within_10m_runs']}/12", f"{groups[g]['mean_patrol_departure_m']:.1f} m", f"{groups[g]['target_route_plans']}" ]
            for g, GROUP_LABEL in (("A", "固定巡检"), ("B", "UGV本地"), ("C", "双端协同"))
        ],
        [1.1, 1.3, 1.25, 0.75, 1.35, 1.0],
        8.0,
    )
    add_body(doc, "解读边界：这些是 12 个配对场景上的描述性结果，没有做显著性检验，也不能当作对所有道路场景的统计泛化。当前没有物理信道约束；下一阶段还需检验带宽、时延、丢包与消息新鲜度对上述增益的影响。")

    add_heading(doc, "验证文件", 1)
    add_body(doc, f"批次总进度：{Path(summary['batch_root']) / 'batch_progress.json'}")
    add_body(doc, f"36 组报告路径清单：{Path(summary['batch_root']) / 'all_validation_paths.tsv'}")
    add_body(doc, f"逐回合指标表：{analysis_dir / 'paired_metrics_36.csv'}")
    doc.core_properties.title = "Town10HD 多场景 A/B/C 配对实验报告"
    doc.core_properties.subject = "36 回合多场景配对实验验收和描述性结果"
    doc.core_properties.author = "CityInspection_GOC Experiment"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    doc.save(output_path)
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
