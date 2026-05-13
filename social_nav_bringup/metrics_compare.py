#!/usr/bin/env python3
"""HuNav evaluator CSV 결과를 self-contained HTML 비교 리포트로 변환합니다."""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from html import escape
from pathlib import Path
from typing import Iterable


BEHAVIOR_NAMES = {
    1: "Regular",
    2: "Impassive",
    3: "Surprised",
    4: "Scared",
    5: "Curious",
    6: "Threatening",
}

METRIC_DESCRIPTIONS = {
    "completed": "미션 목표에 실제로 도달했는지 여부입니다. True면 목표 반경 안에 도착한 것입니다.",
    "robot_on_person_collision": "로봇이 사람과 충돌한 횟수입니다. 낮을수록 좋고, 이상적으로는 0이어야 합니다.",
    "person_on_robot_collision": "사람이 로봇과 충돌한 횟수입니다. 낮을수록 좋고, 이상적으로는 0이어야 합니다.",
    "minimum_distance_to_people": "실행 중 로봇이 사람에게 가장 가까워졌던 최소 거리입니다. 클수록 프록세믹스 측면에서 안전합니다.",
    "avg_distance_to_closest_person": "매 시점마다 가장 가까운 사람까지의 거리를 평균낸 값입니다. 전반적인 여유 공간을 보여줍니다.",
    "intimate_space_intrusions": "친밀 공간(약 0.45m 미만)을 침범한 횟수입니다. 낮을수록 좋습니다.",
    "personal_space_intrusions": "개인 공간(약 0.45m~1.2m)을 침범한 횟수입니다. 낮을수록 좋습니다.",
    "time_to_reach_goal": "기록 시작부터 종료까지 목표에 도달하는 데 걸린 시간입니다. 짧을수록 효율적입니다.",
    "path_length": "로봇이 실제로 이동한 경로 길이입니다. 짧을수록 효율적인 경로로 볼 수 있습니다.",
    "final_distance_to_target": "실험 종료 시점에 목표까지 남아 있는 거리입니다. 0에 가까울수록 좋습니다.",
    "social_force_on_agents": "로봇 존재 때문에 보행자에게 가해진 사회적 힘의 총량입니다. 낮을수록 사람에게 덜 부담을 준 것입니다.",
    "avg_robot_linear_speed": "주행 중 로봇의 평균 선속도입니다. 무조건 낮거나 높다고 좋은 것은 아니며, 안전성과 효율성의 균형을 봐야 합니다.",
}

CHART_DESCRIPTIONS = {
    "avg_distance_to_closest_person": "시간에 따라 가장 가까운 사람과의 거리가 어떻게 변하는지 보여줍니다.",
    "avg_robot_linear_speed": "시간에 따라 로봇 속도가 어떻게 변하는지 보여줍니다.",
    "social_force_on_agents": "시간에 따라 보행자에게 미친 사회적 영향이 커지는 구간을 확인할 수 있습니다.",
    "personal_space_intrusions": "개인 공간 침범이 언제 발생했는지 시간축으로 확인할 수 있습니다.",
}

SUMMARY_METRICS = [
    ("completed", "Mission completed", "higher"),
    ("robot_on_person_collision", "Robot→person collisions", "lower"),
    ("person_on_robot_collision", "Person→robot collisions", "lower"),
    ("minimum_distance_to_people", "Min distance to people (m)", "higher"),
    ("intimate_space_intrusions", "Intimate-space intrusions", "lower"),
    ("personal_space_intrusions", "Personal-space intrusions", "lower"),
    ("time_to_reach_goal", "Time to reach goal (s)", "lower"),
    ("path_length", "Path length (m)", "lower"),
    ("final_distance_to_target", "Final distance to target (m)", "lower"),
    ("social_force_on_agents", "Social force on agents", "lower"),
    ("avg_robot_linear_speed", "Avg robot linear speed (m/s)", "neutral"),
]

BEHAVIOR_METRICS = [
    ("minimum_distance_to_people", "Min distance to people (m)", "higher"),
    ("personal_space_intrusions", "Personal-space intrusions", "lower"),
    ("time_to_reach_goal", "Time to reach goal (s)", "lower"),
    ("social_force_on_agents", "Social force on agents", "lower"),
]

STEP_METRICS = [
    ("avg_distance_to_closest_person", "Closest-person distance over time"),
    ("avg_robot_linear_speed", "Robot linear speed over time"),
    ("social_force_on_agents", "Social force on agents over time"),
    ("personal_space_intrusions", "Personal-space intrusions over time"),
]


@dataclass
class SelectedRun:
    row: dict[str, str]
    source: Path

    @property
    def tag(self) -> str:
        return self.row["experiment_tag"]

    @property
    def run_id(self) -> str:
        return str(self.row["run_id"])

    @property
    def label(self) -> str:
        return f"{self.tag} (run {self.run_id})"


class MetricsCompareError(RuntimeError):
    pass


def sanitize_filename(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9._-]+", "_", value).strip("_") or "report"


def normalize_run_id(value: str | int | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    try:
        return str(int(float(text)))
    except ValueError:
        return text


def parse_number(value: str | None) -> float | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return None
    lowered = text.lower()
    if lowered == "true":
        return 1.0
    if lowered == "false":
        return 0.0
    try:
        return float(text)
    except ValueError:
        return None


def format_value(value: str | None, metric: str | None = None) -> str:
    if value is None or value == "":
        return "—"
    if metric == "completed":
        return "True" if str(value).strip().lower() == "true" or parse_number(value) == 1.0 else "False"
    number = parse_number(value)
    if number is None:
        return str(value)
    if abs(number) >= 100:
        return f"{number:.2f}"
    if abs(number) >= 10:
        return f"{number:.3f}"
    return f"{number:.4f}"


def percent_change(a: str | None, b: str | None) -> str:
    va = parse_number(a)
    vb = parse_number(b)
    if va is None or vb is None:
        return "—"
    if abs(va) < 1e-12:
        if abs(vb) < 1e-12:
            return "0.0%"
        return "—"
    return f"{((vb - va) / abs(va)) * 100:+.1f}%"


def compare_status(a: str | None, b: str | None, direction: str) -> tuple[str, str]:
    if direction == "neutral":
        return ("참고", "neutral")

    va = parse_number(a)
    vb = parse_number(b)
    if va is None or vb is None:
        return ("비교 불가", "neutral")

    eps = 1e-9
    if abs(vb - va) <= eps:
        return ("동일", "neutral")

    improved = vb > va if direction == "higher" else vb < va
    return (("개선", "good") if improved else ("악화", "bad"))


def load_rows(csv_path: Path) -> list[dict[str, str]]:
    if not csv_path.exists():
        raise MetricsCompareError(f"CSV 파일을 찾을 수 없습니다: {csv_path}")
    with csv_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return list(reader)


def sort_rows(rows: Iterable[dict[str, str]]) -> list[dict[str, str]]:
    decorated = []
    for idx, row in enumerate(rows):
        run = normalize_run_id(row.get("run_id"))
        try:
            run_num = int(run) if run is not None else -1
        except ValueError:
            run_num = -1
        decorated.append((run_num, idx, row))
    decorated.sort(key=lambda item: (item[0], item[1]))
    return [row for _, _, row in decorated]


def select_run(rows: list[dict[str, str]], source: Path, tag: str | None, run_id: str | None) -> SelectedRun:
    candidates = rows
    if tag is not None:
        candidates = [row for row in candidates if row.get("experiment_tag") == tag]
        if not candidates:
            raise MetricsCompareError(f"experiment_tag='{tag}' 행을 찾지 못했습니다: {source}")

    if run_id is not None:
        run_id = normalize_run_id(run_id)
        candidates = [row for row in candidates if normalize_run_id(row.get("run_id")) == run_id]
        if not candidates:
            tag_label = f"experiment_tag='{tag}', " if tag is not None else ""
            raise MetricsCompareError(f"{tag_label}run_id='{run_id}' 행을 찾지 못했습니다: {source}")

    ordered = sort_rows(candidates)
    return SelectedRun(row=ordered[-1], source=source)


def auto_select_runs(rows: list[dict[str, str]], source: Path) -> tuple[SelectedRun, SelectedRun]:
    if len(rows) < 2:
        raise MetricsCompareError("비교하려면 metrics.csv에 최소 2개 실행 결과가 필요합니다.")

    unique_tags: list[str] = []
    for row in rows:
        tag = row.get("experiment_tag", "")
        if tag and tag not in unique_tags:
            unique_tags.append(tag)

    if len(unique_tags) >= 2:
        tag_a, tag_b = unique_tags[-2], unique_tags[-1]
        return select_run(rows, source, tag_a, None), select_run(rows, source, tag_b, None)

    ordered = sort_rows(rows)
    return SelectedRun(ordered[-2], source), SelectedRun(ordered[-1], source)


def locate_steps_file(base_dir: Path, prefix: str, tag: str, run_id: str) -> Path | None:
    path = base_dir / f"{prefix}_{tag}_{run_id}.csv"
    return path if path.exists() else None


def load_steps(csv_path: Path | None) -> list[dict[str, str]]:
    if csv_path is None or not csv_path.exists():
        return []
    with csv_path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def tooltip_html(text: str | None) -> str:
    if not text:
        return ""
    escaped = escape(text)
    return (
        '<span class="tooltip" tabindex="0" aria-label="metric description">'
        '<span class="tooltip-icon">?</span>'
        f'<span class="tooltip-text">{escaped}</span>'
        '</span>'
    )


def metric_label_html(metric: str, label: str) -> str:
    return (
        '<span class="metric-label">'
        f'{escape(label)}'
        f'{tooltip_html(METRIC_DESCRIPTIONS.get(metric))}'
        '</span>'
    )


def chart_title_html(metric: str, title: str) -> str:
    return (
        '<span class="metric-label">'
        f'{escape(title)}'
        f'{tooltip_html(CHART_DESCRIPTIONS.get(metric))}'
        '</span>'
    )


def svg_line_chart(
    rows_a: list[dict[str, str]],
    rows_b: list[dict[str, str]],
    metric: str,
    title: str,
    label_a: str,
    label_b: str,
    width: int = 620,
    height: int = 220,
) -> str:
    if not rows_a and not rows_b:
        return ""

    padding = 36
    plot_w = width - padding * 2
    plot_h = height - padding * 2

    def extract_points(rows: list[dict[str, str]]) -> list[tuple[float, float]]:
        pts = []
        for row in rows:
            x = parse_number(row.get("time_stamps"))
            y = parse_number(row.get(metric))
            if x is None or y is None:
                continue
            pts.append((x, y))
        return pts

    pts_a = extract_points(rows_a)
    pts_b = extract_points(rows_b)
    if not pts_a and not pts_b:
        return ""

    xs = [x for x, _ in pts_a + pts_b]
    ys = [y for _, y in pts_a + pts_b]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if math.isclose(min_x, max_x):
        max_x = min_x + 1.0
    if math.isclose(min_y, max_y):
        pad = 1.0 if math.isclose(min_y, 0.0) else abs(min_y) * 0.2
        min_y -= pad
        max_y += pad

    def scale_x(x: float) -> float:
        return padding + ((x - min_x) / (max_x - min_x)) * plot_w

    def scale_y(y: float) -> float:
        return height - padding - ((y - min_y) / (max_y - min_y)) * plot_h

    def polyline(points: list[tuple[float, float]], color: str) -> str:
        if len(points) < 2:
            return ""
        scaled = " ".join(f"{scale_x(x):.1f},{scale_y(y):.1f}" for x, y in points)
        return f'<polyline fill="none" stroke="{color}" stroke-width="2.5" points="{scaled}" />'

    grid_lines = []
    for i in range(5):
        gy = padding + (plot_h / 4) * i
        value = max_y - ((max_y - min_y) / 4) * i
        grid_lines.append(
            f'<line x1="{padding}" y1="{gy:.1f}" x2="{width - padding}" y2="{gy:.1f}" stroke="#e5e7eb" stroke-width="1" />'
            f'<text x="8" y="{gy + 4:.1f}" font-size="11" fill="#6b7280">{value:.2f}</text>'
        )

    return f"""
    <div class=\"chart-card\">
      <div class=\"chart-title\">{chart_title_html(metric, title)}</div>
      <svg viewBox=\"0 0 {width} {height}\" role=\"img\" aria-label=\"{escape(title)}\">
        <rect x=\"{padding}\" y=\"{padding}\" width=\"{plot_w}\" height=\"{plot_h}\" fill=\"#ffffff\" stroke=\"#d1d5db\" />
        {''.join(grid_lines)}
        {polyline(pts_a, '#2563eb')}
        {polyline(pts_b, '#dc2626')}
        <text x=\"{padding}\" y=\"{height - 8}\" font-size=\"11\" fill=\"#6b7280\">time (s)</text>
        <text x=\"{width - padding - 30}\" y=\"{height - 8}\" font-size=\"11\" fill=\"#6b7280\">{max_x:.1f}</text>
      </svg>
      <div class=\"legend\">
        <span><i class=\"swatch blue\"></i>{escape(label_a)}</span>
        <span><i class=\"swatch red\"></i>{escape(label_b)}</span>
      </div>
    </div>
    """


def summary_rows_html(run_a: SelectedRun, run_b: SelectedRun, metric_specs: list[tuple[str, str, str]]) -> str:
    rows_html = []
    for metric, label, direction in metric_specs:
        a = run_a.row.get(metric)
        b = run_b.row.get(metric)
        status_text, status_cls = compare_status(a, b, direction)
        rows_html.append(
            f"""
            <tr>
              <td>{metric_label_html(metric, label)}</td>
              <td>{escape(format_value(a, metric))}</td>
              <td>{escape(format_value(b, metric))}</td>
              <td>{escape(percent_change(a, b))}</td>
              <td><span class=\"badge {status_cls}\">{escape(status_text)}</span></td>
            </tr>
            """
        )
    return "".join(rows_html)


def behavior_sections_html(base_dir: Path, run_a: SelectedRun, run_b: SelectedRun) -> str:
    sections = []
    for behavior_id, behavior_name in BEHAVIOR_NAMES.items():
        path = base_dir / f"metrics_beh_{behavior_id}.csv"
        if not path.exists():
            continue
        rows = load_rows(path)
        try:
            record_a = select_run(rows, path, run_a.tag, run_a.run_id)
            record_b = select_run(rows, path, run_b.tag, run_b.run_id)
        except MetricsCompareError:
            continue

        table_html = summary_rows_html(record_a, record_b, BEHAVIOR_METRICS)
        sections.append(
            f"""
            <section class=\"section\">
              <h2>Behavior {behavior_id} — {escape(behavior_name)}</h2>
              <p class=\"muted\">동일 behavior 타입의 보행자만 따로 필터링해 계산한 결과입니다.</p>
              <table>
                <thead>
                  <tr>
                    <th>Metric</th>
                    <th>{escape(run_a.label)}</th>
                    <th>{escape(run_b.label)}</th>
                    <th>Change</th>
                    <th>판정</th>
                  </tr>
                </thead>
                <tbody>{table_html}</tbody>
              </table>
            </section>
            """
        )
    return "".join(sections)


def generate_report(run_a: SelectedRun, run_b: SelectedRun, metrics_file: Path, output_path: Path) -> Path:
    base_dir = metrics_file.parent

    overall_steps_a = load_steps(locate_steps_file(base_dir, "metrics_steps", run_a.tag, run_a.run_id))
    overall_steps_b = load_steps(locate_steps_file(base_dir, "metrics_steps", run_b.tag, run_b.run_id))

    charts_html = "".join(
        svg_line_chart(overall_steps_a, overall_steps_b, metric, title, run_a.label, run_b.label)
        for metric, title in STEP_METRICS
    )

    html_text = f"""<!doctype html>
<html lang=\"ko\">
<head>
  <meta charset=\"utf-8\" />
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\" />
  <title>HuNav Metrics Report</title>
  <style>
    :root {{
      color-scheme: light dark;
      --bg: #0b1020;
      --card: #111827;
      --card-light: #ffffff;
      --text: #e5e7eb;
      --muted: #94a3b8;
      --line: #cbd5e1;
      --good: #16a34a;
      --bad: #dc2626;
      --neutral: #64748b;
    }}
    * {{ box-sizing: border-box; }}
    body {{ font-family: Inter, system-ui, -apple-system, BlinkMacSystemFont, sans-serif; margin: 0; background: #f8fafc; color: #0f172a; }}
    .wrap {{ max-width: 1180px; margin: 0 auto; padding: 32px 20px 56px; }}
    h1, h2 {{ margin: 0 0 12px; }}
    .hero {{ background: linear-gradient(135deg, #0f172a, #1d4ed8); color: white; border-radius: 20px; padding: 24px; margin-bottom: 24px; }}
    .hero p {{ margin: 10px 0 0; color: #dbeafe; line-height: 1.6; }}
    .cards {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 12px; margin: 18px 0 0; }}
    .card {{ min-width: 0; background: rgba(255,255,255,0.12); border: 1px solid rgba(255,255,255,0.18); border-radius: 14px; padding: 14px 16px; }}
    .card .k {{ font-size: 12px; color: #bfdbfe; margin-bottom: 6px; }}
    .card .v {{ font-weight: 700; font-size: 15px; line-height: 1.45; white-space: normal; overflow-wrap: anywhere; word-break: break-word; }}
    .card .v.path {{ font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 13px; }}
    .section {{ background: white; border: 1px solid #e2e8f0; border-radius: 18px; padding: 22px; margin-bottom: 18px; box-shadow: 0 8px 24px rgba(15,23,42,0.05); }}
    .muted {{ color: #64748b; line-height: 1.6; }}
    table {{ width: 100%; border-collapse: collapse; margin-top: 12px; table-layout: fixed; }}
    th, td {{ text-align: left; padding: 10px 12px; border-bottom: 1px solid #e2e8f0; font-size: 14px; vertical-align: middle; overflow-wrap: anywhere; }}
    th {{ background: #f8fafc; color: #334155; }}
    .metric-label {{ display: inline-flex; align-items: center; gap: 8px; flex-wrap: wrap; }}
    .tooltip {{ position: relative; display: inline-flex; align-items: center; justify-content: center; }}
    .tooltip-icon {{ width: 17px; height: 17px; border-radius: 999px; background: #dbeafe; color: #1d4ed8; font-size: 11px; font-weight: 700; display: inline-flex; align-items: center; justify-content: center; cursor: help; }}
    .tooltip-text {{ position: absolute; left: 50%; bottom: calc(100% + 10px); transform: translateX(-50%); width: min(280px, 80vw); padding: 10px 12px; border-radius: 10px; background: #0f172a; color: white; font-size: 12px; line-height: 1.5; box-shadow: 0 10px 24px rgba(15,23,42,0.25); opacity: 0; visibility: hidden; transition: opacity .15s ease; pointer-events: none; z-index: 20; }}
    .tooltip:hover .tooltip-text, .tooltip:focus-within .tooltip-text {{ opacity: 1; visibility: visible; }}
    .badge {{ display: inline-block; padding: 4px 10px; border-radius: 999px; font-size: 12px; font-weight: 700; }}
    .badge.good {{ background: #dcfce7; color: #166534; }}
    .badge.bad {{ background: #fee2e2; color: #991b1b; }}
    .badge.neutral {{ background: #e2e8f0; color: #334155; }}
    .chart-grid {{ display: grid; grid-template-columns: repeat(auto-fit, minmax(340px, 1fr)); gap: 14px; }}
    .chart-card {{ border: 1px solid #e2e8f0; border-radius: 16px; padding: 14px; background: #fff; min-width: 0; }}
    .chart-title {{ font-weight: 700; margin-bottom: 8px; }}
    .legend {{ display: flex; gap: 14px; font-size: 13px; color: #475569; margin-top: 8px; flex-wrap: wrap; }}
    .swatch {{ display: inline-block; width: 10px; height: 10px; border-radius: 999px; margin-right: 6px; }}
    .blue {{ background: #2563eb; }}
    .red {{ background: #dc2626; }}
    code {{ background: #eff6ff; padding: 2px 6px; border-radius: 8px; }}
  </style>
</head>
<body>
  <div class=\"wrap\">
    <section class=\"hero\">
      <h1>HuNav Evaluator Comparison Report</h1>
      <div class=\"cards\">
        <div class=\"card\"><div class=\"k\">Metrics CSV</div><div class=\"v path\">{escape(str(metrics_file))}</div></div>
        <div class=\"card\"><div class=\"k\">Run A</div><div class=\"v\">{escape(run_a.label)}</div></div>
        <div class=\"card\"><div class=\"k\">Run B</div><div class=\"v\">{escape(run_b.label)}</div></div>
        <div class=\"card\"><div class=\"k\">Output</div><div class=\"v path\">{escape(str(output_path))}</div></div>
      </div>
    </section>

    <section class=\"section\">
      <h2>Overall Summary</h2>
      <p class=\"muted\">전체 보행자 기준 <code>metrics.csv</code> 행을 비교합니다. <strong>개선/악화</strong>는 비교 대상인 Run B를 기준으로 판단합니다.</p>
      <table>
        <thead>
          <tr>
            <th>Metric</th>
            <th>{escape(run_a.label)}</th>
            <th>{escape(run_b.label)}</th>
            <th>Change</th>
            <th>판정</th>
          </tr>
        </thead>
        <tbody>
          {summary_rows_html(run_a, run_b, SUMMARY_METRICS)}
        </tbody>
      </table>
    </section>

    <section class=\"section\">
      <h2>Overall Step Charts</h2>
      <p class=\"muted\">가능한 경우 <code>metrics_steps_&lt;experiment_tag&gt;_&lt;run_id&gt;.csv</code> 를 읽어 시간축 비교 그래프를 생성합니다.</p>
      <div class=\"chart-grid\">{charts_html or '<p class="muted">step CSV를 찾지 못해 그래프를 생략했습니다.</p>'}</div>
    </section>

    {behavior_sections_html(base_dir, run_a, run_b)}
  </div>
</body>
</html>
"""

    output_path.write_text(html_text, encoding="utf-8")
    return output_path


def default_output_path(metrics_file: Path, run_a: SelectedRun, run_b: SelectedRun) -> Path:
    base = (
        f"metrics_report_{sanitize_filename(run_a.tag)}_{sanitize_filename(run_a.run_id)}"
        f"_vs_{sanitize_filename(run_b.tag)}_{sanitize_filename(run_b.run_id)}.html"
    )
    return metrics_file.parent / base


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compare HuNav evaluator CSV results and generate a self-contained HTML report."
    )
    parser.add_argument(
        "--metrics-file",
        default="/home/hunav_webots_ws/metrics.csv",
        help="Path to metrics.csv (default: /home/hunav_webots_ws/metrics.csv)",
    )
    parser.add_argument("--tag-a", help="experiment_tag for run A")
    parser.add_argument("--run-a", help="run_id for run A")
    parser.add_argument("--tag-b", help="experiment_tag for run B")
    parser.add_argument("--run-b", help="run_id for run B")
    parser.add_argument("--output", help="Output HTML path")
    return parser


def main() -> int:
    args = build_parser().parse_args()

    metrics_file = Path(args.metrics_file).expanduser()
    if not metrics_file.exists():
        raise SystemExit(f"metrics.csv not found: {metrics_file}")

    rows = load_rows(metrics_file)
    if args.tag_a or args.run_a or args.tag_b or args.run_b:
        if not (args.tag_a or args.run_a) or not (args.tag_b or args.run_b):
            raise SystemExit("When selecting manually, specify both run A and run B (at least tag or run for each).")
        run_a = select_run(rows, metrics_file, args.tag_a, args.run_a)
        run_b = select_run(rows, metrics_file, args.tag_b, args.run_b)
    else:
        run_a, run_b = auto_select_runs(rows, metrics_file)

    output_path = Path(args.output).expanduser() if args.output else default_output_path(metrics_file, run_a, run_b)
    report_path = generate_report(run_a, run_b, metrics_file, output_path)

    print(f"Run A: {run_a.label}")
    print(f"Run B: {run_b.label}")
    print(f"Metrics: {metrics_file}")
    print(f"Report : {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
