#!/usr/bin/env python3
"""Extract dynamic obstacle-test trajectories from thesis image assets.

The source images are small JPEG screenshots, so the extracted data should be
treated as image-derived approximate trajectory data rather than raw sensor logs.
The script intentionally uses only the Python standard library plus Pillow,
matching the lightweight plotting approach already used in this thesis folder.
"""

from __future__ import annotations

import csv
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from statistics import median

from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = ROOT / "figures" / "chapter05"
DATA_DIR = ROOT / "data" / "chapter05"

TRAJECTORY_CSV = DATA_DIR / "dynamic_obstacle_trajectory.csv"
SUMMARY_CSV = DATA_DIR / "dynamic_obstacle_trajectory_summary.csv"
SVG_PATH = FIGURE_DIR / "动态避障轨迹反演图.svg"
PDF_PATH = FIGURE_DIR / "动态避障轨迹反演图.pdf"

FIELD_WIDTH_M = 8.0
INK_THRESHOLD = 150
SCAN_Y_MIN = 50
SCAN_Y_MAX = 170
Y_BAND_PX = 28
SMOOTH_WINDOW = 7


@dataclass(frozen=True)
class FrameConfig:
    frame: int
    filename: str
    label: str
    color: str


@dataclass
class ObstacleBlob:
    min_x: int
    min_y: int
    max_x: int
    max_y: int

    @property
    def center_x(self) -> float:
        return (self.min_x + self.max_x) / 2

    @property
    def center_y(self) -> float:
        return (self.min_y + self.max_y) / 2


@dataclass
class TrajectoryPoint:
    sample_index: int
    pixel_x: int
    pixel_y_extracted: float
    pixel_y_smoothed: float
    interpolated: bool
    x_m: float
    y_m: float


@dataclass
class Extraction:
    config: FrameConfig
    image_width: int
    image_height: int
    meters_per_pixel: float
    start_x: int
    end_x: int
    center_y: float
    obstacle: ObstacleBlob | None
    points: list[TrajectoryPoint]


FRAMES = [
    FrameConfig(1, "避障能力测试_4.jpg", "阶段1", "#1f77b4"),
    FrameConfig(2, "避障能力测试_5.jpg", "阶段2", "#d62728"),
    FrameConfig(3, "避障能力测试_6.jpg", "阶段3", "#2ca02c"),
]


def luminance(rgb: tuple[int, int, int]) -> float:
    r, g, b = rgb
    return (r + g + b) / 3


def column_ink_y(im: Image.Image, x: int) -> list[int]:
    return [
        y
        for y in range(SCAN_Y_MIN, min(SCAN_Y_MAX, im.height))
        if luminance(im.getpixel((x, y))) < INK_THRESHOLD
    ]


def column_stats(im: Image.Image) -> list[tuple[int, list[int]]]:
    return [(x, column_ink_y(im, x)) for x in range(im.width)]


def is_thick_column(ys: list[int]) -> bool:
    return bool(ys) and len(ys) >= 8 and max(ys) - min(ys) >= 8


def detect_first_thick_blob(im: Image.Image, first_x: int) -> tuple[int, ObstacleBlob | None]:
    thick_run: list[int] = []
    for x in range(first_x, im.width):
        ys = column_ink_y(im, x)
        if is_thick_column(ys):
            thick_run.append(x)
            if len(thick_run) >= 3 and thick_run[-3:] == list(range(thick_run[-3], thick_run[-3] + 3)):
                run_start = thick_run[0]
                run_end = x
                for nx in range(x + 1, im.width):
                    if not is_thick_column(column_ink_y(im, nx)):
                        break
                    run_end = nx
                blob_points = [
                    (bx, by)
                    for bx in range(run_start, run_end + 1)
                    for by in column_ink_y(im, bx)
                ]
                if not blob_points:
                    return run_start - 1, None
                min_x = min(p[0] for p in blob_points)
                max_x = max(p[0] for p in blob_points)
                min_y = min(p[1] for p in blob_points)
                max_y = max(p[1] for p in blob_points)
                return run_start - 1, ObstacleBlob(min_x, min_y, max_x, max_y)
        else:
            thick_run = []
    return im.width - 1, None


def contiguous_clusters(values: list[int]) -> list[list[int]]:
    if not values:
        return []
    clusters: list[list[int]] = []
    current = [values[0]]
    for value in values[1:]:
        if value == current[-1] + 1:
            current.append(value)
        else:
            clusters.append(current)
            current = [value]
    clusters.append(current)
    return clusters


def select_centerline_points(im: Image.Image, start_x: int, end_x: int) -> tuple[float, list[tuple[int, float]]]:
    thin_y_values: list[int] = []
    for x in range(start_x, end_x + 1):
        ys = column_ink_y(im, x)
        if ys and len(ys) <= 5:
            thin_y_values.extend(ys)
    center_y = float(median(thin_y_values)) if thin_y_values else im.height / 2

    selected: list[tuple[int, float]] = []
    previous_y: float | None = None
    for x in range(start_x, end_x + 1):
        ys = [y for y in column_ink_y(im, x) if abs(y - center_y) <= Y_BAND_PX]
        clusters = []
        for cluster in contiguous_clusters(ys):
            cluster_center = sum(cluster) / len(cluster)
            clusters.append((cluster_center, len(cluster)))
        if not clusters:
            continue

        reference_y = previous_y if previous_y is not None else center_y
        cluster_center, _cluster_size = min(clusters, key=lambda item: abs(item[0] - reference_y))
        if previous_y is not None and abs(cluster_center - previous_y) > 8:
            continue
        selected.append((x, cluster_center))
        previous_y = cluster_center
    return center_y, selected


def interpolate_points(known: list[tuple[int, float]]) -> list[tuple[int, float, bool]]:
    if not known:
        return []

    points: list[tuple[int, float, bool]] = []
    for index, (x, y) in enumerate(known[:-1]):
        next_x, next_y = known[index + 1]
        points.append((x, y, False))
        if next_x > x + 1:
            for pixel_x in range(x + 1, next_x):
                ratio = (pixel_x - x) / (next_x - x)
                points.append((pixel_x, y + (next_y - y) * ratio, True))
    points.append((known[-1][0], known[-1][1], False))
    return points


def smooth_y(points: list[tuple[int, float, bool]]) -> list[tuple[int, float, float, bool]]:
    smoothed: list[tuple[int, float, float, bool]] = []
    radius = SMOOTH_WINDOW // 2
    for index, (pixel_x, pixel_y, interpolated) in enumerate(points):
        left = max(0, index - radius)
        right = min(len(points), index + radius + 1)
        smooth_value = sum(point[1] for point in points[left:right]) / (right - left)
        smoothed.append((pixel_x, pixel_y, smooth_value, interpolated))
    return smoothed


def extract_frame(config: FrameConfig) -> Extraction:
    image_path = FIGURE_DIR / config.filename
    im = Image.open(image_path).convert("RGB")
    columns = column_stats(im)
    first_candidates = [x for x, ys in columns if ys]
    if not first_candidates:
        raise RuntimeError(f"No trajectory-like pixels found in {image_path}")

    start_x = first_candidates[0]
    end_x, obstacle = detect_first_thick_blob(im, start_x)
    center_y, known = select_centerline_points(im, start_x, end_x)
    interpolated = interpolate_points(known)
    smoothed = smooth_y(interpolated)
    meters_per_pixel = FIELD_WIDTH_M / im.width

    points = [
        TrajectoryPoint(
            sample_index=index,
            pixel_x=pixel_x,
            pixel_y_extracted=pixel_y,
            pixel_y_smoothed=pixel_y_smoothed,
            interpolated=was_interpolated,
            x_m=pixel_x * meters_per_pixel,
            y_m=(im.height - 1 - pixel_y_smoothed) * meters_per_pixel,
        )
        for index, (pixel_x, pixel_y, pixel_y_smoothed, was_interpolated) in enumerate(smoothed)
    ]

    if not points:
        raise RuntimeError(f"No centerline points extracted from {image_path}")

    return Extraction(
        config=config,
        image_width=im.width,
        image_height=im.height,
        meters_per_pixel=meters_per_pixel,
        start_x=points[0].pixel_x,
        end_x=points[-1].pixel_x,
        center_y=center_y,
        obstacle=obstacle,
        points=points,
    )


def write_csv(extractions: list[Extraction]) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with TRAJECTORY_CSV.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "frame",
                "source_image",
                "sample_index",
                "pixel_x",
                "pixel_y_extracted",
                "pixel_y_smoothed",
                "interpolated",
                "x_m",
                "y_m",
                "meters_per_pixel",
            ]
        )
        for extraction in extractions:
            for point in extraction.points:
                writer.writerow(
                    [
                        extraction.config.frame,
                        extraction.config.filename,
                        point.sample_index,
                        point.pixel_x,
                        f"{point.pixel_y_extracted:.4f}",
                        f"{point.pixel_y_smoothed:.4f}",
                        int(point.interpolated),
                        f"{point.x_m:.4f}",
                        f"{point.y_m:.4f}",
                        f"{extraction.meters_per_pixel:.8f}",
                    ]
                )

    with SUMMARY_CSV.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "frame",
                "source_image",
                "image_width_px",
                "image_height_px",
                "field_width_m",
                "meters_per_pixel",
                "start_pixel_x",
                "end_pixel_x",
                "sample_count",
                "object_pixel_x",
                "object_pixel_y",
                "object_x_m",
                "object_y_m",
            ]
        )
        for extraction in extractions:
            obstacle = extraction.obstacle
            if obstacle:
                object_pixel_x = obstacle.center_x
                object_pixel_y = obstacle.center_y
                object_x_m = object_pixel_x * extraction.meters_per_pixel
                object_y_m = (extraction.image_height - 1 - object_pixel_y) * extraction.meters_per_pixel
                object_values = [
                    f"{object_pixel_x:.2f}",
                    f"{object_pixel_y:.2f}",
                    f"{object_x_m:.4f}",
                    f"{object_y_m:.4f}",
                ]
            else:
                object_values = ["", "", "", ""]

            writer.writerow(
                [
                    extraction.config.frame,
                    extraction.config.filename,
                    extraction.image_width,
                    extraction.image_height,
                    f"{FIELD_WIDTH_M:.2f}",
                    f"{extraction.meters_per_pixel:.8f}",
                    extraction.start_x,
                    extraction.end_x,
                    len(extraction.points),
                    *object_values,
                ]
            )


def build_svg(extractions: list[Extraction]) -> str:
    width = 760
    height = 520
    left = 82
    right = 28
    top = 35
    bottom = 76
    plot_width = width - left - right
    plot_height = height - top - bottom

    all_x = [point.x_m for extraction in extractions for point in extraction.points]
    all_y = [point.y_m for extraction in extractions for point in extraction.points]
    x_min, x_max = 0.0, FIELD_WIDTH_M
    y_min = math.floor((min(all_y) - 0.15) * 10) / 10
    y_max = math.ceil((max(all_y) + 0.15) * 10) / 10
    if y_max - y_min < 0.8:
        center = (y_min + y_max) / 2
        y_min = center - 0.4
        y_max = center + 0.4

    output_width_mm = 114.0
    output_height_mm = output_width_mm * height / width
    final_font_pt = 10.5
    svg_font_pt = final_font_pt * 25.4 * width / (96 * output_width_mm)

    def sx(value: float) -> float:
        return left + (value - x_min) / (x_max - x_min) * plot_width

    def sy(value: float) -> float:
        return top + (y_max - value) / (y_max - y_min) * plot_height

    def text(x: float, y: float, content: str, klass: str = "cn", anchor: str = "middle") -> str:
        return f'<text class="{klass}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}">{content}</text>'

    parts = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{output_width_mm}mm" '
            f'height="{output_height_mm:.3f}mm" viewBox="0 0 {width} {height}">'
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        (
            "<style>"
            f"text{{font-size:{svg_font_pt:.2f}pt;fill:#111;}}"
            '.cn{font-family:SimSun, "宋体", serif;}'
            '.latin,.num{font-family:"Times New Roman", Times, serif;}'
            ".axis{stroke:#222;stroke-width:1.2;}"
            ".grid{stroke:#cfcfcf;stroke-width:0.8;stroke-dasharray:4 4;}"
            "</style>"
        ),
    ]

    x_ticks = [value for value in range(0, int(FIELD_WIDTH_M) + 1)]
    y_step = 0.2
    y_ticks = []
    value = math.ceil(y_min / y_step) * y_step
    while value <= y_max + 1e-9:
        y_ticks.append(round(value, 1))
        value += y_step

    for tick in x_ticks:
        x = sx(tick)
        parts.append(f'<line class="grid" x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height - bottom}"/>')
        parts.append(f'<line class="axis" x1="{x:.1f}" y1="{height - bottom}" x2="{x:.1f}" y2="{height - bottom + 5}"/>')
        parts.append(text(x, height - bottom + 28, f"{tick}", "num"))

    for tick in y_ticks:
        y = sy(tick)
        parts.append(f'<line class="grid" x1="{left}" y1="{y:.1f}" x2="{width - right}" y2="{y:.1f}"/>')
        parts.append(f'<line class="axis" x1="{left - 5}" y1="{y:.1f}" x2="{left}" y2="{y:.1f}"/>')
        parts.append(text(left - 12, y + 5, f"{tick:.1f}", "num", "end"))

    parts.append(f'<line class="axis" x1="{left}" y1="{top}" x2="{left}" y2="{height - bottom}"/>')
    parts.append(f'<line class="axis" x1="{left}" y1="{height - bottom}" x2="{width - right}" y2="{height - bottom}"/>')
    parts.append(
        f'<text x="{left + plot_width / 2:.1f}" y="{height - 22:.1f}" text-anchor="middle">'
        '<tspan class="latin">x</tspan><tspan class="cn"> 坐标 </tspan><tspan class="latin">(m)</tspan></text>'
    )
    parts.append(
        f'<text transform="translate(28 {top + plot_height / 2:.1f}) rotate(-90)" text-anchor="middle">'
        '<tspan class="latin">y</tspan><tspan class="cn"> 坐标 </tspan><tspan class="latin">(m)</tspan></text>'
    )

    for extraction in extractions:
        points = " ".join(f"{sx(point.x_m):.1f},{sy(point.y_m):.1f}" for point in extraction.points)
        parts.append(
            f'<polyline points="{points}" fill="none" stroke="{extraction.config.color}" '
            'stroke-width="2.4" stroke-linejoin="round" stroke-linecap="round"/>'
        )
        first = extraction.points[0]
        last = extraction.points[-1]
        parts.append(
            f'<circle cx="{sx(first.x_m):.1f}" cy="{sy(first.y_m):.1f}" r="4.2" '
            f'fill="{extraction.config.color}"/>'
        )
        parts.append(
            f'<rect x="{sx(last.x_m) - 4.2:.1f}" y="{sy(last.y_m) - 4.2:.1f}" '
            f'width="8.4" height="8.4" fill="white" stroke="{extraction.config.color}" stroke-width="2"/>'
        )
        if extraction.obstacle:
            object_x = extraction.obstacle.center_x * extraction.meters_per_pixel
            object_y = (extraction.image_height - 1 - extraction.obstacle.center_y) * extraction.meters_per_pixel
            parts.append(
                f'<circle cx="{sx(object_x):.1f}" cy="{sy(object_y):.1f}" r="5.2" '
                'fill="white" stroke="#111" stroke-width="1.8"/>'
            )

    legend_x = width - right - 178
    legend_y = top + 12
    parts.append(
        f'<rect x="{legend_x}" y="{legend_y}" width="154" height="96" rx="4" '
        'fill="white" stroke="#999" opacity="0.94"/>'
    )
    for index, extraction in enumerate(extractions):
        y = legend_y + 24 + index * 26
        parts.append(
            f'<line x1="{legend_x + 14}" y1="{y}" x2="{legend_x + 44}" y2="{y}" '
            f'stroke="{extraction.config.color}" stroke-width="2.4" stroke-linecap="round"/>'
        )
        parts.append(text(legend_x + 55, y + 5, extraction.config.label, "cn", "start"))
    y = legend_y + 24 + len(extractions) * 26
    parts.append(f'<circle cx="{legend_x + 29}" cy="{y}" r="5.2" fill="white" stroke="#111" stroke-width="1.8"/>')
    parts.append(text(legend_x + 55, y + 5, "检测物体中心", "cn", "start"))

    parts.append("</svg>")
    return "\n".join(parts)


def convert_svg_to_pdf(svg_path: Path) -> Path | None:
    converter = shutil.which("rsvg-convert")
    if not converter:
        return None
    pdf_path = PDF_PATH if svg_path == SVG_PATH else svg_path.with_suffix(".pdf")
    subprocess.run([converter, "-f", "pdf", "-o", str(pdf_path), str(svg_path)], check=True)
    return pdf_path


def write_svg(extractions: list[Extraction]) -> list[Path]:
    SVG_PATH.write_text(build_svg(extractions), encoding="utf-8")
    written_paths = [SVG_PATH]
    pdf_path = convert_svg_to_pdf(SVG_PATH)
    if pdf_path:
        written_paths.append(pdf_path)
    return written_paths


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    extractions = [extract_frame(config) for config in FRAMES]
    write_csv(extractions)
    figure_paths = write_svg(extractions)

    for extraction in extractions:
        print(
            f"{extraction.config.filename}: {len(extraction.points)} points, "
            f"x={extraction.points[0].x_m:.2f}..{extraction.points[-1].x_m:.2f} m, "
            f"y={min(p.y_m for p in extraction.points):.2f}..{max(p.y_m for p in extraction.points):.2f} m"
        )
    print("wrote", TRAJECTORY_CSV)
    print("wrote", SUMMARY_CSV)
    for figure_path in figure_paths:
        print("wrote", figure_path)


if __name__ == "__main__":
    main()
