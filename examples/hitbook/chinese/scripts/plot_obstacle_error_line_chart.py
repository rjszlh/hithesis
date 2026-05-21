#!/usr/bin/env python3
"""Generate the obstacle ranging relative-error line chart as SVG and PDF."""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = ROOT / "figures" / "chapter05"
SVG_PATH = FIGURE_DIR / "不同类别障碍物相对误差折线图.svg"
PDF_PATH = FIGURE_DIR / "不同类别障碍物相对误差折线图.pdf"

WIDTH = 800
HEIGHT = 500
LEFT = 90
RIGHT = 35
TOP = 35
BOTTOM = 68
PLOT_WIDTH = WIDTH - LEFT - RIGHT
PLOT_HEIGHT = HEIGHT - TOP - BOTTOM
Y_MAX = 10
# Match the thesis text width used by hithesisbook.cls.
OUTPUT_WIDTH_MM = 114.0
OUTPUT_HEIGHT_MM = OUTPUT_WIDTH_MM * HEIGHT / WIDTH

# SVG absolute units are converted into viewBox user units before PDF export.
# The figure is inserted at 0.76\textwidth in the thesis. Use a narrower source
# width so that after LaTeX scales the graphic down, the visible font size still
# matches the thesis "五号" target.
FINAL_FONT_PT = 10.5
SVG_FONT_PT = FINAL_FONT_PT * 25.4 * WIDTH / (96 * OUTPUT_WIDTH_MM)
FONT_SIZE = f"{SVG_FONT_PT:.2f}pt"

DISTANCES = [2, 4, 6, 8, 10]
SERIES = [
    ("行人", [1.0, 1.8, 2.2, 5.8, 7.4], "#1f77b4", "circle", 2.2),
    ("汽车", [0.5, 0.8, 1.2, 1.4, 5.2], "#d62728", "square", 2.2),
    ("摩托车", [3.5, 4.8, 4.8, 5.0, 8.1], "#2ca02c", "triangle", 2.2),
    ("路障", [2.0, 3.0, 3.0, 5.4, 8.6], "#9467bd", "diamond", 2.2),
    ("杆体", [1.0, 3.3, 5.5, 5.5, 5.7], "#ff7f0e", "down", 2.2),
    ("树木", [0.5, 1.5, 2.1, 4.3, 4.3], "#17becf", "plus", 2.2),
]
def sx(distance: float) -> float:
    return LEFT + (distance - min(DISTANCES)) / (max(DISTANCES) - min(DISTANCES)) * PLOT_WIDTH


def sy(error: float) -> float:
    return TOP + (Y_MAX - error) / Y_MAX * PLOT_HEIGHT


def marker(kind: str, x: float, y: float, color: str, *, filled: bool = False) -> str:
    fill = color if filled else "white"
    if kind == "circle":
        return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="5" fill="{fill}" stroke="{color}" stroke-width="2"/>'
    if kind == "square":
        return f'<rect x="{x - 5:.1f}" y="{y - 5:.1f}" width="10" height="10" fill="{fill}" stroke="{color}" stroke-width="2"/>'
    if kind == "triangle":
        points = [(x, y - 6), (x - 5.8, y + 5), (x + 5.8, y + 5)]
        return polygon(points, color, fill)
    if kind == "diamond":
        points = [(x, y - 6), (x - 6, y), (x, y + 6), (x + 6, y)]
        return polygon(points, color, fill)
    if kind == "down":
        points = [(x, y + 6), (x - 5.8, y - 5), (x + 5.8, y - 5)]
        return polygon(points, color, fill)
    return (
        f'<path d="M {x - 6:.1f} {y:.1f} L {x + 6:.1f} {y:.1f} '
        f'M {x:.1f} {y - 6:.1f} L {x:.1f} {y + 6:.1f}" '
        f'stroke="{color}" stroke-width="2.2" stroke-linecap="round" fill="none"/>'
    )


def polygon(points: list[tuple[float, float]], color: str, fill: str) -> str:
    rendered_points = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'<polygon points="{rendered_points}" fill="{fill}" stroke="{color}" stroke-width="2"/>'


def mixed_text(x: float, y: float, chinese: str, latin: str = "", **attrs: str) -> str:
    attr_text = " ".join(f'{key.replace("_", "-")}="{value}"' for key, value in attrs.items())
    latin_part = f'<tspan class="latin">{latin}</tspan>' if latin else ""
    return f'<text {attr_text} x="{x:.1f}" y="{y:.1f}"><tspan class="cn">{chinese}</tspan>{latin_part}</text>'


def polyline(values: list[float], color: str, width: float) -> str:
    points = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in zip(DISTANCES, values))
    return (
        f'<polyline points="{points}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"/>'
    )


def build_svg() -> str:
    parts: list[str] = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{OUTPUT_WIDTH_MM}mm" '
            f'height="{OUTPUT_HEIGHT_MM:.3f}mm" viewBox="0 0 {WIDTH} {HEIGHT}">'
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        (
            "<style>"
            f"text{{font-size:{FONT_SIZE};fill:#111;}}"
            '.cn{font-family:SimSun, "宋体", serif;}'
            '.latin,.num{font-family:"Times New Roman", Times, serif;}'
            ".axis{stroke:#222;stroke-width:1.2;}"
            ".grid{stroke:#d0d0d0;stroke-width:0.8;stroke-dasharray:4 4;}"
            "</style>"
        ),
    ]

    for y in range(0, Y_MAX + 1, 2):
        yy = sy(y)
        parts.append(f'<line class="grid" x1="{LEFT}" y1="{yy:.1f}" x2="{WIDTH - RIGHT}" y2="{yy:.1f}"/>')
        parts.append(f'<line class="axis" x1="{LEFT - 5}" y1="{yy:.1f}" x2="{LEFT}" y2="{yy:.1f}"/>')
        parts.append(f'<text class="num" x="{LEFT - 12}" y="{yy + 5:.1f}" text-anchor="end">{y}</text>')

    for distance in DISTANCES:
        xx = sx(distance)
        parts.append(f'<line class="grid" x1="{xx:.1f}" y1="{TOP}" x2="{xx:.1f}" y2="{HEIGHT - BOTTOM}"/>')
        parts.append(f'<line class="axis" x1="{xx:.1f}" y1="{HEIGHT - BOTTOM}" x2="{xx:.1f}" y2="{HEIGHT - BOTTOM + 5}"/>')
        parts.append(f'<text class="num" x="{xx:.1f}" y="{HEIGHT - BOTTOM + 28}" text-anchor="middle">{distance}</text>')

    parts.append(f'<line class="axis" x1="{LEFT}" y1="{TOP}" x2="{LEFT}" y2="{HEIGHT - BOTTOM}"/>')
    parts.append(f'<line class="axis" x1="{LEFT}" y1="{HEIGHT - BOTTOM}" x2="{WIDTH - RIGHT}" y2="{HEIGHT - BOTTOM}"/>')
    parts.append(mixed_text(LEFT + PLOT_WIDTH / 2, HEIGHT - 18, "真实距离", "(m)", text_anchor="middle"))
    parts.append(
        '<text transform="translate(31 {:.1f}) rotate(-90)" text-anchor="middle">'
        '<tspan class="cn">相对误差</tspan><tspan class="latin"> / %</tspan></text>'.format(TOP + PLOT_HEIGHT / 2)
    )

    for name, values, color, kind, width in SERIES:
        parts.append(polyline(values, color, width))
        for distance, error in zip(DISTANCES, values):
            parts.append(marker(kind, sx(distance), sy(error), color))

    parts.extend(build_legend())
    parts.append("</svg>")
    return "\n".join(parts)


def build_legend() -> list[str]:
    items = SERIES
    legend_x = LEFT + 12
    legend_y = TOP + 12
    legend_cols = 3
    col_width = 122
    row_height = 30
    legend_width = 395
    legend_height = 78
    parts = [
        f'<rect x="{legend_x}" y="{legend_y}" width="{legend_width}" height="{legend_height}" '
        'rx="4" fill="white" stroke="#999" stroke-width="0.8" opacity="0.94"/>'
    ]

    for index, (name, _values, color, kind, width) in enumerate(items):
        col = index % legend_cols
        row = index // legend_cols
        x0 = legend_x + 18 + col * col_width
        y0 = legend_y + 26 + row * row_height
        parts.append(
            f'<line x1="{x0}" y1="{y0}" x2="{x0 + 26}" y2="{y0}" stroke="{color}" '
            f'stroke-width="{width}" stroke-linecap="round"/>'
        )
        parts.append(marker(kind, x0 + 13, y0, color))
        parts.append(f'<text class="cn" x="{x0 + 35}" y="{y0 + 5}">{name}</text>')
    return parts


def main() -> None:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    SVG_PATH.write_text(build_svg(), encoding="utf-8")
    print("wrote", SVG_PATH)
    converter = shutil.which("rsvg-convert")
    if converter:
        subprocess.run([converter, "-f", "pdf", "-o", str(PDF_PATH), str(SVG_PATH)], check=True)
        print("wrote", PDF_PATH)
    elif inkscape := shutil.which("inkscape"):
        subprocess.run([inkscape, str(SVG_PATH), "--export-type=pdf", f"--export-filename={PDF_PATH}"], check=True)
        print("wrote", PDF_PATH)
    else:
        print("rsvg-convert and inkscape not found; skipped PDF:", PDF_PATH)


if __name__ == "__main__":
    main()
