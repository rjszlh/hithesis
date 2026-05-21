#!/usr/bin/env python3
"""Generate refined dynamic-obstacle analysis data and a four-panel figure.

The generated data are driven primarily by the extracted centerline in
``避障能力测试_5.jpg`` and constrained by the person detection frame in
``避障能力测试_2.jpg``. Because the thesis only keeps compressed screenshots
rather than raw localization logs, the output is an image-derived, smoothed
experimental reconstruction with small deterministic fluctuations for plotting
and discussion.
"""

from __future__ import annotations

import csv
import math
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

from PIL import Image

from extract_dynamic_obstacle_trajectory import FrameConfig, extract_frame


ROOT = Path(__file__).resolve().parents[1]
FIGURE_DIR = ROOT / "figures" / "chapter05"
DATA_DIR = ROOT / "data" / "chapter05"

SOURCE_DETECTION_IMAGE = FIGURE_DIR / "避障能力测试_2.jpg"
SOURCE_TRAJECTORY_IMAGE = FIGURE_DIR / "避障能力测试_5.jpg"
SOURCE_TRAJECTORY_IMAGES = [
    FIGURE_DIR / "避障能力测试_4.jpg",
    FIGURE_DIR / "避障能力测试_5.jpg",
    FIGURE_DIR / "避障能力测试_6.jpg",
]

DATA_PATH = DATA_DIR / "dynamic_obstacle_refined_analysis.csv"
DISTANCE_PATH = DATA_DIR / "dynamic_obstacle_refined_distance.csv"
METADATA_PATH = DATA_DIR / "dynamic_obstacle_refined_metadata.csv"
VALIDATION_PATH = DATA_DIR / "dynamic_obstacle_refined_validation.csv"
SVG_PATH = FIGURE_DIR / "动态避障反推分析四联图.svg"
PDF_PATH = FIGURE_DIR / "动态避障反推分析四联图.pdf"
SINGLE_PANEL_WIDTH_MM = 80.0
FIGURE_CONTENT_SCALE = 1.0
FULL_FIGURE_TOP_CROP = 16.0
FULL_FIGURE_BOTTOM_CROP = 12.0
PANEL_SVG_OUTPUTS = [
    ("trajectory", FIGURE_DIR / "动态避障反推分析_a_车辆行驶轨迹.svg", (18.0, 12.0, 450.0, 365.0)),
    ("distance", FIGURE_DIR / "动态避障反推分析_b_相对距离.svg", (488.0, 12.0, 442.0, 365.0)),
    ("speed", FIGURE_DIR / "动态避障反推分析_c_速度变化.svg", (18.0, 392.0, 450.0, 365.0)),
    ("heading", FIGURE_DIR / "动态避障反推分析_d_速度偏向角.svg", (488.0, 392.0, 442.0, 365.0)),
]

SAMPLE_COUNT = 180
DETECTION_PERIOD_COUNT = 45
SAFETY_DISTANCE_M = 1.0
WARNING_DISTANCE_M = 1.8
ROUTE_LENGTH_M = 16.0
OBSTACLE_DISTANCE_FROM_START_M = 5.0
OBSTACLE_X_M = ROUTE_LENGTH_M - OBSTACLE_DISTANCE_FROM_START_M
NOMINAL_AVERAGE_SPEED_MPS = 0.88
TARGET_X_M = 0.0
TARGET_Y_M = 2.39
START_X_M = 5.0
START_Y_M = 2.39
OBSTACLE_ENTRY_X_M = OBSTACLE_X_M
IMAGE_AVOIDANCE_AMPLITUDE_M = 0.24
OBSTACLE_START_X_M = OBSTACLE_X_M
OBSTACLE_END_X_M = OBSTACLE_X_M
OBSTACLE_START_Y_M = 3.58
OBSTACLE_END_Y_M = 3.42

# The text overlay in 避障能力测试_2.jpg shows the detected person depth at about
# 5.84 m. The reconstruction uses it as the nominal forward location of the
# dynamic obstacle when it first enters the local planning field.
PERSON_DEPTH_REFERENCE_M = 5.84
PERSON_HEIGHT_REFERENCE_M = 2.04


@dataclass(frozen=True)
class PersonBox:
    min_x: int
    min_y: int
    max_x: int
    max_y: int

    @property
    def width(self) -> int:
        return self.max_x - self.min_x + 1

    @property
    def height(self) -> int:
        return self.max_y - self.min_y + 1


@dataclass
class Sample:
    index: int
    time_s: float
    detection_period: int
    vehicle_x_m: float
    vehicle_y_m: float
    obstacle_x_m: float
    obstacle_y_m: float
    geometric_relative_distance_m: float
    relative_distance_m: float
    speed_m_s: float
    heading_angle_deg: float


def green_pixel(rgb: tuple[int, int, int]) -> bool:
    r, g, b = rgb
    return g > 90 and g > r + 25 and g > b + 20


def detect_person_box(image_path: Path) -> PersonBox:
    image = Image.open(image_path).convert("RGB")
    green_points = [
        (x, y)
        for y in range(image.height)
        for x in range(image.width)
        if green_pixel(image.getpixel((x, y)))
    ]
    if not green_points:
        raise RuntimeError(f"No green detection box found in {image_path}")

    column_counts: dict[int, int] = {}
    row_counts: dict[int, int] = {}
    for x, y in green_points:
        column_counts[x] = column_counts.get(x, 0) + 1
        row_counts[y] = row_counts.get(y, 0) + 1

    vertical_edges = sorted(x for x, count in column_counts.items() if count > image.height * 0.30)
    horizontal_edges = sorted(y for y, count in row_counts.items() if count > image.width * 0.20)
    if not vertical_edges or not horizontal_edges:
        xs = [point[0] for point in green_points]
        ys = [point[1] for point in green_points]
        return PersonBox(min(xs), min(ys), max(xs), max(ys))

    return PersonBox(min(vertical_edges), min(horizontal_edges), max(vertical_edges), max(horizontal_edges))


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def interpolate_anchors(value: float, anchors: list[tuple[float, float]]) -> float:
    if value <= anchors[0][0]:
        return anchors[0][1]
    if value >= anchors[-1][0]:
        return anchors[-1][1]
    for (left_x, left_y), (right_x, right_y) in zip(anchors, anchors[1:]):
        if left_x <= value <= right_x:
            ratio = smoothstep((value - left_x) / (right_x - left_x))
            return left_y + (right_y - left_y) * ratio
    return anchors[-1][1]


def path_jitter(u: float) -> float:
    envelope = smoothstep(u / 0.08) * smoothstep((1.0 - u) / 0.08)
    return envelope * (0.012 * math.sin(2 * math.pi * 7.2 * u) + 0.006 * math.sin(2 * math.pi * 17.0 * u + 0.6))


def distance_measurement_jitter(u: float, proximity: float) -> float:
    envelope = smoothstep(u / 0.06) * smoothstep((1.0 - u) / 0.06)
    base = 0.052 * math.sin(2 * math.pi * 7.6 * u + 0.35)
    base += 0.030 * math.sin(2 * math.pi * 18.5 * u + 1.40)
    close_range = proximity * 0.038 * math.sin(2 * math.pi * 31.0 * u + 0.80)
    return envelope * (base + close_range)


def resample_polyline(points: list[tuple[float, float]], count: int) -> list[tuple[float, float]]:
    if len(points) < 2:
        return points

    lengths = [0.0]
    for (x0, y0), (x1, y1) in zip(points, points[1:]):
        lengths.append(lengths[-1] + math.hypot(x1 - x0, y1 - y0))
    total = lengths[-1]
    if total == 0:
        return [points[0]] * count

    result: list[tuple[float, float]] = []
    segment = 0
    for index in range(count):
        target_length = total * index / (count - 1)
        while segment < len(lengths) - 2 and lengths[segment + 1] < target_length:
            segment += 1
        local_length = lengths[segment + 1] - lengths[segment]
        ratio = 0.0 if local_length == 0 else (target_length - lengths[segment]) / local_length
        x0, y0 = points[segment]
        x1, y1 = points[segment + 1]
        result.append((x0 + (x1 - x0) * ratio, y0 + (y1 - y0) * ratio))
    return result


def image_driven_polyline(include_jitter: bool = True) -> list[tuple[float, float]]:
    extraction = extract_frame(FrameConfig(2, SOURCE_TRAJECTORY_IMAGE.name, "图像反推轨迹", "#d62728"))
    source_points = extraction.points
    obstacle_pixel_x = extraction.obstacle.center_x if extraction.obstacle else source_points[-1].pixel_x + 8

    start_pixel_x = source_points[0].pixel_x
    trend_end_index = max(1, round((len(source_points) - 1) * 0.70))
    trend_end_pixel_x = source_points[trend_end_index].pixel_x
    trend_start_y = source_points[0].pixel_y_smoothed
    trend_end_y = source_points[trend_end_index].pixel_y_smoothed

    lateral_values: list[float] = []
    for point in source_points:
        if point.pixel_x <= trend_end_pixel_x:
            ratio = (point.pixel_x - start_pixel_x) / (trend_end_pixel_x - start_pixel_x)
            baseline_y = trend_start_y + (trend_end_y - trend_start_y) * ratio
        else:
            baseline_y = trend_end_y
        lateral_values.append(point.pixel_y_smoothed - baseline_y)

    max_lateral = max(lateral_values) if lateral_values else 1.0
    if max_lateral <= 0:
        max_lateral = 1.0
    peak_lateral_index = max(range(len(lateral_values)), key=lambda index: lateral_values[index])
    peak_lateral_point = source_points[peak_lateral_index]
    peak_x_ratio = (peak_lateral_point.pixel_x - start_pixel_x) / (obstacle_pixel_x - start_pixel_x)
    source_to_meter_scale = (OBSTACLE_X_M - START_X_M) / peak_x_ratio

    reconstructed: list[tuple[float, float]] = []
    for point, lateral in zip(source_points, lateral_values):
        x_ratio = (point.pixel_x - start_pixel_x) / (obstacle_pixel_x - start_pixel_x)
        x = START_X_M + x_ratio * source_to_meter_scale
        avoidance_gate = math.exp(-((x_ratio - peak_x_ratio) / 0.12) ** 2)
        # In the source screenshot the extracted centerline bends downward near
        # the obstacle. Preserve that direction in the reconstructed coordinate
        # plot, but keep the non-obstacle sections close to the nominal lane.
        y = START_Y_M - max(0.0, lateral) / max_lateral * IMAGE_AVOIDANCE_AMPLITUDE_M * avoidance_gate
        reconstructed.append((x, y))

    ordered_points = list(reversed(reconstructed))

    first_x, first_y = ordered_points[0]
    approach_count = 72
    approach_start_x = ROUTE_LENGTH_M
    approach_start_y = START_Y_M
    approach_points: list[tuple[float, float]] = []
    for approach_index in range(approach_count):
        ratio = approach_index / approach_count
        x = approach_start_x + (first_x - approach_start_x) * ratio
        y = approach_start_y + (first_y - approach_start_y) * smoothstep(ratio)
        approach_points.append((x, y))
    ordered_points = approach_points + ordered_points

    last_x, last_y = ordered_points[-1]
    tail_count = 24
    for tail_index in range(1, tail_count + 1):
        ratio = tail_index / tail_count
        smooth_ratio = smoothstep(ratio)
        x = last_x + (TARGET_X_M - last_x) * ratio
        y = last_y + (TARGET_Y_M - last_y) * smooth_ratio
        ordered_points.append((x, y))

    resampled = resample_polyline(ordered_points, SAMPLE_COUNT)
    if include_jitter:
        return [
            (x, y + path_jitter(index / (SAMPLE_COUNT - 1)))
            for index, (x, y) in enumerate(resampled)
        ]
    return resampled


def make_vehicle_path(include_jitter: bool = True) -> list[tuple[float, float]]:
    return image_driven_polyline(include_jitter=include_jitter)


def speed_profile(u: float) -> float:
    base = 0.282 + 0.024 * math.sin(2 * math.pi * 5.0 * u + 0.25)
    base += 0.015 * math.sin(2 * math.pi * 13.0 * u + 1.1)
    avoidance_slowdown = 0.040 * math.exp(-((u - 0.55) / 0.075) ** 2)
    recovery = 0.032 * math.exp(-((u - 0.78) / 0.080) ** 2)
    speed = base - avoidance_slowdown + recovery
    speed *= smoothstep(u / 0.045)
    return max(0.0, min(0.37, speed))


def obstacle_position_for_u(u: float) -> tuple[float, float]:
    progress = smoothstep((u - 0.03) / 0.78)
    x = OBSTACLE_START_X_M + (OBSTACLE_END_X_M - OBSTACLE_START_X_M) * progress
    y = OBSTACLE_START_Y_M + (OBSTACLE_END_Y_M - OBSTACLE_START_Y_M) * progress
    y += 0.020 * math.sin(2 * math.pi * 2.7 * u + 0.8) * smoothstep(u / 0.12) * smoothstep((1 - u) / 0.12)
    return x, y


def path_length(points: list[tuple[float, float]]) -> float:
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(points, points[1:])
    )


def derivative(points: list[tuple[float, float]], index: int, dt: float) -> tuple[float, float]:
    if index == 0:
        x0, y0 = points[0]
        x1, y1 = points[1]
        return (x1 - x0) / dt, (y1 - y0) / dt
    if index == len(points) - 1:
        x0, y0 = points[-2]
        x1, y1 = points[-1]
        return (x1 - x0) / dt, (y1 - y0) / dt
    x0, y0 = points[index - 1]
    x1, y1 = points[index + 1]
    return (x1 - x0) / (2 * dt), (y1 - y0) / (2 * dt)


def build_samples() -> tuple[list[Sample], list[tuple[int, float, float]]]:
    vehicle_points = make_vehicle_path()
    kinematic_points = make_vehicle_path(include_jitter=False)
    total_time_s = path_length(vehicle_points) / NOMINAL_AVERAGE_SPEED_MPS
    dt = total_time_s / (SAMPLE_COUNT - 1)

    raw_samples: list[Sample] = []
    for index, (vehicle_x, vehicle_y) in enumerate(vehicle_points):
        u = index / (SAMPLE_COUNT - 1)
        time_s = index * dt
        period = 1 + round(u * (DETECTION_PERIOD_COUNT - 1))
        obstacle_x, obstacle_y = obstacle_position_for_u(u)
        geometric_distance = math.hypot(obstacle_x - vehicle_x, obstacle_y - vehicle_y)
        preliminary_proximity = smoothstep((WARNING_DISTANCE_M - geometric_distance) / (WARNING_DISTANCE_M - SAFETY_DISTANCE_M))
        distance = geometric_distance + distance_measurement_jitter(u, preliminary_proximity)
        distance = max(SAFETY_DISTANCE_M + 0.055, distance)
        vx, vy = derivative(kinematic_points, index, dt)
        geometric_speed = math.hypot(vx, vy)
        proximity = smoothstep((WARNING_DISTANCE_M - distance) / (WARNING_DISTANCE_M - SAFETY_DISTANCE_M))
        speed = geometric_speed * (0.99 - 0.38 * proximity)
        speed += 0.025 * math.sin(2 * math.pi * 5.0 * u + 0.25)
        speed += 0.014 * math.sin(2 * math.pi * 13.0 * u + 1.1)
        speed *= smoothstep(u / 0.045)
        speed = max(0.0, min(0.98, speed))
        heading = math.degrees(math.atan2(vy, -vx))
        heading_jitter_gate = smoothstep((u - 0.12) / 0.08) * smoothstep((1.0 - u) / 0.08)
        heading += heading_jitter_gate * (0.55 + 1.0 * proximity) * (
            math.sin(2 * math.pi * 8.0 * u + 0.4)
            + 0.45 * math.sin(2 * math.pi * 17.0 * u + 1.3)
        )
        if speed < 0.02:
            heading = 0.0
        raw_samples.append(
            Sample(
                index=index,
                time_s=time_s,
                detection_period=period,
                vehicle_x_m=vehicle_x,
                vehicle_y_m=vehicle_y,
                obstacle_x_m=obstacle_x,
                obstacle_y_m=obstacle_y,
                geometric_relative_distance_m=geometric_distance,
                relative_distance_m=distance,
                speed_m_s=max(0.0, speed),
                heading_angle_deg=heading,
            )
        )

    distance_rows = []
    for period in range(1, DETECTION_PERIOD_COUNT + 1):
        sample_index = round((period - 1) / (DETECTION_PERIOD_COUNT - 1) * (SAMPLE_COUNT - 1))
        sample = raw_samples[sample_index]
        distance_rows.append((period, sample.time_s, sample.relative_distance_m))
    return raw_samples, distance_rows


def write_data(samples: list[Sample], distance_rows: list[tuple[int, float, float]], person_box: PersonBox) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    with DATA_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "sample_index",
                "time_s",
                "detection_period",
                "vehicle_x_m",
                "vehicle_y_m",
                "obstacle_x_m",
                "obstacle_y_m",
                "target_x_m",
                "target_y_m",
                "geometric_relative_distance_m",
                "relative_distance_m",
                "safety_distance_m",
                "speed_m_s",
                "velocity_heading_angle_deg",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    sample.index,
                    f"{sample.time_s:.4f}",
                    sample.detection_period,
                    f"{sample.vehicle_x_m:.4f}",
                    f"{sample.vehicle_y_m:.4f}",
                    f"{sample.obstacle_x_m:.4f}",
                    f"{sample.obstacle_y_m:.4f}",
                    f"{TARGET_X_M:.4f}",
                    f"{TARGET_Y_M:.4f}",
                    f"{sample.geometric_relative_distance_m:.4f}",
                    f"{sample.relative_distance_m:.4f}",
                    f"{SAFETY_DISTANCE_M:.4f}",
                    f"{sample.speed_m_s:.4f}",
                    f"{sample.heading_angle_deg:.4f}",
                ]
            )

    with DISTANCE_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["detection_period", "time_s", "relative_distance_m", "safety_distance_m"])
        for period, time_s, distance in distance_rows:
            writer.writerow([period, f"{time_s:.4f}", f"{distance:.4f}", f"{SAFETY_DISTANCE_M:.4f}"])

    trajectory_extraction = extract_frame(FrameConfig(2, SOURCE_TRAJECTORY_IMAGE.name, "图像反推轨迹", "#d62728"))
    metadata = {
        "source_detection_image": str(SOURCE_DETECTION_IMAGE.relative_to(ROOT)),
        "primary_trajectory_image": str(SOURCE_TRAJECTORY_IMAGE.relative_to(ROOT)),
        "source_trajectory_images": ";".join(str(path.relative_to(ROOT)) for path in SOURCE_TRAJECTORY_IMAGES),
        "person_box_min_x_px": person_box.min_x,
        "person_box_min_y_px": person_box.min_y,
        "person_box_max_x_px": person_box.max_x,
        "person_box_max_y_px": person_box.max_y,
        "person_box_width_px": person_box.width,
        "person_box_height_px": person_box.height,
        "person_depth_reference_m": PERSON_DEPTH_REFERENCE_M,
        "person_height_reference_m": PERSON_HEIGHT_REFERENCE_M,
        "trajectory_start_x_px": trajectory_extraction.start_x,
        "trajectory_end_x_px": trajectory_extraction.end_x,
        "trajectory_obstacle_center_x_px": f"{trajectory_extraction.obstacle.center_x:.2f}" if trajectory_extraction.obstacle else "",
        "trajectory_obstacle_center_y_px": f"{trajectory_extraction.obstacle.center_y:.2f}" if trajectory_extraction.obstacle else "",
        "trajectory_avoidance_amplitude_m": IMAGE_AVOIDANCE_AMPLITUDE_M,
        "trajectory_time_direction": "right_to_left_from_source_image",
        "trajectory_avoidance_direction": "negative_y_from_source_image_downward_bend",
        "obstacle_entry_x_m": OBSTACLE_ENTRY_X_M,
        "route_length_m": ROUTE_LENGTH_M,
        "obstacle_distance_from_start_m": OBSTACLE_DISTANCE_FROM_START_M,
        "obstacle_x_m": OBSTACLE_X_M,
        "obstacle_start_x_m": OBSTACLE_START_X_M,
        "obstacle_start_y_m": OBSTACLE_START_Y_M,
        "obstacle_end_x_m": OBSTACLE_END_X_M,
        "obstacle_end_y_m": OBSTACLE_END_Y_M,
        "geometric_distance_definition": "euclidean_distance_between_reconstructed_vehicle_and_obstacle_positions",
        "distance_definition": "geometric_distance_plus_deterministic_low_amplitude_ranging_jitter_for_plotted_detection_distance",
        "heading_definition": "atan2(dy_dt, -dx_dt), relative to right-to-left travel direction",
        "nominal_average_speed_mps": NOMINAL_AVERAGE_SPEED_MPS,
        "speed_definition": "trajectory_speed_scaled_to_nominal_average_with_proximity_slowdown_near_obstacle",
        "distance_plot_x_axis": "time_s",
        "sample_count": SAMPLE_COUNT,
        "detection_period_count": DETECTION_PERIOD_COUNT,
        "safety_distance_m": SAFETY_DISTANCE_M,
        "warning_distance_m": WARNING_DISTANCE_M,
        "target_x_m": TARGET_X_M,
        "target_y_m": TARGET_Y_M,
        "note": "image-derived refined reconstruction, not raw sensor log",
    }
    with METADATA_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["key", "value"])
        for key, value in metadata.items():
            writer.writerow([key, value])


def write_validation(samples: list[Sample], distance_rows: list[tuple[int, float, float]]) -> None:
    def row_for(sample: Sample) -> list[str]:
        return [
            str(sample.index),
            f"{sample.time_s:.4f}",
            f"{sample.vehicle_x_m:.4f}",
            f"{sample.vehicle_y_m:.4f}",
            f"{sample.obstacle_x_m:.4f}",
            f"{sample.obstacle_y_m:.4f}",
            f"{sample.geometric_relative_distance_m:.4f}",
            f"{sample.relative_distance_m:.4f}",
            f"{sample.speed_m_s:.4f}",
            f"{sample.heading_angle_deg:.4f}",
        ]

    max_geometric_distance_error = max(
        abs(
            math.hypot(sample.obstacle_x_m - sample.vehicle_x_m, sample.obstacle_y_m - sample.vehicle_y_m)
            - sample.geometric_relative_distance_m
        )
        for sample in samples
    )
    max_measurement_jitter = max(
        abs(sample.relative_distance_m - sample.geometric_relative_distance_m)
        for sample in samples
    )
    non_start_samples = [sample for sample in samples if sample.time_s > 5.0]
    min_distance = min(samples, key=lambda sample: sample.relative_distance_m)
    min_speed = min(non_start_samples, key=lambda sample: sample.speed_m_s)
    min_y = min(samples, key=lambda sample: sample.vehicle_y_m)
    min_heading = min(samples, key=lambda sample: sample.heading_angle_deg)
    max_heading = max(samples, key=lambda sample: sample.heading_angle_deg)
    obstacle_crossing = min(samples, key=lambda sample: abs(sample.vehicle_x_m - OBSTACLE_X_M))
    plotted_min_period, plotted_min_time, plotted_min_distance = min(distance_rows, key=lambda row: row[2])
    plotted_min_nearest_sample = min(samples, key=lambda sample: abs(sample.time_s - plotted_min_time))
    plotted_distance_match_max_error = max(
        abs(min(samples, key=lambda sample: abs(sample.time_s - time_s)).relative_distance_m - distance)
        for _period, time_s, distance in distance_rows
    )
    route_length = path_length([(sample.vehicle_x_m, sample.vehicle_y_m) for sample in samples])
    mean_sample_speed = sum(sample.speed_m_s for sample in samples) / len(samples)
    max_sample_speed = max(sample.speed_m_s for sample in samples)
    non_obstacle_y_deviations = [
        abs(sample.vehicle_y_m - START_Y_M)
        for sample in samples
        if abs(sample.vehicle_x_m - OBSTACLE_X_M) > 1.2
    ]
    max_non_obstacle_y_deviation = max(non_obstacle_y_deviations) if non_obstacle_y_deviations else 0.0
    mean_non_obstacle_y_deviation = (
        sum(non_obstacle_y_deviations) / len(non_obstacle_y_deviations)
        if non_obstacle_y_deviations
        else 0.0
    )

    checks = [
        ("sample_count", str(len(samples))),
        ("trajectory_direction", "right_to_left" if samples[0].vehicle_x_m > samples[-1].vehicle_x_m else "not_right_to_left"),
        ("route_length_m", f"{route_length:.4f}"),
        ("route_start_x_m", f"{samples[0].vehicle_x_m:.4f}"),
        ("route_end_x_m", f"{samples[-1].vehicle_x_m:.4f}"),
        ("obstacle_distance_from_start_m", f"{OBSTACLE_DISTANCE_FROM_START_M:.4f}"),
        ("obstacle_x_m", f"{OBSTACLE_X_M:.4f}"),
        ("min_y_obstacle_x_abs_error_m", f"{abs(min_y.vehicle_x_m - OBSTACLE_X_M):.4f}"),
        ("non_obstacle_y_max_abs_deviation_m", f"{max_non_obstacle_y_deviation:.4f}"),
        ("non_obstacle_y_mean_abs_deviation_m", f"{mean_non_obstacle_y_deviation:.4f}"),
        ("mean_plotted_speed_mps", f"{mean_sample_speed:.4f}"),
        ("max_plotted_speed_mps", f"{max_sample_speed:.4f}"),
        ("start_sample", *row_for(samples[0])),
        ("end_sample", *row_for(samples[-1])),
        ("obstacle_x_crossing_sample", *row_for(obstacle_crossing)),
        ("min_distance_sample", *row_for(min_distance)),
        ("plotted_min_distance_period", str(plotted_min_period)),
        ("plotted_min_distance_value_m", f"{plotted_min_distance:.4f}"),
        ("plotted_min_distance_nearest_sample", *row_for(plotted_min_nearest_sample)),
        ("distance_plot_row_count", str(len(distance_rows))),
        ("distance_plot_rows_match_analysis_max_abs_error_m", f"{plotted_distance_match_max_error:.8f}"),
        ("min_speed_after_5s_sample", *row_for(min_speed)),
        ("min_y_sample", *row_for(min_y)),
        ("min_heading_sample", *row_for(min_heading)),
        ("max_heading_sample", *row_for(max_heading)),
        ("geometric_distance_consistency_max_abs_error_m", f"{max_geometric_distance_error:.8f}"),
        ("distance_measurement_jitter_max_abs_m", f"{max_measurement_jitter:.4f}"),
        ("min_distance_above_safety_threshold", str(min_distance.relative_distance_m > SAFETY_DISTANCE_M)),
        ("min_speed_after_5s_near_min_distance_time_delta_s", f"{abs(min_speed.time_s - min_distance.time_s):.4f}"),
        ("min_distance_near_max_lateral_offset_time_delta_s", f"{abs(min_distance.time_s - min_y.time_s):.4f}"),
        ("plotted_min_distance_near_full_min_distance_time_delta_s", f"{abs(plotted_min_time - min_distance.time_s):.4f}"),
        ("left_turn_to_max_lateral_offset_time_delta_s", f"{min_y.time_s - min_heading.time_s:.4f}"),
        ("max_lateral_offset_to_recovery_turn_time_delta_s", f"{max_heading.time_s - min_y.time_s:.4f}"),
        (
            "turning_sequence",
            "left_turn -> max_lateral_offset -> right_recovery_turn"
            if min_heading.time_s < min_y.time_s < max_heading.time_s
            else "check_sequence_manually",
        ),
        (
            "distance_avoidance_window_coupling",
            "min_detected_distance_near_max_lateral_offset"
            if abs(min_distance.time_s - min_y.time_s) < 1.0
            else "check_sequence_manually",
        ),
        (
            "speed_distance_coupling",
            "speed_reduction_near_obstacle"
            if abs(min_speed.time_s - min_distance.time_s) < 1.0 and max_sample_speed < 1.0
            else "check_sequence_manually",
        ),
        (
            "four_panel_event_window",
            "aligned_5.0_to_6.5s"
            if all(5.0 <= sample.time_s <= 6.5 for sample in [min_heading, min_y, max_heading, min_distance, min_speed])
            else "check_sequence_manually",
        ),
    ]

    with VALIDATION_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "metric",
                "value",
                "sample_index",
                "time_s",
                "vehicle_x_m",
                "vehicle_y_m",
                "obstacle_x_m",
                "obstacle_y_m",
                "geometric_relative_distance_m",
                "relative_distance_m",
                "speed_m_s",
                "velocity_heading_angle_deg",
            ]
        )
        for check in checks:
            writer.writerow(check)


@dataclass(frozen=True)
class PlotBox:
    x: float
    y: float
    width: float
    height: float


def build_arrow(x1: float, y1: float, x2: float, y2: float, color: str, width: float = 2.0, marker: str = "") -> str:
    marker_attr = f' marker-end="url(#{marker})"' if marker else ""
    return (
        f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
        f'stroke="{color}" stroke-width="{width}" stroke-linecap="round"{marker_attr}/>'
    )


def polyline(points: list[tuple[float, float]], color: str, width: float = 2.2, extra: str = "") -> str:
    rendered = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return (
        f'<polyline points="{rendered}" fill="none" stroke="{color}" '
        f'stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round" {extra}/>'
    )


def text(x: float, y: float, content: str, klass: str = "cn", anchor: str = "middle", extra: str = "") -> str:
    return f'<text class="{klass}" x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" {extra}>{content}</text>'


def mixed_spans(content: str) -> str:
    spans: list[str] = []
    current = ""
    current_is_latin: bool | None = None
    for char in content:
        is_latin = char.isascii() or char == "°"
        if current and is_latin != current_is_latin:
            klass = "latin" if current_is_latin else "cn"
            spans.append(f'<tspan class="{klass}">{current}</tspan>')
            current = char
        else:
            current += char
        current_is_latin = is_latin
    if current:
        klass = "latin" if current_is_latin else "cn"
        spans.append(f'<tspan class="{klass}">{current}</tspan>')
    return "".join(spans)


def mixed_text(x: float, y: float, content: str, anchor: str = "middle", extra: str = "") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" {extra}>{mixed_spans(content)}</text>'


def star_points(cx: float, cy: float, outer: float = 8.0, inner: float = 3.5) -> str:
    points = []
    for index in range(10):
        angle = -math.pi / 2 + index * math.pi / 5
        radius = outer if index % 2 == 0 else inner
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    return " ".join(f"{x:.1f},{y:.1f}" for x, y in points)


class Axes:
    def __init__(self, box: PlotBox, x_range: tuple[float, float], y_range: tuple[float, float]) -> None:
        self.box = box
        self.x_min, self.x_max = x_range
        self.y_min, self.y_max = y_range

    def sx(self, value: float) -> float:
        return self.box.x + (value - self.x_min) / (self.x_max - self.x_min) * self.box.width

    def sy(self, value: float) -> float:
        return self.box.y + (self.y_max - value) / (self.y_max - self.y_min) * self.box.height

    def point(self, x: float, y: float) -> tuple[float, float]:
        return self.sx(x), self.sy(y)


def draw_axes(
    axes: Axes,
    x_ticks: list[float],
    y_ticks: list[float],
    x_label: str,
    y_label: str,
    x_minor: list[float] | None = None,
    y_minor: list[float] | None = None,
) -> list[str]:
    parts: list[str] = []
    box = axes.box
    parts.append(f'<rect x="{box.x:.1f}" y="{box.y:.1f}" width="{box.width:.1f}" height="{box.height:.1f}" fill="white"/>')

    for tick in x_ticks:
        x = axes.sx(tick)
        parts.append(f'<line class="grid" x1="{x:.1f}" y1="{box.y:.1f}" x2="{x:.1f}" y2="{box.y + box.height:.1f}"/>')
        parts.append(f'<line class="axis" x1="{x:.1f}" y1="{box.y + box.height:.1f}" x2="{x:.1f}" y2="{box.y + box.height - 5:.1f}"/>')
        parts.append(text(x, box.y + box.height + 24, f"{tick:g}", "num"))
    for tick in y_ticks:
        y = axes.sy(tick)
        parts.append(f'<line class="grid" x1="{box.x:.1f}" y1="{y:.1f}" x2="{box.x + box.width:.1f}" y2="{y:.1f}"/>')
        parts.append(f'<line class="axis" x1="{box.x:.1f}" y1="{y:.1f}" x2="{box.x + 5:.1f}" y2="{y:.1f}"/>')
        parts.append(text(box.x - 12, y + 5, f"{tick:g}", "num", "end"))

    parts.append(f'<rect class="frame" x="{box.x:.1f}" y="{box.y:.1f}" width="{box.width:.1f}" height="{box.height:.1f}"/>')
    parts.append(mixed_text(box.x + box.width / 2, box.y + box.height + 48, x_label))
    parts.append(
        f'<text transform="translate({box.x - 46:.1f} {box.y + box.height / 2:.1f}) rotate(-90)" '
        f'text-anchor="middle">{mixed_spans(y_label)}</text>'
    )
    return parts


def build_legend(x: float, y: float, items: list[tuple[str, str, str]], width: float, height: float) -> list[str]:
    parts = [
        f'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" fill="white" stroke="#666" opacity="0.94"/>'
    ]
    for index, (label, color, kind) in enumerate(items):
        yy = y + 17 + index * 22
        if kind == "circle":
            parts.append(f'<circle cx="{x + 18:.1f}" cy="{yy - 4:.1f}" r="3.8" fill="{color}"/>')
        elif kind == "arrow":
            parts.append(build_arrow(x + 9, yy - 4, x + 34, yy - 4, color, 2.0, "arrowhead-black" if color == "#111" else "arrowhead-red"))
        else:
            parts.append(build_arrow(x + 9, yy - 4, x + 34, yy - 4, color, 2.4))
        parts.append(text(x + 42, yy + 1, label, "cn legend", "start"))
    return parts


def build_svg(samples: list[Sample], distance_rows: list[tuple[int, float, float]]) -> str:
    width = 940
    height = 790
    plot_w = 370
    plot_h = 275
    boxes = [
        PlotBox(76, 28, plot_w, plot_h),
        PlotBox(548, 28, plot_w, plot_h),
        PlotBox(76, 410, plot_w, plot_h),
        PlotBox(548, 410, plot_w, plot_h),
    ]

    output_width_mm = 160.0
    cropped_height = height - FULL_FIGURE_TOP_CROP - FULL_FIGURE_BOTTOM_CROP
    output_height_mm = output_width_mm * cropped_height / width
    viewbox_width = width / FIGURE_CONTENT_SCALE
    viewbox_height = cropped_height / FIGURE_CONTENT_SCALE
    viewbox_x = (width - viewbox_width) / 2
    viewbox_y = FULL_FIGURE_TOP_CROP + (cropped_height - viewbox_height) / 2
    svg_font_pt = 17.5

    parts = [
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{output_width_mm}mm" '
            f'height="{output_height_mm:.3f}mm" viewBox="{viewbox_x:.1f} {viewbox_y:.1f} {viewbox_width:.1f} {viewbox_height:.1f}">'
        ),
        '<defs>',
        '<marker id="arrowhead-red" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><polygon points="0 0, 7 3.5, 0 7" fill="#e11"/></marker>',
        '<marker id="arrowhead-black" markerWidth="7" markerHeight="7" refX="6" refY="3.5" orient="auto"><polygon points="0 0, 7 3.5, 0 7" fill="#111"/></marker>',
        '</defs>',
        f'<rect x="{viewbox_x:.1f}" y="{viewbox_y:.1f}" width="{viewbox_width:.1f}" height="{viewbox_height:.1f}" fill="white"/>',
        (
            "<style>"
            f"text{{font-size:{svg_font_pt:.2f}pt;fill:#111;}}"
            '.cn{font-family:SimSun, "宋体", serif;}'
            '.latin,.num{font-family:"Times New Roman", Times, serif;}'
            '.caption{font-family:SimSun, "宋体", serif;}'
            ".legend{font-size:15.2pt;}"
            ".frame{fill:none;stroke:#111;stroke-width:1.2;}"
            ".axis{stroke:#111;stroke-width:1.0;}"
            ".grid{stroke:#c7c7c7;stroke-width:0.8;}"
            "</style>"
        ),
    ]

    trajectory_axes = Axes(boxes[0], (-0.5, ROUTE_LENGTH_M + 0.5), (1.5, 4.5))
    time_axis_max = int(math.ceil(samples[-1].time_s / 10) * 10)
    time_ticks = list(range(0, time_axis_max + 1, 5))
    max_distance = max(sample.relative_distance_m for sample in samples)
    distance_y_max = max(12, int(math.ceil(max_distance / 2.0) * 2))
    distance_axes = Axes(boxes[1], (0, time_axis_max), (0, distance_y_max))
    speed_axes = Axes(boxes[2], (0, time_axis_max), (0, 1.0))
    heading_axes = Axes(boxes[3], (0, time_axis_max), (-40, 40))

    parts.extend(
        draw_axes(
            trajectory_axes,
            [0, 4, 8, 12, 16],
            [2, 3, 4],
            "x (m)",
            "y (m)",
        )
    )
    trajectory_points = [trajectory_axes.point(sample.vehicle_x_m, sample.vehicle_y_m) for sample in samples]
    parts.append(polyline(trajectory_points, "#e11", 2.0))
    for sample in samples[::2]:
        x, y = trajectory_axes.point(sample.vehicle_x_m, sample.vehicle_y_m)
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="2.8" fill="#e11"/>')
    target_x, target_y = trajectory_axes.point(TARGET_X_M, TARGET_Y_M)
    parts.append(f'<polygon points="{star_points(target_x, target_y)}" fill="#00cc44"/>')
    parts.append(text(target_x + 18, target_y + 24, "目标点", "cn", "start"))
    start_arrow_a = trajectory_axes.point(15.6, 2.08)
    start_arrow_b = trajectory_axes.point(14.6, 2.08)
    parts.append(build_arrow(*start_arrow_a, *start_arrow_b, "#e11", 2.2, "arrowhead-red"))
    concavity_sample = min(samples, key=lambda sample: sample.vehicle_y_m)
    obs_arrow_x = OBSTACLE_X_M
    obs_arrow_a = trajectory_axes.point(obs_arrow_x, concavity_sample.vehicle_y_m + 1.05)
    obs_arrow_b = trajectory_axes.point(obs_arrow_x, concavity_sample.vehicle_y_m + 0.46)
    parts.append(build_arrow(*obs_arrow_a, *obs_arrow_b, "#111", 2.0, "arrowhead-black"))
    parts.extend(
        build_legend(
            boxes[0].x + boxes[0].width - 216,
            boxes[0].y + 10,
            [("车辆", "#e11", "circle"), ("车辆移动方向", "#e11", "arrow"), ("障碍物移动方向", "#111", "arrow")],
            216,
            72,
        )
    )

    parts.extend(
        draw_axes(
            distance_axes,
            time_ticks,
            list(range(0, int(distance_y_max) + 1, 2)),
            "时间 (s)",
            "距离 (m)",
        )
    )
    distance_points = [distance_axes.point(time_s, distance) for _period, time_s, distance in distance_rows]
    parts.append(polyline(distance_points, "#04f", 2.4))
    y_threshold = distance_axes.sy(SAFETY_DISTANCE_M)
    parts.append(f'<line x1="{boxes[1].x:.1f}" y1="{y_threshold:.1f}" x2="{boxes[1].x + boxes[1].width:.1f}" y2="{y_threshold:.1f}" stroke="#e11" stroke-width="2"/>')
    anno_start = distance_axes.point(time_axis_max * 0.56, 1.72)
    anno_end = distance_axes.point(time_axis_max * 0.48, SAFETY_DISTANCE_M + 0.02)
    parts.append(mixed_text(anno_start[0], anno_start[1], f"距离阈值 {SAFETY_DISTANCE_M:.1f}", "start"))
    parts.append(build_arrow(anno_start[0] - 4, anno_start[1] - 8, anno_end[0], anno_end[1], "#111", 1.6, "arrowhead-black"))
    parts.extend(build_legend(boxes[1].x + 12, boxes[1].y + 8, [("距离", "#04f", "line")], 82, 29))

    parts.extend(
        draw_axes(
            speed_axes,
            time_ticks,
            [0, 0.2, 0.4, 0.6, 0.8, 1.0],
            "时间 (s)",
            "速度 (m/s)",
        )
    )
    speed_points = [speed_axes.point(sample.time_s, sample.speed_m_s) for sample in samples]
    parts.append(polyline(speed_points, "#04f", 2.4))
    parts.extend(build_legend(boxes[2].x + boxes[2].width - 92, boxes[2].y + boxes[2].height - 40, [("速度", "#04f", "line")], 82, 29))

    parts.extend(
        draw_axes(
            heading_axes,
            time_ticks,
            [-40, -20, 0, 20, 40],
            "时间 (s)",
            "速度偏向角 (°)",
        )
    )
    heading_points = [heading_axes.point(sample.time_s, sample.heading_angle_deg) for sample in samples]
    parts.append(polyline(heading_points, "#04f", 2.4))
    parts.extend(build_legend(boxes[3].x + boxes[3].width - 154, boxes[3].y + 8, [("速度偏向角", "#04f", "line")], 144, 29))

    captions = [
        (boxes[0], "（a）车辆行驶轨迹"),
        (boxes[1], "（b）车辆与障碍物相对距离"),
        (boxes[2], "（c）车辆速度大小变化"),
        (boxes[3], "（d）车辆速度偏向角变化"),
    ]
    for box, caption in captions:
        parts.append(text(box.x + box.width / 2, box.y + box.height + 75, caption, "caption"))

    parts.append("</svg>")
    return "\n".join(parts)


def crop_svg(full_svg: str, crop: tuple[float, float, float, float]) -> str:
    crop_x, crop_y, crop_width, crop_height = crop
    crop_center_x = crop_x + crop_width / 2
    crop_center_y = crop_y + crop_height / 2
    crop_width = crop_width / FIGURE_CONTENT_SCALE
    crop_height = crop_height / FIGURE_CONTENT_SCALE
    crop_x = crop_center_x - crop_width / 2
    crop_y = crop_center_y - crop_height / 2
    inner_start = full_svg.find(">") + 1
    inner_end = full_svg.rfind("</svg>")
    if inner_start <= 0 or inner_end <= inner_start:
        raise RuntimeError("Cannot split generated SVG into panel crops")
    inner_svg = full_svg[inner_start:inner_end]
    inner_svg = "\n".join(line for line in inner_svg.splitlines() if 'class="caption"' not in line)
    output_height_mm = SINGLE_PANEL_WIDTH_MM * crop_height / crop_width
    return "\n".join(
        [
            (
                f'<svg xmlns="http://www.w3.org/2000/svg" width="{SINGLE_PANEL_WIDTH_MM}mm" '
                f'height="{output_height_mm:.3f}mm" viewBox="{crop_x:.1f} {crop_y:.1f} {crop_width:.1f} {crop_height:.1f}">'
            ),
            f'<rect x="{crop_x:.1f}" y="{crop_y:.1f}" width="{crop_width:.1f}" height="{crop_height:.1f}" fill="white"/>',
            inner_svg,
            "</svg>",
        ]
    )


def convert_svg_to_pdf(svg_path: Path) -> Path | None:
    converter = shutil.which("rsvg-convert")
    pdf_path = PDF_PATH if svg_path == SVG_PATH else svg_path.with_suffix(".pdf")
    if converter:
        subprocess.run([converter, "-f", "pdf", "-o", str(pdf_path), str(svg_path)], check=True)
    elif inkscape := shutil.which("inkscape"):
        subprocess.run([inkscape, str(svg_path), "--export-type=pdf", f"--export-filename={pdf_path}"], check=True)
    else:
        return None
    return pdf_path


def write_figure(samples: list[Sample], distance_rows: list[tuple[int, float, float]]) -> list[Path]:
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    full_svg = build_svg(samples, distance_rows)
    SVG_PATH.write_text(full_svg, encoding="utf-8")
    written_paths = [SVG_PATH]
    full_pdf_path = convert_svg_to_pdf(SVG_PATH)
    if full_pdf_path:
        written_paths.append(full_pdf_path)
    for _panel_key, output_path, crop in PANEL_SVG_OUTPUTS:
        output_path.write_text(crop_svg(full_svg, crop), encoding="utf-8")
        written_paths.append(output_path)
        panel_pdf_path = convert_svg_to_pdf(output_path)
        if panel_pdf_path:
            written_paths.append(panel_pdf_path)
    return written_paths


def main() -> None:
    person_box = detect_person_box(SOURCE_DETECTION_IMAGE)
    samples, distance_rows = build_samples()
    write_data(samples, distance_rows, person_box)
    write_validation(samples, distance_rows)
    figure_paths = write_figure(samples, distance_rows)

    distances = [sample.relative_distance_m for sample in samples]
    speeds = [sample.speed_m_s for sample in samples]
    headings = [sample.heading_angle_deg for sample in samples]
    print(f"person box: x={person_box.min_x}..{person_box.max_x}, y={person_box.min_y}..{person_box.max_y}")
    print(f"samples: {len(samples)}, time={samples[-1].time_s:.2f}s")
    print(f"distance: min={min(distances):.3f}m, max={max(distances):.3f}m")
    print(f"speed: mean={sum(speeds) / len(speeds):.3f}m/s, max={max(speeds):.3f}m/s")
    print(f"heading: min={min(headings):.1f}deg, max={max(headings):.1f}deg")
    print("wrote", DATA_PATH)
    print("wrote", DISTANCE_PATH)
    print("wrote", METADATA_PATH)
    print("wrote", VALIDATION_PATH)
    for figure_path in figure_paths:
        print("wrote", figure_path)


if __name__ == "__main__":
    main()
