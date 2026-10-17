"""Obstacle points → colour-frame outline polygons and distances (spec §4.4)."""
from __future__ import annotations

import cv2
import numpy as np

from .contract import MAX_OBSTACLES, MAX_PACKET_BYTES, MAX_POLY_POINTS, MIN_POLY_POINTS, encoded_size
from .projection import DepthToColorProjector

CLOSE_KERNEL = np.ones((9, 9), np.uint8)
DILATE_KERNEL = np.ones((5, 5), np.uint8)
MIN_AREA_PX2 = 150.0
EPSILON_FRACTION = 0.01
EPSILON_GROWTH = 1.5
DISTANCE_PERCENTILE = 5.0
THIN_POLY_POINTS = 6


def approximate_polygon(contour: np.ndarray, width: int, height: int) -> list[list[int]]:
    epsilon = EPSILON_FRACTION * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    while len(approx) > MAX_POLY_POINTS:
        epsilon *= EPSILON_GROWTH
        approx = cv2.approxPolyDP(contour, epsilon, True)
    points = approx.reshape(-1, 2)
    if len(points) < MIN_POLY_POINTS:
        # A long thin outline can collapse to two points; keep it drawable.
        points = cv2.boxPoints(cv2.minAreaRect(contour))
    points = np.rint(points).astype(np.int64)
    points[:, 0] = np.clip(points[:, 0], 0, width - 1)
    points[:, 1] = np.clip(points[:, 1], 0, height - 1)
    return [[int(x), int(y)] for x, y in points]


def outline_contours(color_mask: np.ndarray) -> list[np.ndarray]:
    """Close/dilate the sparse projected mask and return outer contours ≥ 150 px²."""
    mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)
    mask = cv2.dilate(mask, DILATE_KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return [c for c in contours if cv2.contourArea(c) >= MIN_AREA_PX2]


def build_obstacles(
    points: np.ndarray, fwd: np.ndarray, lat: np.ndarray,
    projector: DepthToColorProjector, *, half_width_m: float, front_offset_m: float,
) -> list[dict]:
    """Return at most four ``{dist_m, in_path, poly}`` dicts, nearest first."""
    width, height = projector.camera.width, projector.camera.height
    pixels, kept = projector.project(points)
    if len(pixels) == 0:
        return []
    fwd, lat = np.asarray(fwd)[kept], np.asarray(lat)[kept]
    mask = np.zeros((height, width), np.uint8)
    mask[pixels[:, 1], pixels[:, 0]] = 255
    labels = np.zeros((height, width), np.uint16)
    contours = outline_contours(mask)
    for index, contour in enumerate(contours[: np.iinfo(np.uint16).max - 1], start=1):
        cv2.drawContours(labels, [contour], -1, color=index, thickness=cv2.FILLED)
    point_labels = labels[pixels[:, 1], pixels[:, 0]]
    obstacles = []
    for index, contour in enumerate(contours, start=1):
        inside = point_labels == index
        if not inside.any():
            continue
        distance = float(np.percentile(fwd[inside], DISTANCE_PERCENTILE)) - front_offset_m
        obstacles.append({
            "dist_m": round(max(0.0, distance), 2),
            "in_path": bool((np.abs(lat[inside]) <= half_width_m).any()),
            "poly": approximate_polygon(contour, width, height),
        })
    obstacles.sort(key=lambda item: item["dist_m"])
    return obstacles[:MAX_OBSTACLES]


def fit_packet_size(payload: dict, limit: int = MAX_PACKET_BYTES) -> dict:
    """Halve dense polygons, then drop the farthest obstacles until it fits."""
    if encoded_size(payload) <= limit:
        return payload
    obstacles = [
        dict(item, poly=item["poly"][::2] if len(item["poly"]) > THIN_POLY_POINTS else item["poly"])
        for item in payload["obstacles"]
    ]
    payload = dict(payload, obstacles=obstacles)
    while obstacles and encoded_size(payload) > limit:
        obstacles = obstacles[:-1]
        payload = dict(payload, obstacles=obstacles)
    return payload
