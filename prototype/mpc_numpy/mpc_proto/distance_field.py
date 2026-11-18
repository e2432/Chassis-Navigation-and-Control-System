"""Static obstacle distance field (numpy mirror of C++ GridDistanceField).

Used only for W6 C++/Python consistency checks and offline validation. The
chamfer pass order, clamping and gradient normalization intentionally match
mpc_controller/src/distance_field.cpp so QP matrices can be compared exactly.
"""
from __future__ import annotations

import numpy as np


class GridDistanceField:
    def __init__(self) -> None:
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.distance = np.zeros((0, 0))

    @classmethod
    def from_occupied(
        cls,
        occupied: np.ndarray,
        resolution: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        max_distance: float = 5.0,
    ) -> "GridDistanceField":
        field = cls()
        field.build(occupied, resolution, origin_x, origin_y, max_distance)
        return field

    def build(
        self,
        occupied: np.ndarray,
        resolution: float,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        max_distance: float = 5.0,
    ) -> None:
        occupied = np.asarray(occupied, dtype=bool)
        if occupied.ndim != 2:
            raise ValueError("occupied must be 2-D")
        height, width = occupied.shape
        if width <= 0 or height <= 0 or resolution <= 0.0:
            raise ValueError("invalid grid dimensions")

        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        cap = max(0.0, float(max_distance))
        distance = np.full((height, width), cap, dtype=float)
        distance[occupied] = 0.0
        diagonal = resolution * np.sqrt(2.0)

        for y in range(height):
            for x in range(width):
                value = distance[y, x]
                if x > 0:
                    value = min(value, distance[y, x - 1] + resolution)
                if y > 0:
                    value = min(value, distance[y - 1, x] + resolution)
                    if x > 0:
                        value = min(value, distance[y - 1, x - 1] + diagonal)
                    if x + 1 < width:
                        value = min(value, distance[y - 1, x + 1] + diagonal)
                distance[y, x] = min(value, cap)

        for y in range(height - 1, -1, -1):
            for x in range(width - 1, -1, -1):
                value = distance[y, x]
                if x + 1 < width:
                    value = min(value, distance[y, x + 1] + resolution)
                if y + 1 < height:
                    value = min(value, distance[y + 1, x] + resolution)
                    if x + 1 < width:
                        value = min(value, distance[y + 1, x + 1] + diagonal)
                    if x > 0:
                        value = min(value, distance[y + 1, x - 1] + diagonal)
                distance[y, x] = min(value, cap)

        self.distance = distance

    def empty(self) -> bool:
        return self.width <= 0 or self.height <= 0 or self.distance.size == 0

    def _at(self, ix: int, iy: int) -> float:
        ix = int(np.clip(ix, 0, self.width - 1))
        iy = int(np.clip(iy, 0, self.height - 1))
        return float(self.distance[iy, ix])

    def _gradient_x(self, ix: int, iy: int) -> float:
        left = max(ix - 1, 0)
        right = min(ix + 1, self.width - 1)
        span = right - left
        if span == 0:
            return 0.0
        return (self._at(right, iy) - self._at(left, iy)) / (span * self.resolution)

    def _gradient_y(self, ix: int, iy: int) -> float:
        lower = max(iy - 1, 0)
        upper = min(iy + 1, self.height - 1)
        span = upper - lower
        if span == 0:
            return 0.0
        return (self._at(ix, upper) - self._at(ix, lower)) / (span * self.resolution)

    def sample(self, point: np.ndarray) -> tuple[float, np.ndarray] | None:
        if self.empty():
            return None
        point = np.asarray(point, dtype=float)
        fx = (point[0] - self.origin_x) / self.resolution - 0.5
        fy = (point[1] - self.origin_y) / self.resolution - 0.5
        if fx < -0.5 or fx > self.width - 0.5 or fy < -0.5 or fy > self.height - 0.5:
            return None

        cx = float(np.clip(fx, 0.0, self.width - 1))
        cy = float(np.clip(fy, 0.0, self.height - 1))
        ix = int(np.floor(cx))
        iy = int(np.floor(cy))
        ix1 = min(ix + 1, self.width - 1)
        iy1 = min(iy + 1, self.height - 1)
        tx = cx - ix
        ty = cy - iy

        def bilinear(v00: float, v10: float, v01: float, v11: float) -> float:
            top = v00 * (1.0 - tx) + v10 * tx
            bottom = v01 * (1.0 - tx) + v11 * tx
            return top * (1.0 - ty) + bottom * ty

        distance = bilinear(
            self._at(ix, iy), self._at(ix1, iy),
            self._at(ix, iy1), self._at(ix1, iy1))
        gradient = np.array([
            bilinear(
                self._gradient_x(ix, iy), self._gradient_x(ix1, iy),
                self._gradient_x(ix, iy1), self._gradient_x(ix1, iy1)),
            bilinear(
                self._gradient_y(ix, iy), self._gradient_y(ix1, iy),
                self._gradient_y(ix, iy1), self._gradient_y(ix1, iy1)),
        ])
        norm = float(np.linalg.norm(gradient))
        if norm > 1e-9:
            gradient = gradient / norm
        return distance, gradient
