"""
Lidar.py
Load lidar point cloud data from lidar_data.json and return particles.

Data format (lidar_data.json):
  {
    "frames": [
      {
        "timestamp": float,
        "points": [[x, y, z], ...]
      },
      ...
    ]
  }
"""

import json
import os
from typing import List, Tuple

# Each point: (x, y, z)
Point3D = Tuple[float, float, float]


class LidarLoader:
    def __init__(self, data_path: str = None):
        if data_path is None:
            data_path = os.path.join(os.path.dirname(__file__), "lidar_data.json")
        self.data_path = data_path
        self.frames: list = []
        self._current_frame: int = 0
        self._load()

    def _load(self):
        with open(self.data_path, "r") as f:
            raw = json.load(f)
        self.frames = raw.get("frames", [])
        print(f"[Lidar] Loaded {len(self.frames)} frame(s) from {self.data_path}")

    # ── public API ────────────────────────────────────────

    def get_frame(self, frame_index: int) -> List[Point3D]:
        """Return point list of a specific frame index."""
        if not self.frames:
            return []
        idx = frame_index % len(self.frames)
        return [tuple(p) for p in self.frames[idx]["points"]]

    def get_current_frame(self) -> List[Point3D]:
        """Return current frame points."""
        return self.get_frame(self._current_frame)

    def next_frame(self) -> List[Point3D]:
        """Advance to next frame and return its points."""
        self._current_frame = (self._current_frame + 1) % len(self.frames)
        return self.get_current_frame()

    def frame_count(self) -> int:
        return len(self.frames)

    def current_frame_index(self) -> int:
        return self._current_frame

    def timestamp(self, frame_index: int = None) -> float:
        idx = frame_index if frame_index is not None else self._current_frame
        return self.frames[idx % len(self.frames)].get("timestamp", 0.0)
