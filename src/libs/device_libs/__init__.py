from __future__ import annotations

import threading

import numpy as np
from typing import Callable, Optional, NamedTuple, Any, Generator, Iterator
from dataclasses import dataclass
from collections import deque
from queue import Queue
import time

try:
    import cv2
except ModuleNotFoundError:  # optional dependency
    cv2 = None

class FrameData(NamedTuple):
    frame: np.ndarray
    videoCapture: cv2.VideoCapture
    ret: bool
    prevTime: float
    currentTime: float
    fps: float



def video_read_iterator(video_capture: cv2.VideoCapture) -> Iterator[FrameData | None]:
    _require_cv2()
    prev_time = time.perf_counter()
    try:
        while video_capture.isOpened():
            ret, frame = video_capture.read()
            current_time = time.perf_counter()
            if not ret:
                break
            yield FrameData(
                frame=frame,
                videoCapture=video_capture,
                ret=ret,
                prevTime=prev_time,
                currentTime=current_time,
                fps=1 / (current_time - prev_time)
            )
            prev_time = current_time
    finally:
        video_capture.release()

    yield None

# def video_read_queue_thread(video_capture: cv2.VideoCapture)-> Queue[FrameData | None]:
#     frame_queue = Queue(maxsize=1)
#     def read_thread():
#         prev_time = time.perf_counter()
#         try:
#             while video_capture.isOpened():
#                 ret, frame = video_capture.read()
#                 current_time = time.perf_counter()
#                 if not ret:
#                     break
#                 frame_queue.put(FrameData(
#                     frame=frame,
#                     videoCapture=video_capture,
#                     ret=ret,
#                     prevTime=prev_time,
#                     currentTime=current_time,
#                     fps=1 / (current_time - prev_time)
#                 ))
#                 prev_time = current_time
#         finally:
#             video_capture.release()
#             frame_queue.put(None)
#
#     threading.Thread(target=read_thread, daemon=True).start()
#     return frame_queue



def is_available_video_device(device_id: int, require_frame: bool = True) -> bool:
    _require_cv2()
    cap = cv2.VideoCapture(device_id, cv2.CAP_ANY)
    try:
        if not cap.isOpened():
            return False
        ok = True
        if require_frame:
            # try to grab/read a single frame
            ret, frame = cap.read()
            ok = bool(ret and frame is not None)
        return ok
    finally:
        cap.release()


def first_video_capture() -> cv2.VideoCapture | None:
    _require_cv2()
    device_id = detect_first_video_device()
    if device_id is None:
        return None
    return cv2.VideoCapture(device_id)


def video_captures(max_devices: int = 10) -> list[cv2.VideoCapture]:
    _require_cv2()
    available = detect_video_devices(max_devices)
    return [cv2.VideoCapture(device_id) for device_id in available]


def detect_first_video_device() -> int | None:
    for i in range(10):
        if is_available_video_device(i):
            return i
    return None


def detect_video_devices(max_devices: int = 10, require_frame: bool = True) -> list[int]:
    available: list[int] = []
    for i in range(max_devices):
        if is_available_video_device(i, require_frame):
            available.append(i)
    return available


def _require_cv2() -> None:
    if cv2 is None:
        raise ModuleNotFoundError("No module named 'cv2'. Install opencv-python to use device_libs.")