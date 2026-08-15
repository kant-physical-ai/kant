import math


def calculate_fps(prev_time: float, current_time: float):
    fps = 1 / (current_time - prev_time)
    return fps

def degrees_to_radians(degrees: float):
    return degrees * (math.pi / 180)

def radians_to_degrees(radians: float):
    return radians * (180 / math.pi)

def angle_wrap(a: float) -> float:
    """Wrap angle to [-π, π]."""
    while a >  math.pi: a -= 2 * math.pi
    while a < -math.pi: a += 2 * math.pi
    return a