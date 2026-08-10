import math


def calculate_fps(prev_time: float, current_time: float):
    fps = 1 / (current_time - prev_time)
    return fps

# 각도를 라디안으로 변환
def degrees_to_radians(degrees: float):
    return degrees * (math.pi / 180)

# 라디안을 각도로 변환
def radians_to_degrees(radians: float):
    return radians * (180 / math.pi)