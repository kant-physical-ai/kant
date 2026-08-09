import numpy as np

a = np.array([0.8, 0.82, 0.85, 0.79])      # list → 1차원 배열
scan = np.zeros(360)                          # 0으로 채운 배열 (LiDAR 1회전)
angles = np.linspace(0, 2*np.pi, 360)         # 0~2π 균등 분할
img = np.random.rand(480, 640, 3)             # 480×640 RGB 임의 이미지

print(a.shape)      # (4,)          — 각 차원의 크기
print(img.shape)    # (480, 640, 3) — 행, 열, 채널
print(a.dtype)      # float64       — 원소 타입 (전부 동일!)
print(img.ndim)     # 3             — 차원 수


"""mean"""

traj = np.array([[0.0, 0.0, 0.0],
                 [0.5, 0.1, 0.0],
                 [1.0, 0.3, 0.0]])   # shape (3, 3): 시점 3개 × 좌표 3

print(traj.mean(axis=0))    # 시점 방향으로 평균 → 좌표별 평균 위치, shape (3,)
print(traj.mean(axis=1))    # 좌표 방향으로 평균 → 시점별 평균(의미 없음 예시), shape (3,)


"""slicing"""
scan = np.random.rand(360) * 10       # LiDAR 스캔 (0~10m)

print(scan[0:90])        # 전방 90° 구간
print(scan[::2])         # 한 칸 걸러 다운샘플링
print(scan[-10:])        # 마지막 10개

img = np.random.rand(480, 640, 3)
print(img[100:200, 300:400])     # 이미지의 관심 영역(ROI) 잘라내기
print(img[:, :, 0])              # R 채널만 → shape (480, 640)
print(img[::2, ::2])             # 가로세로 절반 해상도로 다운샘플




roi = img[100:200, 300:400]
roi[:] = 0                 # ROI를 검게 칠하면...
# img의 해당 영역도 검게 변합니다! 같은 메모리를 보고 있기 때문
safe = img[100:200, 300:400].copy()   # 독립 사본이 필요하면 copy()









"""boolean"""
scan = np.array([0.3, 8.2, 0.5, 12.0, 0.45, 9.9])

mask = scan < 1.0          # array([True, False, True, False, True, False])
near = scan[mask]          # array([0.3, 0.5, 0.45]) — 1m 이내 장애물만
scan[scan > 10.0] = 10.0   # 10m 초과 값을 10으로 클리핑 (센서 최대거리 처리)

# 복합 조건: & (and), | (or) — 괄호 필수!
valid = scan[(scan > 0.1) & (scan < 10.0)]   # 유효 측정만



"""broadcasting"""

A = np.random.rand(480, 640, 3)   # 이미지
m = np.array([0.5, 1.0, 2.0])     # 채널별 계수, shape (3,)

A * m
# A:  (480, 640, 3)
# m:  (          3)   ← 뒤에서부터: 3=3 ✓, 나머지는 m에 차원이 없으므로 자동 확장
# 결과: (480, 640, 3) — R×0.5, G×1.0, B×2.0 이 한 줄로!


scan_mm = A

# ① 궤적 전체를 원점 이동: (N,3) - (3,)
traj_centered = traj - traj[0]

# ② 궤적 전체 회전: (3,3) 행렬 @ (N,3) → 행렬곱 + 전치 관용구
R = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])   # z축 90° 회전
traj_rot = traj @ R.T                              # (N,3) — N개의 점을 한 번에 회전!

# ③ 스칼라도 브로드캐스팅: 모든 거리값을 mm → m
scan_m = scan_mm * 0.001


# ❌ 반복문 사고
velocity_log = np.random.rand(1000) * 2 - 1   # -1~1 m/s 속도 로그
count = 0
for v in velocity_log:
    if abs(v) > 1.0:
        count += 1
ratio = count / len(velocity_log)

# ✅ 벡터화 사고: "조건 → 마스크 → 집계"
ratio = (np.abs(velocity_log) > 1.0).mean()   # True=1, False=0의 평균 = 비율