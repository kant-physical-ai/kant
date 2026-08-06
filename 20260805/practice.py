import sys
import numpy as np
import matplotlib.pyplot as plt

# %matplotlib inline
# %load_ext autoreload
# %autoreload 2

print('실행 중인 파이썬:', sys.executable)   # .venv 경로가 맞는지 확인!


# ── 자가 채점 도우미 ──────────────────────────────
_score = {}

def check(label, ok, hint=''):
    """조건 하나를 확인하고 결과를 출력한다."""
    ok = bool(ok)
    print(('  PASS  ' if ok else '  FAIL  ') + label
          + ('' if ok else '   ->  ' + hint))
    return ok

def grade(no, *conds):
    """문항 하나의 채점 결과를 기록한다."""
    _score[no] = all(conds)
    print(f"[문항 {no}] {'통과' if _score[no] else '미통과'}")

def summary():
    """전체 통과 현황을 요약한다."""
    passed = sum(_score.values())
    print(f'통과 {passed} / 시도 {len(_score)} 문항')
    bad = [k for k, v in _score.items() if not v]
    print('다시 볼 문항:', ', '.join(map(str, bad)) if bad else '없음')


###################################
# data setting

# random seed generator
rng = np.random.default_rng(0)

# 0>= random <10
scan = rng.random(360) * 10  # (0~10)  360개

# linspace는 Linearly Spaced(선형 간격으로 균등하게 나눈다)
# 180 degree = 3.14159 radian
# 360 degree = 2 * 3.14159 radian = 6.28318 radian
# start 0,
angles = np.linspace(0, 2*np.pi, 360)

print('scan  :', scan.shape, scan.dtype)
print('angles:', angles.shape)
print('처음 5개 거리:', np.round(scan[:5], 3))


# 문항 1. 유효 측정만 남기기
# 목표 — 센서 사양을 벗어난 측정을 걸러낸다.
#
# 주어진 것
# - scan — 거리 배열 (360,)

# 구현할 것
# - mask — 0.1 m 초과 그리고 10 m 미만인 곳이 True 인 불리언 배열 (360,)
# - valid — 유효한 거리만 남긴 1차원 배열
# - ratio — 유효 측정의 비율 (0~1 사이 실수)

# 기대 결과
# - valid.shape -> (247,)
# - 유효 비율 -> 0.686

# 힌트: 복합 조건은 & 로 잇고 각 조건을 괄호로 감싸야 합니다. 비율은 mask.mean() 한 줄로도 됩니다.


# boolean numpy
mask = (scan > 0.1) & (scan < 10.0)
valid = scan[mask]
ratio = mask.mean()



# ── 자가 채점 (이 셀은 수정하지 마세요) ──
_m = (scan > 0.1) & (scan < 10.0)
grade(1,
      check('mask 가 불리언 배열', getattr(mask, 'dtype', None) == bool,
            '비교 연산 결과를 그대로 담으세요'),
      check('mask 의 내용이 정확', np.array_equal(mask, _m),
            '0.1 초과 AND 10 미만 - 등호 포함 여부와 괄호를 확인하세요'),
      check('valid 가 1차원', np.ndim(valid) == 1),
      check('valid 의 내용이 정확', np.array_equal(valid, scan[_m])),
      check('ratio 가 정확', np.isclose(ratio, _m.mean()),
            'ratio = 유효 개수 / 전체 개수'))





# 문항 2. 최근접 장애물이
# 목표 — 유효 측정 중 가장 가까운 것의 거리와 그 각도를 찾는다.
#
# 주어진 것
# valid, mask, angles

# 구현할 것
# near_dist — 최소 거리 [m] (스칼라)
# near_deg — 그 측정의 각도 [deg], 0~360 범위

# 기대 결과
# 최근접 0.101 m @ 265.7 deg

# 힌트: np.argmin(valid) 는 valid 안에서의 위치입니다. 원래 각도를 찾으려면 angles[mask] 로 각도도 같이 걸러 두고 같은 인덱스를 쓰세요.

near_dist = scan[_m].min() #  valid.min()
# np.argmin은 배열에서 가장 작은 값(최솟값)이 위치한 '인덱스(위치 번호)'를 반환하는 함수입니다
near_deg  = np.degrees(angles[_m][np.argmin(scan[_m])])  # TODO   (도 단위)

print(f'최근접 {near_dist:.3f} m @ {near_deg:.1f} deg')


# ── 자가 채점 (이 셀은 수정하지 마세요) ──
_i = np.argmin(scan[_m])
grade(2,
      check('near_dist 가 최솟값', np.isclose(near_dist, scan[_m].min())),
      check('near_deg 가 그 측정의 각도',
            np.isclose(near_deg, np.degrees(angles[_m][_i])),
            '무효 측정을 걸러낸 뒤의 인덱스를 각도에도 똑같이 적용하세요'),
      check('near_deg 가 0~360 범위', 0 <= near_deg <= 360))




# 문항 3. 극좌표 → 직교좌표 (반복문 금지)
# 목표 — 유효 측정 전부를 로봇 중심 직교좌표로 옮긴다.
#
# 주어진 것
# valid (유효 거리), mask, angles

# 구현할 것
# xy — shape (N, 2) 배열. 각 행이 [x, y]
# x = r·cos(θ), y = r·sin(θ)

# 기대 결과
# xy.shape -> (247, 2)
# 힌트: np.column_stack([x, y]) 또는 np.stack([x, y], axis=1). for 문을 쓰면 이 문항은 통과해도 목적을 놓친 것입니다.



ang_valid = angles[mask]      # 유효 측정의 각도만 (mask가 True인 각도만 추출)
xy        = np.column_stack([valid * np.cos(ang_valid), valid * np.sin(ang_valid)])  # (N, 2) 형태로 합치기


print(xy.shape)
print(np.round(xy[:3], 3))

# ── 자가 채점 (이 셀은 수정하지 마세요) ──
_r, _a = scan[_m], angles[_m]
_xy = np.column_stack([_r*np.cos(_a), _r*np.sin(_a)])
grade(3,
      check('xy 의 shape 가 (N, 2)', np.shape(xy) == _xy.shape,
            f'{_xy.shape} 가 나와야 합니다'),
      check('xy 의 값이 정확', np.allclose(xy, _xy),
            'x 는 cos, y 는 sin 입니다. 열 순서를 확인하세요'),
      check('원점까지 거리가 보존됨',
            np.allclose(np.linalg.norm(xy, axis=1), _r),
            '변환은 길이를 바꾸지 않아야 합니다'))





# 문항 4. 전방 위험 구간 판정을
# 목표 — 로봇 전방 ±30° 안에 1.5 m 이내 장애물이 있으면 정지 판정을 낸다.
#
# 주어진 것
# valid, ang_valid (문항 3에서 만든 유효 각도)

# 구현할 것
#
# front — 각도가 전방 ±30° 안인 곳이 True 인 마스크
# danger — 전방이면서 1.5 m 이내인 곳이 True 인 마스크
# stop — 위험이 하나라도 있으면 True (파이썬 bool)

# 기대 결과
# 전방 위험 측정 12개 -> 정지
# 힌트: 각도는 0~2π 범위입니다. 전방 ±30° 는 θ < π/6 또는 θ > 2π - π/6 두 구간으로 나뉩니다. | 로 잇고 괄호에 주의하세요. 마지막은 .any().


minRadian = 0
maxRadian = 2 * np.pi
degree30radian = np.pi / 6

front  = (ang_valid < degree30radian) | (ang_valid > (maxRadian - degree30radian))
danger = front & (valid < 1.5)
stop   = danger.any()

print(f'전방 위험 측정 {danger.sum()}개 -> ' + ('정지' if stop else '주행'))


# ── 자가 채점 (이 셀은 수정하지 마세요) ──
pi = 2 * np.pi
_f = (_a < np.pi/6) | (_a > pi - np.pi / 6)
_d = _f & (_r < 1.5)
grade(4,
      check('front 마스크가 정확', np.array_equal(front, _f),
            '0 근처와 2pi 근처 두 구간을 OR 로 이어야 합니다'),
      check('danger 마스크가 정확', np.array_equal(danger, _d)),
      check('stop 이 bool', isinstance(bool(stop), bool)),
      check('stop 판정이 정확', bool(stop) == bool(_d.any())))



# 문항 5. 시각화
# 목표 — 위 결과를 한 그림으로 확인한다.
#
# 그릴 것
# - 유효 측정 전체를 회색 점으로
# - 위험 구간 측정을 빨간 점으로
# - 로봇 위치(원점)를 삼각형 마커로
# - ax.set_aspect('equal') 로 축 비율을 맞출 것
# 이 문항은 자동 채점 대신 눈으로 확인합니다. 6강 본문의 06_lidar_극좌표.png 오른쪽 그림과 비슷하면 성공입니다.

fig, ax = plt.subplots(figsize=(5.5, 5.5))
ax.scatter(xy[:, 0], xy[:, 1], s=12, c='0.7', label='valid')
ax.scatter(xy[danger, 0], xy[danger, 1], s=18, c='r', label='danger')
ax.plot(0, 0, marker='^', markersize=8, color='k', linestyle='None', label='robot')

ax.set_aspect('equal')
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
ax.legend()
plt.show()




# 문항 6. 회전은 길이를 바꾸지 않는다
# 목표 — 점군 전체를 z축 45° 회전시키고, 원점까지의 거리가 보존되는지 확인한다.
#
# 주어진 것
#
# xy — (N, 2) 점군
# 구현할 것
#
# R — 2×2 회전행렬 (45°)
# xy_rot — 회전된 (N, 2) 점군
# 기대 결과
#
# 회전 전후 거리 최대 차이 -> 0.0 (부동소수점 오차 수준)
# 힌트: 점이 행으로 쌓여 있으므로 xy @ R.T 입니다(6강 브로드캐스팅 절). R @ xy 는 shape 가 맞지 않습니다.


th = np.radians(45)
R = np.array([[np.cos(th), -np.sin(th)],
              [np.sin(th),  np.cos(th)]])
# R.T는 파이썬 넘파이(NumPy) 배열에서 행렬의 전치(Transpose, 행과 열을 서로 맞바꾸는 것)를 의미합니다.
# .T는 transpose() 메서드의 단축 속성입니다.
xy_rot = xy @ R.T

ax.clear()
fig, ax = plt.subplots(figsize=(5.5, 5.5))
ax.scatter(xy_rot[:, 0], xy_rot[:, 1], s=12, c='0.7', label='valid')
ax.scatter(xy_rot[danger, 0], xy_rot[danger, 1], s=18, c='r', label='danger')
ax.plot(0, 0, marker='^', markersize=8, color='k', linestyle='None', label='robot')
ax.set_aspect('equal')
ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
ax.legend()
plt.show()

d0 = np.linalg.norm(xy, axis=1)
d1 = np.linalg.norm(xy_rot, axis=1)
print('거리 최대 차이:', np.abs(d0 - d1).max())



# ── 자가 채점 (이 셀은 수정하지 마세요) ──
_c, _s = np.cos(np.radians(45)), np.sin(np.radians(45))
_R = np.array([[_c, -_s], [_s, _c]])
grade(6,
      check('R 이 2x2', np.shape(R) == (2, 2)),
      check('R 이 45도 회전행렬', np.allclose(R, _R),
            '[[cos, -sin], [sin, cos]] 순서를 확인하세요'),
      check('xy_rot 의 shape 가 유지됨', np.shape(xy_rot) == np.shape(xy)),
      check('회전이 길이를 보존', np.allclose(d0, d1),
            '길이가 변했다면 곱하는 순서나 전치를 확인하세요'))



# 문항 7. 반복문 vs 벡터화 속도 비교
# 목표 — 6강 본문의 속도 차이를 직접 측정한다.
#
# 할 것
#
# 문항 1의 전처리를 for 문 버전으로 작성한다 (loop_filter)
# %timeit 으로 두 버전의 시간을 각각 잰다
# 몇 배 차이인지 아래 마크다운 셀에 적는다

def loop_filter(scan):
    """for 문으로 유효 측정만 골라내기 (비교용)"""
    out = []
    for val in scan:
        if 0.1 < val < 10.0:
            out.append(val)
    return np.array(out)

print(np.array_equal(loop_filter(scan), scan[mask]))   # True 여야 합니다

summary()