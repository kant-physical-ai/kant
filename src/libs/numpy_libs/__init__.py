import numpy as np


def array_equal(a: np.ndarray, b: np.ndarray):
    return np.array_equal(a, b)

def sum(a: np.ndarray):
    return a.sum()

def mean(a: np.ndarray):
    return a.mean()

def min(a: np.ndarray):
    return a.min()

def max(a: np.ndarray):
    return a.max()

def isclose(a: any, b: any):
    return np.isclose(a, b)


def degree(a: np.ndarray):
    return np.degrees(a)

def cos(a: np.ndarray):
    return np.cos(a)

def tan(a: np.ndarray):
    return np.tan(a)

def arctan(a: np.ndarray):
    return np.arctan(a)

def arctan2(a: np.ndarray, b: np.ndarray):
    return np.arctan2(a, b)

def degrees(a: np.ndarray):
    return np.degrees(a)

def radians(a: np.ndarray):
    return np.radians(a)

def column_stack(arrays: list):
    """
    1차원 배열들을 세로 열(Column) 방향으로 나란히 붙여서 2차원 행렬로 만드는 함수입니다.
    책꽂이에 책을 왼쪽부터 오른쪽으로 하나씩 세워서 꽂는 모습을 상상하시면 됩니다.
    약 아래와 같이 3개짜리 1차원 배열 2개가 있다고 가정해 보겠습니다.
    x = [x1, x2, x3]
    y = [y1, y2, y3]
    np.column_stack([x, y])를 실행하면 두 배열을 세로 기둥 삼아 나란히 옆으로 붙입니다.
    [ x1, y1 ]
    [ x2, y2 ]
    [ x3, y3 ]
    :param arrays:
    :return:
    """
    return np.column_stack(arrays)

def linspace(start: float, stop: float, num: int):
    """
    start 부터 end 까지 균등하게 나눈 각도 배열을 생성합니다.
    :param start: start
    :param stop: end
    :param num:  size
    :return:
    """
    return np.linspace(start, stop, num)

def radians(a: np.ndarray):
    return np.radians(a)

def arctan2(a: np.ndarray, b: np.ndarray):
    return np.arctan2(a, b)

def angle(a: np.ndarray):
    """
    배열의 각 요소에 대해 각도를 계산합니다.
    수학에서 복소수는 x + yi(파이썬에서는 x + yj)로 표현합니다.
    이를 가로축이 실수, 세로축이 허수인 평면에 점으로 찍고 원점과 연결하면 하나의 선이 생깁니다.
    np.angle은 이 선이 반시계 방향으로 몇 라디안만큼 돌아갔는지를 계산해 줍니다.
    기본 반환 값: 라디안(radian) 단위범위: -π 부터 π 까지 (즉, -180° ~ 180° 사이의 값으로 표현)
    """
    return np.angle(a)

def sqrt(a: np.ndarray):
    """
    배열의 각 요소에 대해 제곱근을 계산합니다.
    :param a: 입력 배열
    :return: 제곱근 배열
    """
    return np.sqrt(a)

def argmin(a: np.ndarray):
    """
    배열에서 가장 작은 값(최솟값)이 위치한 '인덱스(위치 번호)'를 반환하는 함수입니다.
    :param a: parameter a
    """
    return a.argmin()
#
def argmax(a: np.ndarray):
    """
    배열에서 가장 큰 값(최댓값)이 위치한 '인덱스(위치 번호)'를 반환하는 함수입니다.
    :param a: parameter a
    """
    return a.argmax()

def sin(a: np.ndarray):
    return np.sin(a)


def shape(a: np.ndarray):
    """
     몇 차원이고, 각 차원마다 데이터가 몇 개씩 들어있는지를 튜플 ( ) 형태로 보여주는 속성입니다.

    # 데이터 5개가 한 줄로 있는 1차원 배열
    arr1 = np.array([10, 20, 30, 40, 50])
    print(arr1.shape)
    # 출력: (5,)  <- "1차원이고 데이터가 5개 있다"라는 뜻

    # 2행 3열짜리 2차원 배열
    arr2 = np.array([[1, 2, 3],
                     [4, 5, 6]])
    print(arr2.shape)
    # 출력: (2, 3)  <- "세로로 2줄, 가로로 3줄 있다"라는 뜻

    :param a:
    :return:
    """
    return a.shape