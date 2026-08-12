import pytest
import numpy as np

from libs import math_libs, numpy_libs, device_libs, system_libs
from libs.numpy_libs import sin, sqrt, mean, array_equal, linspace


def test_import_all_subpackages():
    assert math_libs is not None
    assert device_libs is not None
    assert system_libs is not None
    assert numpy_libs is not None


def test_system_libs():
    assert system_libs.python_version_info().major == 3
    assert isinstance(system_libs.python_platform(), str)


def test_numpy_sin():
    assert sin(0.5) == pytest.approx(np.sin(0.5))


def test_numpy_sqrt():
    assert sqrt(np.array([4.0, 9.0])) == pytest.approx(np.array([2.0, 3.0]))


def test_numpy_mean():
    assert mean(np.array([1, 2, 3, 4])) == 2.5


def test_numpy_array_equal():
    assert array_equal(np.array([1, 2]), np.array([1, 2]))
    assert not array_equal(np.array([1, 2]), np.array([1, 3]))


def test_numpy_linspace():
    result = linspace(0, 90, 4)
    assert result == pytest.approx(np.array([0, 30, 60, 90]))
