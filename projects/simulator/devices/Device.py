from __future__ import annotations

import random
from abc import ABC, abstractmethod
from typing import Generic, TypeVar

READ = TypeVar("READ")
WINPUT = TypeVar("WINPUT")
WRETURN = TypeVar("WRETURN")


class Device(ABC, Generic[READ, WINPUT, WRETURN]):
    def __init__(self, name: str, error_rate: float = 0.0, hz: float = 0.0) -> None:
        self.name = name
        self.error_rate = error_rate
        self.hz = hz

    def _apply_error(self, value: float) -> float:
        if self.error_rate <= 0.0 or value == 0.0:
            return value
        return value * (1.0 + random.uniform(-self.error_rate, self.error_rate))

    @abstractmethod
    def read(self) -> READ:
        ...

    @abstractmethod
    def write(self, value: WINPUT) -> WRETURN:
        ...