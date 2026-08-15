from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Generic, TypeVar

ARGS = TypeVar("ARGS")
RETURN = TypeVar("RETURN")


class Updater(ABC, Generic[ARGS, RETURN]):
    @abstractmethod
    def update(self, *args: ARGS, **kwargs: Any) -> RETURN:
        ...