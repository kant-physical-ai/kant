from __future__ import annotations

from libs import Volume3D


class Link:
    def __init__(self, name: str, volume: Volume3D) -> None:
        self.name = name
        self.volume = volume
