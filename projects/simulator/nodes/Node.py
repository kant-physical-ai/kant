from __future__ import annotations

from abc import ABC

from simulator.devices.DeviceManager import DeviceManager


class Node(ABC):
    def __init__(self, name: str, device_manager: DeviceManager) -> None:
        self.name = name
        self.device_manager = device_manager