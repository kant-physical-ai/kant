from __future__ import annotations

from pathlib import Path
from typing import Any, TypeVar, overload
from xml.etree import ElementTree

from simulator.devices.Device import Device
from simulator.devices.Encoder import Encoder
from simulator.devices.Imu import Imu
from simulator.devices.Lidar import Lidar
from simulator.devices.Motor import Motor

T = TypeVar("T", bound=Device)


def _param(element: ElementTree.Element, name: str) -> str | None:
    for param in element.findall("param"):
        if param.get("name") == name:
            return param.text.strip() if param.text else None
    return None


class DeviceManager:
    def __init__(self, urdf_path: str | Path) -> None:
        self.urdf_path = Path(urdf_path)
        self.devices: list[Device[Any, Any, Any]] = []

    def build(self) -> list[Device[Any, Any, Any]]:
        device_types: dict[str, type[Device]] = {
            "motor": Motor,
            "encoder": Encoder,
            "imu": Imu,
            "lidar": Lidar,
        }

        root = ElementTree.parse(str(self.urdf_path)).getroot()

        self.devices.clear()
        for ros2_control in root.findall("ros2_control"):
            for joint in ros2_control.findall("joint"):
                name = joint.get("name")
                if name is None:
                    continue
                device_cls = device_types.get(_param(joint, "type") or "")
                if device_cls is None:
                    continue
                error_rate = float(_param(joint, "error_rate") or 0.0)
                hz = float(_param(joint, "hz") or 0.0)
                if device_cls is Lidar:
                    num_rays = int(_param(joint, "num_rays") or 36)
                    max_range = float(_param(joint, "max_range") or 2.0)
                    elevation_min = float(_param(joint, "elevation_min") or 0.0)
                    elevation_max = float(_param(joint, "elevation_max") or 0.0)
                    elevation_rays = int(_param(joint, "elevation_rays") or 1)
                    self.devices.append(Lidar(name, num_rays=num_rays, max_range=max_range,
                                              elevation_min=elevation_min,
                                              elevation_max=elevation_max,
                                              elevation_rays=elevation_rays,
                                              error_rate=error_rate, hz=hz))
                else:
                    self.devices.append(device_cls(name, error_rate=error_rate, hz=hz))

        return self.devices

    @overload
    def find_devices(self) -> list[Device]:
        ...

    @overload
    def find_devices(self, name: str) -> list[Device]:
        ...

    @overload
    def find_devices(self, device_type: type[T]) -> list[T]:
        ...

    @overload
    def find_devices(self, device_type: type[T], name: str) -> list[T]:
        ...

    def find_devices(self, *args: Any, **kwargs: Any) -> list[Any]:
        device_type, name = self._resolve_filters(args, kwargs)
        return [d for d in self.devices
                if (device_type is None or isinstance(d, device_type))
                and (name is None or d.name == name)]

    @overload
    def find_devices_group_name(self) -> dict[str, Device]:
        ...

    @overload
    def find_devices_group_name(self, name: str) -> dict[str, Device]:
        ...

    @overload
    def find_devices_group_name(self, device_type: type[T]) -> dict[str, T]:
        ...

    @overload
    def find_devices_group_name(self, device_type: type[T], name: str) -> dict[str, T]:
        ...

    def find_devices_group_name(self, *args: Any, **kwargs: Any) -> dict[str, Any]:
        device_type, name = self._resolve_filters(args, kwargs)
        return {d.name: d for d in self.devices
                if (device_type is None or isinstance(d, device_type))
                and (name is None or d.name == name)}

    @overload
    def find_first_device(self) -> Device | None:
        ...

    @overload
    def find_first_device(self, name: str) -> Device | None:
        ...

    @overload
    def find_first_device(self, device_type: type[T]) -> T | None:
        ...

    @overload
    def find_first_device(self, device_type: type[T], name: str) -> T | None:
        ...

    def find_first_device(self, *args: Any, **kwargs: Any) -> Device | None:
        device_type, name = self._resolve_filters(args, kwargs)
        for device in self.devices:
            if device_type is not None and not isinstance(device, device_type):
                continue
            if name is not None and device.name != name:
                continue
            return device
        return None

    @overload
    def find_last_device(self) -> Device | None:
        ...

    @overload
    def find_last_device(self, name: str) -> Device | None:
        ...

    @overload
    def find_last_device(self, device_type: type[T]) -> T | None:
        ...

    @overload
    def find_last_device(self, device_type: type[T], name: str) -> T | None:
        ...

    def find_last_device(self, *args: Any, **kwargs: Any) -> Device | None:
        device_type, name = self._resolve_filters(args, kwargs)
        found = [d for d in self.devices
                 if (device_type is None or isinstance(d, device_type))
                 and (name is None or d.name == name)]
        return found[-1] if found else None

    @staticmethod
    def _resolve_filters(args: tuple[Any, ...],
                         kwargs: dict[str, Any]) -> tuple[type[Device] | None, str | None]:
        device_type: type[Device] | None = kwargs.get("device_type")
        name: str | None = kwargs.get("name")
        if args:
            first = args[0]
            if isinstance(first, str):
                name = first
            else:
                device_type = first
        if len(args) > 1:
            name = args[1]
        return device_type, name