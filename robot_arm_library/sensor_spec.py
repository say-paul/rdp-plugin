"""Caryam graph adapter for the independent MuJoCo Sensor Library."""

from __future__ import annotations

import site
import sys

try:
    import mujoco_sensor_library as _sensors
except ModuleNotFoundError as error:
    if error.name != "mujoco_sensor_library":
        raise
    # Blender does not always include Python's user site on its import path.
    user_site = site.getusersitepackages()
    if user_site not in sys.path:
        site.addsitedir(user_site)
    try:
        import mujoco_sensor_library as _sensors
    except ModuleNotFoundError as missing:
        if missing.name != "mujoco_sensor_library":
            raise
        raise ModuleNotFoundError(
            "Install the separate mujoco-sensor-library package for Blender's Python "
            "and the external simulation Python; see the Caryam installation guide.",
            name="mujoco_sensor_library",
        ) from missing

from .behavior_graph import PortSpec

if not hasattr(_sensors, "compile_imu"):
    raise ImportError("Caryam requires mujoco-sensor-library >=0.1.1; update the separate sensor package or run make install")

CameraSensorSpec = _sensors.CameraSensorSpec
LidarSensorSpec = _sensors.LidarSensorSpec
SENSOR_CATEGORIES = _sensors.SENSOR_CATEGORIES
CAMERA_CATEGORIES = _sensors.CAMERA_CATEGORIES


class SensorDefinition(_sensors.SensorDefinition):
    def output_ports(self) -> tuple[PortSpec, ...]:
        from .port_catalog import sensor_port_templates

        value = self.to_dict()
        if self.category == "lidar" and self.lidar is None:
            value["lidar"] = LidarSensorSpec().to_dict()
        if self.category in CAMERA_CATEGORIES and self.camera is None:
            value["camera"] = CameraSensorSpec().to_dict()
        return sensor_port_templates(value)
