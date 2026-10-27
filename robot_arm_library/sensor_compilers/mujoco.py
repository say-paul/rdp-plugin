"""Compatibility imports for the independent library's MJCF compiler."""

from ..sensor_spec import _sensors

compile_lidar = _sensors.compile_lidar
compile_camera = _sensors.compile_camera
compile_imu = _sensors.compile_imu
compile_depth_camera = _sensors.compile_depth_camera
