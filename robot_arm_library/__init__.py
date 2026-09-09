"""Blender add-on entry point with a parser-friendly package import."""

bl_info = {
    "name": "MuJoCo Robot Arm Library",
    "author": "Caryam",
    "version": (0, 7, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar > Robot Library",
    "description": "Discover robot arms from a MuJoCo setup and drag them into the scene",
    "category": "3D View",
}

from .setup_parser import RobotDefinition, discover_robot_definitions


try:
    from .blender_addon import bl_info, register, unregister
except ModuleNotFoundError as error:
    if error.name != "bpy":
        raise

    def register() -> None:
        raise RuntimeError("The Blender add-on must be registered from Blender")

    def unregister() -> None:
        return None


__all__ = ["RobotDefinition", "discover_robot_definitions", "register", "unregister"]
