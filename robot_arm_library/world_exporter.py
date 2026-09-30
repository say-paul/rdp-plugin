"""Export imported MuJoCo robot instances as a composed MJCF world."""

from __future__ import annotations

import json
import re
import xml.etree.ElementTree as ElementTree
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class RobotInstance:
    instance_id: str
    robot_id: str
    source_path: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    qpos: dict[str, float]

    @property
    def prefix(self) -> str:
        value = re.sub(r"[^A-Za-z0-9_]", "_", self.instance_id).strip("_")
        return (value or "robot") + "_"


@dataclass(frozen=True)
class SceneMeshInstance:
    instance_id: str
    source_path: str
    rgba: tuple[float, float, float, float]
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    dynamic: bool = False
    mass: float = 1.0

    @property
    def asset_name(self) -> str:
        value = re.sub(r"[^A-Za-z0-9_]", "_", self.instance_id).strip("_")
        return "scene_" + (value or "mesh")


def _format_values(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.17g}" for value in values)


def _relative_model_path(source_path: str, output_path: Path) -> str:
    source = Path(source_path).expanduser().resolve()
    try:
        return source.relative_to(output_path.parent.resolve()).as_posix()
    except ValueError:
        return source.as_posix()


def _model_has_free_joint(source_path: Path) -> bool:
    try:
        root = ElementTree.parse(source_path).getroot()
    except (ElementTree.ParseError, OSError):
        return False
    return any(
        element.tag == "freejoint" or (element.tag == "joint" and element.get("type") == "free")
        for element in root.iter()
    )


def build_world_xml(
    instances: Iterable[RobotInstance],
    output_path: str | Path,
    scene_meshes: Iterable[SceneMeshInstance] = (),
) -> ElementTree.ElementTree:
    """Build a MuJoCo 3.2+ world using ``asset/model`` and ``attach``."""
    output = Path(output_path).expanduser().resolve()
    root = ElementTree.Element("mujoco", {"model": "caryam_world"})
    ElementTree.SubElement(root, "compiler", {"autolimits": "true"})
    ElementTree.SubElement(root, "option", {"timestep": "0.002", "gravity": "0 0 -9.81"})
    asset = ElementTree.SubElement(root, "asset")
    ElementTree.SubElement(
        asset,
        "texture",
        {
            "name": "floor_checker",
            "type": "2d",
            "builtin": "checker",
            "width": "512",
            "height": "512",
            "rgb1": "0.04 0.11 0.32",
            "rgb2": "0.17 0.37 0.74",
        },
    )
    ElementTree.SubElement(
        asset,
        "material",
        {"name": "floor_checker", "texture": "floor_checker", "texrepeat": "8 8", "reflectance": "0.15"},
    )
    worldbody = ElementTree.SubElement(root, "worldbody")
    ElementTree.SubElement(
        worldbody,
        "geom",
        {
            "name": "ground",
            "type": "plane",
            "size": "0 0 0.05",
            "material": "floor_checker",
            "contype": "1",
            "conaffinity": "1",
        },
    )
    ElementTree.SubElement(
        worldbody,
        "light",
        {"name": "scene_light", "directional": "true", "dir": "0 0 -1", "diffuse": "0.8 0.8 0.8"},
    )

    seen_ids: set[str] = set()
    for instance in instances:
        if instance.instance_id in seen_ids:
            raise ValueError(f"Duplicate robot instance id: {instance.instance_id}")
        seen_ids.add(instance.instance_id)
        source = Path(instance.source_path).expanduser()
        if source.suffix.lower() != ".xml":
            raise ValueError(f"MuJoCo export requires an XML source: {source}")
        model_name = f"model_{instance.prefix[:-1]}"
        ElementTree.SubElement(
            asset,
            "model",
            {"name": model_name, "file": _relative_model_path(instance.source_path, output)},
        )
        attach_attributes = {"model": model_name, "prefix": instance.prefix}
        if _model_has_free_joint(source):
            ElementTree.SubElement(worldbody, "attach", attach_attributes)
        else:
            body = ElementTree.SubElement(
                worldbody,
                "body",
                {
                    "name": instance.instance_id,
                    "pos": _format_values(instance.position),
                    "quat": _format_values(instance.quaternion),
                },
            )
            ElementTree.SubElement(body, "attach", attach_attributes)

    seen_scene_ids: set[str] = set()
    for scene_mesh in scene_meshes:
        if scene_mesh.instance_id in seen_scene_ids:
            raise ValueError(f"Duplicate scene mesh id: {scene_mesh.instance_id}")
        seen_scene_ids.add(scene_mesh.instance_id)
        source = Path(scene_mesh.source_path).expanduser()
        if source.suffix.lower() != ".obj":
            raise ValueError(f"MuJoCo scene export requires an OBJ mesh: {source}")
        mesh_name = scene_mesh.asset_name
        mesh_attributes = {"name": mesh_name, "file": _relative_model_path(scene_mesh.source_path, output)}
        if scene_mesh.scale != (1.0, 1.0, 1.0):
            mesh_attributes["scale"] = _format_values(scene_mesh.scale)
        ElementTree.SubElement(asset, "mesh", mesh_attributes)
        geom_attributes = {
            "name": mesh_name,
            "type": "mesh",
            "mesh": mesh_name,
            "rgba": _format_values(scene_mesh.rgba),
            "contype": "1",
            "conaffinity": "1",
        }
        body = ElementTree.SubElement(
            worldbody,
            "body",
            {
                "name": mesh_name,
                "pos": _format_values(scene_mesh.position),
                "quat": _format_values(scene_mesh.quaternion),
            },
        )
        if scene_mesh.dynamic:
            ElementTree.SubElement(body, "freejoint", {"name": f"{mesh_name}_free"})
            geom_attributes["mass"] = f"{scene_mesh.mass:.17g}"
        ElementTree.SubElement(body, "geom", geom_attributes)

    return ElementTree.ElementTree(root)


def _manifest_value(instance: RobotInstance) -> dict[str, object]:
    value = asdict(instance)
    value["position"] = list(instance.position)
    value["quaternion"] = list(instance.quaternion)
    value["prefix"] = instance.prefix
    value["free_base"] = _model_has_free_joint(Path(instance.source_path))
    return value


def _scene_mesh_manifest_value(instance: SceneMeshInstance) -> dict[str, object]:
    value = asdict(instance)
    value["rgba"] = list(instance.rgba)
    value["position"] = list(instance.position)
    value["quaternion"] = list(instance.quaternion)
    value["scale"] = list(instance.scale)
    value["asset_name"] = instance.asset_name
    return value


def _render_script(manifest_name: str, world_name: str) -> str:
    return f'''"""Render a Caryam world export with MuJoCo."""

from pathlib import Path
import json
import time

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
manifest = json.loads((ROOT / {manifest_name!r}).read_text(encoding="utf-8"))
model = mujoco.MjModel.from_xml_path(str(ROOT / {world_name!r}))
data = mujoco.MjData(model)


def robot_joint_ids(prefix):
    joint_ids = set()
    for body_id in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if not body_name or not body_name.startswith(prefix):
            continue
        first_joint = int(model.body_jntadr[body_id])
        joint_count = int(model.body_jntnum[body_id])
        joint_ids.update(range(first_joint, first_joint + joint_count))
    return sorted(joint_ids)


def joint_qpos_width(joint_id):
    start = int(model.jnt_qposadr[joint_id])
    end = int(model.jnt_qposadr[joint_id + 1]) if joint_id + 1 < model.njnt else model.nq
    return start, end


def multiply_quaternions(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


for robot in manifest["robots"]:
    prefix = robot["prefix"]
    joint_ids = robot_joint_ids(prefix)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, prefix + "home")
    if home_key >= 0:
        for joint_id in joint_ids:
            start, end = joint_qpos_width(joint_id)
            data.qpos[start:end] = model.key_qpos[home_key, start:end]
    for joint_name, value in robot["qpos"].items():
        full_name = prefix + joint_name
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full_name)
        if joint_id < 0:
            continue
        qpos_address = int(model.jnt_qposadr[joint_id])
        if int(model.jnt_type[joint_id]) in {{2, 3}}:
            data.qpos[qpos_address] = float(value)
    if robot.get("free_base"):
        free_joint = next(
            (joint_id for joint_id in joint_ids if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE)),
            -1,
        )
        if free_joint >= 0:
            qpos_address = int(model.jnt_qposadr[free_joint])
            for index, value in enumerate(robot["position"]):
                data.qpos[qpos_address + index] += float(value)
            orientation = multiply_quaternions(robot["quaternion"], data.qpos[qpos_address + 3:qpos_address + 7])
            data.qpos[qpos_address + 3:qpos_address + 7] = orientation

mujoco.mj_forward(model, data)
with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = (0.0, 2.0, 1.2)
    viewer.cam.distance = 7.0
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -20.0
    next_step_time = time.perf_counter()
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        next_step_time += model.opt.timestep
        remaining = next_step_time - time.perf_counter()
        if remaining > 0:
            time.sleep(remaining)
        else:
            next_step_time = time.perf_counter()
'''


def write_world_export(
    output_path: str | Path,
    instances: Iterable[RobotInstance],
    scene_meshes: Iterable[SceneMeshInstance] = (),
) -> tuple[Path, Path, Path]:
    """Write a MuJoCo world, robot pose data, and a runnable physics viewer script."""
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    instance_list = list(instances)
    scene_mesh_list = list(scene_meshes)
    world_tree = build_world_xml(instance_list, output, scene_mesh_list)
    ElementTree.indent(world_tree, space="  ")
    world_tree.write(output, encoding="utf-8", xml_declaration=True)

    manifest_path = output.with_suffix(".json")
    manifest_path.write_text(
        json.dumps(
            {
                "version": 1,
                "world_xml": output.name,
                "robots": [_manifest_value(instance) for instance in instance_list],
                "scene_meshes": [_scene_mesh_manifest_value(instance) for instance in scene_mesh_list],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    render_path = output.with_name("render_world.py")
    render_path.write_text(_render_script(manifest_path.name, output.name), encoding="utf-8")
    return output, manifest_path, render_path