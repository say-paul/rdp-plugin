"""Export imported MuJoCo robot instances as a composed MJCF world."""

from __future__ import annotations

import json
import math
import re
import xml.etree.ElementTree as ElementTree
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import Iterable

from .sensor_compilers.mujoco import compile_depth_camera, compile_lidar
from .sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition


_PRIMITIVE_COLLISION_TYPES = {"BOX": "box", "SPHERE": "sphere", "CAPSULE": "capsule", "CYLINDER": "cylinder"}


@dataclass(frozen=True)
class SensorMount:
    sensor: SensorDefinition
    mount_name: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    body_name: str = ""


@dataclass(frozen=True)
class RobotMount:
    target_kind: str
    target_instance_id: str
    target_body_name: str = ""
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class RobotInstance:
    instance_id: str
    robot_id: str
    source_path: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    qpos: dict[str, float]
    sensors: tuple[SensorMount, ...] = ()
    mount: RobotMount | None = None

    @property
    def prefix(self) -> str:
        value = re.sub(r"[^A-Za-z0-9_]", "_", self.instance_id).strip("_")
        return (value or "robot") + "_"


@dataclass(frozen=True)
class SceneMeshInstance:
    """Scene geometry with asset-only scale and an optional rigid root-body mount."""

    instance_id: str
    source_path: str
    rgba: tuple[float, float, float, float]
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    dynamic: bool = False
    mass: float = 1.0
    collision_shape: str = "MESH"
    collision_enabled: bool = True
    friction: float | None = None
    restitution: float = 0.0
    mount: RobotMount | None = None

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


def _model_has_free_joint(source_path: Path, visited: set[Path] | None = None) -> bool:
    source_path = source_path.expanduser().resolve()
    visited = visited if visited is not None else set()
    if source_path in visited:
        return False
    visited.add(source_path)
    try:
        root = ElementTree.parse(source_path).getroot()
    except (ElementTree.ParseError, OSError):
        return False
    if any(
        element.tag == "freejoint" or (element.tag == "joint" and element.get("type") == "free")
        for element in root.iter()
    ):
        return True
    for include in root.iter("include"):
        raw = include.get("file")
        if raw and _model_has_free_joint(source_path.parent / raw, visited):
            return True
    return False


def _sensorized_model_path(output_path: Path, instance: RobotInstance) -> Path:
    return output_path.with_name(f"{output_path.stem}_assets") / f"{instance.prefix}sensorized.xml"


def _write_sensorized_model(output_path: Path, instance: RobotInstance) -> Path:
    """Add body-mounted sensors to a copy of the source MJCF."""
    source_path = Path(instance.source_path).expanduser().resolve()
    tree = ElementTree.parse(source_path)
    root = tree.getroot()
    body_names = {body.get("name") for body in root.iter("body")}
    requested_names = {mount.body_name for mount in instance.sensors if mount.body_name}
    if requested_names - body_names and root.find("include") is not None:
        try:
            import mujoco
            root = ElementTree.fromstring(mujoco.MjSpec.from_file(str(source_path)).to_xml())
        except (ImportError, AttributeError) as error:
            raise ValueError("MuJoCo is required to mount sensors in included MJCF bodies") from error
        tree = ElementTree.ElementTree(root)
    compiler = root.find("compiler")
    asset_dirs: dict[str, Path] = {}
    if compiler is not None:
        for key in ("assetdir", "meshdir", "texturedir"):
            raw = compiler.get(key)
            if raw:
                resolved = Path(raw)
                if not resolved.is_absolute():
                    resolved = (source_path.parent / resolved).resolve()
                compiler.set(key, str(resolved))
                asset_dirs[key] = resolved
    for include in root.iter("include"):
        raw = include.get("file")
        if raw and not Path(raw).is_absolute():
            include.set("file", str((source_path.parent / raw).resolve()))
    for asset in root.findall(".//asset/*[@file]"):
        raw = asset.get("file")
        if not raw or Path(raw).is_absolute():
            continue
        preferred = asset_dirs.get("meshdir") if asset.tag == "mesh" else asset_dirs.get("texturedir")
        base = preferred or asset_dirs.get("assetdir") or source_path.parent
        asset.set("file", str((base / raw).resolve()))

    existing_sensor_section = root.find("sensor")
    for mount in instance.sensors:
        if not mount.body_name:
            continue
        target = next((body for body in root.iter("body") if body.get("name") == mount.body_name), None)
        if target is None:
            raise ValueError(f"Sensor mount body '{mount.body_name}' is not in {source_path}")
        mount_body = ElementTree.SubElement(target, "body", {
            "name": mount.mount_name,
            "pos": _format_values(mount.position),
            "quat": _format_values(mount.quaternion),
        })
        if mount.sensor.category == "lidar":
            sites, rangefinders = compile_lidar(mount.sensor.lidar or LidarSensorSpec(), mount.mount_name)
            mount_body.extend(sites)
            if existing_sensor_section is None:
                existing_sensor_section = ElementTree.SubElement(root, "sensor")
            existing_sensor_section.extend(rangefinders)
        elif mount.sensor.category == "depth_camera":
            mount_body.append(compile_depth_camera(mount.sensor.camera or CameraSensorSpec(), mount.mount_name))
    generated = _sensorized_model_path(output_path, instance)
    generated.parent.mkdir(parents=True, exist_ok=True)
    ElementTree.indent(tree, space="  ")
    tree.write(generated, encoding="utf-8", xml_declaration=True)
    return generated


def build_world_xml(
    instances: Iterable[RobotInstance],
    output_path: str | Path,
    scene_meshes: Iterable[SceneMeshInstance] = (),
) -> ElementTree.ElementTree:
    """Build a MuJoCo 3.2+ world using ``asset/model`` and ``attach``."""
    output = Path(output_path).expanduser().resolve()
    instance_list = list(instances)
    scene_mesh_list = list(scene_meshes)
    if any(instance.mount is not None for instance in instance_list) or any(mesh.mount is not None for mesh in scene_mesh_list):
        return _build_mounted_world(instance_list, output, scene_mesh_list)
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
    sensor_elements: list[ElementTree.Element] = []
    for instance in instance_list:
        if instance.instance_id in seen_ids:
            raise ValueError(f"Duplicate robot instance id: {instance.instance_id}")
        seen_ids.add(instance.instance_id)
        source = Path(instance.source_path).expanduser()
        if source.suffix.lower() != ".xml":
            raise ValueError(f"MuJoCo export requires an XML source: {source}")
        model_name = f"model_{instance.prefix[:-1]}"
        model_source = _sensorized_model_path(output, instance) if any(mount.body_name for mount in instance.sensors) else source
        ElementTree.SubElement(
            asset,
            "model",
            {"name": model_name, "file": _relative_model_path(str(model_source), output)},
        )
        attach_attributes = {"model": model_name, "prefix": instance.prefix}
        if _model_has_free_joint(source):
            ElementTree.SubElement(worldbody, "attach", attach_attributes)
            sensor_parent = worldbody
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
            sensor_parent = body

        for mount in instance.sensors:
            if mount.body_name:
                continue
            mount_name = f"{instance.prefix}{mount.mount_name}"
            mount_body = ElementTree.SubElement(
                sensor_parent,
                "body",
                {
                    "name": mount_name,
                    "pos": _format_values(mount.position),
                    "quat": _format_values(mount.quaternion),
                },
            )
            if mount.sensor.category == "lidar":
                sites, rangefinders = compile_lidar(mount.sensor.lidar or LidarSensorSpec(), mount_name)
                mount_body.extend(sites)
                sensor_elements.extend(rangefinders)
            elif mount.sensor.category == "depth_camera":
                mount_body.append(compile_depth_camera(mount.sensor.camera or CameraSensorSpec(), mount_name))

    seen_scene_ids: set[str] = set()
    for scene_mesh in scene_mesh_list:
        if scene_mesh.instance_id in seen_scene_ids:
            raise ValueError(f"Duplicate scene mesh id: {scene_mesh.instance_id}")
        seen_scene_ids.add(scene_mesh.instance_id)
        source = Path(scene_mesh.source_path).expanduser()
        if source.suffix.lower() != ".obj":
            raise ValueError(f"MuJoCo scene export requires an OBJ mesh: {source}")
        if scene_mesh.dynamic and not scene_mesh.collision_enabled:
            raise ValueError(f"Dynamic scene mesh needs collision geometry: {scene_mesh.instance_id}")
        if scene_mesh.dynamic and (not math.isfinite(scene_mesh.mass) or scene_mesh.mass <= 0):
            raise ValueError(f"Dynamic scene mesh needs positive mass: {scene_mesh.instance_id}")
        if scene_mesh.friction is not None and (not math.isfinite(scene_mesh.friction) or scene_mesh.friction < 0):
            raise ValueError(f"Scene mesh friction must be nonnegative: {scene_mesh.instance_id}")
        if not math.isfinite(scene_mesh.restitution) or not 0 <= scene_mesh.restitution <= 1:
            raise ValueError(f"Scene mesh restitution must be between 0 and 1: {scene_mesh.instance_id}")
        mesh_name = scene_mesh.asset_name
        mesh_attributes = {"name": mesh_name, "file": _relative_model_path(scene_mesh.source_path, output)}
        if scene_mesh.scale != (1.0, 1.0, 1.0):
            mesh_attributes["scale"] = _format_values(scene_mesh.scale)
        ElementTree.SubElement(asset, "mesh", mesh_attributes)
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
        primitive_type = _PRIMITIVE_COLLISION_TYPES.get(scene_mesh.collision_shape)
        if primitive_type and scene_mesh.collision_enabled:
            ElementTree.SubElement(
                body,
                "geom",
                {
                    "name": f"{mesh_name}_visual",
                    "type": "mesh",
                    "mesh": mesh_name,
                    "rgba": _format_values(scene_mesh.rgba),
                    "contype": "0",
                    "conaffinity": "0",
                    "mass": "0",
                },
            )
        geom_attributes = {
            "name": mesh_name,
            "type": primitive_type if primitive_type and scene_mesh.collision_enabled else "mesh",
            "mesh": mesh_name,
            "rgba": "0 0 0 0" if primitive_type and scene_mesh.collision_enabled else _format_values(scene_mesh.rgba),
            "contype": "1" if scene_mesh.collision_enabled else "0",
            "conaffinity": "1" if scene_mesh.collision_enabled else "0",
        }
        if scene_mesh.dynamic:
            geom_attributes["mass"] = f"{scene_mesh.mass:.17g}"
        if scene_mesh.friction is not None and scene_mesh.collision_enabled:
            geom_attributes["friction"] = _format_values((scene_mesh.friction, 0.005, 0.0001))
        if scene_mesh.restitution > 0 and scene_mesh.collision_enabled:
            # MuJoCo has no restitution geom attribute. Approximate Blender's bounce
            # with an underdamped contact spring in MuJoCo's direct solref form.
            bounce = min(scene_mesh.restitution, 0.999999)
            log_bounce = math.log(bounce)
            damping_ratio = -log_bounce / math.sqrt(math.pi**2 + log_bounce**2)
            geom_attributes["solref"] = _format_values((-10000, -200 * damping_ratio))
        ElementTree.SubElement(body, "geom", geom_attributes)

    if sensor_elements:
        sensors = ElementTree.SubElement(root, "sensor")
        sensors.extend(sensor_elements)
    return ElementTree.ElementTree(root)


def _mount_order(
    instances: list[RobotInstance], scene_meshes: list[SceneMeshInstance],
) -> list[RobotInstance | SceneMeshInstance]:
    if len({instance.instance_id for instance in instances}) != len(instances):
        raise ValueError("Duplicate robot instance id")
    if len({instance.prefix for instance in instances}) != len(instances):
        raise ValueError("Robot instance names produce duplicate MuJoCo prefixes")
    if len({mesh.instance_id for mesh in scene_meshes}) != len(scene_meshes):
        raise ValueError("Duplicate scene mesh id")
    if len({mesh.asset_name for mesh in scene_meshes}) != len(scene_meshes):
        raise ValueError("Scene mesh names produce duplicate MuJoCo names")
    nodes: dict[tuple[str, str], RobotInstance | SceneMeshInstance] = {
        **{("robot", instance.instance_id): instance for instance in instances},
        **{("scene", mesh.instance_id): mesh for mesh in scene_meshes},
    }
    order: list[RobotInstance | SceneMeshInstance] = []
    visiting: set[tuple[str, str]] = set()
    visited: set[tuple[str, str]] = set()

    def visit(key: tuple[str, str]) -> None:
        if key in visiting:
            raise ValueError("Rigid mounts contain a cycle")
        if key in visited:
            return
        visiting.add(key)
        instance = nodes[key]
        mount = instance.mount
        if mount is not None:
            if not all(math.isfinite(value) for value in (*mount.position, *mount.quaternion)):
                raise ValueError("Mount poses must be finite")
            if len(mount.position) != 3 or len(mount.quaternion) != 4 or sum(value * value for value in mount.quaternion) < 1e-12:
                raise ValueError("Mount poses require a position and nonzero quaternion")
            if mount.target_kind not in {"robot", "scene"}:
                raise ValueError(f"Unsupported mount target kind: {mount.target_kind}")
            target_key = (mount.target_kind, mount.target_instance_id)
            if target_key not in nodes:
                raise ValueError(f"Missing {mount.target_kind} mount target: {mount.target_instance_id}")
            if mount.target_kind == "robot" and not mount.target_body_name:
                raise ValueError("A robot mount requires a target MJCF body name")
            visit(target_key)
        visiting.remove(key)
        visited.add(key)
        order.append(instance)

    for key in nodes:
        visit(key)
    return order


def _load_mount_spec(source: Path, mujoco):
    spec = mujoco.MjSpec.from_file(str(source))
    for kind, directory in (
        ("meshes", spec.compiler.meshdir), ("textures", spec.compiler.texturedir),
        ("hfields", ""), ("skins", ""),
    ):
        for asset in getattr(spec, kind):
            if asset.file:
                asset.file = str((source.parent / directory / asset.file).resolve())
            if kind == "textures":
                asset.cubefiles = [
                    str((source.parent / directory / name).resolve()) if name else "" for name in asset.cubefiles
                ]
    return spec


def _mounted_spec(instance: RobotInstance, source: Path, mujoco):
    spec = _load_mount_spec(source, mujoco)
    bases = list(spec.worldbody.bodies)
    if len(bases) != 1:
        raise ValueError(f"Mounted robot '{instance.instance_id}' must have exactly one base body")
    base = bases[0]
    base_joints = list(base.joints)
    if any(joint.type != mujoco.mjtJoint.mjJNT_FREE for joint in base_joints):
        raise ValueError(f"Mounted robot '{instance.instance_id}' base must be fixed or free, not articulated")
    if not base_joints:
        return spec
    model = spec.compile()
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    base_id = 1
    base.pos = data.xpos[base_id].tolist()
    base.quat = data.xquat[base_id].tolist()
    qpos_indices: set[int] = set()
    qvel_indices: set[int] = set()
    for joint_id in range(int(model.body_jntadr[base_id]), int(model.body_jntadr[base_id] + model.body_jntnum[base_id])):
        qpos_indices.update(range(int(model.jnt_qposadr[joint_id]), int(model.jnt_qposadr[joint_id]) + 7))
        qvel_indices.update(range(int(model.jnt_dofadr[joint_id]), int(model.jnt_dofadr[joint_id]) + 6))
    for index, key in enumerate(spec.keys):
        key.qpos = [float(value) for address, value in enumerate(model.key_qpos[index]) if address not in qpos_indices]
        key.qvel = [float(value) for address, value in enumerate(model.key_qvel[index]) if address not in qvel_indices]
    for joint in base_joints:
        spec.delete(joint)
    spec.compile()
    return spec


def _add_root_sensors(spec, instance: RobotInstance, mujoco):
    root = ElementTree.fromstring(spec.to_xml())
    size = root.find("size")
    if size is not None:
        size.attrib.pop("nkey", None)
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("The robot model has no worldbody for its root sensors")
    for mount in instance.sensors:
        if mount.body_name:
            continue
        body = ElementTree.SubElement(worldbody, "body", {
            "name": mount.mount_name, "pos": _format_values(mount.position),
            "quat": _format_values(mount.quaternion),
        })
        if mount.sensor.category == "lidar":
            sites, sensors = compile_lidar(mount.sensor.lidar or LidarSensorSpec(), mount.mount_name)
            body.extend(sites)
            section = root.find("sensor")
            if section is None:
                section = ElementTree.SubElement(root, "sensor")
            section.extend(sensors)
        elif mount.sensor.category == "depth_camera":
            body.append(compile_depth_camera(mount.sensor.camera or CameraSensorSpec(), mount.mount_name))
    return mujoco.MjSpec.from_string(ElementTree.tostring(root, encoding="unicode"))


def _build_mounted_world(
    instances: list[RobotInstance], output: Path, scene_meshes: list[SceneMeshInstance],
) -> ElementTree.ElementTree:
    try:
        import mujoco
    except ImportError as error:
        raise ValueError("MuJoCo must be available in Blender to export rigid mounts") from error
    spec_type = getattr(mujoco, "MjSpec", None)
    if spec_type is None or not hasattr(spec_type, "attach"):
        raise ValueError("Rigid mounts require MuJoCo's MjSpec.attach API; upgrade MuJoCo")
    order = _mount_order(instances, scene_meshes)
    root = build_world_xml([], output).getroot()
    if root is None:
        raise ValueError("The exported MuJoCo world has no root")
    world = spec_type.from_string(ElementTree.tostring(root, encoding="unicode"))
    robots = {instance.instance_id: instance for instance in instances}
    scenes = {mesh.instance_id: mesh for mesh in scene_meshes}
    for instance in order:
        mount = instance.mount
        if isinstance(instance, SceneMeshInstance):
            if mount is not None and (not math.isfinite(instance.mass) or instance.mass <= 0):
                raise ValueError(f"Mounted scene mesh needs positive mass: {instance.instance_id}")
            scene_root = build_world_xml([], output, [replace(instance, mount=None)]).getroot()
            assert scene_root is not None
            asset = scene_root.find("asset")
            worldbody = scene_root.find("worldbody")
            assert asset is not None and worldbody is not None
            for element in list(asset):
                if element.tag == "mesh":
                    element.set("file", str((output.parent / element.get("file", "")).resolve()))
                else:
                    asset.remove(element)
            for element in list(worldbody):
                if element.tag != "body":
                    worldbody.remove(element)
                elif mount is not None:
                    element.set("pos", "0 0 0")
                    element.set("quat", "1 0 0 0")
                    for joint in element.findall("freejoint"):
                        element.remove(joint)
                    geom = element.find(f"geom[@name='{instance.asset_name}']")
                    assert geom is not None
                    geom.set("mass", f"{instance.mass:.17g}")
            child = spec_type.from_string(ElementTree.tostring(scene_root, encoding="unicode"))
            prefix = ""
            frame = world.worldbody.add_frame() if mount is None else None
        else:
            source = Path(instance.source_path).expanduser().resolve()
            if source.suffix.lower() != ".xml":
                raise ValueError(f"MuJoCo export requires an XML source: {source}")
            if any(sensor.body_name for sensor in instance.sensors):
                source = _sensorized_model_path(output, instance)
            child = _load_mount_spec(source, mujoco) if mount is None else _mounted_spec(instance, source, mujoco)
            if any(not sensor.body_name for sensor in instance.sensors):
                child = _add_root_sensors(child, instance, mujoco)
            prefix = instance.prefix
            frame = None
            if mount is None:
                if _model_has_free_joint(source):
                    frame = world.worldbody.add_frame()
                else:
                    frame = world.worldbody.add_frame(pos=instance.position, quat=instance.quaternion)
        if mount is not None:
            if mount.target_kind == "scene":
                target_name = scenes[mount.target_instance_id].asset_name
            else:
                target_name = robots[mount.target_instance_id].prefix + mount.target_body_name
            target = world.body(target_name)
            if target is None:
                raise ValueError(f"Missing MuJoCo mount body: {target_name}")
            frame = target.add_frame(pos=mount.position, quat=mount.quaternion)
        world.attach(child, prefix=prefix, frame=frame)
    world.compile()
    root = ElementTree.fromstring(world.to_xml())
    # Unprefixed specs serialize empty unnamed nested defaults, which MJCF cannot reload.
    for section in root.iter("default"):
        for nested in section.findall("default"):
            if not nested.attrib and len(nested) == 0:
                section.remove(nested)
    return ElementTree.ElementTree(root)


def _manifest_value(instance: RobotInstance) -> dict[str, object]:
    value = asdict(instance)
    value["position"] = list(instance.position)
    value["quaternion"] = list(instance.quaternion)
    value["prefix"] = instance.prefix
    value["free_base"] = instance.mount is None and _model_has_free_joint(Path(instance.source_path))
    value["sensors"] = [
        {
            "mount_name": mount.mount_name,
            "position": list(mount.position),
            "quaternion": list(mount.quaternion),
            "body_name": mount.body_name,
            "sensor": mount.sensor.to_dict(),
        }
        for mount in instance.sensors
    ]
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
import sys
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
            rotation = robot["quaternion"]
            inverse_rotation = (rotation[0], -rotation[1], -rotation[2], -rotation[3])
            local_position = (0.0, *data.qpos[qpos_address:qpos_address + 3])
            rotated_position = multiply_quaternions(
                multiply_quaternions(rotation, local_position), inverse_rotation,
            )
            for index, value in enumerate(robot["position"]):
                data.qpos[qpos_address + index] = rotated_position[index + 1] + float(value)
            orientation = multiply_quaternions(rotation, data.qpos[qpos_address + 3:qpos_address + 7])
            data.qpos[qpos_address + 3:qpos_address + 7] = orientation

mujoco.mj_forward(model, data)

for robot in manifest["robots"]:
    for mount in robot.get("sensors", []):
        mount_name = robot["prefix"] + mount["mount_name"]
        definition = mount["sensor"]
        if definition["category"] == "lidar":
            spec = definition.get("lidar") or {{}}
            channels = int(spec.get("channels", 1))
            samples = int(spec.get("horizontal_samples", 360))
            readings = []
            for channel in range(channels):
                row = []
                for index in range(samples):
                    suffix = f"{{channel:02d}}_{{index:02d}}" if channels > 1 else f"{{index:02d}}"
                    name = f"{{mount_name}}_rf_{{suffix}}"
                    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
                    if sensor_id < 0:
                        row.append(float("nan"))
                    else:
                        row.append(float(data.sensordata[int(model.sensor_adr[sensor_id])]))
                readings.append(row)
            print(f"{{mount_name}} lidar scan: {{readings[0] if channels == 1 else readings}}")
        elif definition["category"] == "depth_camera":
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, mount_name)
            if camera_id < 0:
                continue
            spec = definition.get("camera") or {{}}
            height = int(spec.get("resolution_height", 480))
            width = int(spec.get("resolution_width", 640))
            model.vis.global_.offheight = max(model.vis.global_.offheight, height)
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
            renderer = mujoco.Renderer(
                model,
                height=height,
                width=width,
            )
            try:
                renderer.enable_depth_rendering()
                renderer.update_scene(data, camera=camera_id)
                import numpy as np

                depth_path = ROOT / f"{{mount_name}}_depth.npy"
                np.save(depth_path, renderer.render())
                print(f"{{mount_name}} depth frame saved to {{depth_path}}")
            finally:
                renderer.close()

if "--sensors-only" in sys.argv:
    raise SystemExit(0)

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
    for instance in instance_list:
        if any(mount.body_name for mount in instance.sensors):
            _write_sensorized_model(output, instance)
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
