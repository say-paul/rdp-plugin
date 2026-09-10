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


def _format_values(values: Iterable[float]) -> str:
    return " ".join(f"{float(value):.17g}" for value in values)


def _relative_model_path(source_path: str, output_path: Path) -> str:
    source = Path(source_path).expanduser().resolve()
    try:
        return source.relative_to(output_path.parent.resolve()).as_posix()
    except ValueError:
        return source.as_posix()


def build_world_xml(instances: Iterable[RobotInstance], output_path: str | Path) -> ElementTree.ElementTree:
    """Build a MuJoCo 3.2+ world using ``asset/model`` and ``attach``."""
    output = Path(output_path).expanduser().resolve()
    root = ElementTree.Element("mujoco", {"model": "caryam_world"})
    ElementTree.SubElement(root, "compiler", {"autolimits": "true"})
    ElementTree.SubElement(root, "option", {"timestep": "0.002", "gravity": "0 0 -9.81"})
    asset = ElementTree.SubElement(root, "asset")
    worldbody = ElementTree.SubElement(root, "worldbody")
    ElementTree.SubElement(
        worldbody,
        "geom",
        {"name": "ground", "type": "plane", "size": "0 0 0.05", "rgba": "0.18 0.2 0.22 1"},
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
        body = ElementTree.SubElement(
            worldbody,
            "body",
            {
                "name": instance.instance_id,
                "pos": _format_values(instance.position),
                "quat": _format_values(instance.quaternion),
            },
        )
        ElementTree.SubElement(body, "attach", {"model": model_name, "prefix": instance.prefix})

    return ElementTree.ElementTree(root)


def _manifest_value(instance: RobotInstance) -> dict[str, object]:
    value = asdict(instance)
    value["position"] = list(instance.position)
    value["quaternion"] = list(instance.quaternion)
    value["prefix"] = instance.prefix
    return value


def _render_script(manifest_name: str, world_name: str) -> str:
    return f'''"""Render a Caryam world export with MuJoCo."""

from pathlib import Path
import json

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
manifest = json.loads((ROOT / {manifest_name!r}).read_text(encoding="utf-8"))
model = mujoco.MjModel.from_xml_path(str(ROOT / {world_name!r}))
data = mujoco.MjData(model)

for robot in manifest["robots"]:
    prefix = robot["prefix"]
    for joint_name, value in robot["qpos"].items():
        full_name = prefix + joint_name
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, full_name)
        if joint_id < 0:
            continue
        qpos_address = int(model.jnt_qposadr[joint_id])
        if int(model.jnt_type[joint_id]) in {{2, 3}}:
            data.qpos[qpos_address] = float(value)

mujoco.mj_forward(model, data)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
'''


def write_world_export(output_path: str | Path, instances: Iterable[RobotInstance]) -> tuple[Path, Path, Path]:
    """Write ``.xml``, ``.json`` pose data, and a runnable render script."""
    output = Path(output_path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    instance_list = list(instances)
    if not instance_list:
        raise ValueError("There are no imported MuJoCo robots to export")
    world_tree = build_world_xml(instance_list, output)
    ElementTree.indent(world_tree, space="  ")
    world_tree.write(output, encoding="utf-8", xml_declaration=True)

    manifest_path = output.with_suffix(".json")
    manifest_path.write_text(
        json.dumps(
            {"version": 1, "world_xml": output.name, "robots": [_manifest_value(instance) for instance in instance_list]},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    render_path = output.with_name("render_world.py")
    render_path.write_text(_render_script(manifest_path.name, output.name), encoding="utf-8")
    return output, manifest_path, render_path