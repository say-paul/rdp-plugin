"""Read robot-arm definitions from common MuJoCo setup file shapes."""

from __future__ import annotations

import ast
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable
import xml.etree.ElementTree as ElementTree


MODEL_KEYS = (
    "model_path",
    "mjcf_path",
    "xml_path",
    "urdf_path",
    "model",
    "asset_path",
    "asset",
    "file",
    "path",
)
NAME_KEYS = ("name", "robot_name", "display_name", "id")
DESCRIPTION_KEYS = ("description", "label", "comment")
CATEGORY_KEYS = ("category", "type")
SUPPORTED_SETUP_SUFFIXES = {".json", ".py", ".yaml", ".yml", ".xml"}


@dataclass(frozen=True)
class RobotDefinition:
    robot_id: str
    name: str
    model_path: str
    source_path: str
    description: str = ""
    category: str = ""


def _string(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _slug(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "robot-arm"


def _first_string(mapping: dict[str, Any], keys: Iterable[str]) -> str:
    for key in keys:
        value = _string(mapping.get(key))
        if value:
            return value
    return ""


def _looks_like_model_path(value: str) -> bool:
    return Path(value).suffix.lower() in {".xml", ".urdf", ".obj", ".stl", ".ply", ".fbx", ".blend", ".glb", ".gltf"}


def _records_from_value(value: Any, source_path: Path, hint: str = "") -> list[RobotDefinition]:
    records: list[RobotDefinition] = []
    if isinstance(value, list):
        for item in value:
            records.extend(_records_from_value(item, source_path))
        return records
    if not isinstance(value, dict):
        return records

    model_path = _first_string(value, MODEL_KEYS)
    name = _first_string(value, NAME_KEYS) or hint
    if model_path and name:
        robot_id = _first_string(value, ("id", "robot_id")) or _slug(name)
        description = _first_string(value, DESCRIPTION_KEYS)
        category = _first_string(value, CATEGORY_KEYS)
        records.append(RobotDefinition(robot_id, name, model_path, str(source_path), description, category))

    for key, child in value.items():
        if isinstance(child, str) and key not in MODEL_KEYS and _looks_like_model_path(child):
            records.append(RobotDefinition(_slug(key), key, child, str(source_path)))
            continue
        if isinstance(child, (dict, list)):
            child_hint = _string(key) if isinstance(child, dict) else ""
            records.extend(_records_from_value(child, source_path, child_hint))
    return records


def _parse_json(path: Path) -> list[RobotDefinition]:
    value = json.loads(path.read_text(encoding="utf-8"))
    robots = value.get("robots") if isinstance(value, dict) else None
    has_entry_points = isinstance(robots, dict) and any(
        isinstance(robot, dict) and "entry_points" in robot for robot in robots.values()
    )
    if has_entry_points:
        return _parse_menagerie_registry(value, path)
    return _records_from_value(value, path)


def _menagerie_root(registry_path: Path) -> Path:
    candidates = [registry_path.parent, *registry_path.parents]
    for candidate in candidates:
        if candidate.name == "mujoco_menagerie":
            return candidate
        if (candidate / "catalog.py").is_file() and (candidate / "README.md").is_file():
            return candidate
    return registry_path.parent


def _parse_menagerie_registry(value: dict[str, Any], registry_path: Path) -> list[RobotDefinition]:
    root = _menagerie_root(registry_path)
    records: list[RobotDefinition] = []
    for robot_id, robot in sorted(value.get("robots", {}).items()):
        if not isinstance(robot, dict):
            continue
        entry_points = robot.get("entry_points", {})
        if not isinstance(entry_points, dict):
            continue
        entry_name = _string(robot.get("default_model"))
        entry = entry_points.get(entry_name, {})
        if not isinstance(entry, dict) or not _string(entry.get("file")):
            entry = next(
                (candidate for candidate in entry_points.values() if isinstance(candidate, dict) and candidate.get("kind") == "robot"),
                {},
            )
        model_file = _string(entry.get("file")) if isinstance(entry, dict) else ""
        if not model_file:
            continue
        model_path = root / robot_id / model_file
        display_name = _string(robot.get("display_name")) or robot_id.replace("_", " ").title()
        category = _string(robot.get("category"))
        records.append(
            RobotDefinition(
                robot_id,
                display_name,
                str(model_path),
                str(registry_path),
                f"{category} model from MuJoCo Menagerie" if category else "MuJoCo Menagerie model",
                category,
            )
        )
    return records


def _menagerie_display_name(model_dir: Path, xml_path: Path, include_variant: bool) -> str:
    readme = model_dir / "README.md"
    if readme.exists():
        try:
            match = re.search(r"^#\s+(.+?)(?:\s+Description)?\s+\(MJCF\)", readme.read_text(encoding="utf-8"), re.MULTILINE)
        except OSError:
            match = None
        if match:
            name = match.group(1).strip()
            if include_variant and xml_path.stem not in {model_dir.name, "scene"}:
                return f"{name} ({xml_path.stem})"
            return name
    return f"{model_dir.name.replace('_', ' ').title()} ({xml_path.stem})"


def _parse_menagerie_directory(root: Path) -> list[RobotDefinition]:
    records: list[RobotDefinition] = []
    ignored_directories = {".git", ".github", "assets", "opensource", "test", "python"}
    for model_dir in sorted(root.iterdir()):
        if not model_dir.is_dir() or model_dir.name in ignored_directories:
            continue
        xml_files = sorted(model_dir.glob("*.xml"))
        model_files = [
            xml for xml in xml_files
            if not xml.stem.startswith("scene") and not xml.stem.endswith("_mjx")
        ]
        selected_files = model_files or [xml for xml in xml_files if xml.name == "scene.xml"]
        for xml_path in selected_files:
            records.append(
                RobotDefinition(
                    f"{model_dir.name}/{xml_path.stem}",
                    _menagerie_display_name(model_dir, xml_path, len(selected_files) > 1),
                    str(xml_path.resolve()),
                    str(xml_path),
                    "MuJoCo Menagerie model",
                )
            )
    return records


def _find_menagerie_root(path: Path) -> Path | None:
    start = path if path.is_dir() else path.parent
    for candidate in (start, *start.parents):
        if candidate.name == "mujoco_menagerie" or (candidate / "catalog.py").exists():
            return candidate
    return None


def _parse_python(path: Path) -> list[RobotDefinition]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    records: list[RobotDefinition] = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.Assign, ast.AnnAssign, ast.Expr)):
            continue
        value_node = node.value
        try:
            value = ast.literal_eval(value_node)
        except (ValueError, TypeError, SyntaxError):
            continue
        records.extend(_records_from_value(value, path))
    return records


def _parse_yaml(path: Path) -> list[RobotDefinition]:
    try:
        import yaml
    except ImportError:
        return []
    return _records_from_value(yaml.safe_load(path.read_text(encoding="utf-8")), path)


def _parse_xml(path: Path) -> list[RobotDefinition]:
    ElementTree.parse(path)
    return [RobotDefinition(_slug(path.stem), path.stem, path.name, str(path))]


def _parse_file(path: Path) -> list[RobotDefinition]:
    if path.suffix.lower() == ".json":
        return _parse_json(path)
    if path.suffix.lower() == ".py":
        return _parse_python(path)
    if path.suffix.lower() in {".yaml", ".yml"}:
        return _parse_yaml(path)
    if path.suffix.lower() == ".xml":
        return _parse_xml(path)
    return []


def _setup_files(setup_path: Path) -> list[Path]:
    if setup_path.is_file():
        return [setup_path] if setup_path.suffix.lower() in SUPPORTED_SETUP_SUFFIXES else []
    if not setup_path.is_dir():
        return []
    return sorted(path for path in setup_path.rglob("*") if path.is_file() and path.suffix.lower() in SUPPORTED_SETUP_SUFFIXES)


def discover_robot_definitions(setup_path: str | Path) -> list[RobotDefinition]:
    """Discover and de-duplicate robot definitions below a setup path."""
    path = Path(setup_path).expanduser().resolve()
    menagerie_root = _find_menagerie_root(path)
    if menagerie_root is not None:
        registry_files = [file for file in _setup_files(menagerie_root) if file.name == "registry.json"]
        setup_files = registry_files or []
        if not setup_files:
            return _parse_menagerie_directory(menagerie_root)
    else:
        setup_files = _setup_files(path)
    registry_files = [file for file in setup_files if file.name == "registry.json"]
    files_to_parse = registry_files or setup_files
    records: list[RobotDefinition] = []
    for setup_file in files_to_parse:
        try:
            records.extend(_parse_file(setup_file))
        except (OSError, ValueError, SyntaxError, ElementTree.ParseError):
            continue

    unique: list[RobotDefinition] = []
    seen: set[tuple[str, str, str]] = set()
    for record in records:
        key = (record.robot_id, record.name, record.model_path)
        if key not in seen:
            seen.add(key)
            unique.append(record)
    return unique