"""Reversible main-thread Blender viewport integration for external MuJoCo."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import shutil

import bpy
from bpy.app.handlers import persistent
from mathutils import Matrix, Quaternion, Vector

from .. import mounting
from .client import SimulationClient


_session = None
_status = "Stopped"
_updating = False
_closing_clients = []


def active() -> bool:
    return _session is not None or bool(_closing_clients)


def status() -> str:
    return "Stopping…" if _closing_clients else _status


def state() -> str:
    return _session.state if _session is not None else ("STOPPING" if _closing_clients else "IDLE")


def _depth(obj) -> int:
    count = 0
    while obj.parent is not None:
        count += 1
        obj = obj.parent
    return count


def _matrix_close(left, right, tolerance=2e-5) -> bool:
    return all(abs(left[r][c] - right[r][c]) < tolerance for r in range(4) for c in range(4))


def _validate_transform(obj, *, robot=False):
    matrix = obj.matrix_world
    position, rotation, scale = matrix.decompose()
    if not all(math.isfinite(value) for row in matrix for value in row):
        raise ValueError(f"Non-finite transform on {obj.name}")
    if min(scale) <= 1e-8 or matrix.determinant() <= 0:
        raise ValueError(f"Live physics requires positive, nonzero scale: {obj.name}")
    if not _matrix_close(matrix, Matrix.LocRotScale(position, rotation, scale)):
        raise ValueError(f"Live physics cannot represent shear on {obj.name}; apply parent transforms first")
    if robot and any(abs(value - 1) > 1e-5 for value in scale):
        raise ValueError(f"Live MJCF robots require unit world scale: {obj.name}")


@dataclass
class Binding:
    obj: object
    body: str
    scale: object
    offset: object


class AuthoringState:
    def __init__(self, scene, objects):
        self.scene = scene
        self.frame = scene.frame_current
        self.subframe = scene.frame_subframe
        self.objects = []
        self.mutes = []
        self.animations = []
        self.rigid_world = scene.rigidbody_world
        self.rigid_enabled = self.rigid_world.enabled if self.rigid_world else None
        for obj in sorted(objects, key=_depth):
            self.objects.append((obj, obj.parent, obj.matrix_basis.copy(), obj.matrix_parent_inverse.copy(),
                                 float(obj["qpos"]) if "qpos" in obj else None))

    def suspend(self):
        if self.rigid_world:
            self.rigid_world.enabled = False
        for obj, *_ in self.objects:
            animation = obj.animation_data
            if animation:
                self.animations.append((animation, animation.action, animation.use_nla, getattr(animation, "action_slot", None)))
                animation.action = None
                animation.use_nla = False
                for driver in animation.drivers:
                    self.mutes.append((driver, driver.mute))
                    driver.mute = True
            for constraint in obj.constraints:
                self.mutes.append((constraint, constraint.mute))
                constraint.mute = True
        bpy.context.view_layer.update()

    def restore(self):
        for item, muted in self.mutes:
            try:
                item.mute = muted
            except (ReferenceError, RuntimeError):
                pass
        for animation, action, use_nla, slot in self.animations:
            try:
                animation.action = action
                if slot is not None:
                    animation.action_slot = slot
                animation.use_nla = use_nla
            except (ReferenceError, RuntimeError):
                pass
        for obj, parent, basis, inverse, qpos in self.objects:
            try:
                # Restore surviving objects without recreating intentionally deleted data.
                if parent is not None:
                    try:
                        parent.name
                    except ReferenceError:
                        parent = None
                obj.parent = parent
                obj.matrix_parent_inverse = inverse
                obj.matrix_basis = basis
                if qpos is not None:
                    obj["qpos"] = qpos
                    obj.update_tag(refresh={"OBJECT"})
            except (ReferenceError, RuntimeError):
                pass
        try:
            if self.rigid_world:
                self.rigid_world.enabled = self.rigid_enabled
            self.scene.frame_set(self.frame, subframe=self.subframe)
            bpy.context.view_layer.update()
        except (ReferenceError, RuntimeError):
            pass


class Session:
    def __init__(self, scene, robots, meshes, client):
        self.scene = scene
        self.client = client
        self.state = "STARTING"
        self.bindings = []
        self.time = 0.0
        self.lag = 0.0
        self.applied = False
        self.expected = []
        self.names = tuple(sorted(obj.as_pointer() for obj in scene.objects))
        self.settings = (tuple(scene.robot_mujoco_gravity), scene.robot_mujoco_timestep,
                         scene.robot_mujoco_rate, scene.robot_mujoco_python)
        participants = set()
        body_names = set()
        for robot in robots:
            root = scene.objects.get(robot.instance_id)
            if root is None or root.get("import_mode") != "mujoco_compiled":
                raise ValueError(f"{robot.instance_id}: Live physics needs a compiled MJCF preview; reimport with MuJoCo")
            _validate_transform(root, robot=True)
            bodies = [obj for obj in root.children_recursive
                      if "mujoco_body_id" in obj and mounting.robot_root(obj) == root]
            if not bodies:
                raise ValueError(f"{root.name}: no compiled body mappings")
            for obj in bodies:
                _validate_transform(obj, robot=True)
                original_name = str(obj.get("mujoco_body_name", ""))
                if not original_name:
                    raise ValueError(f"{obj.name}: missing original MuJoCo body name; reimport robot")
                name = robot.prefix + original_name
                if name in body_names:
                    raise ValueError(f"Duplicate MuJoCo body binding: {name}")
                body_names.add(name)
                self.bindings.append(Binding(obj, name, Vector((1, 1, 1)), Matrix.Identity(4)))
            participants.add(root)
            participants.update(root.children_recursive)
        for mesh in meshes:
            obj = scene.objects.get(mesh.instance_id)
            if obj is None:
                raise ValueError(f"Exported mesh disappeared: {mesh.instance_id}")
            _validate_transform(obj)
            name = mesh.asset_name
            if name in body_names:
                raise ValueError(f"Duplicate MuJoCo body binding: {name}")
            body_names.add(name)
            self.bindings.append(Binding(obj, name, obj.matrix_world.to_scale(), Matrix.Identity(4)))
            participants.add(obj)
            participants.update(obj.children_recursive)
        if not self.bindings:
            raise ValueError("No supported robots or meshes to simulate")
        for obj in list(participants):
            parent = obj.parent
            while parent is not None:
                participants.add(parent)
                parent = parent.parent
        self.bindings.sort(key=lambda binding: _depth(binding.obj))
        self.body_names = sorted(body_names)
        self.authoring = AuthoringState(scene, participants)
        self.capture_expected()

    def capture_expected(self):
        self.expected = [(obj, obj.name, obj.parent, obj.matrix_basis.copy(), obj.matrix_parent_inverse.copy(),
                          float(obj["qpos"]) if "qpos" in obj else None)
                         for obj, *_ in self.authoring.objects]
        self.physics_signature = self._physics_signature()

    def _physics_signature(self):
        result = []
        for obj, *_ in self.authoring.objects:
            rigid = obj.rigid_body
            mount = obj.caryam_mount
            result.append((obj.hide_render, obj.get("model_path"), obj.get("mujoco_dynamic"),
                           mount.enabled, mount.target, mount.source_anchor,
                           (rigid.type, rigid.enabled, rigid.kinematic, rigid.mass, rigid.friction,
                            rigid.restitution, rigid.collision_shape) if rigid else None))
        return result

    def validate_scene(self):
        if bpy.context.scene != self.scene:
            raise RuntimeError("Scene changed; live physics stopped")
        if self.scene.frame_current != self.authoring.frame or self.scene.frame_subframe != self.authoring.subframe:
            raise RuntimeError("Timeline changed; live physics stopped")
        if tuple(sorted(obj.as_pointer() for obj in self.scene.objects)) != self.names:
            raise RuntimeError("Scene objects changed; Stop and Start a fresh physics snapshot")
        current = (tuple(self.scene.robot_mujoco_gravity), self.scene.robot_mujoco_timestep,
                   self.scene.robot_mujoco_rate, self.scene.robot_mujoco_python)
        if current != self.settings or self._physics_signature() != self.physics_signature:
            raise RuntimeError("Physics settings changed; Start a fresh snapshot")
        for obj, name, parent, basis, inverse, qpos in self.expected:
            if obj.name != name or obj.parent != parent or not _matrix_close(obj.matrix_basis, basis) or not _matrix_close(obj.matrix_parent_inverse, inverse):
                raise RuntimeError(f"Authoring transform changed: {name}; Start a fresh snapshot")
            if qpos is not None and float(obj["qpos"]) != qpos:
                raise RuntimeError("Joint pose changed; Stop physics before editing the authoring pose")

    def apply(self, frame):
        positions = frame["positions"]
        quaternions = frame["quaternions"]
        if len(positions) != len(self.body_names) or len(quaternions) != len(self.body_names):
            raise ValueError("Runtime returned an incomplete body frame")
        poses = {name: Matrix.LocRotScale(Vector(position), Quaternion(quaternion), Vector((1, 1, 1)))
                 for name, position, quaternion in zip(self.body_names, positions, quaternions)}
        for binding in self.bindings:
            obj = binding.obj
            world = poses[binding.body] @ binding.offset @ Matrix.Diagonal((*binding.scale, 1.0))
            # Cancel the real parent frame rather than decomposing a sheared local transform.
            obj.matrix_parent_inverse = obj.parent.matrix_world.inverted() if obj.parent else Matrix.Identity(4)
            obj.matrix_basis = world
            bpy.context.view_layer.update()
        self.last_frame = frame
        self.time = frame["time"]
        self.lag = float(frame.get("lag", 0.0))
        self.applied = True
        self.capture_expected()

    def tick(self):
        self.validate_scene()
        for event in self.client.poll():
            kind = event["type"]
            if kind == "error":
                raise RuntimeError(event["message"])
            if kind == "ready":
                if event["bodies"] != self.body_names:
                    raise ValueError("Runtime body bindings disagree with the exported scene")
                self.authoring.suspend()
                self.capture_expected()
                self.client.command("start")
            elif kind == "ack":
                command = event["command"]
                if command in {"start", "resume"}:
                    self.state = "RUNNING"
                elif command in {"pause", "reset"}:
                    self.state = "PAUSED"
        frame = self.client.latest_frame()
        if frame is not None:
            self.apply(frame)


def start(scene):
    global _session, _status
    if active():
        raise RuntimeError("Stop the active MuJoCo session first")
    from .. import blender_addon as addon
    client = SimulationClient(scene.robot_mujoco_python.strip() or shutil.which("python") or "python",
                              rate_hz=scene.robot_mujoco_rate)
    session = None
    try:
        bpy.context.view_layer.update()
        output = client.directory / "world.xml"
        robots = addon._world_robot_instances()
        meshes = addon._world_scene_meshes(output)
        session = Session(scene, robots, meshes, client)
        request = {"version": 1, "output_path": str(output), "robots": [asdict(robot) for robot in robots],
                   "scene_meshes": [asdict(mesh) for mesh in meshes],
                   "world_options": {"gravity": list(scene.robot_mujoco_gravity), "timestep": scene.robot_mujoco_timestep}}
        _session = session
        _status = "Starting external MuJoCo…"
        client.start(request, session.body_names)
        if not bpy.app.timers.is_registered(_tick):
            bpy.app.timers.register(_tick, first_interval=0.01)
    except Exception:
        _session = None
        if session is not None:
            session.authoring.restore()
        client.close()
        raise


def command(name):
    if _session is None:
        raise RuntimeError("No live MuJoCo session")
    _session.client.command(name)


def stop(reason="Stopped", *, wait=False):
    global _session, _status
    session, _session = _session, None
    if bpy.app.timers.is_registered(_tick):
        bpy.app.timers.unregister(_tick)
    if session is not None:
        session.authoring.restore()
        session.client.close(wait=False)
        _closing_clients.append(session.client)
    if wait:
        for client in _closing_clients:
            client.close(wait=True)
    _reap_clients()
    if _closing_clients and not bpy.app.timers.is_registered(_reap_clients):
        bpy.app.timers.register(_reap_clients, first_interval=.02)
    _status = reason
    _redraw()


def _reap_clients():
    _closing_clients[:] = [client for client in _closing_clients if not client.closed]
    if _closing_clients:
        return .02
    if bpy.app.timers.is_registered(_reap_clients):
        bpy.app.timers.unregister(_reap_clients)
    _redraw()
    return None


def _redraw():
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            area.tag_redraw()


def _tick():
    global _status, _updating
    session = _session
    if session is None:
        return None
    try:
        _updating = True
        session.tick()
        _status = f"{session.state.title()} · {session.time:.3f} s · lag {session.lag:.3f} s"
        _redraw()
    except Exception as error:
        stop(f"Error: {error}")
        return None
    finally:
        _updating = False
    return 1.0 / max(1, session.scene.robot_mujoco_rate)


@persistent
def _stop_for_lifecycle(*_args):
    stop(wait=True)


@persistent
def _geometry_changed(scene, depsgraph):
    if _updating or _session is None or _session.scene != scene:
        return
    participants = {obj for obj, *_ in _session.authoring.objects}
    for update in depsgraph.updates:
        original = getattr(update.id, "original", update.id)
        if isinstance(original, bpy.types.Object) and original in participants and update.is_updated_geometry:
            stop("Geometry changed; Start a fresh physics snapshot")
            return


def register_handlers():
    unregister_handlers()
    for name in ("save_pre", "load_pre", "undo_pre", "redo_pre"):
        getattr(bpy.app.handlers, name).append(_stop_for_lifecycle)
    bpy.app.handlers.depsgraph_update_post.append(_geometry_changed)


def unregister_handlers():
    stop(wait=True)
    for name in ("save_pre", "load_pre", "undo_pre", "redo_pre"):
        handlers = getattr(bpy.app.handlers, name)
        for handler in list(handlers):
            if handler.__module__ == __name__:
                handlers.remove(handler)
    handlers = bpy.app.handlers.depsgraph_update_post
    for handler in list(handlers):
        if handler.__module__ == __name__:
            handlers.remove(handler)


class ROBOT_OT_live_physics(bpy.types.Operator):
    bl_idname = "robot_library.live_physics"
    bl_label = "MuJoCo Live Physics"
    action: bpy.props.EnumProperty(items=(("START", "Start", ""), ("PAUSE", "Pause", ""),
                                         ("RESUME", "Resume", ""), ("RESET", "Reset", ""), ("STOP", "Stop", "")))

    def execute(self, context):
        try:
            if self.action == "START":
                start(context.scene)
            elif self.action == "STOP":
                stop()
            else:
                command(self.action.lower())
        except (OSError, RuntimeError, ValueError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}


def draw(layout, scene):
    settings = layout.column()
    settings.enabled = not active()
    settings.label(text="MuJoCo · external Python", icon="PHYSICS")
    settings.prop(scene, "robot_mujoco_python", text="Python")
    settings.prop(scene, "robot_mujoco_gravity", text="Gravity")
    settings.prop(scene, "robot_mujoco_timestep", text="Timestep (s)")
    settings.prop(scene, "robot_mujoco_rate", text="Viewport Hz")
    layout.label(text="Live in Blender · all exportable scene objects")
    row = layout.row(align=True)
    button = row.row(align=True)
    button.enabled = not active()
    button.operator("robot_library.live_physics", text="Start", icon="PLAY").action = "START"
    button = row.row(align=True)
    button.enabled = state() in {"RUNNING", "PAUSED"}
    paused = state() == "PAUSED"
    button.operator("robot_library.live_physics", text="Resume" if paused else "Pause", icon="PLAY" if paused else "PAUSE").action = "RESUME" if paused else "PAUSE"
    button.operator("robot_library.live_physics", text="Reset", icon="LOOP_BACK").action = "RESET"
    button = row.row(align=True)
    button.enabled = active()
    button.operator("robot_library.live_physics", text="Stop", icon="CANCEL").action = "STOP"
    layout.label(text=status(), icon="ERROR" if status().startswith("Error") else "INFO")
    layout.label(text="Active mesh: dynamic · other meshes: fixed")


class ROBOT_PT_scene_mujoco(bpy.types.Panel):
    bl_label = "MuJoCo World"
    bl_idname = "ROBOT_PT_scene_mujoco"
    bl_space_type = "PROPERTIES"
    bl_region_type = "WINDOW"
    bl_context = "scene"

    def draw(self, context):
        draw(self.layout, context.scene)


CLASSES = (ROBOT_OT_live_physics, ROBOT_PT_scene_mujoco)
