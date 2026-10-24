# Robot Flow Dataflow Graph, ROS Control Bridge, Robot Build OCI/bootc Tab, and Multi-Repo Sensor Menagerie Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the Caryam Blender add-on's linear "Behavior Graph" into a `flow.yaml`-style dataflow graph of named application nodes (sensors → AI models → actions), give the existing "Ros Control" panel a real ROS 1/2 topic/RViz bridge, add a new "Robot Build" sidebar tab that generates a bootc-based OCI Containerfile and can build/flash it with `podman`, and generalize asset discovery into a multi-repo registry that adds a simulator-agnostic virtual sensor catalog (Lidar, Depth Camera) compiled into real MuJoCo sensor data.

## Execution status — 2026-10-23

The feature code for Phases 1–9 is implemented in the working tree. The final validation ran 46 pure-Python tests, the nine Blender smoke scripts listed below, real MuJoCo compilations of an articulated SO-101 wrist mount and an included Talos head mount, and a headless lidar/depth capture. Packaging was checked with `make zip PACKAGE_ZIP=/tmp/caryam-plan-validation.zip` to preserve the pre-existing repository archive. ROS and podman commands were tested with injected runners; live ROS, image builds, and disk flashing were not run. The task checkboxes below retain the original task-by-task TDD and commit instructions and were not bulk-marked as completed. Per-task commits were not created because this working tree already contained edits in the same files before execution.

**Architecture:** Four independent feature tracks share one codebase layout convention already used by this repo: pure-Python, `bpy`-free logic lives in its own module under `robot_arm_library/` and is unit-tested with `pytest`/`unittest`; Blender-specific wiring (scene properties, operators, panels) lives in `robot_arm_library/blender_addon.py` and is exercised by headless `blender --background --python tests/blender_*_test.py` scripts, mirroring `tests/blender_behavior_export_test.py`. Phase 1 and Phase 1.5 establish typed graph ports plus catalog-backed robot/sensor I/O contracts and pre-run link validation; Phases 2–3 surface them as dynamic multi-socket nodes and flow.yaml I/O in Blender. Phase 4 adds a subprocess-backed `ros_bridge.py` module. Phase 5 adds a subprocess-backed `container_builder.py` module. Phases 6–9 add a fourth track: `robot_arm_library/asset_sources.py` generalizes today's single `robot_setup_path` into a multi-repo registry; `robot_arm_library/sensor_spec.py` defines a simulator-agnostic sensor schema modeled on Isaac Sim's camera/lidar attribute catalog; `robot_arm_library/sensor_compilers/mujoco.py` turns a sensor spec into real `<rangefinder>`/`<camera>` MJCF elements; and Blender UI lets a sensor be attached to a specific body of an already-imported robot. All new modules follow the same "pure functions + dependency-injected runner/data" shape as the rest of the codebase so they're testable without Blender, ROS, podman, or MuJoCo installed.

**Tech Stack:** Python ≥3.10, Blender ≥3.6 `bpy` API, `pytest` (config already in `pyproject.toml`), optional `PyYAML` (already an optional/soft dependency pattern in `setup_parser.py`), ROS 1 (`rostopic`, `rviz`) or ROS 2 (`ros2 topic`, `rviz2`) CLIs, `podman` CLI, `bootc`, optional `mujoco>=3.2` + NumPy (already an implicit dependency of `mujoco`, used only inside the generated `render_world.py` script to save depth frames).

**Spec:** No separate spec document exists yet. This plan is derived directly from the project owner's chat requirements and handwritten notes (paraphrased below) plus the current state of `robot_arm_library/`. A follow-up planning exchange (2026-09-20, Plan Mode) added Requirement 4 below about virtual sensors; it was researched via web search and NVIDIA Isaac Sim's public sensor documentation before Phases 6–9 were drafted. Where the requirements were ambiguous, this plan records an explicit **Assumption** and a resolved default — flag any wrong assumption before execution starts.

## Caryam + Intrinsic Core MVP Plan

The separate [Caryam + Intrinsic Core MVP Plan](2026-09-30-caryam-intrinsic-core-mvp.md) defines the optional Core integration, including the dataflow pipeline, stream rules, and runtime boundary.


Source requirements (paraphrased from chat):
1. "Robot Joints Behavior graph is looking weird but it should be a flow graph of how the sensors are connected to each AI, just like `/home/saypaul/git_repos/rdp-demo/custom_bots/franka/flow.yaml` done through an application."
2. "Ros control is basically a ros bridge to connect any ros topics, rviz etc."
3. "Plan to add another side tab to Robot Build which is creating an OCI container to be able to use bootc for final image build and flash in system." Notebook notes add: "Create container (for boot) → OCI name option → use podman."
4. "For virtual sensors (like lidar scan or depth cameras), suggest a similar repo like `mujoco_menagerie`, and the code needs to be available to run the virtual sensor in sim (MuJoCo/Gazebo)." Follow-up clarification narrowed this to: asset discovery must generalize to multiple registered repos (robot **and** sensor menageries, not one hardcoded path); a sensor definition must not be "part of created for MuJoCo, Gazebo" — i.e. it must be simulator-agnostic, with per-simulator compilers; and a sensor attaches to a specific body/site of an already-imported robot.

## Assumptions (confirm or override before execution)

1. **Scope:** implement all three requirements as sequential phases in one plan, since the request bundled all three and this step only asked for a plan.
2. **Port typing:** application nodes get full typed input/output ports with tensor shapes (mirroring `flow.yaml`'s `inputs`/`outputs` blocks), edited in Blender as a comma-separated shape string per port (Blender has no native inline "list of ints" widget).
3. **File format:** Robot Flow graphs gain `.yaml`/`.yml` save/load (flow.yaml-shaped) *in addition to* the existing `.json` format — the JSON format and its schema are unchanged and stay the default.
4. **flow.yaml `dataflow:` semantics:** the example file's `dataflow:` list mixes true node chains (`oak1, oak1_2 → image_transform → smolvla`) with raw external-signal routes that reference names outside the `application:` block (`serial → actuators`, `command → dds_1.command`). Reproducing the latter exactly would require guessing an external tool's parser we don't have source for. This plan treats `dataflow:` as a **regenerated summary** on export (derived from which named ports are wired to which) and, on import, infers `BehaviorLink`s by **matching output port names to input port names** across `application` entries — the same signal-name-matching idea `flow.yaml` already relies on. Exactly reproducing `dds_1.*`/`https_1.*`-style external routing is called out as explicit **out of scope / follow-up**, not guessed at.
5. **ROS bridge transport:** implemented by shelling out to `rostopic`/`ros2 topic`/`rviz`/`rviz2` CLIs (dependency-injectable subprocess runner), not a hand-rolled rosbridge websocket client — Blender's bundled Python has no guaranteed `websocket-client`/`roslibpy`, and the repo already shells out to MuJoCo the same way (`ROBOT_OT_export_and_render`).
6. **Container tooling:** generates a `Containerfile` and shells out to `podman build` / `podman run … bootc install to-disk`, rather than vendoring an OCI-build library — matches the notebook's explicit "use podman" instruction and the existing "delegate to external CLI" pattern.
7. **Sensor catalog repo research:** no community "Menagerie" repo exists specifically for virtual sensors today. `mujoco_menagerie` itself only ships lidar/camera **meshes** (e.g. `anybotics_anymal_c/assets/lidar.obj`), never actual MuJoCo `<sensor>` elements. The closest structural references found are [hakoniwa-mujoco-robots](https://github.com/hakoniwalab/hakoniwa-mujoco-robots) (reusable lidar/RGB-D/IMU components under `config/sensors/`), [Gazebo Sensors](https://gazebosim.org/libs/sensors/) (SDF `camera`/`depth_camera`/`gpu_lidar` types), [gz-mujoco's SDFormat→MJCF sensor mapping](https://github.com/gazebosim/gz-mujoco/issues/16) (`gpu_lidar → sensor/rangefinder`, `depth_camera → body/camera`, `imu → accelerometer + gyro`, `force_torque → force + torque`), and NVIDIA Isaac Sim's [camera/depth](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_camera_depth_sensors.html) and [non-visual](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/assets/usd_assets_nonvisual_sensors.html) sensor catalogs (physically meaningful, renderer-independent attributes: `focalLength`, `horizontalAperture`, `clippingRange` for cameras; `scanRateBaseHz`, channel count, FOV, range for lidar). This plan builds a new catalog convention rather than adopting an existing repo, since none fit the "Blender-addon-discoverable, mujoco_menagerie-shaped" requirement.
8. **Multi-repo asset registry:** `Scene.robot_setup_path` (single string) is **kept** as the always-present "Primary" source for full backward compatibility with existing tests (`tests/blender_import_test.py`, `tests/blender_behavior_export_test.py`, `tests/blender_so101_joint_test.py` all set it directly and must keep working unchanged). A new `Scene.robot_asset_sources` collection holds *additional* sources (robot menageries, sensor menageries, or "auto"-detect both), merged with the primary source at refresh time — this satisfies "generalized to add more repos" without breaking any existing single-path call site.
9. **Sensor spec scope for this plan:** the canonical `SensorDefinition` schema (Phase 6) is simulator-agnostic and Isaac-Sim-attribute-shaped, but only a **MuJoCo compiler** (Phase 8, `sensor_compilers/mujoco.py`) is implemented now. Gazebo SDF and Isaac Sim USD compilers are explicit, deliberate **follow-up work** — the schema is designed so those compilers can be added later as sibling modules without changing `SensorDefinition`. The v1 catalog covers exactly the two sensor types named in the request — **Lidar** (`sensor_spec.LidarSensorSpec`) and **Depth Camera** (`sensor_spec.CameraSensorSpec`) — using the `gz-mujoco` mapping (`gpu_lidar → rangefinder` fan, `depth_camera → camera` + depth render). IMU and force/torque are named in the mapping table for future extension but not compiled in this plan.
10. **Sensor mounting UX:** a sensor is attached to a specific body/site of an already-imported robot (an empty parented under that body, carrying the sensor id and mount transform as custom properties), mirroring how `robot_library.add_robot`/`drag_robot` place robots today — not a free-floating world placement.

## Global Constraints

- Target Python `>=3.10` (from `pyproject.toml`); Blender `>=3.6` (from `bl_info["blender"]` in `robot_arm_library/blender_addon.py`).
- No new hard pip dependencies inside Blender's bundled Python. Any optional dependency (PyYAML) must degrade to a clear, caught error, mirroring `setup_parser.py`'s `_parse_yaml`'s `try: import yaml / except ImportError: return []` pattern.
- All new `bpy`-independent logic lives in its own module under `robot_arm_library/` (no `import bpy` at module scope), so it can run under plain `pytest`. Existing examples: `behavior_graph.py`, `world_exporter.py`, `setup_parser.py`.
- Pure-Python tests run via `pytest tests/<file>.py -v` (config already present in `pyproject.toml`) and must pass under `python -m unittest discover -s tests` too (README-documented command), so keep them `unittest.TestCase`-based like the existing suite.
- Every Blender-dependent behavior change gets a headless smoke script runnable via `blender --background --factory-startup --python tests/<name>.py`, following the exact pattern already in `tests/blender_behavior_export_test.py` (import `robot_arm_library`, call `robot_arm_library.register()`, exercise behavior with `assert`, `print("... passed")`, then `robot_arm_library.unregister()`).
- Preserve backward compatibility of `BehaviorGraph`/`BehaviorNode`/`BehaviorLink` public constructors and existing saved `.json` graphs — the positional-argument call sites in `default_behavior_graph()` and the assertions in `tests/test_behavior_graph.py` must keep working unchanged after Phase 1.
- New Blender operators/panels follow the existing naming convention: `bl_idname` prefixed `robot_library.`, classes prefixed `ROBOT_OT_`/`ROBOT_PT_`, registered by adding to the `CLASSES` tuple in dependency order (a `PropertyGroup` used as another class's property type must appear earlier in `CLASSES`), and any new `bpy.types.Scene` attribute added to `_remove_scene_properties()`'s name list.
- Destructive operators (disk flashing) must call `context.window_manager.invoke_confirm(self, event)` from `invoke()` before `execute()` runs, matching the risk profile of "flash a bootc image to a real disk."
- Default container tool is `podman` (explicit user requirement).
- `robot_arm_library/sensor_spec.py` and `robot_arm_library/sensor_compilers/` stay `bpy`-free, following the same "pure module" rule as `behavior_graph.py`/`world_exporter.py`/`ros_bridge.py`/`container_builder.py`.
- Every new discovery/compiler function returns plain dataclasses/`ElementTree.Element`s, never writes files itself unless it is explicitly an export/write function (mirrors `world_exporter.write_world_export` vs. `build_world_xml`) — keeps unit tests fast and filesystem-free where possible.

---

## Phase 1 — Dataflow Graph Engine (pure Python)

**Objective:** Give `behavior_graph.py` a typed, multi-port "application" node kind and `flow.yaml`-shaped (de)serialization, with zero impact on existing single-port nodes/graphs.

### Entry Criteria
- `robot_arm_library/behavior_graph.py` and `tests/test_behavior_graph.py` are in their current state (as read on 2026-09-20): linear `NODE_KINDS = {"start", "sensor", "ai_model", "condition", "action", "end"}`, single-value `BehaviorLink(source, target)`.
- `pytest` and `python -m unittest discover -s tests` both pass on `main` before starting (baseline green).

### Exit Criteria
- `BehaviorNode` supports optional `handler`, `inputs: tuple[PortSpec, ...]`, `outputs: tuple[PortSpec, ...]` fields; `"application"` is a valid `kind`.
- `BehaviorLink` supports optional `source_port`/`target_port` (default `"value"`), so every existing 2-arg call site keeps working unchanged.
- `BehaviorGraph.validate()` rejects application nodes with no ports and links that reference unknown ports on application nodes.
- `BehaviorGraph.run()` executes application nodes via a new `application_runner` callback that receives/returns **named** port dictionaries, without changing the behavior of `start`/`sensor`/`ai_model`/`condition`/`action`/`end` nodes.
- `BehaviorGraph.to_flow_dict()` / `from_flow_dict()` and `save_flow_yaml()` / `load_flow_yaml()` exist and round-trip application-node graphs.
- All new code has zero `import bpy`.

### Phase Validation Plan
1. Run `pytest tests/test_behavior_graph.py -v` — all pre-existing tests plus all Phase 1 tests below pass.
2. Run `python -m unittest discover -s tests` from repo root — full suite still green (confirms no regression to `test_setup_parser.py` etc.).
3. Manually inspect one round-tripped `.json` file saved by `default_behavior_graph().save(...)` before and after this phase — byte-identical except for the added `"handler": ""`, `"inputs": []`, `"outputs": []` keys per node and `"source_port": "value"`, `"target_port": "value"` keys per link (confirms additive, non-breaking schema change).

### Task 1: Typed ports on `BehaviorNode` and `BehaviorLink`

**Files:**
- Modify: `robot_arm_library/behavior_graph.py`
- Test: `tests/test_behavior_graph.py`

**Interfaces:**
- Produces: `PortSpec(name: str, shape: tuple[int, ...] = ())` with `.to_dict()`/`PortSpec.from_dict()`; `BehaviorNode.input_names() -> tuple[str, ...]`; `BehaviorNode.output_names() -> tuple[str, ...]`; extended `NODE_KINDS` including `"application"`.
- Consumes: nothing new (extends existing `BehaviorNode`/`BehaviorLink`/`BehaviorGraph.to_dict`/`from_dict`).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_behavior_graph.py — add inside BehaviorGraphTests
    def test_application_node_round_trips_ports_and_handler(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode(
                    "camera",
                    "application",
                    "Camera",
                    handler="python scripts/applications/camera.py",
                    outputs=(PortSpec("oak1", (1, 3, 480, 640)),),
                ),
            ],
            links=[],
        )

        restored = BehaviorGraph.from_dict(graph.to_dict())

        camera = restored.node_map()["camera"]
        self.assertEqual(camera.handler, "python scripts/applications/camera.py")
        self.assertEqual(camera.outputs, (PortSpec("oak1", (1, 3, 480, 640)),))
        self.assertEqual(camera.output_names(), ("oak1",))

    def test_application_node_without_ports_is_invalid(self):
        graph = BehaviorGraph(nodes=[BehaviorNode("empty_app", "application")], links=[])

        self.assertIn("Application node has no ports: empty_app", graph.validate())
```

Add the import at the top of the test file: `from robot_arm_library.behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, PortSpec, default_behavior_graph`.

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_behavior_graph.py -k application_node -v`
Expected: FAIL with `ImportError: cannot import name 'PortSpec'` (or `TypeError: __init__() got an unexpected keyword argument 'handler'`).

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/behavior_graph.py

GRAPH_VERSION = 1
NODE_KINDS = {"start", "sensor", "ai_model", "condition", "action", "end", "application"}


@dataclass(frozen=True)
class PortSpec:
    name: str
    shape: tuple[int, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "shape": list(self.shape)}

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "PortSpec":
        shape = value.get("shape", [])
        parsed_shape = tuple(int(part) for part in shape) if isinstance(shape, (list, tuple)) else ()
        return cls(name=str(value.get("name", "")), shape=parsed_shape)


@dataclass
class BehaviorNode:
    node_id: str
    kind: str
    label: str = ""
    data: dict[str, Any] = field(default_factory=dict)
    position: tuple[float, float] = (0.0, 0.0)
    handler: str = ""
    inputs: tuple[PortSpec, ...] = ()
    outputs: tuple[PortSpec, ...] = ()

    def input_names(self) -> tuple[str, ...]:
        return tuple(port.name for port in self.inputs)

    def output_names(self) -> tuple[str, ...]:
        return tuple(port.name for port in self.outputs)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.node_id,
            "kind": self.kind,
            "label": self.label or self.node_id,
            "data": self.data,
            "position": list(self.position),
            "handler": self.handler,
            "inputs": [port.to_dict() for port in self.inputs],
            "outputs": [port.to_dict() for port in self.outputs],
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BehaviorNode":
        position = value.get("position", (0.0, 0.0))
        if not isinstance(position, (list, tuple)) or len(position) != 2:
            position = (0.0, 0.0)
        data = value.get("data", {})
        inputs = tuple(
            PortSpec.from_dict(port) for port in value.get("inputs", []) if isinstance(port, dict)
        )
        outputs = tuple(
            PortSpec.from_dict(port) for port in value.get("outputs", []) if isinstance(port, dict)
        )
        return cls(
            node_id=str(value.get("id", "")),
            kind=str(value.get("kind", "")),
            label=str(value.get("label", "")),
            data=dict(data) if isinstance(data, dict) else {},
            position=(float(position[0]), float(position[1])),
            handler=str(value.get("handler", "")),
            inputs=inputs,
            outputs=outputs,
        )


@dataclass(frozen=True)
class BehaviorLink:
    source: str
    target: str
    source_port: str = "value"
    target_port: str = "value"

    def to_dict(self) -> dict[str, str]:
        return {
            "source": self.source,
            "target": self.target,
            "source_port": self.source_port,
            "target_port": self.target_port,
        }
```

Update `BehaviorGraph.from_dict` to parse the new link fields:

```python
    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "BehaviorGraph":
        nodes = value.get("nodes", [])
        links = value.get("links", [])
        graph = cls(
            name=str(value.get("name", "Robot Behavior")),
            nodes=[BehaviorNode.from_dict(node) for node in nodes if isinstance(node, dict)],
            links=[
                BehaviorLink(
                    str(link.get("source", "")),
                    str(link.get("target", "")),
                    str(link.get("source_port", "value")),
                    str(link.get("target_port", "value")),
                )
                for link in links
                if isinstance(link, dict)
            ],
        )
        errors = graph.validate()
        if errors:
            raise ValueError("Invalid behavior graph: " + "; ".join(errors))
        return graph
```

Update `validate()` to add the ports check (insert after the existing "Unsupported node kind" block, before the link-existence loop):

```python
        errors.extend(
            f"Application node has no ports: {node.node_id}"
            for node in self.nodes
            if node.kind == "application" and not node.inputs and not node.outputs
        )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_behavior_graph.py -v`
Expected: PASS, including all pre-existing tests (`test_default_graph_runs_sensor_ai_and_action`, `test_cycles_are_rejected`, `test_world_export_writes_render_bundle`).

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/behavior_graph.py tests/test_behavior_graph.py
git commit -m "feat(behavior-graph): add typed ports for application nodes"
```

### Task 2: Execute application nodes with named ports in `BehaviorGraph.run()`

**Files:**
- Modify: `robot_arm_library/behavior_graph.py`
- Test: `tests/test_behavior_graph.py`

**Interfaces:**
- Consumes: `PortSpec`, `BehaviorNode.inputs/outputs`, `BehaviorLink.source_port/target_port` from Task 1.
- Produces: `ApplicationRunner = Callable[[BehaviorNode, dict[str, Any]], dict[str, Any]]`; `BehaviorGraph.run(..., application_runner: ApplicationRunner | None = None)`. Also adds a link-port-validity check to `validate()` consumed by Phase 2/3 callers.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_behavior_graph.py — add inside BehaviorGraphTests
    def test_run_executes_application_node_with_named_ports(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode("camera", "application", outputs=(PortSpec("oak1"),)),
                BehaviorNode(
                    "image_transform",
                    "application",
                    inputs=(PortSpec("oak1"),),
                    outputs=(PortSpec("smolvla"),),
                ),
            ],
            links=[BehaviorLink("camera", "image_transform", "oak1", "oak1")],
        )

        def application_runner(node, inputs):
            if node.node_id == "camera":
                return {"oak1": "frame"}
            return {"smolvla": inputs["oak1"] + "-transformed"}

        values = graph.run(lambda data: None, application_runner=application_runner)

        self.assertEqual(values["image_transform"], {"smolvla": "frame-transformed"})

    def test_run_without_application_runner_raises_clear_error(self):
        graph = BehaviorGraph(nodes=[BehaviorNode("app", "application", outputs=(PortSpec("x"),))], links=[])

        with self.assertRaisesRegex(ValueError, "No application runner configured for node: app"):
            graph.run(lambda data: None)

    def test_validate_rejects_unknown_port_on_link(self):
        graph_dict = {
            "nodes": [
                BehaviorNode("app", "application", outputs=(PortSpec("known"),)).to_dict(),
                BehaviorNode("sink", "application", inputs=(PortSpec("known"),)).to_dict(),
            ],
            "links": [BehaviorLink("app", "sink", "missing", "known").to_dict()],
        }
        graph = BehaviorGraph(
            nodes=[BehaviorNode.from_dict(n) for n in graph_dict["nodes"]],
            links=[BehaviorLink(l["source"], l["target"], l["source_port"], l["target_port"]) for l in graph_dict["links"]],
        )

        self.assertIn("Unknown output port 'missing' on app", graph.validate())
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_behavior_graph.py -k "application_node_with_named_ports or without_application_runner or unknown_port_on_link" -v`
Expected: FAIL — `run()` raises `ValueError: Unsupported node kind: application`, and the unknown-port test finds no matching validation error.

- [ ] **Step 3: Write minimal implementation**

Add near the other callback type aliases:

```python
ApplicationRunner = Callable[["BehaviorNode", dict[str, Any]], dict[str, Any]]
```

Extend `validate()` with the port-existence check (add after the ports-check block from Task 1, before the `known_ids` link-existence loop so the messages appear in a sensible order — but functionally order doesn't matter):

```python
        node_map = self.node_map()
        for link in self.links:
            source_node = node_map.get(link.source)
            if source_node is not None and source_node.kind == "application" and link.source_port not in source_node.output_names():
                errors.append(f"Unknown output port '{link.source_port}' on {link.source}")
            target_node = node_map.get(link.target)
            if target_node is not None and target_node.kind == "application" and link.target_port not in target_node.input_names():
                errors.append(f"Unknown input port '{link.target_port}' on {link.target}")
```

(Note: `validate()` already computes `node_map = self.node_map()`/`known_ids` further down for the link-existence checks — reuse that existing `node_map` variable instead of recomputing; just place this new loop right next to it.)

Extend `run()`:

```python
    def run(
        self,
        sensor_reader: SensorReader,
        ai_runner: AIRunner | None = None,
        action_sink: ActionSink | None = None,
        application_runner: ApplicationRunner | None = None,
    ) -> dict[str, Any]:
        errors = self.validate()
        if errors:
            raise ValueError("Invalid behavior graph: " + "; ".join(errors))
        node_map = self.node_map()
        predecessors: dict[str, list[str]] = {node_id: [] for node_id in node_map}
        port_links: dict[str, list[BehaviorLink]] = {node_id: [] for node_id in node_map}
        for link in self.links:
            if link.source in node_map and link.target in node_map:
                predecessors[link.target].append(link.source)
                port_links[link.target].append(link)
        values: dict[str, Any] = {}
        port_outputs: dict[str, dict[str, Any]] = {}
        for node_id in self.execution_order():
            node = node_map[node_id]
            inputs = [values[source] for source in predecessors[node_id] if source in values]
            if node.kind in {"start", "end"}:
                values[node_id] = inputs[0] if inputs else None
            elif node.kind == "sensor":
                values[node_id] = sensor_reader(node.data)
            elif node.kind == "ai_model":
                if ai_runner is None:
                    raise ValueError(f"No AI runner configured for node: {node_id}")
                node_input = inputs[-1] if len(inputs) == 1 else inputs
                values[node_id] = ai_runner(node.data, {"input": node_input, "values": values.copy()})
            elif node.kind == "condition":
                values[node_id] = bool(inputs[-1]) if inputs else bool(node.data.get("value", False))
            elif node.kind == "action":
                action_input = inputs[-1] if len(inputs) == 1 else inputs
                if action_sink is not None:
                    action_sink(node.data, action_input)
                values[node_id] = action_input
            elif node.kind == "application":
                if application_runner is None:
                    raise ValueError(f"No application runner configured for node: {node_id}")
                named_inputs = {
                    link.target_port: port_outputs.get(link.source, {}).get(link.source_port, values.get(link.source))
                    for link in port_links[node_id]
                }
                outputs = application_runner(node, named_inputs)
                port_outputs[node_id] = dict(outputs)
                values[node_id] = outputs
        return values
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_behavior_graph.py -v`
Expected: PASS for all tests, including the pre-existing `test_default_graph_runs_sensor_ai_and_action` (unchanged code path for non-application kinds).

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/behavior_graph.py tests/test_behavior_graph.py
git commit -m "feat(behavior-graph): run application nodes with named port I/O"
```

### Task 3: `flow.yaml`-shaped (de)serialization

**Files:**
- Modify: `robot_arm_library/behavior_graph.py`
- Test: `tests/test_behavior_graph.py`

**Interfaces:**
- Consumes: `PortSpec`, `BehaviorNode`, `BehaviorLink`, `BehaviorGraph.validate()` from Tasks 1–2.
- Produces: `BehaviorGraph.to_flow_dict() -> dict`, `BehaviorGraph.from_flow_dict(value: dict) -> BehaviorGraph`, `BehaviorGraph.save_flow_yaml(path)`, `BehaviorGraph.load_flow_yaml(path) -> BehaviorGraph`. Consumed by Phase 3's Blender save/load operators.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_behavior_graph.py — add near the top
import importlib.util
import tempfile
from pathlib import Path

# tests/test_behavior_graph.py — add inside BehaviorGraphTests
    def test_to_flow_dict_emits_application_entries_and_dataflow_chain(self):
        graph = BehaviorGraph(
            nodes=[
                BehaviorNode("camera", "application", handler="python scripts/camera.py", outputs=(PortSpec("oak1", (1, 3, 480, 640)),)),
                BehaviorNode(
                    "image_transform",
                    "application",
                    handler="python scripts/transform.py",
                    inputs=(PortSpec("oak1", (1, 3, 480, 640)),),
                    outputs=(PortSpec("smolvla", (2, 3, 512, 512)),),
                ),
            ],
            links=[BehaviorLink("camera", "image_transform", "oak1", "oak1")],
        )

        flow = graph.to_flow_dict()

        camera_entry = next(entry for entry in flow["application"] if entry["name"] == "camera")
        self.assertEqual(camera_entry["outputs"], {"oak1": [1, 3, 480, 640]})
        self.assertIn("oak1 \u2192 image_transform \u2192 smolvla", flow["dataflow"])

    def test_from_flow_dict_infers_links_from_matching_port_names(self):
        flow = {
            "application": [
                {"name": "camera", "type": "python", "outputs": {"oak1": [1, 3, 480, 640]}},
                {
                    "name": "image_transform",
                    "type": "python",
                    "inputs": {"oak1": [1, 3, 480, 640]},
                    "outputs": {"smolvla": [2, 3, 512, 512]},
                },
            ],
        }

        graph = BehaviorGraph.from_flow_dict(flow)

        self.assertEqual(len(graph.links), 1)
        link = graph.links[0]
        self.assertEqual((link.source, link.source_port, link.target, link.target_port), ("camera", "oak1", "image_transform", "oak1"))

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML not installed")
    def test_save_and_load_flow_yaml_round_trip(self):
        graph = BehaviorGraph.from_flow_dict(
            {
                "application": [
                    {"name": "camera", "type": "python", "outputs": {"oak1": [1, 3, 480, 640]}},
                    {"name": "image_transform", "type": "python", "inputs": {"oak1": [1, 3, 480, 640]}, "outputs": {"smolvla": []}},
                ],
            }
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "flow.yaml"
            graph.save_flow_yaml(path)

            restored = BehaviorGraph.load_flow_yaml(path)

        self.assertEqual({node.node_id for node in restored.nodes}, {"camera", "image_transform"})
        self.assertEqual(len(restored.links), 1)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_behavior_graph.py -k "flow_dict or flow_yaml" -v`
Expected: FAIL with `AttributeError: type object 'BehaviorGraph' has no attribute 'to_flow_dict'`.

- [ ] **Step 3: Write minimal implementation**

Add to `BehaviorGraph` in `robot_arm_library/behavior_graph.py`:

```python
    def to_flow_dict(self) -> dict[str, Any]:
        """Convert application nodes into a flow.yaml-shaped mapping."""
        applications: list[dict[str, Any]] = []
        for node in self.nodes:
            if node.kind != "application":
                continue
            entry: dict[str, Any] = {"name": node.node_id, "type": "python"}
            if node.handler:
                entry["handler"] = node.handler
            if node.inputs:
                entry["inputs"] = {port.name: list(port.shape) for port in node.inputs}
            if node.outputs:
                entry["outputs"] = {port.name: list(port.shape) for port in node.outputs}
            applications.append(entry)

        dataflow: list[str] = []
        for node in self.nodes:
            if node.kind != "application":
                continue
            incoming_ports = sorted({link.target_port for link in self.links if link.target == node.node_id})
            outgoing_ports = sorted({link.source_port for link in self.links if link.source == node.node_id})
            if not incoming_ports and not outgoing_ports:
                continue
            left = ", ".join(incoming_ports) if incoming_ports else node.node_id
            line = f"{left} \u2192 {node.node_id}"
            if outgoing_ports:
                line += f" \u2192 {', '.join(outgoing_ports)}"
            dataflow.append(line)
        return {"application": applications, "dataflow": dataflow}

    @classmethod
    def from_flow_dict(cls, value: dict[str, Any]) -> "BehaviorGraph":
        """Build a graph from a flow.yaml-shaped mapping.

        Links are inferred by matching output port names to input port names
        across application entries; the ``dataflow`` list (if present) is
        treated as documentation, not the source of truth, because it may
        contain external routing entries (e.g. ``dds_1.command``) that do not
        correspond to any application node.
        """
        applications = value.get("application", [])
        nodes: list[BehaviorNode] = []
        producers: dict[str, tuple[str, str]] = {}
        consumers: list[tuple[str, str, str]] = []
        for index, entry in enumerate(applications):
            if not isinstance(entry, dict):
                continue
            name = str(entry.get("name", f"application_{index}"))
            inputs = tuple(
                PortSpec(str(port_name), tuple(int(v) for v in shape) if isinstance(shape, list) else ())
                for port_name, shape in (entry.get("inputs") or {}).items()
            )
            outputs = tuple(
                PortSpec(str(port_name), tuple(int(v) for v in shape) if isinstance(shape, list) else ())
                for port_name, shape in (entry.get("outputs") or {}).items()
            )
            nodes.append(
                BehaviorNode(
                    node_id=name,
                    kind="application",
                    label=name,
                    handler=str(entry.get("handler", "")),
                    inputs=inputs,
                    outputs=outputs,
                    position=(float(index) * 220.0, 0.0),
                )
            )
            for port in outputs:
                producers[port.name] = (name, port.name)
            for port in inputs:
                consumers.append((name, port.name, port.name))
        links = [
            BehaviorLink(producers[signal][0], target_node, producers[signal][1], target_port)
            for target_node, target_port, signal in consumers
            if signal in producers
        ]
        graph = cls(name=str(value.get("name", "Robot Flow")), nodes=nodes, links=links)
        errors = graph.validate()
        if errors:
            raise ValueError("Invalid flow definition: " + "; ".join(errors))
        return graph

    def save_flow_yaml(self, path: str | Path) -> None:
        try:
            import yaml
        except ImportError as error:
            raise RuntimeError("PyYAML is required to save a flow.yaml file") from error
        Path(path).write_text(
            yaml.safe_dump(self.to_flow_dict(), sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )

    @classmethod
    def load_flow_yaml(cls, path: str | Path) -> "BehaviorGraph":
        try:
            import yaml
        except ImportError as error:
            raise RuntimeError("PyYAML is required to load a flow.yaml file") from error
        value = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError("flow.yaml must contain a mapping")
        return cls.from_flow_dict(value)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_behavior_graph.py -v`
Expected: PASS for all tests (the PyYAML round-trip test auto-skips if `pyyaml` isn't installed in the environment — run `python -m pip install pyyaml` first if you want that test to execute rather than skip).

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/behavior_graph.py tests/test_behavior_graph.py
git commit -m "feat(behavior-graph): add flow.yaml import/export"
```

---

## Phase 1.5 — Catalog-Backed Robot and Sensor I/O Contracts

**Objective:** Make the dataflow graph resolve and validate I/O contracts before any inference runs. Robot state and command ports use one standard template derived from the robot's already-resolved degree of freedom (DOF); sensor ports remain concrete, catalog-defined contracts because each physical or virtual sensor has different outputs, calibration, rates, and simulator requirements.

This is the pure-Python foundation for an OmniGraph-style Blender extension: a sensor's sensing function is independent from its optional visible mesh, a mounted sensor contributes declared output sockets to the graph, and a graph link is legal only when both port contracts are compatible. It deliberately does **not** discover sensor repos, create Blender UI, mount a sensor, or compile simulator elements; those responsibilities remain in Phases 6–9.

### Entry Criteria
- Phase 1 is complete: `PortSpec`, named `BehaviorLink` ports, and `BehaviorGraph.validate()` already exist.
- Robot Control already resolves each imported robot's controllable joint list or DOF, so this phase does not create a per-robot joint catalog.
- No graph validation currently compares the source and target port shapes or semantic contracts.

### Exit Criteria
- `PortSpec` remains backward compatible but can additionally carry optional `dtype`, `semantic`, `unit`, `rate_hz`, and `variable` metadata; existing shape-only ports and saved JSON still load unchanged.
- A pure-Python robot I/O template returns standard state outputs (`joint_positions`, `joint_velocities`, `joint_efforts`) and command inputs (`joint_position_command`) with shape `[1, dof]`, derived from the resolved DOF.
- A pure-Python catalog adapter resolves a concrete sensor's declared ports into graph-node templates without importing `bpy` or requiring a simulator. It represents camera/depth, lidar, IMU, contact/force-torque, tactile, and radar categories as catalog contracts; only Lidar and Depth Camera execution is implemented later in this plan.
- `BehaviorGraph.validate()` reports an actionable error when a linked source and target have incompatible declared tensor shape, data type, semantic type, or units. Unspecified legacy metadata stays permissive for backward compatibility.
- The adapter records declared simulator backend capabilities (for example `mujoco: camera_depth` or `mujoco: rangefinder_fan`) without compiling, rendering, or executing sensor data.

### Contract Rules
- Robot joint state and command vectors always use `[1, dof]`; Robot Control remains the authority for joint names, limits, and the actual live values.
- Sensor output names and contracts are concrete catalog facts. For example, an RGB-D camera exposes `rgb: [1, 3, height, width]` and `depth: [1, 1, height, width]`; a 2D lidar exposes `ranges: [1, samples]` and optionally `angles: [1, samples]`.
- Shape-only ports require an exact match when both shapes are non-empty. `variable: true` allows a catalog-declared variable-length source only when the target explicitly accepts variable data.
- `dtype`, `semantic`, and `unit` must match when both endpoints declare the field. A matching vector length alone must not allow a motor command to connect to a joint-state input.
- Feedback control is represented as per-tick dataflow: Robot State is a source and Robot Command is a sink for the same robot instance. This avoids treating a physical control loop as an invalid same-tick graph cycle.

### Phase Validation Plan
1. Run `pytest tests/test_port_catalog.py tests/test_behavior_graph.py -v` — template resolution and contract mismatch cases pass while all existing graph tests stay green.
2. Run `python -m unittest discover -s tests` — full pure-Python suite remains green.
3. Manually inspect the generated template for a 7-DOF robot and a catalog camera: robot state/command shapes are `[1, 7]`; camera output shapes match its configured resolution.

### Task 3.5: Extend `PortSpec` with compatible optional contract metadata

**Files:**
- Modify: `robot_arm_library/behavior_graph.py`
- Test: `tests/test_behavior_graph.py`

**Interfaces:**
- Produces: `PortSpec(name, shape=(), dtype="", semantic="", unit="", rate_hz=None, variable=False)` and `PortSpec.compatibility_error(target) -> str | None`.
- Consumes: existing `BehaviorNode.inputs`/`outputs` and `BehaviorLink` fields from Phase 1.

- [ ] Add tests that prove shape mismatch, conflicting `dtype`, conflicting `semantic`, and conflicting `unit` are rejected, while an unspecified field on either endpoint preserves legacy permissiveness.
- [ ] Serialize the new fields only when populated; `PortSpec.from_dict()` accepts older shape-only dictionaries unchanged.
- [ ] In `BehaviorGraph.validate()`, resolve the source-output and target-input `PortSpec` for every application-to-application link and append the compatibility error when both endpoints are known.
- [ ] Run `pytest tests/test_behavior_graph.py -v`.

### Task 3.6: Add generic robot templates and catalog-to-node contract resolution

**Files:**
- Create: `robot_arm_library/port_catalog.py`
- Test: `tests/test_port_catalog.py`

**Interfaces:**
- Produces: `robot_joint_state_ports(dof: int) -> tuple[PortSpec, ...]`, `robot_joint_command_ports(dof: int) -> tuple[PortSpec, ...]`, `sensor_port_templates(entry: dict[str, Any]) -> tuple[PortSpec, ...]`, and `catalog_node_template(node_id, entry, direction) -> BehaviorNode`.
- Consumed later by: Phase 2 catalog-node creation, Phase 6 `SensorDefinition` parsing, Phase 7 source discovery, Phase 8 simulator compilers, and Phase 9 sensor mounting.

- [ ] Write tests for a 6-DOF and 7-DOF robot template, asserting standard port names, `[1, dof]` shapes, `float32` data type, and `rad` units for joint positions/commands.
- [ ] Write tests for depth-camera, lidar, IMU, force-torque, tactile, and radar catalog entries. The tests must assert that each category produces concrete output contracts and preserves declared backend capability metadata without invoking a simulator.
- [ ] Implement the adapter as a `bpy`-free parser of normalized catalog dictionaries. It must return clear `ValueError` messages for unsupported categories or missing required dimensions.
- [ ] Build application-node templates with resolved ports so Phase 2 can create Blender sockets from a selected catalog item instead of requiring users to type every sensor port by hand.
- [ ] Run `pytest tests/test_port_catalog.py tests/test_behavior_graph.py -v` and `python -m unittest discover -s tests`.

### Explicit Non-Goals
- This phase does not replace the canonical `SensorDefinition` schema, `registry.json` discovery, multi-repo asset sources, MuJoCo compiler, or Blender mounting UI described in Phases 6–9.
- This phase does not execute handlers, render a camera/depth frame, emit a lidar scan, or provide a Gazebo/Isaac Sim backend.
- This phase does not duplicate Robot Control's resolved joint names, limits, or drivers; it only publishes the standard graph contract derived from DOF.

---

## Phase 2 — Blender Multi-Socket Application Node UI

**Objective:** Let the Blender node editor create/edit "Application" nodes with an arbitrary number of named input/output sockets (backed by Phase 1's `PortSpec` and Phase 1.5 catalog contracts), fixing the "single Value socket" limitation that made the graph look wrong for fan-in/fan-out sensor→AI wiring.

### Entry Criteria
- Phases 1 and 1.5 complete and merged (`PortSpec`, ported `BehaviorNode`/`BehaviorLink`, `application_runner`, catalog port templates, and link-contract validation all exist and are tested).
- `robot_arm_library/blender_addon.py` is in its current state: `CaryamBehaviorNode` has exactly one `NodeSocketFloat` input and one output named `"Value"`.
- A local Blender ≥3.6 executable is available on `PATH` for running headless tests (`blender --version` succeeds).

### Exit Criteria
- `CaryamBehaviorNode` has a `kind` option `"application"` that shows an editable list of input/output ports (name + comma-separated shape) with add/remove buttons, and dynamically creates/destroys matching Blender node sockets.
- `_graph_from_tree()` converts Blender application nodes/links into `BehaviorNode`/`BehaviorLink` objects carrying the correct `handler`, `inputs`, `outputs`, `source_port`, `target_port`.
- `_populate_behavior_tree()` reconstructs Blender application nodes (ports + sockets) and wires links to the correct named sockets from a loaded `BehaviorGraph`.
- The editor can create a node from a catalog template so standard Robot State/Robot Command nodes and selected sensor nodes receive catalog-defined, read-only initial sockets; manually-created application nodes remain editable.
- `_run_behavior_graph()` supplies a default `application_runner` so **Run Graph** still works for application nodes without an external `handler`, and raises a clear, caught error for nodes that do declare a `handler` (external execution is out of scope for this phase).

### Phase Validation Plan
1. Run `pytest tests/test_behavior_graph.py -v` — still green (Phase 2 must not touch Phase 1 files' public behavior).
2. Run `blender --background --factory-startup --python tests/blender_behavior_export_test.py` — still passes unchanged (regression check on the pre-existing single-socket flow).
3. Run the new `blender --background --factory-startup --python tests/blender_application_node_test.py` (Task 6) — passes, proving multi-socket application nodes can be created, wired, saved to the tree, converted to a `BehaviorGraph`, and run.
4. Manual check (documented, not automated): open Blender, **Robot Control → Robot Flow → Open Behavior Editor**, add an Application node from the toolbar, click "Add Input" twice and "Add Output" once, confirm three new sockets appear on the node and can be dragged to connect to another node's sockets.

### Task 4: `CaryamPortSpec` property group and dynamic socket sync on `CaryamBehaviorNode`

**Files:**
- Modify: `robot_arm_library/blender_addon.py`

**Interfaces:**
- Consumes: nothing new from other tasks (this task is additive to the existing `CaryamBehaviorNode` class).
- Produces: `CaryamPortSpec` (PropertyGroup with `port_name: StringProperty`, `shape_csv: StringProperty`, `shape_tuple() -> tuple[int, ...]`); `CaryamBehaviorNode.handler: StringProperty`; `CaryamBehaviorNode.input_ports` / `.output_ports: CollectionProperty(type=CaryamPortSpec)`; `CaryamBehaviorNode.sync_sockets() -> None`. Consumed by Task 5 (UI) and Task 6 (graph conversion).

- [ ] **Step 1: Write the failing (headless) test**

```python
# tests/blender_application_node_test.py
"""Validate dynamic application-node sockets in Blender without a full export cycle."""

import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon

robot_arm_library.register()
scene = bpy.context.scene

tree = bpy.data.node_groups.new("Port Sync Test", "CARYAM_BEHAVIOR_TREE")
scene.robot_behavior_tree = tree
node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
node.kind = "application"

port = node.input_ports.add()
port.port_name = "oak1"
port.shape_csv = "1,3,480,640"
node.sync_sockets()

assert list(node.inputs.keys()) == ["oak1"], list(node.inputs.keys())
assert list(node.outputs.keys()) == [], list(node.outputs.keys())

node.kind = "sensor"
assert list(node.inputs.keys()) == ["value"], list(node.inputs.keys())
assert list(node.outputs.keys()) == ["value"], list(node.outputs.keys())

print("Application node socket sync test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: FAIL — `node.kind = "application"` raises `TypeError` (enum item `'application'` not found) since the `kind` `EnumProperty` doesn't have that option yet, or `AttributeError: 'CaryamBehaviorNode' object has no attribute 'input_ports'`.

- [ ] **Step 3: Write minimal implementation**

Add a new `PropertyGroup` right before `CaryamBehaviorNode` in `robot_arm_library/blender_addon.py`:

```python
class CaryamPortSpec(PropertyGroup):
    port_name: StringProperty(name="Name", default="value")
    shape_csv: StringProperty(name="Shape", default="", description="Comma-separated tensor shape, e.g. 1,3,480,640")

    def shape_tuple(self) -> tuple[int, ...]:
        if not self.shape_csv.strip():
            return ()
        try:
            return tuple(int(part) for part in self.shape_csv.split(","))
        except ValueError:
            return ()
```

Edit `CaryamBehaviorNode`:

```python
class CaryamBehaviorNode(Node):
    bl_idname = "CARYAM_BEHAVIOR_NODE"
    bl_label = "Behavior Node"

    node_id: StringProperty(name="Id", default="node")

    def _on_kind_update(self, context: bpy.types.Context) -> None:
        self.sync_sockets()

    kind: EnumProperty(
        name="Kind",
        items=(
            ("start", "Start", "Begin execution"),
            ("sensor", "Virtual Sensor", "Read a Blender or MuJoCo value"),
            ("ai_model", "AI Model", "Transform sensor data into a decision"),
            ("condition", "Condition", "Evaluate a boolean value"),
            ("action", "Robot Action", "Write a robot joint command"),
            ("application", "Application", "A named application with typed input/output ports"),
            ("end", "End", "Finish execution"),
        ),
        default="sensor",
        update=_on_kind_update,
    )
    sensor_path: StringProperty(name="Sensor", default="joint.qpos")
    model_name: StringProperty(name="Model", default="rule_based")
    robot_id: StringProperty(name="Robot")
    joint_name: StringProperty(name="Joint")
    target: FloatProperty(name="Target", default=0.0)
    handler: StringProperty(name="Handler", default="")
    input_ports: CollectionProperty(type=CaryamPortSpec)
    output_ports: CollectionProperty(type=CaryamPortSpec)

    def sync_sockets(self) -> None:
        if self.kind == "application":
            wanted_inputs = [port.port_name for port in self.input_ports if port.port_name]
            wanted_outputs = [port.port_name for port in self.output_ports if port.port_name]
        else:
            wanted_inputs = ["value"]
            wanted_outputs = ["value"]
        existing_inputs = {socket.name: socket for socket in self.inputs}
        existing_outputs = {socket.name: socket for socket in self.outputs}
        for name, socket in list(existing_inputs.items()):
            if name not in wanted_inputs:
                self.inputs.remove(socket)
        for name in wanted_inputs:
            if name not in existing_inputs:
                self.inputs.new("NodeSocketFloat", name)
        for name, socket in list(existing_outputs.items()):
            if name not in wanted_outputs:
                self.outputs.remove(socket)
        for name in wanted_outputs:
            if name not in existing_outputs:
                self.outputs.new("NodeSocketFloat", name)

    def init(self, context: bpy.types.Context) -> None:
        self.sync_sockets()

    def draw_label(self) -> str:
        return self.label or self.kind.replace("_", " ").title()
```

(Leave `draw_buttons` untouched for now — Task 5 extends it.)

Register `CaryamPortSpec` before `CaryamBehaviorNode` in the `CLASSES` tuple:

```python
CLASSES = (
    RobotLibraryItem,
    CaryamBehaviorTree,
    CaryamPortSpec,
    CaryamBehaviorNode,
    ...
)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: `Application node socket sync test passed` printed, exit code 0.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_application_node_test.py
git commit -m "feat(blender-addon): dynamic multi-socket application nodes"
```

### Task 5: Port add/remove operators and application node UI

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_application_node_test.py`

**Interfaces:**
- Consumes: `CaryamPortSpec`, `CaryamBehaviorNode.sync_sockets()` from Task 4.
- Produces: `ROBOT_OT_add_port` (`bl_idname = "robot_library.add_behavior_port"`), `ROBOT_OT_remove_port` (`bl_idname = "robot_library.remove_behavior_port"`), both with a `direction: EnumProperty({"input", "output"})`; `ROBOT_OT_remove_port` also takes `port_index: IntProperty`.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_application_node_test.py — append before the final print()/unregister()
node2 = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
node2.kind = "application"
bpy.context.space_data = None  # active_node is set via context override below in real UI; call operator methods directly instead

# Exercise the operators' execute() logic directly against the node under test,
# since headless background mode has no active NODE_EDITOR area/context.
node2.input_ports.clear()
entry = node2.input_ports.add()
entry.port_name = "input_1"
node2.sync_sockets()
assert list(node2.inputs.keys()) == ["input_1"]

entry2 = node2.output_ports.add()
entry2.port_name = "output_1"
node2.sync_sockets()
assert list(node2.outputs.keys()) == ["output_1"]

node2.output_ports.remove(0)
node2.sync_sockets()
assert list(node2.outputs.keys()) == []

print("Application node port add/remove test passed")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: PASS already for the `sync_sockets()`-only assertions added above (Task 4 already implemented this), but this step exists to lock in the port-collection behavior *before* wiring operators/UI on top of it — treat a clean pass here as the green baseline for Step 3's UI-only addition. If it fails, re-check Task 4's `sync_sockets()` implementation first.

- [ ] **Step 3: Write minimal implementation**

Add operators (near the other `ROBOT_OT_*` classes, after `ROBOT_OT_add_behavior_node`):

```python
class ROBOT_OT_add_port(Operator):
    bl_idname = "robot_library.add_behavior_port"
    bl_label = "Add Behavior Port"
    bl_options = {"REGISTER", "UNDO"}

    direction: EnumProperty(items=(("input", "Input", ""), ("output", "Output", "")), default="input")

    def execute(self, context: bpy.types.Context) -> set[str]:
        node = context.active_node
        if not isinstance(node, CaryamBehaviorNode) or node.kind != "application":
            self.report({"WARNING"}, "Select an Application node first")
            return {"CANCELLED"}
        ports = node.input_ports if self.direction == "input" else node.output_ports
        entry = ports.add()
        entry.port_name = f"{self.direction}_{len(ports)}"
        node.sync_sockets()
        return {"FINISHED"}


class ROBOT_OT_remove_port(Operator):
    bl_idname = "robot_library.remove_behavior_port"
    bl_label = "Remove Behavior Port"
    bl_options = {"REGISTER", "UNDO"}

    direction: EnumProperty(items=(("input", "Input", ""), ("output", "Output", "")), default="input")
    port_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        node = context.active_node
        if not isinstance(node, CaryamBehaviorNode) or node.kind != "application":
            return {"CANCELLED"}
        ports = node.input_ports if self.direction == "input" else node.output_ports
        if 0 <= self.port_index < len(ports):
            ports.remove(self.port_index)
        node.sync_sockets()
        return {"FINISHED"}
```

Extend `CaryamBehaviorNode.draw_buttons`:

```python
    def draw_buttons(self, context: bpy.types.Context, layout: bpy.types.UILayout) -> None:
        layout.prop(self, "kind", text="")
        if self.kind == "sensor":
            layout.prop(self, "sensor_path")
        elif self.kind == "ai_model":
            layout.prop(self, "model_name")
        elif self.kind == "action":
            layout.prop(self, "robot_id")
            layout.prop(self, "joint_name")
            layout.prop(self, "target")
        elif self.kind == "application":
            layout.prop(self, "handler")
            columns = layout.row()
            input_column = columns.column()
            input_column.label(text="Inputs")
            for index, port in enumerate(self.input_ports):
                port_row = input_column.row(align=True)
                port_row.prop(port, "port_name", text="")
                port_row.prop(port, "shape_csv", text="")
                remove = port_row.operator("robot_library.remove_behavior_port", text="", icon="X")
                remove.direction = "input"
                remove.port_index = index
            add_input = input_column.operator("robot_library.add_behavior_port", text="Add Input", icon="ADD")
            add_input.direction = "input"
            output_column = columns.column()
            output_column.label(text="Outputs")
            for index, port in enumerate(self.output_ports):
                port_row = output_column.row(align=True)
                port_row.prop(port, "port_name", text="")
                port_row.prop(port, "shape_csv", text="")
                remove = port_row.operator("robot_library.remove_behavior_port", text="", icon="X")
                remove.direction = "output"
                remove.port_index = index
            add_output = output_column.operator("robot_library.add_behavior_port", text="Add Output", icon="ADD")
            add_output.direction = "output"
```

Add both operators to `CLASSES` (after `ROBOT_OT_add_behavior_node`):

```python
    ROBOT_OT_add_behavior_node,
    ROBOT_OT_add_port,
    ROBOT_OT_remove_port,
```

Also add `"application"` as a toolbar quick-add option in `ROBOT_PT_behavior_editor.draw()` and to `ROBOT_OT_add_behavior_node.kind`'s items:

```python
# ROBOT_OT_add_behavior_node.kind items — add one entry:
            ("application", "Application", ""),

# ROBOT_PT_behavior_editor.draw() — extend the icon tuple:
        for kind, icon in (("sensor", "DRIVER_SENSOR"), ("ai_model", "LIGHT_DATA"), ("action", "CONSTRAINT_BONE"), ("application", "SCRIPT")):
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: both print statements (`Application node socket sync test passed`, `Application node port add/remove test passed`) print; exit code 0.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_application_node_test.py
git commit -m "feat(blender-addon): add/remove application node ports from the UI"
```

### Task 6: Wire application nodes through `_graph_from_tree`, `_populate_behavior_tree`, and `_run_behavior_graph`

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_application_node_test.py`

**Interfaces:**
- Consumes: `CaryamPortSpec`, `input_ports`/`output_ports`/`handler`/`sync_sockets()` (Task 4/5); `PortSpec`, `BehaviorNode(handler=..., inputs=..., outputs=...)`, `BehaviorLink(source, target, source_port, target_port)`, `BehaviorGraph.run(..., application_runner=...)` (Phase 1).
- Produces: application-node-aware `_graph_from_tree`, `_populate_behavior_tree`, and a default `application_runner` inside `_run_behavior_graph` — consumed by Phase 3's save/load operators.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_application_node_test.py — append before print()/unregister(), reusing `tree`/`node`/`node2` from earlier in the file
from robot_arm_library.behavior_graph import BehaviorGraph, PortSpec

node.node_id = "camera"
node.kind = "application"
node.input_ports.clear()
node.output_ports.clear()
out_port = node.output_ports.add()
out_port.port_name = "oak1"
out_port.shape_csv = "1,3,480,640"
node.sync_sockets()

node2.node_id = "image_transform"
node2.kind = "application"
node2.input_ports.clear()
node2.output_ports.clear()
in_port = node2.input_ports.add()
in_port.port_name = "oak1"
in_port.shape_csv = "1,3,480,640"
out_port2 = node2.output_ports.add()
out_port2.port_name = "smolvla"
node2.sync_sockets()

tree.links.new(node.outputs["oak1"], node2.inputs["oak1"])

graph = blender_addon._graph_from_tree(tree)
assert graph.validate() == [], graph.validate()
camera_node = graph.node_map()["camera"]
assert camera_node.outputs == (PortSpec("oak1", (1, 3, 480, 640)),)
link = graph.links[0]
assert (link.source, link.source_port, link.target, link.target_port) == ("camera", "oak1", "image_transform", "oak1")

scene.robot_behavior_tree = tree
values = blender_addon._run_behavior_graph(graph)
assert values["image_transform"]["smolvla"] is None  # passthrough default runner has no camera output wired in this test

print("Application node graph conversion test passed")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: FAIL — `_graph_from_tree` builds `BehaviorNode`s without `inputs`/`outputs`/`handler` (both come back empty), and `_run_behavior_graph` raises `ValueError: No application runner configured for node: camera`.

- [ ] **Step 3: Write minimal implementation**

Edit `_graph_from_tree` in `robot_arm_library/blender_addon.py`:

```python
def _graph_from_tree(tree: CaryamBehaviorTree) -> BehaviorGraph:
    nodes = []
    for node in tree.nodes:
        if not isinstance(node, CaryamBehaviorNode):
            continue
        if node.kind == "application":
            inputs = tuple(PortSpec(p.port_name, p.shape_tuple()) for p in node.input_ports if p.port_name)
            outputs = tuple(PortSpec(p.port_name, p.shape_tuple()) for p in node.output_ports if p.port_name)
            handler = node.handler
        else:
            inputs = ()
            outputs = ()
            handler = ""
        nodes.append(
            BehaviorNode(
                node.node_id,
                node.kind,
                node.label or node.name,
                _node_data(node),
                (float(node.location.x), float(node.location.y)),
                handler,
                inputs,
                outputs,
            )
        )
    links = []
    for link in tree.links:
        if not (isinstance(link.from_node, CaryamBehaviorNode) and isinstance(link.to_node, CaryamBehaviorNode)):
            continue
        source_port = link.from_socket.name if link.from_node.kind == "application" else "value"
        target_port = link.to_socket.name if link.to_node.kind == "application" else "value"
        links.append(BehaviorLink(link.from_node.node_id, link.to_node.node_id, source_port, target_port))
    return BehaviorGraph(tree.name, nodes, links)
```

Add `PortSpec` to the import line at the top of `blender_addon.py`:

```python
from .behavior_graph import BehaviorGraph, BehaviorLink, BehaviorNode, PortSpec, default_behavior_graph
```

Edit `_populate_behavior_tree`:

```python
def _populate_behavior_tree(tree: CaryamBehaviorTree, graph: BehaviorGraph) -> None:
    tree.nodes.clear()
    node_map: dict[str, CaryamBehaviorNode] = {}
    for item in graph.nodes:
        node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
        node.node_id = item.node_id
        node.kind = item.kind if item.kind in {"start", "sensor", "ai_model", "condition", "action", "end", "application"} else "sensor"
        node.label = item.label
        node.location = item.position
        node.handler = item.handler
        data = item.data
        node.sensor_path = str(data.get("sensor", "joint.qpos"))
        node.model_name = str(data.get("model", "rule_based"))
        node.robot_id = str(data.get("robot_id", ""))
        node.joint_name = str(data.get("joint", ""))
        node.target = float(data.get("target", 0.0))
        if node.kind == "application":
            node.input_ports.clear()
            for port in item.inputs:
                entry = node.input_ports.add()
                entry.port_name = port.name
                entry.shape_csv = ",".join(str(value) for value in port.shape)
            node.output_ports.clear()
            for port in item.outputs:
                entry = node.output_ports.add()
                entry.port_name = port.name
                entry.shape_csv = ",".join(str(value) for value in port.shape)
        node.sync_sockets()
        node_map[item.node_id] = node
    for link in graph.links:
        source = node_map.get(link.source)
        target = node_map.get(link.target)
        if source is None or target is None:
            continue
        source_socket = source.outputs.get(link.source_port) if source.kind == "application" else source.outputs[0]
        target_socket = target.inputs.get(link.target_port) if target.kind == "application" else target.inputs[0]
        if source_socket is not None and target_socket is not None:
            tree.links.new(source_socket, target_socket)
```

(Note: `node.sync_sockets()` is now called unconditionally for every node kind, which is safe and idempotent — for non-application kinds it just (re)creates the single `"value"` socket pair, replacing the old direct `self.inputs.new(...)` calls that `init()` used to do.)

Edit `_run_behavior_graph` to supply a default application runner:

```python
def _run_behavior_graph(graph: BehaviorGraph) -> dict[str, object]:
    fallback_root = _control_root(bpy.context)

    def sensor_reader(data: dict[str, object]) -> object:
        ...  # unchanged

    def ai_runner(data: dict[str, object], context: dict[str, object]) -> object:
        ...  # unchanged

    def action_sink(data: dict[str, object], value: object) -> None:
        ...  # unchanged

    def application_runner(node: BehaviorNode, inputs: dict[str, object]) -> dict[str, object]:
        if node.handler:
            raise ValueError(
                f"Application node '{node.node_id}' declares an external handler "
                f"({node.handler}); running it is not implemented inside Blender yet"
            )
        first_input = next(iter(inputs.values()), None)
        return {port.name: first_input for port in node.outputs}

    values = graph.run(sensor_reader, ai_runner, action_sink, application_runner)
    bpy.context.scene.frame_set(bpy.context.scene.frame_current)
    bpy.context.view_layer.update()
    return values
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_application_node_test.py`
Expected: `Application node graph conversion test passed` prints; exit code 0. Also re-run `blender --background --factory-startup --python tests/blender_behavior_export_test.py` to confirm the pre-existing single-socket flow still passes unchanged.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_application_node_test.py
git commit -m "feat(blender-addon): convert application nodes to/from BehaviorGraph"
```

---

## Phase 3 — flow.yaml File I/O Wiring in Blender

**Objective:** Let **Save**/**Load** in the Robot Flow node editor read/write `.yaml`/`.yml` files using Phase 1's flow.yaml (de)serialization, alongside the existing `.json` format.

### Entry Criteria
- Phase 1 and Phase 2 complete (application nodes exist end-to-end from Blender UI through `BehaviorGraph`).
- `PyYAML` installed in the environment used for the headless test (`python -c "import yaml"` succeeds) — if unavailable, this phase's test still validates the JSON path and the clear-error path for `.yaml` without PyYAML.

### Exit Criteria
- `ROBOT_OT_save_behavior` writes `graph.save_flow_yaml(path)` when `filepath` ends in `.yaml`/`.yml`, else `graph.save(path)` as before.
- `ROBOT_OT_load_behavior` reads `BehaviorGraph.load_flow_yaml(path)` for `.yaml`/`.yml`, else `BehaviorGraph.load(path)` as before.
- Both operators' `filter_glob` includes `*.yaml;*.yml` alongside `*.json` so the file browser shows flow files.
- A `RuntimeError` from a missing PyYAML surfaces as a Blender `self.report({"ERROR"}, ...)` instead of an unhandled traceback.

### Phase Validation Plan
1. Run `blender --background --factory-startup --python tests/blender_flow_yaml_io_test.py` (Task 7) — passes, proving a graph with an application node round-trips through `.yaml` via the actual operators (not just the underlying `BehaviorGraph` methods tested in Phase 1).
2. Manually confirm (documented) that saving to a `.json` path still produces byte-for-byte the same shape as before Phase 1/2/3 (only the additive fields from Task 1 appear).

### Task 7: Extension-aware save/load operators

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_flow_yaml_io_test.py` (new)

**Interfaces:**
- Consumes: `BehaviorGraph.save_flow_yaml`/`load_flow_yaml` (Phase 1 Task 3), `_graph_from_tree`/`_populate_behavior_tree` (Phase 2 Task 6).

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_flow_yaml_io_test.py
"""Validate flow.yaml save/load through the Blender operators."""

import sys
import tempfile
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon

robot_arm_library.register()
scene = bpy.context.scene

tree = bpy.data.node_groups.new("Flow YAML IO Test", "CARYAM_BEHAVIOR_TREE")
scene.robot_behavior_tree = tree
node = tree.nodes.new("CARYAM_BEHAVIOR_NODE")
node.node_id = "camera"
node.kind = "application"
port = node.output_ports.add()
port.port_name = "oak1"
node.sync_sockets()

with tempfile.TemporaryDirectory() as directory:
    path = str(Path(directory) / "flow.yaml")
    result = bpy.ops.robot_library.save_behavior(filepath=path)
    assert result == {"FINISHED"}, result
    assert Path(path).exists()

    new_tree = bpy.data.node_groups.new("Flow YAML IO Test Reloaded", "CARYAM_BEHAVIOR_TREE")
    scene.robot_behavior_tree = new_tree
    result = bpy.ops.robot_library.load_behavior(filepath=path)
    assert result == {"FINISHED"}, result
    loaded_node = next(n for n in new_tree.nodes if isinstance(n, blender_addon.CaryamBehaviorNode) and n.node_id == "camera")
    assert loaded_node.kind == "application"
    assert list(loaded_node.output_ports)[0].port_name == "oak1"

print("flow.yaml save/load operator test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_flow_yaml_io_test.py`
Expected: FAIL — `robot_library.save_behavior` currently always calls `graph.save(self.filepath)`, so it writes JSON content to a `.yaml`-named file; `BehaviorGraph.load(path)` on `load_behavior` then raises `json.JSONDecodeError` when reading that YAML-formatted-but-`.json`-parsed content (or, if PyYAML happens to accept the JSON subset, the subsequent assertions about `output_ports` still fail because `load_behavior` never calls `load_flow_yaml`).

- [ ] **Step 3: Write minimal implementation**

```python
class ROBOT_OT_save_behavior(Operator, ExportHelper):
    bl_idname = "robot_library.save_behavior"
    bl_label = "Save Behavior Graph"
    bl_options = {"REGISTER"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json;*.yaml;*.yml", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        graph = _graph_from_tree(tree)
        errors = graph.validate()
        if errors:
            self.report({"ERROR"}, "; ".join(errors))
            return {"CANCELLED"}
        path = Path(self.filepath)
        try:
            if path.suffix.lower() in {".yaml", ".yml"}:
                graph.save_flow_yaml(path)
            else:
                graph.save(path)
        except RuntimeError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        self.report({"INFO"}, f"Saved {path.name}")
        return {"FINISHED"}


class ROBOT_OT_load_behavior(Operator, ImportHelper):
    bl_idname = "robot_library.load_behavior"
    bl_label = "Load Behavior Graph"
    bl_options = {"REGISTER", "UNDO"}

    filename_ext = ".json"
    filter_glob: StringProperty(default="*.json;*.yaml;*.yml", options={"HIDDEN"})

    def execute(self, context: bpy.types.Context) -> set[str]:
        path = Path(self.filepath)
        try:
            if path.suffix.lower() in {".yaml", ".yml"}:
                graph = BehaviorGraph.load_flow_yaml(path)
            else:
                graph = BehaviorGraph.load(path)
        except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        tree = _behavior_tree(context) or _ensure_behavior_tree(context.scene)
        _populate_behavior_tree(tree, graph)
        return {"FINISHED"}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_flow_yaml_io_test.py`
Expected: `flow.yaml save/load operator test passed` prints; exit code 0. (If PyYAML is missing, `graph.save_flow_yaml` raises `RuntimeError`, the operator returns `{"CANCELLED"}`, and the test's `assert result == {"FINISHED"}` fails loudly — install `pyyaml` in Blender's Python via `<blender-python> -m pip install pyyaml` before running this test.)

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_flow_yaml_io_test.py
git commit -m "feat(blender-addon): save/load Robot Flow graphs as flow.yaml"
```

---

## Phase 4 — ROS Control Bridge

**Objective:** Replace the placeholder "RCS control is not configured" panel with a working bridge that lists ROS topics, echoes one message, publishes one message, and launches RViz, for either ROS 1 or ROS 2, via CLI subprocesses.

### Entry Criteria
- No dependency on Phases 1–3; can start in parallel with them.
- `robot_arm_library/blender_addon.py`'s `ROBOT_PT_rcs_control` panel exists in its current placeholder form (`bl_label = "RCS Control"`, single `layout.label(text="RCS control is not configured", ...)`).

### Exit Criteria
- `robot_arm_library/ros_bridge.py` exists with `list_topics`, `echo_topic`, `publish_topic`, `launch_rviz`, all accepting an injectable `runner`/`popen` callable defaulting to `subprocess.run`/`subprocess.Popen`, and raising `RosCommandError` on nonzero exit codes.
- `ROBOT_PT_rcs_control` (kept at its existing `bl_idname`/tab placement) shows: a ROS 1/ROS 2 distro selector, a "Launch RViz" button, a "List Topics" button, a topic name field, an "Echo Once" button, message-type/payload fields, a "Publish Once" button, and a status label.
- None of the new operators require a real ROS installation to be *registered*; they only fail at `execute()` time if the underlying CLI is missing, with a caught, reported error.

### Phase Validation Plan
1. Run `pytest tests/test_ros_bridge.py -v` — all pure-function tests pass using mocked `subprocess.run`/`Popen`, no real ROS install needed.
2. Run `blender --background --factory-startup --python tests/blender_ros_bridge_test.py` — passes using monkeypatched `ros_bridge` functions (no real ROS install needed in CI).
3. Manual check (documented, requires a real ROS 2 install): on a machine with `ros2` on `PATH` and a running publisher (e.g. `ros2 topic pub /chatter std_msgs/String "data: hi"`), open the **Robot Control → RCS Control** panel, select `ros2`, click **List Topics**, confirm `/chatter` appears in the status text; set **Topic** to `/chatter` and click **Echo Once**, confirm the message payload appears.

### Task 8: `ros_bridge.py` pure CLI-wrapping module

**Files:**
- Create: `robot_arm_library/ros_bridge.py`
- Test: `tests/test_ros_bridge.py`

**Interfaces:**
- Produces: `RosCommandError(RuntimeError)`; `list_topics(distro: str, runner=subprocess.run) -> list[str]`; `echo_topic(distro: str, topic: str, runner=subprocess.run, timeout: float = 3.0) -> str`; `publish_topic(distro: str, topic: str, message_type: str, payload_yaml: str, runner=subprocess.run) -> None`; `launch_rviz(distro: str, popen=subprocess.Popen) -> subprocess.Popen`. Consumed by Task 9.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_ros_bridge.py
import subprocess
import unittest
from unittest.mock import MagicMock

from robot_arm_library.ros_bridge import (
    RosCommandError,
    echo_topic,
    launch_rviz,
    list_topics,
    publish_topic,
)


class RosBridgeTests(unittest.TestCase):
    def test_list_topics_parses_stdout_lines(self):
        runner = MagicMock(return_value=subprocess.CompletedProcess([], 0, stdout="/joint_states\n/tf\n", stderr=""))

        topics = list_topics("ros2", runner=runner)

        self.assertEqual(topics, ["/joint_states", "/tf"])
        runner.assert_called_once_with(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=5.0, check=False
        )

    def test_list_topics_raises_on_failure(self):
        runner = MagicMock(return_value=subprocess.CompletedProcess([], 1, stdout="", stderr="no master"))

        with self.assertRaises(RosCommandError):
            list_topics("ros1", runner=runner)

    def test_echo_topic_returns_trimmed_stdout(self):
        runner = MagicMock(return_value=subprocess.CompletedProcess([], 0, stdout="data: 1.0\n", stderr=""))

        result = echo_topic("ros2", "/joint_states", runner=runner)

        self.assertEqual(result, "data: 1.0")

    def test_publish_topic_invokes_expected_command(self):
        runner = MagicMock(return_value=subprocess.CompletedProcess([], 0, stdout="", stderr=""))

        publish_topic("ros1", "/cmd", "std_msgs/Float64", "data: 0.5", runner=runner)

        runner.assert_called_once_with(
            ["rostopic", "pub", "-1", "/cmd", "std_msgs/Float64", "data: 0.5"],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )

    def test_launch_rviz_starts_detached_process(self):
        popen = MagicMock()

        launch_rviz("ros2", popen=popen)

        popen.assert_called_once_with(["rviz2"], start_new_session=True)

    def test_unsupported_distro_raises_value_error(self):
        with self.assertRaises(ValueError):
            list_topics("ros3", runner=MagicMock())


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_ros_bridge.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_arm_library.ros_bridge'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/ros_bridge.py
"""Thin CLI-based bridge for inspecting and driving ROS 1 or ROS 2 topics."""

from __future__ import annotations

import subprocess


class RosCommandError(RuntimeError):
    """Raised when a ROS CLI command exits with a nonzero status."""


def _topic_list_command(distro: str) -> list[str]:
    if distro == "ros1":
        return ["rostopic", "list"]
    if distro == "ros2":
        return ["ros2", "topic", "list"]
    raise ValueError(f"Unsupported ROS distro: {distro}")


def _topic_echo_command(distro: str, topic: str) -> list[str]:
    if distro == "ros1":
        return ["rostopic", "echo", "-n", "1", topic]
    if distro == "ros2":
        return ["ros2", "topic", "echo", "--once", topic]
    raise ValueError(f"Unsupported ROS distro: {distro}")


def _topic_publish_command(distro: str, topic: str, message_type: str, payload_yaml: str) -> list[str]:
    if distro == "ros1":
        return ["rostopic", "pub", "-1", topic, message_type, payload_yaml]
    if distro == "ros2":
        return ["ros2", "topic", "pub", "--once", topic, message_type, payload_yaml]
    raise ValueError(f"Unsupported ROS distro: {distro}")


def _rviz_command(distro: str) -> list[str]:
    if distro == "ros1":
        return ["rviz"]
    if distro == "ros2":
        return ["rviz2"]
    raise ValueError(f"Unsupported ROS distro: {distro}")


def list_topics(distro: str, runner=subprocess.run) -> list[str]:
    result = runner(_topic_list_command(distro), capture_output=True, text=True, timeout=5.0, check=False)
    if result.returncode != 0:
        raise RosCommandError(result.stderr.strip() or "Failed to list topics")
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def echo_topic(distro: str, topic: str, runner=subprocess.run, timeout: float = 3.0) -> str:
    result = runner(_topic_echo_command(distro, topic), capture_output=True, text=True, timeout=timeout, check=False)
    if result.returncode != 0:
        raise RosCommandError(result.stderr.strip() or f"Failed to echo {topic}")
    return result.stdout.strip()


def publish_topic(distro: str, topic: str, message_type: str, payload_yaml: str, runner=subprocess.run) -> None:
    result = runner(
        _topic_publish_command(distro, topic, message_type, payload_yaml),
        capture_output=True,
        text=True,
        timeout=5.0,
        check=False,
    )
    if result.returncode != 0:
        raise RosCommandError(result.stderr.strip() or f"Failed to publish to {topic}")


def launch_rviz(distro: str, popen=subprocess.Popen) -> subprocess.Popen:
    return popen(_rviz_command(distro), start_new_session=True)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_ros_bridge.py -v`
Expected: PASS for all 6 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/ros_bridge.py tests/test_ros_bridge.py
git commit -m "feat(ros-bridge): add CLI-backed ROS 1/2 topic and RViz helpers"
```

### Task 9: Wire ROS Control panel and operators in Blender

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_ros_bridge_test.py` (new)

**Interfaces:**
- Consumes: `ros_bridge.list_topics`, `.echo_topic`, `.publish_topic`, `.launch_rviz`, `.RosCommandError` (Task 8).
- Produces: `bpy.types.Scene.robot_ros_distro/topic/message_type/payload/status`; `ROBOT_OT_ros_list_topics`, `ROBOT_OT_ros_echo_topic`, `ROBOT_OT_ros_publish_topic`, `ROBOT_OT_ros_launch_rviz`.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_ros_bridge_test.py
"""Validate ROS Control operators in Blender without a real ROS install."""

import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.ros_bridge as ros_bridge

robot_arm_library.register()
scene = bpy.context.scene
scene.robot_ros_distro = "ros2"

ros_bridge.list_topics = lambda distro, runner=None: ["/joint_states", "/tf"]
result = bpy.ops.robot_library.ros_list_topics()
assert result == {"FINISHED"}, result
assert scene.robot_ros_status == "/joint_states; /tf", scene.robot_ros_status

ros_bridge.echo_topic = lambda distro, topic, runner=None, timeout=3.0: "data: 1.0"
scene.robot_ros_topic = "/joint_states"
result = bpy.ops.robot_library.ros_echo_topic()
assert result == {"FINISHED"}, result
assert scene.robot_ros_status == "data: 1.0", scene.robot_ros_status

publish_calls = []
ros_bridge.publish_topic = lambda distro, topic, message_type, payload, runner=None: publish_calls.append((distro, topic, message_type, payload))
scene.robot_ros_message_type = "std_msgs/Float64"
scene.robot_ros_payload = "data: 0.5"
result = bpy.ops.robot_library.ros_publish_topic()
assert result == {"FINISHED"}, result
assert publish_calls == [("ros2", "/joint_states", "std_msgs/Float64", "data: 0.5")]

print("ROS control operator test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_ros_bridge_test.py`
Expected: FAIL with `AttributeError: 'Scene' object has no attribute 'robot_ros_distro'` (properties not registered yet) — or, once properties exist, `RuntimeError: Operator bpy.ops.robot_library.ros_list_topics.poll() failed` because the operator class doesn't exist yet.

- [ ] **Step 3: Write minimal implementation**

Add operators (near `ROBOT_OT_joint_window`, before the `ROBOT_UL_library` class):

```python
class ROBOT_OT_ros_list_topics(Operator):
    bl_idname = "robot_library.ros_list_topics"
    bl_label = "List ROS Topics"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge

        try:
            topics = ros_bridge.list_topics(context.scene.robot_ros_distro)
        except ros_bridge.RosCommandError as error:
            context.scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        context.scene.robot_ros_status = "; ".join(topics) if topics else "No topics found"
        return {"FINISHED"}


class ROBOT_OT_ros_echo_topic(Operator):
    bl_idname = "robot_library.ros_echo_topic"
    bl_label = "Echo ROS Topic"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge

        scene = context.scene
        try:
            message = ros_bridge.echo_topic(scene.robot_ros_distro, scene.robot_ros_topic)
        except ros_bridge.RosCommandError as error:
            scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_ros_status = message
        return {"FINISHED"}


class ROBOT_OT_ros_publish_topic(Operator):
    bl_idname = "robot_library.ros_publish_topic"
    bl_label = "Publish ROS Topic"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge

        scene = context.scene
        try:
            ros_bridge.publish_topic(
                scene.robot_ros_distro, scene.robot_ros_topic, scene.robot_ros_message_type, scene.robot_ros_payload
            )
        except ros_bridge.RosCommandError as error:
            scene.robot_ros_status = f"Error: {error}"
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_ros_status = f"Published to {scene.robot_ros_topic}"
        return {"FINISHED"}


class ROBOT_OT_ros_launch_rviz(Operator):
    bl_idname = "robot_library.ros_launch_rviz"
    bl_label = "Launch RViz"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import ros_bridge

        try:
            ros_bridge.launch_rviz(context.scene.robot_ros_distro)
        except (OSError, FileNotFoundError) as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        return {"FINISHED"}
```

Replace the `ROBOT_PT_rcs_control` panel body:

```python
class ROBOT_PT_rcs_control(Panel):
    bl_label = "ROS Control"
    bl_idname = "ROBOT_PT_rcs_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Control"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_ros_distro", text="ROS")
        layout.operator("robot_library.ros_launch_rviz", text="Launch RViz", icon="RESTRICT_VIEW_OFF")
        layout.operator("robot_library.ros_list_topics", text="List Topics", icon="FILE_REFRESH")
        layout.prop(scene, "robot_ros_topic", text="Topic")
        layout.operator("robot_library.ros_echo_topic", text="Echo Once", icon="PLAY")
        layout.prop(scene, "robot_ros_message_type", text="Type")
        layout.prop(scene, "robot_ros_payload", text="Payload")
        layout.operator("robot_library.ros_publish_topic", text="Publish Once", icon="EXPORT")
        layout.label(text=scene.robot_ros_status)
```

Add to `CLASSES` (after `ROBOT_OT_joint_window`):

```python
    ROBOT_OT_joint_window,
    ROBOT_OT_ros_list_topics,
    ROBOT_OT_ros_echo_topic,
    ROBOT_OT_ros_publish_topic,
    ROBOT_OT_ros_launch_rviz,
```

Add scene properties in `register()` (after `bpy.types.Scene.robot_behavior_tree = ...`):

```python
    bpy.types.Scene.robot_ros_distro = EnumProperty(
        name="ROS", items=(("ros1", "ROS 1", ""), ("ros2", "ROS 2", "")), default="ros2"
    )
    bpy.types.Scene.robot_ros_topic = StringProperty(name="Topic", default="/joint_states")
    bpy.types.Scene.robot_ros_message_type = StringProperty(name="Type", default="std_msgs/Float64")
    bpy.types.Scene.robot_ros_payload = StringProperty(name="Payload", default="data: 0.0")
    bpy.types.Scene.robot_ros_status = StringProperty(name="ROS Status", default="Not checked")
```

Add the five new attribute names to `_remove_scene_properties()`'s tuple:

```python
    for name in (
        "robot_library", "robot_library_index", "robot_library_status", "robot_library_filter",
        "robot_setup_path", "robot_library_auto_spread", "robot_library_include_collisions",
        "robot_library_control_root", "robot_mujoco_python", "robot_behavior_tree",
        "robot_ros_distro", "robot_ros_topic", "robot_ros_message_type", "robot_ros_payload", "robot_ros_status",
    ):
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_ros_bridge_test.py`
Expected: `ROS control operator test passed` prints; exit code 0.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_ros_bridge_test.py
git commit -m "feat(blender-addon): wire ROS Control panel to ros_bridge"
```

---

## Phase 5 — Robot Build Tab (OCI Container + bootc Flash)

**Objective:** Add a new "Robot Build" sidebar tab that generates a bootc-based `Containerfile`, builds it with `podman`, and flashes the resulting image to a target disk via `bootc install to-disk`.

### Entry Criteria
- No dependency on Phases 1–4; can start in parallel with them.
- No `robot_arm_library/container_builder.py` module and no "Robot Build" sidebar category exist yet.

### Exit Criteria
- `robot_arm_library/container_builder.py` exists with `DEFAULT_BASE_IMAGE`, `render_containerfile`, `write_containerfile`, `build_image_command`, `bootc_install_command`.
- A new "Robot Build" sidebar tab (`ROBOT_PT_robot_build`) exposes: OCI image name, base image, tool (`podman`/`docker`, defaulting to `podman`), a "Generate Containerfile" button, a "Build Image" button, a target-disk field, and a "Flash to Disk" button that requires confirmation before running.
- The flash operator never runs without `context.window_manager.invoke_confirm` first.

### Phase Validation Plan
1. Run `pytest tests/test_container_builder.py -v` — passes without podman/bootc installed (pure string/list-building functions).
2. Run `blender --background --factory-startup --python tests/blender_container_build_test.py` — passes with `subprocess.Popen` monkeypatched to a recording stub, proving the operators build the right commands and write the right file without actually invoking podman.
3. Manual check (documented, requires podman + bootc on a Linux host): open **Robot Build**, set an image name, click **Generate Containerfile**, confirm `<blend file directory>/build/Containerfile` exists and starts with `FROM quay.io/centos-bootc/centos-bootc:stream9`; click **Build Image** and confirm `podman images` lists the new tag after the background build finishes.

### Task 10: `container_builder.py` pure command/template module

**Files:**
- Create: `robot_arm_library/container_builder.py`
- Test: `tests/test_container_builder.py`

**Interfaces:**
- Produces: `DEFAULT_BASE_IMAGE: str`; `render_containerfile(base_image: str, packages: list[str], entrypoint: str) -> str`; `write_containerfile(directory: Path, content: str) -> Path`; `build_image_command(tool: str, tag: str, containerfile: Path, context_dir: Path) -> list[str]`; `bootc_install_command(tool: str, image: str, target_disk: str) -> list[str]`. Consumed by Task 11.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_container_builder.py
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.container_builder import (
    DEFAULT_BASE_IMAGE,
    bootc_install_command,
    build_image_command,
    render_containerfile,
    write_containerfile,
)


class ContainerBuilderTests(unittest.TestCase):
    def test_render_containerfile_includes_base_image_and_packages(self):
        content = render_containerfile(DEFAULT_BASE_IMAGE, ["ros-humble-desktop"], "/opt/caryam/entrypoint.sh")

        self.assertIn(f"FROM {DEFAULT_BASE_IMAGE}", content)
        self.assertIn("RUN dnf install -y ros-humble-desktop", content)
        self.assertIn("ENTRYPOINT", content)

    def test_render_containerfile_without_packages_skips_install_line(self):
        content = render_containerfile(DEFAULT_BASE_IMAGE, [], "/opt/caryam/entrypoint.sh")

        self.assertNotIn("dnf install", content)

    def test_write_containerfile_creates_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_containerfile(Path(directory) / "build", "FROM scratch\n")

            self.assertTrue(path.exists())
            self.assertEqual(path.name, "Containerfile")
            self.assertEqual(path.read_text(encoding="utf-8"), "FROM scratch\n")

    def test_build_image_command_uses_requested_tool(self):
        command = build_image_command("podman", "caryam-robot:latest", Path("/tmp/build/Containerfile"), Path("/tmp/build"))

        self.assertEqual(
            command,
            ["podman", "build", "-t", "caryam-robot:latest", "-f", "/tmp/build/Containerfile", "/tmp/build"],
        )

    def test_bootc_install_command_targets_requested_disk(self):
        command = bootc_install_command("podman", "caryam-robot:latest", "/dev/sda")

        self.assertIn("/dev/sda", command)
        self.assertIn("bootc", command)
        self.assertIn("caryam-robot:latest", command)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_container_builder.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_arm_library.container_builder'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/container_builder.py
"""Generate bootc-based OCI container build and flash commands for Caryam robots."""

from __future__ import annotations

from pathlib import Path

DEFAULT_BASE_IMAGE = "quay.io/centos-bootc/centos-bootc:stream9"


def render_containerfile(base_image: str, packages: list[str], entrypoint: str) -> str:
    package_line = f"RUN dnf install -y {' '.join(packages)} && dnf clean all\n" if packages else ""
    return (
        f"FROM {base_image}\n"
        f"{package_line}"
        "COPY caryam_payload/ /opt/caryam/\n"
        f"ENTRYPOINT [{entrypoint!r}]\n"
    )


def write_containerfile(directory: Path, content: str) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "Containerfile"
    path.write_text(content, encoding="utf-8")
    return path


def build_image_command(tool: str, tag: str, containerfile: Path, context_dir: Path) -> list[str]:
    return [tool, "build", "-t", tag, "-f", str(containerfile), str(context_dir)]


def bootc_install_command(tool: str, image: str, target_disk: str) -> list[str]:
    return [
        tool, "run", "--rm", "--privileged", "--pid=host",
        "-v", "/dev:/dev",
        "-v", "/var/lib/containers:/var/lib/containers",
        "--security-opt", "label=type:unconfined_t",
        image, "bootc", "install", "to-disk", "--wipe", target_disk,
    ]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_container_builder.py -v`
Expected: PASS for all 5 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/container_builder.py tests/test_container_builder.py
git commit -m "feat(container-builder): add bootc Containerfile and command builders"
```

### Task 11: "Robot Build" sidebar tab and operators

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_container_build_test.py` (new)

**Interfaces:**
- Consumes: `container_builder.DEFAULT_BASE_IMAGE`, `.render_containerfile`, `.write_containerfile`, `.build_image_command`, `.bootc_install_command` (Task 10); existing `_blend_directory()` helper.
- Produces: `bpy.types.Scene.robot_build_oci_name/base_image/tool/target_disk/status`; `ROBOT_OT_generate_containerfile`, `ROBOT_OT_build_container_image`, `ROBOT_OT_flash_bootc_image`; `ROBOT_PT_robot_build` panel under the "Robot Build" sidebar category.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_container_build_test.py
"""Validate the Robot Build tab operators without invoking real podman/bootc."""

import subprocess
import sys
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon

robot_arm_library.register()
scene = bpy.context.scene
scene.robot_build_oci_name = "caryam-robot"

result = bpy.ops.robot_library.generate_containerfile()
assert result == {"FINISHED"}, result
containerfile = blender_addon._blend_directory() / "build" / "Containerfile"
assert containerfile.exists(), containerfile
assert containerfile.read_text(encoding="utf-8").startswith("FROM ")

recorded_commands = []


def fake_popen(command, **kwargs):
    recorded_commands.append(command)

    class _FakeProcess:
        pid = 0

    return _FakeProcess()


original_popen = subprocess.Popen
subprocess.Popen = fake_popen
try:
    result = bpy.ops.robot_library.build_container_image()
    assert result == {"FINISHED"}, result
    assert recorded_commands[-1][:2] == ["podman", "build"], recorded_commands

    scene.robot_build_target_disk = "/dev/sda"
    result = bpy.ops.robot_library.flash_bootc_image()
    assert result == {"FINISHED"}, result
    assert "/dev/sda" in recorded_commands[-1], recorded_commands
finally:
    subprocess.Popen = original_popen

print("Robot Build container operator test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_container_build_test.py`
Expected: FAIL with `AttributeError: 'Scene' object has no attribute 'robot_build_oci_name'`.

- [ ] **Step 3: Write minimal implementation**

Add near the other `ROBOT_OT_*` classes (after `ROBOT_OT_ros_launch_rviz`):

```python
class ROBOT_OT_generate_containerfile(Operator):
    bl_idname = "robot_library.generate_containerfile"
    bl_label = "Generate Containerfile"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import container_builder

        scene = context.scene
        content = container_builder.render_containerfile(scene.robot_build_base_image, [], "/opt/caryam/entrypoint.sh")
        path = container_builder.write_containerfile(_blend_directory() / "build", content)
        scene.robot_build_status = f"Wrote {path}"
        self.report({"INFO"}, f"Wrote {path}")
        return {"FINISHED"}


class ROBOT_OT_build_container_image(Operator):
    bl_idname = "robot_library.build_container_image"
    bl_label = "Build Container Image"
    bl_options = {"REGISTER"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import container_builder

        scene = context.scene
        build_dir = _blend_directory() / "build"
        containerfile = build_dir / "Containerfile"
        if not containerfile.exists():
            self.report({"ERROR"}, "Generate the Containerfile first")
            return {"CANCELLED"}
        tag = f"{scene.robot_build_oci_name}:latest"
        command = container_builder.build_image_command(scene.robot_build_tool, tag, containerfile, build_dir)
        try:
            subprocess.Popen(command, cwd=str(build_dir), start_new_session=True)
        except OSError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_build_status = f"Building {tag}"
        return {"FINISHED"}


class ROBOT_OT_flash_bootc_image(Operator):
    bl_idname = "robot_library.flash_bootc_image"
    bl_label = "Flash to Disk"
    bl_options = {"REGISTER"}

    def invoke(self, context: bpy.types.Context, event: bpy.types.Event) -> set[str]:
        return context.window_manager.invoke_confirm(self, event)

    def execute(self, context: bpy.types.Context) -> set[str]:
        from . import container_builder

        scene = context.scene
        tag = f"{scene.robot_build_oci_name}:latest"
        command = container_builder.bootc_install_command(scene.robot_build_tool, tag, scene.robot_build_target_disk)
        try:
            subprocess.Popen(command, start_new_session=True)
        except OSError as error:
            self.report({"ERROR"}, str(error))
            return {"CANCELLED"}
        scene.robot_build_status = f"Flashing {tag} to {scene.robot_build_target_disk}"
        return {"FINISHED"}
```

Add the panel (near `ROBOT_PT_rcs_control`):

```python
class ROBOT_PT_robot_build(Panel):
    bl_label = "Container Build"
    bl_idname = "ROBOT_PT_robot_build"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Build"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        layout.prop(scene, "robot_build_oci_name", text="Image Name")
        layout.prop(scene, "robot_build_base_image", text="Base Image")
        layout.prop(scene, "robot_build_tool", text="Tool")
        layout.operator("robot_library.generate_containerfile", text="Generate Containerfile", icon="FILE_NEW")
        layout.operator("robot_library.build_container_image", text="Build Image", icon="PACKAGE")
        layout.separator()
        layout.prop(scene, "robot_build_target_disk", text="Target Disk")
        layout.operator("robot_library.flash_bootc_image", text="Flash to Disk", icon="EXPORT")
        layout.label(text=scene.robot_build_status)
```

Add the `container_builder` import for `DEFAULT_BASE_IMAGE` at the top of `blender_addon.py`:

```python
from . import container_builder
```

Add new classes to `CLASSES` (after `ROBOT_OT_ros_launch_rviz` for operators, after `ROBOT_PT_rcs_control` for the panel):

```python
    ROBOT_OT_ros_launch_rviz,
    ROBOT_OT_generate_containerfile,
    ROBOT_OT_build_container_image,
    ROBOT_OT_flash_bootc_image,
    ...
    ROBOT_PT_rcs_control,
    ROBOT_PT_robot_build,
    ROBOT_PT_behavior_editor,
```

Add scene properties in `register()`:

```python
    bpy.types.Scene.robot_build_oci_name = StringProperty(name="Image", default="caryam-robot")
    bpy.types.Scene.robot_build_base_image = StringProperty(name="Base Image", default=container_builder.DEFAULT_BASE_IMAGE)
    bpy.types.Scene.robot_build_tool = EnumProperty(
        name="Tool", items=(("podman", "Podman", ""), ("docker", "Docker", "")), default="podman"
    )
    bpy.types.Scene.robot_build_target_disk = StringProperty(name="Target Disk", default="/dev/sda")
    bpy.types.Scene.robot_build_status = StringProperty(name="Build Status", default="Not built")
```

Add the five new attribute names to `_remove_scene_properties()`'s tuple, alongside the Phase 4 additions:

```python
        "robot_build_oci_name", "robot_build_base_image", "robot_build_tool",
        "robot_build_target_disk", "robot_build_status",
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_container_build_test.py`
Expected: `Robot Build container operator test passed` prints; exit code 0.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_container_build_test.py
git commit -m "feat(blender-addon): add Robot Build tab for OCI/bootc container flashing"
```

---

## Phase 6 — Simulator-Agnostic Sensor Specification

**Objective:** Model a virtual sensor (Lidar, Depth Camera) the same way `setup_parser.RobotDefinition` already models a robot, but carrying Isaac-Sim-shaped physical attributes instead of just a mesh path, so the same definition can later be compiled for MuJoCo, Gazebo, or Isaac Sim without changing its schema.

### Entry Criteria
- Phase 1.5 defines the shared port-contract and catalog-adapter API consumed by sensor definitions; Phase 6 supplies its concrete sensor-schema and discovery implementation.
- No `robot_arm_library/sensor_spec.py` exists yet; `robot_arm_library/setup_parser.py` only discovers robots.

### Exit Criteria
- `robot_arm_library/sensor_spec.py` exists with `LidarSensorSpec`, `CameraSensorSpec`, and `SensorDefinition` dataclasses, each with `to_dict()`/`from_dict()` JSON round-trip, catalog-resolved output contracts, declarative simulator-backend capabilities, and zero `import bpy`.
- `setup_parser.discover_sensor_definitions(setup_path)` reads a `registry.json` shaped like `{"sensors": {"<id>": {"category": "lidar"|"depth_camera", ...}}}` and returns `SensorDefinition` records, returning `[]` (not raising) for any directory that has no such file — mirroring `discover_robot_definitions`'s tolerant style.
- A documented sensor-menagerie repo convention exists in `README.md` so a user-created `~/git_repos/sensor_menagerie` repo is discoverable by Phase 7's registry.

### Phase Validation Plan
1. Run `pytest tests/test_sensor_spec.py -v` — JSON round-trip and discovery tests pass.
2. Run `python -m unittest discover -s tests` — full suite still green (new file is additive, no existing module is touched).

### Task 12: `LidarSensorSpec`, `CameraSensorSpec`, `SensorDefinition`

**Files:**
- Create: `robot_arm_library/sensor_spec.py`
- Test: `tests/test_sensor_spec.py` (new)

**Interfaces:**
- Produces: `LidarSensorSpec(channels, horizontal_fov, vertical_fov, horizontal_samples, range_min, range_max, scan_rate_hz)`; `CameraSensorSpec(focal_length, horizontal_aperture, vertical_aperture, clipping_near, clipping_far, resolution_width, resolution_height, depth_enabled)` with `.vertical_fov_degrees() -> float`; `SensorDefinition(sensor_id, name, category, source_path, description, lidar, camera)`. Consumed by Task 13 (discovery), Task 17 (MuJoCo compiler), Task 18 (world export), Task 20 (Blender UI).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_sensor_spec.py
import unittest

from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition


class SensorSpecTests(unittest.TestCase):
    def test_lidar_spec_round_trips_through_dict(self):
        spec = LidarSensorSpec(channels=16, horizontal_fov=270.0, horizontal_samples=540, range_max=20.0)

        restored = LidarSensorSpec.from_dict(spec.to_dict())

        self.assertEqual(restored, spec)

    def test_camera_spec_round_trips_and_computes_vertical_fov(self):
        spec = CameraSensorSpec(focal_length=1.93, vertical_aperture=2.453)

        restored = CameraSensorSpec.from_dict(spec.to_dict())

        self.assertEqual(restored, spec)
        self.assertGreater(spec.vertical_fov_degrees(), 0.0)

    def test_sensor_definition_requires_known_category(self):
        with self.assertRaises(ValueError):
            SensorDefinition.from_dict("bad-1", {"category": "sonar"}, "source.json")

    def test_sensor_definition_round_trips_lidar_payload(self):
        entry = {"category": "lidar", "name": "Front Lidar", "lidar": LidarSensorSpec(horizontal_samples=360).to_dict()}

        definition = SensorDefinition.from_dict("lidar-1", entry, "sensor_menagerie/lidar-1/registry.json")

        self.assertEqual(definition.category, "lidar")
        self.assertIsNotNone(definition.lidar)
        self.assertEqual(definition.lidar.horizontal_samples, 360)
        self.assertEqual(definition.to_dict()["lidar"]["horizontal_samples"], 360)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_sensor_spec.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_arm_library.sensor_spec'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/sensor_spec.py
"""Simulator-agnostic virtual sensor specifications (lidar, depth camera).

Physical attribute names follow the same shape NVIDIA Isaac Sim uses for its
USD camera/lidar catalogs (focal_length, horizontal_aperture, clipping_range,
scan_rate_hz, channels, ...) so a single SensorDefinition can later be
compiled for MuJoCo, Gazebo SDF, or Isaac Sim USD without changing this
schema. Only the MuJoCo compiler (robot_arm_library/sensor_compilers/mujoco.py)
ships in this plan; other backends are explicit follow-up work.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

SENSOR_CATEGORIES = {"lidar", "depth_camera"}


@dataclass(frozen=True)
class LidarSensorSpec:
    channels: int = 1
    horizontal_fov: float = 360.0
    vertical_fov: float = 0.0
    horizontal_samples: int = 360
    range_min: float = 0.05
    range_max: float = 30.0
    scan_rate_hz: float = 10.0

    def to_dict(self) -> dict[str, Any]:
        return {
            "channels": self.channels,
            "horizontal_fov": self.horizontal_fov,
            "vertical_fov": self.vertical_fov,
            "horizontal_samples": self.horizontal_samples,
            "range_min": self.range_min,
            "range_max": self.range_max,
            "scan_rate_hz": self.scan_rate_hz,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "LidarSensorSpec":
        return cls(
            channels=int(value.get("channels", 1)),
            horizontal_fov=float(value.get("horizontal_fov", 360.0)),
            vertical_fov=float(value.get("vertical_fov", 0.0)),
            horizontal_samples=int(value.get("horizontal_samples", 360)),
            range_min=float(value.get("range_min", 0.05)),
            range_max=float(value.get("range_max", 30.0)),
            scan_rate_hz=float(value.get("scan_rate_hz", 10.0)),
        )


@dataclass(frozen=True)
class CameraSensorSpec:
    focal_length: float = 1.93
    horizontal_aperture: float = 3.896
    vertical_aperture: float = 2.453
    clipping_near: float = 0.01
    clipping_far: float = 1000.0
    resolution_width: int = 640
    resolution_height: int = 480
    depth_enabled: bool = True

    def to_dict(self) -> dict[str, Any]:
        return {
            "focal_length": self.focal_length,
            "horizontal_aperture": self.horizontal_aperture,
            "vertical_aperture": self.vertical_aperture,
            "clipping_near": self.clipping_near,
            "clipping_far": self.clipping_far,
            "resolution_width": self.resolution_width,
            "resolution_height": self.resolution_height,
            "depth_enabled": self.depth_enabled,
        }

    @classmethod
    def from_dict(cls, value: dict[str, Any]) -> "CameraSensorSpec":
        return cls(
            focal_length=float(value.get("focal_length", 1.93)),
            horizontal_aperture=float(value.get("horizontal_aperture", 3.896)),
            vertical_aperture=float(value.get("vertical_aperture", 2.453)),
            clipping_near=float(value.get("clipping_near", 0.01)),
            clipping_far=float(value.get("clipping_far", 1000.0)),
            resolution_width=int(value.get("resolution_width", 640)),
            resolution_height=int(value.get("resolution_height", 480)),
            depth_enabled=bool(value.get("depth_enabled", True)),
        )

    def vertical_fov_degrees(self) -> float:
        return math.degrees(2.0 * math.atan((self.vertical_aperture / 2.0) / self.focal_length))


@dataclass(frozen=True)
class SensorDefinition:
    sensor_id: str
    name: str
    category: str
    source_path: str
    description: str = ""
    lidar: LidarSensorSpec | None = None
    camera: CameraSensorSpec | None = None

    def to_dict(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "id": self.sensor_id,
            "name": self.name,
            "category": self.category,
            "description": self.description,
        }
        if self.lidar is not None:
            value["lidar"] = self.lidar.to_dict()
        if self.camera is not None:
            value["camera"] = self.camera.to_dict()
        return value

    @classmethod
    def from_dict(cls, sensor_id: str, value: dict[str, Any], source_path: str) -> "SensorDefinition":
        category = str(value.get("category", ""))
        if category not in SENSOR_CATEGORIES:
            raise ValueError(f"Unsupported sensor category: {category}")
        lidar = LidarSensorSpec.from_dict(value["lidar"]) if category == "lidar" and "lidar" in value else None
        camera = CameraSensorSpec.from_dict(value["camera"]) if category == "depth_camera" and "camera" in value else None
        return cls(
            sensor_id=sensor_id,
            name=str(value.get("name", sensor_id)),
            category=category,
            source_path=source_path,
            description=str(value.get("description", "")),
            lidar=lidar,
            camera=camera,
        )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_sensor_spec.py -v`
Expected: PASS for all 4 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/sensor_spec.py tests/test_sensor_spec.py
git commit -m "feat(sensor-spec): add simulator-agnostic lidar/depth-camera specs"
```

### Task 13: `discover_sensor_definitions()` and the sensor-menagerie `registry.json` convention

**Files:**
- Modify: `robot_arm_library/setup_parser.py`
- Modify: `README.md`
- Test: `tests/test_sensor_spec.py`

**Interfaces:**
- Consumes: `SensorDefinition.from_dict` (Task 12).
- Produces: `setup_parser.discover_sensor_definitions(setup_path: str | Path) -> list[SensorDefinition]`. Consumed by Phase 7's `asset_sources.discover_from_sources`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_sensor_spec.py — add inside SensorSpecTests
    def test_discover_sensor_definitions_reads_registry_json(self):
        import tempfile
        from pathlib import Path

        from robot_arm_library.setup_parser import discover_sensor_definitions

        with tempfile.TemporaryDirectory() as directory:
            registry_path = Path(directory) / "registry.json"
            registry_path.write_text(
                '{"sensors": {"lidar-1": {"category": "lidar", "name": "Front Lidar", '
                '"lidar": {"horizontal_samples": 180}}}}',
                encoding="utf-8",
            )

            definitions = discover_sensor_definitions(directory)

        self.assertEqual(len(definitions), 1)
        self.assertEqual(definitions[0].sensor_id, "lidar-1")
        self.assertEqual(definitions[0].lidar.horizontal_samples, 180)

    def test_discover_sensor_definitions_returns_empty_for_missing_registry(self):
        import tempfile

        from robot_arm_library.setup_parser import discover_sensor_definitions

        with tempfile.TemporaryDirectory() as directory:
            self.assertEqual(discover_sensor_definitions(directory), [])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_sensor_spec.py -k discover_sensor_definitions -v`
Expected: FAIL with `ImportError: cannot import name 'discover_sensor_definitions' from 'robot_arm_library.setup_parser'`.

- [ ] **Step 3: Write minimal implementation**

Add to `robot_arm_library/setup_parser.py` (near the bottom, after `discover_robot_definitions`):

```python
from .sensor_spec import SensorDefinition  # add to the existing import block at the top of the file


def discover_sensor_definitions(setup_path: str | Path) -> list[SensorDefinition]:
    """Discover sensor definitions from a sensor-menagerie-shaped directory.

    Expects a ``registry.json`` at the root of ``setup_path`` shaped like
    ``{"sensors": {"<id>": {"category": "lidar"|"depth_camera", ...}}}``,
    mirroring the ``registry.json`` convention ``_parse_menagerie_registry``
    already reads for robots.
    """
    path = Path(setup_path).expanduser().resolve()
    registry_path = path if path.is_file() and path.name == "registry.json" else path / "registry.json"
    if not registry_path.exists():
        return []
    try:
        value = json.loads(registry_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    sensors = value.get("sensors") if isinstance(value, dict) else None
    if not isinstance(sensors, dict):
        return []
    definitions: list[SensorDefinition] = []
    for sensor_id, entry in sorted(sensors.items()):
        if not isinstance(entry, dict):
            continue
        try:
            definitions.append(SensorDefinition.from_dict(sensor_id, entry, str(registry_path)))
        except (ValueError, KeyError):
            continue
    return definitions
```

Document the convention in `README.md` (append a new section after "## Load MuJoCo Menagerie"):

```markdown
## Sensor menagerie repos (virtual sensors)

The Sensor Library panel discovers virtual sensors (Lidar, Depth Camera) the same way the Robot Library discovers MuJoCo Menagerie robots: point an asset source at a directory containing a `registry.json` shaped like:

\```json
{
  "sensors": {
    "front-lidar": {
      "category": "lidar",
      "name": "Front Lidar",
      "lidar": {"horizontal_fov": 270.0, "horizontal_samples": 540, "range_max": 20.0}
    },
    "chest-depth-camera": {
      "category": "depth_camera",
      "name": "Chest Depth Camera",
      "camera": {"focal_length": 1.93, "horizontal_aperture": 3.896, "vertical_aperture": 2.453}
    }
  }
}
\```

Create a repo following this convention (e.g. `~/git_repos/sensor_menagerie`) and add it as an additional asset source with kind `sensor` (see "Multiple asset sources" below) to make its sensors available for **Attach to Selected Body**.
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_sensor_spec.py -v`
Expected: PASS for all 6 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/setup_parser.py README.md tests/test_sensor_spec.py
git commit -m "feat(setup-parser): discover sensor definitions from a registry.json"
```

---

## Phase 7 — Generalized Multi-Repo Asset Source Registry

**Objective:** Let the add-on merge robot and sensor discovery across an arbitrary number of registered repos instead of one hardcoded `robot_setup_path`, while keeping every existing single-path test and workflow unchanged (Assumption 8).

### Entry Criteria
- Phase 6 complete (`sensor_spec.py`, `setup_parser.discover_sensor_definitions` exist and are tested).
- `blender_addon.py` currently has exactly one `Scene.robot_setup_path: StringProperty` and one `scene.robot_library` collection; `_refresh_library()` and `_refresh_active_library()` are the only readers/writers of that path.

### Exit Criteria
- `robot_arm_library/asset_sources.py` exists with `AssetSource(path, kind, label, enabled)` and `discover_from_sources(sources) -> tuple[list[RobotDefinition], list[SensorDefinition]]`.
- `Scene.robot_setup_path` still exists, unchanged in meaning, and is always treated as the "Primary" `kind="auto"` source — every existing headless test that sets it directly keeps passing without modification.
- `Scene.robot_asset_sources: CollectionProperty` holds zero or more additional sources; `ROBOT_OT_add_asset_source`/`ROBOT_OT_remove_asset_source` manage them; `ROBOT_PT_library` shows them in a small list.
- `scene.sensor_library: CollectionProperty(type=SensorLibraryItem)` is populated by the same refresh pass that populates `scene.robot_library`.

### Phase Validation Plan
1. Run `pytest tests/test_asset_sources.py -v` — merge logic tests pass without Blender.
2. Run `blender --background --factory-startup --python tests/blender_import_test.py` — **unchanged file, must still pass**, proving the primary-source backward-compatibility guarantee holds.
3. Run `blender --background --factory-startup --python tests/blender_multi_source_test.py` (new) — proves a second, sensor-kind source populates `scene.sensor_library` alongside the primary robot source.

### Task 14: `AssetSource` and `discover_from_sources()`

**Files:**
- Create: `robot_arm_library/asset_sources.py`
- Test: `tests/test_asset_sources.py` (new)

**Interfaces:**
- Consumes: `setup_parser.discover_robot_definitions`, `setup_parser.discover_sensor_definitions` (Phase 6/existing).
- Produces: `AssetSource(path, kind="auto", label="", enabled=True)`; `discover_from_sources(sources: Iterable[AssetSource]) -> tuple[list[RobotDefinition], list[SensorDefinition]]`. Consumed by Task 15 (`_refresh_libraries`).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_asset_sources.py
import json
import tempfile
import unittest
from pathlib import Path

from robot_arm_library.asset_sources import AssetSource, discover_from_sources


class AssetSourcesTests(unittest.TestCase):
    def test_auto_source_discovers_robots_and_sensors_independently(self):
        with tempfile.TemporaryDirectory() as robot_dir, tempfile.TemporaryDirectory() as sensor_dir:
            (Path(robot_dir) / "robot.json").write_text(
                json.dumps({"name": "Test Arm", "model_path": "arm.xml"}), encoding="utf-8"
            )
            (Path(sensor_dir) / "registry.json").write_text(
                json.dumps({"sensors": {"lidar-1": {"category": "lidar", "name": "Lidar"}}}), encoding="utf-8"
            )

            robots, sensors = discover_from_sources(
                [AssetSource(robot_dir, "auto"), AssetSource(sensor_dir, "auto")]
            )

        self.assertEqual([robot.name for robot in robots], ["Test Arm"])
        self.assertEqual([sensor.sensor_id for sensor in sensors], ["lidar-1"])

    def test_disabled_source_is_skipped(self):
        with tempfile.TemporaryDirectory() as sensor_dir:
            (Path(sensor_dir) / "registry.json").write_text(
                json.dumps({"sensors": {"lidar-1": {"category": "lidar", "name": "Lidar"}}}), encoding="utf-8"
            )

            robots, sensors = discover_from_sources([AssetSource(sensor_dir, "auto", enabled=False)])

        self.assertEqual(robots, [])
        self.assertEqual(sensors, [])

    def test_sensor_only_kind_skips_robot_discovery(self):
        with tempfile.TemporaryDirectory() as robot_dir:
            (Path(robot_dir) / "robot.json").write_text(
                json.dumps({"name": "Test Arm", "model_path": "arm.xml"}), encoding="utf-8"
            )

            robots, sensors = discover_from_sources([AssetSource(robot_dir, "sensor")])

        self.assertEqual(robots, [])
        self.assertEqual(sensors, [])

    def test_unsupported_kind_raises(self):
        with self.assertRaises(ValueError):
            discover_from_sources([AssetSource("/tmp", "unknown")])


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_asset_sources.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_arm_library.asset_sources'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/asset_sources.py
"""Merge robot and sensor discovery results across multiple registered repos."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .sensor_spec import SensorDefinition
from .setup_parser import RobotDefinition, discover_robot_definitions, discover_sensor_definitions

SOURCE_KINDS = {"auto", "robot", "sensor"}


@dataclass(frozen=True)
class AssetSource:
    path: str
    kind: str = "auto"
    label: str = ""
    enabled: bool = True


def discover_from_sources(sources: Iterable[AssetSource]) -> tuple[list[RobotDefinition], list[SensorDefinition]]:
    robots: list[RobotDefinition] = []
    sensors: list[SensorDefinition] = []
    for source in sources:
        if not source.enabled:
            continue
        if source.kind not in SOURCE_KINDS:
            raise ValueError(f"Unsupported asset source kind: {source.kind}")
        if source.kind in {"auto", "robot"}:
            robots.extend(discover_robot_definitions(source.path))
        if source.kind in {"auto", "sensor"}:
            sensors.extend(discover_sensor_definitions(source.path))
    return robots, sensors
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_asset_sources.py -v`
Expected: PASS for all 4 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/asset_sources.py tests/test_asset_sources.py
git commit -m "feat(asset-sources): merge robot/sensor discovery across multiple repos"
```

### Task 15: Wire the registry into Blender scene state and UI

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_multi_source_test.py` (new)

**Interfaces:**
- Consumes: `AssetSource`, `discover_from_sources` (Task 14).
- Produces: `Scene.robot_asset_sources`, `Scene.sensor_library`, `Scene.sensor_library_index`; `_refresh_libraries(scene)` (renames `_refresh_library`); `ROBOT_OT_add_asset_source`, `ROBOT_OT_remove_asset_source`; `SensorLibraryItem`, `RobotAssetSource` PropertyGroups. Consumed by Phase 9's Sensor Library panel.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_multi_source_test.py
"""Validate multi-repo asset discovery without breaking the single-path default."""

import json
import sys
import tempfile
from pathlib import Path

import bpy

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library

robot_arm_library.register()
scene = bpy.context.scene

with tempfile.TemporaryDirectory() as sensor_dir:
    registry_path = Path(sensor_dir) / "registry.json"
    registry_path.write_text(
        json.dumps({"sensors": {"lidar-1": {"category": "lidar", "name": "Front Lidar"}}}), encoding="utf-8"
    )

    scene.robot_setup_path = str(robot_arm_library.blender_addon._addon_directory() / "demo_robot_setup.json")
    source = scene.robot_asset_sources.add()
    source.path = sensor_dir
    source.kind = "sensor"

    bpy.ops.robot_library.refresh()

    assert len(scene.robot_library) > 0, "primary source must still populate robot_library"
    assert len(scene.sensor_library) == 1, [item.sensor_id for item in scene.sensor_library]
    assert scene.sensor_library[0].sensor_id == "lidar-1"

print("Multi-source asset discovery test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_multi_source_test.py`
Expected: FAIL with `AttributeError: 'Scene' object has no attribute 'robot_asset_sources'`.

- [ ] **Step 3: Write minimal implementation**

Add new `PropertyGroup`s near `RobotLibraryItem`:

```python
class SensorLibraryItem(PropertyGroup):
    sensor_id: StringProperty()
    name: StringProperty()
    category: StringProperty()
    source_path: StringProperty()
    description: StringProperty()


class RobotAssetSource(PropertyGroup):
    path: StringProperty(name="Path", default="")
    kind: EnumProperty(
        name="Kind",
        items=(("auto", "Auto", ""), ("robot", "Robot", ""), ("sensor", "Sensor", "")),
        default="auto",
    )
    label: StringProperty(name="Label", default="")
    enabled: BoolProperty(name="Enabled", default=True)
```

Add both to `CLASSES` (before `ROBOT_PT_library`, alongside `RobotLibraryItem`):

```python
CLASSES = (
    RobotLibraryItem,
    SensorLibraryItem,
    RobotAssetSource,
    CaryamBehaviorTree,
    ...
```

Rename `_refresh_library` to `_refresh_libraries` and rewrite its body:

```python
def _refresh_libraries(scene: bpy.types.Scene) -> None:
    from .asset_sources import AssetSource, discover_from_sources

    sources = [AssetSource(str(_resolve_setup_path(scene.robot_setup_path)), "auto", "Primary")]
    for item in scene.robot_asset_sources:
        if not item.path.strip():
            continue
        sources.append(
            AssetSource(str(_resolve_setup_path(item.path)), item.kind, item.label or item.path, item.enabled)
        )

    scene.robot_library.clear()
    scene.sensor_library.clear()
    try:
        robots, sensors = discover_from_sources(sources)
    except Exception as error:
        scene.robot_library_status = f"Cannot read setup: {error}"
        return

    for definition in robots:
        item = scene.robot_library.add()
        item.robot_id = definition.robot_id
        item.name = definition.name
        item.model_path = definition.model_path
        item.source_path = definition.source_path
        item.description = definition.description
        item.category = definition.category

    for definition in sensors:
        item = scene.sensor_library.add()
        item.sensor_id = definition.sensor_id
        item.name = definition.name
        item.category = definition.category
        item.source_path = definition.source_path
        item.description = definition.description

    status_parts = []
    if robots:
        status_parts.append(f"{len(robots)} robot(s)")
    if sensors:
        status_parts.append(f"{len(sensors)} sensor(s)")
    scene.robot_library_status = f"Found {' and '.join(status_parts)}" if status_parts else "No robots or sensors found"
    scene.robot_library_index = min(scene.robot_library_index, max(len(scene.robot_library) - 1, 0))
    scene.sensor_library_index = min(scene.sensor_library_index, max(len(scene.sensor_library) - 1, 0))
```

Update the two call sites (`ROBOT_OT_refresh.execute` and `_refresh_active_library`) from `_refresh_library(...)` to `_refresh_libraries(...)`.

Add the add/remove operators (near `ROBOT_OT_clear_filter`):

```python
class ROBOT_OT_add_asset_source(Operator):
    bl_idname = "robot_library.add_asset_source"
    bl_label = "Add Asset Source"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context: bpy.types.Context) -> set[str]:
        item = context.scene.robot_asset_sources.add()
        item.path = str(Path.home() / "git_repos" / "sensor_menagerie")
        item.kind = "sensor"
        item.label = "New Source"
        return {"FINISHED"}


class ROBOT_OT_remove_asset_source(Operator):
    bl_idname = "robot_library.remove_asset_source"
    bl_label = "Remove Asset Source"
    bl_options = {"REGISTER", "UNDO"}

    source_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        sources = context.scene.robot_asset_sources
        if 0 <= self.source_index < len(sources):
            sources.remove(self.source_index)
        return {"FINISHED"}
```

Add both to `CLASSES` (after `ROBOT_OT_clear_filter`). Extend `ROBOT_PT_library.draw()` — after the existing `path_row`/`filter_row` block, add:

```python
        sources_box = layout.box()
        sources_box.label(text="Additional Asset Sources")
        for index, source in enumerate(scene.robot_asset_sources):
            source_row = sources_box.row(align=True)
            source_row.prop(source, "enabled", text="")
            source_row.prop(source, "path", text="")
            source_row.prop(source, "kind", text="")
            remove = source_row.operator("robot_library.remove_asset_source", text="", icon="X")
            remove.source_index = index
        sources_box.operator("robot_library.add_asset_source", text="Add Source", icon="ADD")
```

Add scene properties in `register()` (after `bpy.types.Scene.robot_library = ...`):

```python
    bpy.types.Scene.robot_asset_sources = CollectionProperty(type=RobotAssetSource)
    bpy.types.Scene.sensor_library = CollectionProperty(type=SensorLibraryItem)
    bpy.types.Scene.sensor_library_index = IntProperty(name="Sensor", default=0, min=0)
```

Add the three new names to `_remove_scene_properties()`'s tuple, alongside the existing entries:

```python
        "robot_asset_sources", "sensor_library", "sensor_library_index",
```

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_multi_source_test.py`
Expected: `Multi-source asset discovery test passed` prints; exit code 0. Also re-run `blender --background --factory-startup --python tests/blender_import_test.py` to confirm the primary-source path is unaffected.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_multi_source_test.py
git commit -m "feat(blender-addon): generalize asset discovery to multiple registered sources"
```

---

## Phase 8 — MuJoCo Sensor Compiler (Lidar Fan + Depth Camera)

**Objective:** Turn a `SensorDefinition` plus a mount name into real MJCF elements, and extend the existing world export/render pipeline to actually produce lidar scans and depth frames.

### Entry Criteria
- Phase 6 complete (`sensor_spec.py` exists and is tested).
- `robot_arm_library/world_exporter.py` is in its current state: `RobotInstance` has no `sensors` field; `build_world_xml` only emits `<asset><model>` + `<worldbody><body><attach>`.

### Exit Criteria
- `robot_arm_library/sensor_compilers/mujoco.py` exists with `compile_lidar(spec, mount_name) -> tuple[list[Element], list[Element]]` (site elements, `<rangefinder>` elements) and `compile_depth_camera(spec, mount_name) -> Element` (`<camera>` element), both pure and `bpy`-free.
- `RobotInstance` gains an optional `sensors: tuple[SensorMount, ...] = ()` field; `build_world_xml` inserts a child `<body>` per mount under the robot's body, appends compiled `<site>` elements there, and collects `<rangefinder>` elements into a single top-level `<sensor>` block.
- `render_world.py`'s generated script prints lidar scan readings and saves a `.npy` depth frame per attached sensor, in addition to its existing pose-restoring/viewer behavior.
- Existing `test_world_export_writes_render_bundle` (in `tests/test_behavior_graph.py`) keeps passing unchanged (`sensors` defaults to `()`).

### Phase Validation Plan
1. Run `pytest tests/test_sensor_compilers_mujoco.py -v` — structural checks (site/rangefinder counts, names, `fovy` math) pass without MuJoCo installed.
2. Run `pytest tests/test_world_exporter.py -v` — exported `world.xml` contains the expected `<rangefinder>`/`<camera>` elements and the manifest JSON carries sensor mount data.
3. Run `pytest tests/test_behavior_graph.py -v` — the pre-existing `test_world_export_writes_render_bundle` still passes (no `sensors` argument needed).
4. Manual check (documented, requires `mujoco>=3.2` installed): export a world with an attached lidar/depth camera, run the generated `render_world.py`, and confirm it prints a lidar scan list and writes a `<mount_name>_depth.npy` file before opening the viewer.

### Task 16: `sensor_compilers/mujoco.py`

**Files:**
- Create: `robot_arm_library/sensor_compilers/__init__.py` (empty)
- Create: `robot_arm_library/sensor_compilers/mujoco.py`
- Test: `tests/test_sensor_compilers_mujoco.py` (new)

**Interfaces:**
- Consumes: `LidarSensorSpec`, `CameraSensorSpec` (Phase 6).
- Produces: `compile_lidar(spec, mount_name) -> tuple[list[ElementTree.Element], list[ElementTree.Element]]`; `compile_depth_camera(spec, mount_name) -> ElementTree.Element`. Consumed by Task 17 (`world_exporter.py`).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_sensor_compilers_mujoco.py
import unittest

from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec
from robot_arm_library.sensor_compilers.mujoco import compile_depth_camera, compile_lidar


class MujocoSensorCompilerTests(unittest.TestCase):
    def test_compile_lidar_creates_one_site_and_rangefinder_per_sample(self):
        spec = LidarSensorSpec(horizontal_fov=90.0, horizontal_samples=4, range_max=12.0)

        sites, rangefinders = compile_lidar(spec, "front_lidar")

        self.assertEqual(len(sites), 4)
        self.assertEqual(len(rangefinders), 4)
        self.assertEqual(sites[0].get("name"), "front_lidar_rf_00")
        self.assertEqual(rangefinders[0].get("site"), "front_lidar_rf_00")
        self.assertEqual(rangefinders[0].get("cutoff"), "12")
        self.assertEqual(rangefinders[-1].get("name"), "front_lidar_rf_03")

    def test_compile_lidar_rejects_zero_samples(self):
        spec = LidarSensorSpec(horizontal_samples=0)

        with self.assertRaises(ValueError):
            compile_lidar(spec, "front_lidar")

    def test_compile_depth_camera_derives_fovy_from_aperture_and_focal_length(self):
        spec = CameraSensorSpec(focal_length=1.93, vertical_aperture=2.453)

        camera = compile_depth_camera(spec, "chest_cam")

        self.assertEqual(camera.tag, "camera")
        self.assertEqual(camera.get("name"), "chest_cam")
        self.assertAlmostEqual(float(camera.get("fovy")), spec.vertical_fov_degrees(), places=4)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_sensor_compilers_mujoco.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'robot_arm_library.sensor_compilers'`.

- [ ] **Step 3: Write minimal implementation**

```python
# robot_arm_library/sensor_compilers/__init__.py
```

(empty file, marks the package)

```python
# robot_arm_library/sensor_compilers/mujoco.py
"""Compile simulator-agnostic sensor specs into MuJoCo MJCF elements.

Follows the gz-mujoco SDFormat->MJCF mapping (gpu_lidar -> sensor/rangefinder,
depth_camera -> body/camera) and the mujoco_ros2_control lidar recipe (a fan
of rangefinder sensors replicated around a mount, one per horizontal sample).
"""

from __future__ import annotations

import xml.etree.ElementTree as ElementTree

from ..sensor_spec import CameraSensorSpec, LidarSensorSpec


def compile_lidar(spec: LidarSensorSpec, mount_name: str) -> tuple[list[ElementTree.Element], list[ElementTree.Element]]:
    """Return (site_elements, rangefinder_elements) for a horizontal lidar fan.

    ``site_elements`` must be inserted as children of the mount ``<body>``;
    each site is yawed so its local +Z axis (MuJoCo's rangefinder measurement
    axis) points at a different horizontal angle, fanning across
    ``horizontal_fov``. ``rangefinder_elements`` must be inserted into the
    top-level MJCF ``<sensor>`` block. The exact yaw/pitch convention should
    be re-verified against the exported model's ``eulerseq`` during manual
    validation (Phase Validation Plan step 4); the values below use MuJoCo's
    default ``xyz`` sequence.
    """
    if spec.horizontal_samples < 1:
        raise ValueError("Lidar needs at least one horizontal sample")
    half_fov = spec.horizontal_fov / 2.0
    step = spec.horizontal_fov / max(spec.horizontal_samples - 1, 1) if spec.horizontal_samples > 1 else 0.0
    site_elements: list[ElementTree.Element] = []
    rangefinder_elements: list[ElementTree.Element] = []
    for index in range(spec.horizontal_samples):
        angle_degrees = -half_fov + step * index
        ray_name = f"{mount_name}_rf_{index:02d}"
        site_elements.append(
            ElementTree.Element("site", {"name": ray_name, "euler": f"0 90 {angle_degrees:.6g}", "size": "0.002"})
        )
        rangefinder_elements.append(
            ElementTree.Element("rangefinder", {"name": ray_name, "site": ray_name, "cutoff": f"{spec.range_max:.6g}"})
        )
    return site_elements, rangefinder_elements


def compile_depth_camera(spec: CameraSensorSpec, mount_name: str) -> ElementTree.Element:
    """Return a <camera> element for the mount body (MuJoCo cameras are declared inline in <body>)."""
    fovy = spec.vertical_fov_degrees()
    return ElementTree.Element("camera", {"name": mount_name, "mode": "fixed", "fovy": f"{fovy:.6g}"})
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_sensor_compilers_mujoco.py -v`
Expected: PASS for all 3 tests.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/sensor_compilers/__init__.py robot_arm_library/sensor_compilers/mujoco.py tests/test_sensor_compilers_mujoco.py
git commit -m "feat(sensor-compilers): compile lidar/depth-camera specs to MJCF"
```

### Task 17: Wire compiled sensors into `world_exporter.py` and `render_world.py`

**Files:**
- Modify: `robot_arm_library/world_exporter.py`
- Test: `tests/test_world_exporter.py` (new)

**Interfaces:**
- Consumes: `compile_lidar`, `compile_depth_camera` (Task 16); `SensorDefinition` (Phase 6).
- Produces: `SensorMount(sensor, mount_name, position, quaternion)`; `RobotInstance.sensors: tuple[SensorMount, ...] = ()`. Consumed by Phase 9's `_world_robot_instances()`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_world_exporter.py
import json
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

from robot_arm_library.sensor_spec import CameraSensorSpec, LidarSensorSpec, SensorDefinition
from robot_arm_library.world_exporter import RobotInstance, SensorMount, write_world_export


class WorldExporterSensorTests(unittest.TestCase):
    def test_export_includes_compiled_lidar_and_depth_camera(self):
        lidar = SensorDefinition(
            "lidar-1", "Front Lidar", "lidar", "sensor_menagerie/lidar-1", lidar=LidarSensorSpec(horizontal_samples=8)
        )
        camera = SensorDefinition(
            "depth-1", "Chest Depth", "depth_camera", "sensor_menagerie/depth-1", camera=CameraSensorSpec()
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text("<mujoco><worldbody><body name='arm'/></worldbody></mujoco>", encoding="utf-8")
            instance = RobotInstance(
                "arm_1",
                "arm",
                str(source),
                (0, 0, 0),
                (1, 0, 0, 0),
                {},
                sensors=(
                    SensorMount(lidar, "front_lidar", (0.1, 0.0, 0.2), (1, 0, 0, 0)),
                    SensorMount(camera, "chest_cam", (0.0, 0.0, 0.3), (1, 0, 0, 0)),
                ),
            )

            world_path, manifest_path, _render_path = write_world_export(root / "world.xml", [instance])

            world_root = ElementTree.parse(world_path).getroot()
            self.assertEqual(len(world_root.findall(".//sensor/rangefinder")), 8)
            self.assertIsNotNone(world_root.find(".//body/body/camera"))
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest["robots"][0]["sensors"][0]["sensor"]["category"], "lidar")

    def test_export_without_sensors_matches_previous_behavior(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "arm.xml"
            source.write_text("<mujoco><worldbody><body name='arm'/></worldbody></mujoco>", encoding="utf-8")
            instance = RobotInstance("arm_1", "arm", str(source), (0, 0, 0), (1, 0, 0, 0), {"joint": 0.5})

            world_path, _manifest_path, _render_path = write_world_export(root / "world.xml", [instance])

            world_root = ElementTree.parse(world_path).getroot()
            self.assertIsNone(world_root.find("sensor"))


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_world_exporter.py -v`
Expected: FAIL with `ImportError: cannot import name 'SensorMount' from 'robot_arm_library.world_exporter'`.

- [ ] **Step 3: Write minimal implementation**

Add imports and the new dataclass near the top of `robot_arm_library/world_exporter.py`:

```python
from .sensor_compilers.mujoco import compile_depth_camera, compile_lidar
from .sensor_spec import SensorDefinition


@dataclass(frozen=True)
class SensorMount:
    sensor: SensorDefinition
    mount_name: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
```

Extend `RobotInstance`:

```python
@dataclass(frozen=True)
class RobotInstance:
    instance_id: str
    robot_id: str
    source_path: str
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    qpos: dict[str, float]
    sensors: tuple[SensorMount, ...] = ()

    @property
    def prefix(self) -> str:
        value = re.sub(r"[^A-Za-z0-9_]", "_", self.instance_id).strip("_")
        return (value or "robot") + "_"
```

Edit `build_world_xml`'s per-instance loop to attach sensor mounts and collect top-level sensor elements:

```python
    seen_ids: set[str] = set()
    sensor_elements: list[ElementTree.Element] = []
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
        for mount in instance.sensors:
            mount_name = f"{instance.prefix}{mount.mount_name}"
            mount_body = ElementTree.SubElement(
                body,
                "body",
                {"name": mount_name, "pos": _format_values(mount.position), "quat": _format_values(mount.quaternion)},
            )
            if mount.sensor.category == "lidar" and mount.sensor.lidar is not None:
                sites, rangefinders = compile_lidar(mount.sensor.lidar, mount_name)
                for site in sites:
                    mount_body.append(site)
                sensor_elements.extend(rangefinders)
            elif mount.sensor.category == "depth_camera" and mount.sensor.camera is not None:
                mount_body.append(compile_depth_camera(mount.sensor.camera, mount_name))

    if sensor_elements:
        sensor_section = ElementTree.SubElement(root, "sensor")
        for element in sensor_elements:
            sensor_section.append(element)

    return ElementTree.ElementTree(root)
```

(Remove the old bare `return ElementTree.ElementTree(root)` that previously ended the function immediately after the loop — it moves to after the new `if sensor_elements:` block above.)

Update `_manifest_value` to serialize sensor mounts explicitly:

```python
def _manifest_value(instance: RobotInstance) -> dict[str, object]:
    value = asdict(instance)
    value["position"] = list(instance.position)
    value["quaternion"] = list(instance.quaternion)
    value["prefix"] = instance.prefix
    value["sensors"] = [
        {
            "mount_name": mount.mount_name,
            "position": list(mount.position),
            "quaternion": list(mount.quaternion),
            "sensor": mount.sensor.to_dict(),
        }
        for mount in instance.sensors
    ]
    return value
```

Extend `_render_script` to capture sensor readings before opening the viewer:

```python
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

for robot in manifest["robots"]:
    for sensor in robot.get("sensors", []):
        mount_name = robot["prefix"] + sensor["mount_name"]
        category = sensor["sensor"]["category"]
        if category == "lidar":
            readings = []
            for index in range(sensor["sensor"]["lidar"]["horizontal_samples"]):
                sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{{mount_name}}_rf_{{index:02d}}")
                if sensor_id < 0:
                    continue
                address = int(model.sensor_adr[sensor_id])
                readings.append(float(data.sensordata[address]))
            print(f"{{mount_name}} lidar scan: {{readings}}")
        elif category == "depth_camera":
            camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, mount_name)
            if camera_id < 0:
                continue
            spec = sensor["sensor"]["camera"]
            renderer = mujoco.Renderer(model, height=spec["resolution_height"], width=spec["resolution_width"])
            renderer.enable_depth_rendering()
            renderer.update_scene(data, camera=camera_id)
            depth = renderer.render()
            import numpy as np

            depth_path = ROOT / f"{{mount_name}}_depth.npy"
            np.save(depth_path, depth)
            print(f"{{mount_name}} depth frame saved to {{depth_path}}")
            renderer.close()

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
'''
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_world_exporter.py tests/test_behavior_graph.py -v`
Expected: PASS for both new tests and the pre-existing `test_world_export_writes_render_bundle`.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/world_exporter.py tests/test_world_exporter.py
git commit -m "feat(world-exporter): attach compiled lidar/depth-camera sensors to exported worlds"
```

---

## Phase 9 — Blender UI: Sensor Library Panel and Attach-to-Body

**Objective:** Replace the `ROBOT_PT_sensor_library` stub with a real, discoverable, attachable sensor library, and thread attached sensors through world export automatically.

### Entry Criteria
- Phases 6–8 complete (`SensorDefinition`, multi-source `scene.sensor_library`, MuJoCo compiler, and sensor-aware `world_exporter.py` all exist and are tested).
- `ROBOT_PT_sensor_library` is still the stub `"Sensor library is not configured"` panel.

### Exit Criteria
- `ROBOT_PT_sensor_library` lists `scene.sensor_library` and offers an **Attach to Selected Body** button.
- `ROBOT_OT_attach_sensor` creates a `"Sensor: <name>"` empty parented to the selected body (must resolve to an imported robot via `_selected_robot_root`), carrying `sensor_id`/`sensor_category`/`sensor_source_path` custom properties.
- `_world_robot_instances()` automatically includes every attached sensor as a `SensorMount` on the corresponding `RobotInstance`, so **Export**/**Render** (existing buttons) carry sensors through with no extra step.

### Phase Validation Plan
1. Run `blender --background --factory-startup --python tests/blender_sensor_attach_test.py` (new) — import a robot, attach a lidar and a depth camera to two different bodies, export the world, and assert the resulting `world.xml` contains both compiled sensor blocks at the correct body.
2. Re-run `blender --background --factory-startup --python tests/blender_behavior_export_test.py` — unaffected (`_world_robot_instances()` still returns an empty `sensors` tuple when nothing is attached).

### Task 18: `ROBOT_UL_sensor_library`, `ROBOT_PT_sensor_library`, `ROBOT_OT_attach_sensor`

**Files:**
- Modify: `robot_arm_library/blender_addon.py`
- Test: `tests/blender_sensor_attach_test.py` (new)

**Interfaces:**
- Consumes: `SensorLibraryItem`, `scene.sensor_library` (Phase 7); `_selected_robot_root`, `_robot_collection` (existing helpers).
- Produces: `ROBOT_OT_attach_sensor` (`bl_idname = "robot_library.attach_sensor"`); `_sensor_mounts(root) -> list[SensorMount]`, wired into `_world_robot_instances()`.

- [ ] **Step 1: Write the failing test**

```python
# tests/blender_sensor_attach_test.py
"""Validate attaching a sensor to a robot body and exporting it in the world."""

import json
import sys
import tempfile
from pathlib import Path
from xml.etree import ElementTree

import bpy
from mathutils import Vector

repository_root = Path(__file__).resolve().parents[1]
if str(repository_root) not in sys.path:
    sys.path.insert(0, str(repository_root))

import robot_arm_library
import robot_arm_library.blender_addon as blender_addon

robot_arm_library.register()
scene = bpy.context.scene
scene.robot_setup_path = str(Path.home() / "git_repos" / "mujoco_menagerie")
bpy.ops.robot_library.refresh()
item = next(item for item in scene.robot_library if item.robot_id == "robotstudio_so101/so101")
root = blender_addon._place_robot(item, Vector((0.0, 0.0, 0.0)))

with tempfile.TemporaryDirectory() as sensor_dir:
    registry_path = Path(sensor_dir) / "registry.json"
    registry_path.write_text(
        json.dumps(
            {
                "sensors": {
                    "lidar-1": {"category": "lidar", "name": "Front Lidar", "lidar": {"horizontal_samples": 6}},
                    "depth-1": {"category": "depth_camera", "name": "Chest Depth"},
                }
            }
        ),
        encoding="utf-8",
    )
    source = scene.robot_asset_sources.add()
    source.path = sensor_dir
    source.kind = "sensor"
    bpy.ops.robot_library.refresh()

    assert len(scene.sensor_library) == 2, [item.sensor_id for item in scene.sensor_library]

    bpy.context.view_layer.objects.active = root
    lidar_index = next(i for i, item in enumerate(scene.sensor_library) if item.sensor_id == "lidar-1")
    depth_index = next(i for i, item in enumerate(scene.sensor_library) if item.sensor_id == "depth-1")
    result = bpy.ops.robot_library.attach_sensor(sensor_index=lidar_index)
    assert result == {"FINISHED"}, result
    result = bpy.ops.robot_library.attach_sensor(sensor_index=depth_index)
    assert result == {"FINISHED"}, result

    mounts = blender_addon._sensor_mounts(root)
    assert {mount.sensor.category for mount in mounts} == {"lidar", "depth_camera"}, mounts

    with tempfile.TemporaryDirectory() as export_dir:
        world_path, _manifest_path, _render_path = blender_addon._export_world_file(str(Path(export_dir) / "world.xml"))
        world_root = ElementTree.parse(world_path).getroot()
        assert len(world_root.findall(".//sensor/rangefinder")) == 6, ElementTree.tostring(world_root)
        assert world_root.find(".//body/body/camera") is not None

print("Sensor attach and export test passed")
robot_arm_library.unregister()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `blender --background --factory-startup --python tests/blender_sensor_attach_test.py`
Expected: FAIL with `RuntimeError: Operator bpy.ops.robot_library.attach_sensor.poll() failed, context is incorrect` (operator does not exist yet).

- [ ] **Step 3: Write minimal implementation**

Add `SensorMount` to the import line for `world_exporter` in `blender_addon.py`:

```python
from .world_exporter import RobotInstance, SensorMount, write_world_export
```

Add the UIList and panel (replace the existing `ROBOT_PT_sensor_library` stub entirely):

```python
class ROBOT_UL_sensor_library(UIList):
    def draw_item(
        self,
        context: bpy.types.Context,
        layout: bpy.types.UILayout,
        data: object,
        item: SensorLibraryItem,
        icon: int,
        active_data: object,
        active_propname: str,
        index: int,
        flt_flag: int = 0,
    ) -> None:
        icon_name = "OUTLINER_OB_CAMERA" if item.category == "depth_camera" else "OUTLINER_OB_FORCE_FIELD"
        layout.label(text=f"{index + 1:03d}  {item.name}", icon=icon_name)


class ROBOT_PT_sensor_library(Panel):
    bl_label = "Sensor Library"
    bl_idname = "ROBOT_PT_sensor_library"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Library"

    def draw(self, context: bpy.types.Context) -> None:
        layout = self.layout
        scene = context.scene
        if not scene.sensor_library:
            layout.label(text="No sensors discovered", icon="INFO")
            layout.label(text="Add a sensor menagerie source above and refresh", icon="INFO")
            return
        layout.template_list(
            "ROBOT_UL_sensor_library", "sensor_library", scene, "sensor_library", scene, "sensor_library_index", rows=4
        )
        item = scene.sensor_library[scene.sensor_library_index]
        layout.label(text=item.category.replace("_", " ").title())
        attach = layout.operator("robot_library.attach_sensor", text="Attach to Selected Body", icon="CONSTRAINT_BONE")
        attach.sensor_index = scene.sensor_library_index
```

Add the attach operator (near `ROBOT_OT_joint_window`):

```python
class ROBOT_OT_attach_sensor(Operator):
    bl_idname = "robot_library.attach_sensor"
    bl_label = "Attach Sensor to Body"
    bl_options = {"REGISTER", "UNDO"}

    sensor_index: IntProperty()

    def execute(self, context: bpy.types.Context) -> set[str]:
        scene = context.scene
        if self.sensor_index < 0 or self.sensor_index >= len(scene.sensor_library):
            self.report({"WARNING"}, "Select a sensor first")
            return {"CANCELLED"}
        target = context.active_object
        robot_root = _selected_robot_root(target)
        if target is None or robot_root is None:
            self.report({"WARNING"}, "Select a body belonging to an imported robot first")
            return {"CANCELLED"}
        item = scene.sensor_library[self.sensor_index]
        mount = bpy.data.objects.new(f"Sensor: {item.name}", None)
        mount.empty_display_type = "SPHERE"
        mount.empty_display_size = 0.03
        mount["sensor_id"] = item.sensor_id
        mount["sensor_category"] = item.category
        mount["sensor_source_path"] = item.source_path
        _robot_collection().objects.link(mount)
        mount.parent = target
        mount.matrix_parent_inverse = target.matrix_world.inverted()
        return {"FINISHED"}
```

Add a helper that resolves attached sensors back into `SensorMount`s, near `_world_robot_instances`:

```python
def _sensor_mounts(root: bpy.types.Object) -> list[SensorMount]:
    from .setup_parser import discover_sensor_definitions

    mounts: list[SensorMount] = []
    definitions_by_source: dict[str, dict[str, object]] = {}
    for obj in bpy.data.objects:
        if not obj.name.startswith("Sensor: ") or not obj.get("sensor_id"):
            continue
        ancestor = obj
        while ancestor.parent is not None and ancestor != root:
            ancestor = ancestor.parent
        if ancestor != root:
            continue
        source_path = str(obj.get("sensor_source_path", ""))
        if source_path not in definitions_by_source:
            definitions_by_source[source_path] = {
                definition.sensor_id: definition
                for definition in discover_sensor_definitions(Path(source_path).parent)
            }
        definition = definitions_by_source[source_path].get(str(obj.get("sensor_id", "")))
        if definition is None:
            continue
        local_matrix = root.matrix_world.inverted() @ obj.matrix_world
        position = tuple(float(value) for value in local_matrix.translation)
        quaternion = tuple(float(value) for value in local_matrix.to_quaternion())
        mount_name = obj.name.removeprefix("Sensor: ").lower().replace(" ", "_")
        mounts.append(SensorMount(definition, mount_name, position, quaternion))
    return mounts
```

Wire it into `_world_robot_instances()`'s `RobotInstance(...)` construction:

```python
        instances.append(
            RobotInstance(
                instance_id=root.name,
                robot_id=str(root.get("robot_id", root.name)),
                source_path=source_path,
                position=position,
                quaternion=quaternion,
                qpos=qpos,
                sensors=tuple(_sensor_mounts(root)),
            )
        )
```

Add `ROBOT_UL_sensor_library` and `ROBOT_OT_attach_sensor` to `CLASSES` (`ROBOT_UL_sensor_library` alongside `ROBOT_UL_library`; `ROBOT_OT_attach_sensor` alongside `ROBOT_OT_joint_window`); `ROBOT_PT_sensor_library` is already present in `CLASSES` from the original stub and needs no reordering.

- [ ] **Step 4: Run test to verify it passes**

Run: `blender --background --factory-startup --python tests/blender_sensor_attach_test.py`
Expected: `Sensor attach and export test passed` prints; exit code 0.

- [ ] **Step 5: Commit**

```bash
git add robot_arm_library/blender_addon.py tests/blender_sensor_attach_test.py
git commit -m "feat(blender-addon): attach sensors to robot bodies and export them"
```

## Explicit Non-Goals for Phases 6–9

- No Gazebo SDF compiler and no Isaac Sim USD compiler are implemented — `sensor_spec.py`'s schema is deliberately kept simulator-agnostic (Isaac-Sim-attribute-shaped) so `sensor_compilers/gazebo.py` / `sensor_compilers/isaac_usd.py` can be added later without changing `SensorDefinition` (Assumption 9).
- No new external `sensor_menagerie` git repo is created by this plan — Task 13 only documents the convention in `README.md`; populating a real repo is follow-up work.
- IMU and force/torque sensor categories are not compiled, even though the `gz-mujoco` mapping table used as a reference (Assumption 7) already covers them (`imu → accelerometer + gyro`, `force_torque → force + torque`) for a future phase.
- The exact yaw/pitch Euler convention used in `compile_lidar`'s ray fan (Task 16) is flagged in-code as needing manual verification against MuJoCo's `eulerseq` — the automated tests check structural correctness (counts, names, cutoff), not physical ray-hit accuracy; that is covered by Phase 8's manual validation step.

---

## Cross-Phase Final Validation

Run once after all 9 phases are complete, in this order, from the repository root:

1. `pytest tests/ -v` (or `python -m unittest discover -s tests`) — every pure-Python test across all phases is green in one run, including `test_sensor_spec.py`, `test_asset_sources.py`, `test_sensor_compilers_mujoco.py`, and `test_world_exporter.py`.
2. `blender --background --factory-startup --python tests/blender_smoke_test.py`
3. `blender --background --factory-startup --python tests/blender_behavior_export_test.py`
4. `blender --background --factory-startup --python tests/blender_application_node_test.py`
5. `blender --background --factory-startup --python tests/blender_flow_yaml_io_test.py`
6. `blender --background --factory-startup --python tests/blender_ros_bridge_test.py`
7. `blender --background --factory-startup --python tests/blender_container_build_test.py`
8. `blender --background --factory-startup --python tests/blender_import_test.py` — confirms Phase 7's primary-source generalization is still backward compatible.
9. `blender --background --factory-startup --python tests/blender_multi_source_test.py`
10. `blender --background --factory-startup --python tests/blender_sensor_attach_test.py`
11. `make zip` — confirms the add-on still packages cleanly with all new modules (`ros_bridge.py`, `container_builder.py`, `sensor_spec.py`, `asset_sources.py`, `sensor_compilers/`) and updated `blender_addon.py`/`setup_parser.py`/`world_exporter.py` included.

## Self-Review Notes (completed against this plan)

- **Spec coverage:** Requirement 1 (flow-graph-shaped Robot Flow) → Phases 1–3. Requirement 2 (ROS bridge for topics/rviz) → Phase 4. Requirement 3 (OCI/bootc Robot Build tab, podman) → Phase 5. Requirement 4 (sensor-menagerie-style repo + running virtual sensors in sim) → Phases 6–9. All four notebook/chat requirements have a covering phase.
- **Placeholder scan:** no "TBD"/"add error handling"/"similar to Task N" phrasing remains; every step has literal code or an exact command. The one explicitly flagged uncertainty (`compile_lidar`'s Euler convention, Task 16) is called out as a documented, tested-for-structure-not-physics assumption rather than a silent placeholder.
- **Type consistency check:** `PortSpec`, `BehaviorNode.handler/inputs/outputs`, `BehaviorLink.source_port/target_port`, `ApplicationRunner`, `CaryamPortSpec.port_name/shape_csv/shape_tuple()`, `CaryamBehaviorNode.input_ports/output_ports/handler/sync_sockets()` are each defined once (Phase 1 Task 1/2/3, Phase 2 Task 4) and reused identically in every later task (Phase 2 Task 5/6, Phase 3 Task 7). `LidarSensorSpec`/`CameraSensorSpec`/`SensorDefinition` (Phase 6 Task 12) are reused identically by `discover_sensor_definitions` (Task 13), `AssetSource`/`discover_from_sources` (Task 14), `compile_lidar`/`compile_depth_camera` (Task 16), `SensorMount`/`RobotInstance.sensors` (Task 17), and `_sensor_mounts`/`ROBOT_OT_attach_sensor` (Task 18) with no renamed fields across tasks.

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-09-20-robot-flow-ros-oci-build.md`. Two execution options:

1. **Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.
2. **Inline Execution** — Execute tasks in this session using `executing-plans`, batch execution with checkpoints.

Which approach — and which phase should start first? Phases 1, 4, 5, and 6 have no dependencies on each other and could run in parallel; Phases 2–3 depend on 1, and Phases 7–9 depend on 6 (7 also feeds 9).
