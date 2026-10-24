# MuJoCo Robot Arm Library for Blender

This Blender add-on discovers robot arms from a `deep_mind/mujoco_manager/robot_setup` file or directory and makes them available in the 3D View sidebar.

## Install

Create the Blender add-on archive from the repository root:

```bash
make zip
```

This creates `robot_arm_library.zip` for the Blender installation steps below.

1. In Blender, open **Edit > Preferences > Add-ons > Install**.
2. Select `robot_arm_library.zip` from this repository.
3. Enable **MuJoCo Robot Arm Library**.
4. Open the 3D View sidebar with `N`, then choose the **Robot Library** tab.
5. The add-on defaults to `~/git_repos/mujoco_menagerie` when that folder exists. Set **Setup** to the existing Menagerie or `deep_mind/mujoco_manager...` directory and press refresh to load your real robots.

The parser accepts JSON, Python literal dictionaries/lists, YAML when PyYAML is available in Blender, and MuJoCo XML files. A setup entry can use `model_path`, `mjcf_path`, `xml_path`, `urdf_path`, `asset_path`, `asset`, `file`, or `path`.

## Load MuJoCo Menagerie

Clone the upstream repository locally:

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```

Set **Setup** to the cloned `mujoco_menagerie` folder and click refresh. Selecting a single XML such as `agilex_piper/piper.xml` also walks up to the Menagerie root automatically. The add-on reads the curated Menagerie registry when available. For older checkouts without that generated registry, it scans each model directory, selects the root-level robot XMLs, and skips scene, MJX, and nested asset helper files. The library will show the Menagerie models and their categories; it does not copy or redistribute Menagerie assets into this add-on.

### Multiple asset sources and virtual sensors

The **Robot Library** tab keeps **Setup** as its primary source. Use **Additional Asset Sources → Add Source** to register other robot or sensor repositories, choose `Auto`, `Robot`, or `Sensor`, then refresh. The **Sensor Library** panel lists discovered sensors. A sensor repository needs a `registry.json` at its root:

```json
{
  "sensors": {
    "front-lidar": {
      "category": "lidar",
      "name": "Front Lidar",
      "lidar": {"channels": 1, "horizontal_fov": 270, "horizontal_samples": 540, "range_min": 0.05, "range_max": 20},
      "backend_capabilities": {"mujoco": "rangefinder_fan"}
    },
    "chest-depth": {
      "category": "depth_camera",
      "name": "Chest Depth Camera",
      "camera": {"focal_length": 1.93, "horizontal_aperture": 3.896, "vertical_aperture": 2.453, "resolution_width": 640, "resolution_height": 480},
      "backend_capabilities": {"mujoco": "camera_depth"}
    }
  }
}
```

Create your own repository with this convention, then register its path as a `Sensor` source. Select an imported robot body and press **Attach to Selected Body**. The new sensor empty starts at that body's origin; move or rotate it to set its mount pose. The mount stores a copy of the definition in the Blender scene. On export, Caryam writes a sensorized MJCF copy beside the world XML, adds rangefinder sites or a depth camera under the selected body, and records the mount in `world.json`. The generated `render_world.py` prints lidar readings and saves depth frames as `.npy` files when MuJoCo renders the world. The sensor definition is simulator-independent; this release includes the MuJoCo compiler for lidar and depth cameras.

Each item has two placement modes:

- **Add to World** places the selected arm at the 3D cursor.
- **Drag into Viewport** starts a modal drag. Hold the mouse, move into a 3D viewport, and release on the ground plane.

**Robot Geometry** defaults to **Fast Assembly**. It displays each MuJoCo geometry as a small bounding box with an outline, so dense robot meshes do not fill Blender's viewport memory. The boxes follow the robot's original body and joint hierarchy. Choose **Detailed Mesh** before adding a robot when you need its visual surfaces in Blender; **Mesh Detail** can reduce the imported triangles, but detailed meshes must still be loaded before they can be simplified. The setting applies to newly added robots; add the robot again to change its preview. These boxes are viewport approximations, not CAD solids. MuJoCo **Play** uses the original MJCF geometry, mass, collisions, and joints. A Blender `world.usd` snapshot contains the Blender preview geometry, so use Detailed Mesh if the USD snapshot needs full robot surfaces. Fast Assembly requires the MuJoCo Python package inside Blender; if unavailable, the add-on shows a placeholder and reports a warning instead of loading dense meshes.

MuJoCo XML entries are compiled with the official MuJoCo Python package when available. In Detailed Mesh mode, Blender geometry is built from the compiled mesh vertices and faces. In both modes, body and geom poses come from MuJoCo forward kinematics, and joint markers use MuJoCo's compiled anchors and axes. The add-on adds the Python user-site directory automatically, allowing Blender to use an existing `pip install mujoco`. The hand-written XML importer remains a fallback when MuJoCo is unavailable.

For authoritative scale and joint placement, install MuJoCo 3.2 or newer for the Python used alongside Blender:

```bash
python -m pip install --user "mujoco>=3.2"
```

## Export and render a world

After placing one or more XML-backed robots, open the **Sim** sidebar tab. Its **MuJoCo** section provides **Play**, which exports `caryam_world.xml` beside the current Blender file and immediately opens the MuJoCo physics viewer. **Gazebo** is reserved for future simulator support. Use **Export** when you want to choose a different world file path. Joint posing is available in the **Joints** panel under **Robot Control**. The add-on writes companion files beside the chosen world path:

- `world.xml` composes the imported MJCF models with MuJoCo's `attach` element and preserves their Blender world transforms.
- `world.json` records the current hinge and slide qpos values.
- `render_world.py` applies those qpos values and opens the MuJoCo passive viewer.
- `world.usd` is a Blender USD snapshot of the scene for interchange with USD-based tools.

Visible Blender meshes are exported as individual MuJoCo solids. To make a cube fall, select it and use Blender's **Object > Rigid Body > Add Active** command, or add a rigid body in Blender's **Physics Properties**. Make sure **Dynamic** is on and **Animated** is off. Use **Add Passive** for fixed obstacles. Set the collision shape, mass, friction, and bounciness in Blender's Physics Properties. Caryam reads those settings when you press **Export** or **Play**; it does not maintain separate physics controls. The MuJoCo panel shows whether the selected mesh will export as dynamic or fixed. Press **Play** again after changing Blender settings: each MuJoCo run receives a new scene snapshot, not live updates from the Blender viewport. Objects without a rigid body stay fixed colliders; object names do not control physics. The `mujoco_dynamic` custom property remains available for existing scenes.

Active bodies receive a free joint and mass. Passive bodies stay fixed. Disabling a rigid body exports its visible mesh without collision. Blender's box, sphere, capsule, and cylinder collision shapes become MuJoCo primitive colliders while retaining the original visible mesh. Mesh and convex-hull shapes use MuJoCo's mesh collision, which uses a convex hull; concave obstacles need multiple convex pieces for accurate contact. Bounce is approximated with MuJoCo contact solver settings and may need tuning. Kinematic Blender rigid bodies export at their current pose as fixed bodies; animation is not replayed in MuJoCo.

Blender's cone collision shape falls back to MuJoCo's convex mesh collision. Imported robots keep their original MJCF collision geoms, mass, and joints; changing a Blender rigid body on a robot's visual mesh does not alter that source model.

Blender rigid-body constraints are not yet translated into MuJoCo joints or equality constraints. Connected objects therefore export as separate bodies. Blender object constraints are not exported as MuJoCo constraints either.

The exporter keeps each object's Blender material diffuse color in the MuJoCo view. The USD snapshot exports Blender materials; image textures and shader nodes are not yet translated into MuJoCo materials. The viewer advances physics in real time so falling and contacts are visible.
Free-base MJCF robots preserve their Blender root position and rotation, so separately arranged robots spawn with the same spacing in MuJoCo instead of overlapping at world origin.

The **MuJoCo Python** field controls which Python executable launches the renderer. The composed world uses MuJoCo 3.2+'s model attachment API; the original MJCF and its assets remain external files, so keep their relative paths valid after export.

To capture mounted sensor output without opening the viewer, run `MUJOCO_GL=egl python render_world.py --sensors-only` from the export directory. It prints lidar readings and saves depth frames there.

## Data graph editor

Use **Open Data Graph** to switch the current area to the **Caryam Data Graph** node editor. Add nodes from the sidebar and connect typed ports in the node view. Save/load `.json` for the full Caryam graph or `.yaml`/`.yml` for application nodes in `flow.yaml` form. YAML I/O needs PyYAML in Blender's Python; the add-on reports a clear error if it is unavailable. Each JSON port records its data type, tensor shape, delivery mode (`stream`, `state`, `event`, or `command`), timestamp and validity requirements, and optional dtype, semantic type, unit, rate, and variable-length contract. Links with incompatible declared contracts are rejected. **Catalog Nodes** adds a Robot State source, Robot Command sink, or the selected Sensor Library item with generated ports; manually created Application ports remain editable.

Use the **Joint Controller** and **Joint Target** nodes to model a simulation command path. The controller rejects stale, invalid, and out-of-limit targets; the target node publishes the compact `joint_target` command proposal. A node's component type is fixed when it is added; edit only its own settings, never its behavior category. **RGB Camera** emits `rgb_image` in RGB8 format, **Depth Camera** emits `depth_image` in 32FC1 format, **IMU** emits `imu`, and **Tactile Array** emits `tactile_array`; their sensor settings are serialized with the graph. The tactile array must reference a configured MuJoCo tactile geom and mesh. **Virtual Lidar** always emits a `point_cloud` port in `ROS PointCloud2` format, with editable maximum range and horizontal-sample settings. Virtual Sensor nodes can read `joint.qpos` or a Blender custom property using `object:ObjectName.property`.

## ROS Control

Open **Robot Control → ROS Control** to choose ROS 1 or ROS 2, list topics, echo one message, publish one message, or launch RViz. The controls use the installed `rostopic`/`ros2` and `rviz`/`rviz2` command-line tools. The add-on registers without ROS installed and reports a command error when a tool is missing.

## Robot Build

Open the **Robot Build** sidebar tab to set an OCI image name and bootc base image. **Generate Containerfile** writes `build/Containerfile` and a small entrypoint payload beside the current `.blend` file. **Build Image** starts `podman build` (or Docker when selected) in the background. The status shows the launched command's intent; inspect the tool's output to confirm completion. Set the exact `/dev/...` disk and use **Flash to Disk** to open a confirmation dialog before the bootc install command runs. This erases the selected disk. The command follows the [bootc installation instructions](https://bootc.dev/bootc/bootc-installation.7.html).

## Intrinsic Core simulation

Start the external bridge in the Core simulation environment, not inside Blender:

```bash
python -m robot_arm_library.core_bridge --host 127.0.0.1 --port 8765
```

In Blender, open **Sim > Intrinsic Core**, set the bridge URL and Core endpoint (default `localhost:17080`), then select **Run**. The bridge runs the data graph in simulation mode and exposes status, compact blackboard outputs, and recent logs over local JSON HTTP. It publishes only `caryam.perception.part_pose`, `caryam.imu.tilt_exceeded`, `caryam.control.joint_target`, and `caryam.control.command_valid`; raw sensor frames remain in the flow transport. The current MVP never sends a physical robot command.

## Joint controls and animation

After importing a compiled model, press **Open Joint Controller** in the library or open the dedicated **Robot Joints** sidebar tab. The controller targets the most recently imported robot automatically; selecting any root or child switches it to that robot. It exposes each hinge joint in radians and each slide joint in meters, using the limits compiled by MuJoCo.

- Change a `qpos` slider to pose the robot and its complete downstream body hierarchy.
- Press **Keyframe** to insert the current joint pose at the active timeline frame.
- Move to another frame, change the sliders, and keyframe again to create an animation.
- Press **Reset** to restore and keyframe the MuJoCo home pose at the current frame.

These controls provide accurate kinematic posing and animation. Version 0.6 does not yet step MuJoCo dynamics from the Blender timeline; Blender rigid-body physics is not a substitute for MuJoCo's articulated dynamics.

## Collision geometry

Enable **Load Collision Geoms** before adding a robot to import MuJoCo collision groups alongside the visual model. Collision objects are tagged with `is_collision`, shown as red wireframes, and excluded from rendering. The option affects newly imported robots; reimport an existing robot after changing it.

## Local validation

Run the parser tests from the repository root:

```bash
python -m unittest discover -s tests
```

If Blender is installed on your `PATH`, run the add-on smoke test headlessly:

```bash
blender --background --factory-startup --python tests/blender_smoke_test.py
```

To test a real Menagerie import, run:

```bash
blender --background --factory-startup --python tests/blender_import_test.py
```

The feature smoke scripts are `tests/blender_flow_yaml_io_test.py`, `tests/blender_ros_bridge_test.py`, `tests/blender_container_build_test.py`, `tests/blender_multi_source_test.py`, and `tests/blender_sensor_attach_test.py`. Run each with the same `blender --background --factory-startup --python` command.
