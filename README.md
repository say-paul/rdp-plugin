# MuJoCo Robot Arm Library for Blender

This Blender add-on discovers robot arms from a `deep_mind/mujoco_manager/robot_setup` file or directory and makes them available in the 3D View sidebar.

## Install

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

Each item has two placement modes:

- **Add to World** places the selected arm at the 3D cursor.
- **Drag into Viewport** starts a modal drag. Hold the mouse, move into a 3D viewport, and release on the ground plane.

MuJoCo XML entries are compiled with the official MuJoCo Python package when available. Blender geometry is built from the compiled mesh vertices and faces, body and geom poses come from MuJoCo forward kinematics, and joint markers use MuJoCo's compiled anchors and axes. The add-on adds the Python user-site directory automatically, allowing Blender to use an existing `pip install mujoco`. The hand-written XML importer remains a fallback when MuJoCo is unavailable.

For authoritative scale and joint placement, install MuJoCo for the system Python used alongside Blender:

```bash
python -m pip install --user mujoco
```

## Joint controls and animation

After importing a compiled model, select its robot root or any child body. The **Joint Controls** section exposes each hinge joint in radians and each slide joint in meters, using the limits compiled by MuJoCo.

- Change a `qpos` slider to pose the robot and its complete downstream body hierarchy.
- Press **Keyframe** to insert the current joint pose at the active timeline frame.
- Move to another frame, change the sliders, and keyframe again to create an animation.
- Press **Reset** to restore and keyframe the MuJoCo home pose at the current frame.

These controls provide accurate kinematic posing and animation. Version 0.5 does not yet step MuJoCo dynamics from the Blender timeline; Blender rigid-body physics is not a substitute for MuJoCo's articulated dynamics.

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