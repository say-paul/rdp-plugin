# Caryam + Intrinsic Core MVP Plan

**Goal:** Use Caryam as the project editor for a sensor-to-command dataflow. Run the flow on Intrinsic Core in simulation; use Intrinsic's Behavior Tree only to start, supervise, and stop it.

1. **Define the Caryam flow contract.** Keep the named-port graph as **Robot Flow**, not a Behavior Tree. Start with:

   ```text
   sensor -> application/model -> joint_controller -> joint_target
   ```

   - `sensor` reads camera, lidar, IMU, force/torque, or joint-state data.
   - `application/model` turns sensor data into facts such as `part_pose` or orientation.
   - `joint_controller` combines a requested target with current joint state, kinematics, and limits.
   - `joint_target` is a command proposal containing positions, optional velocity/acceleration limits, timestamp, and validity.

   Each port declares its type, shape, timestamp, validity, and delivery mode: `stream`, `state`, `event`, or `command`.

2. **Separate data and control semantics.** Data links transform typed values each tick. Control nodes are limited to `Start`, `Stop`, `Wait`, `Retry`, `Success`, and `Failure`; loops occur by repeated ticks, not graph cycles.

3. **Create a Caryam-to-Core bridge.** Run an external Python process, `caryam-core-bridge`, using the Intrinsic SDK. The Caryam UI calls it over local HTTP or JSON-RPC; it connects to Core at `host:17080`. Do not install the SDK or run Core inside Blender.

4. **Implement one Core Flow Runner.** Build one Intrinsic Skill or Service that runs a compiled Caryam flow on the Core host. It streams sensor data between flow nodes and publishes only compact outputs to the blackboard:

   ```text
   caryam.perception.part_pose
   caryam.imu.tilt_exceeded
   caryam.control.joint_target
   caryam.control.command_valid
   ```

   Keep raw camera frames, point clouds, and raw IMU samples in the stream transport, not the blackboard.

5. **Add a minimal Intrinsic Behavior Tree.** Start the Flow Runner, check data freshness and `command_valid`, then invoke a validated command Skill. Reject stale, invalid, out-of-limit, or faulted commands before they reach the robot controller.

6. **Integrate the Caryam UI.** Add Core connection settings, `Run in Core Simulation`, `Stop`, and status/log views. Reuse the Blender graph editor first; later use the same project format in a standalone Caryam desktop or web application.

7. **Validate in simulation only.** Use an Ubuntu 26.04 Intrinsic Core simulator. Acceptance test: a mock sensor input produces a valid `joint_target`, Caryam shows status and outputs, and stale or invalid inputs prevent command execution.

**MVP boundaries:** retain the current MuJoCo `world.xml` exporter; defer an Intrinsic/Gazebo workcell exporter, full Behavior Tree authoring, and real-hardware control.