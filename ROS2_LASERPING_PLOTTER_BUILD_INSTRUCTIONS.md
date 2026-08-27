# Build Instructions: ROS 2 LaserPING Robot-Arm Plotter Prototype

## Objective

Build a working prototype that reads four Dynamixel joint positions and one LaserPING distance measurement from an STM32 over a serial connection, records the synchronized samples, converts each valid laser measurement into a 3D point using the robot arm kinematics, and displays the accumulated result in RViz 2 as a point cloud.

This is a prototype. Prefer a simple, observable, configurable implementation over premature optimization. Do not invent final mechanical dimensions or calibration values. Put unknown values in configuration files, document them clearly, and provide safe example defaults.

## First action in the target workspace

Before editing anything:

1. Inspect the repository structure, README, build system, existing ROS packages, STM32 firmware, URDF/Xacro files, and any serial or Dynamixel code.
2. Preserve unrelated user changes.
3. Reuse existing drivers, robot descriptions, naming conventions, and package structure where practical.
4. Determine the ROS 2 distribution from the workspace or environment. If it cannot be determined, target ROS 2 Jazzy and clearly document that assumption.
5. Report any pin, joint-ID, timer, UART, link-length, joint-direction, or laser-mount conflicts found in the repository before changing those specific values.

## Prototype architecture

Use this data path:

```text
Dynamixel encoders + LaserPING
              |
              v
           STM32
              |  newline-delimited CSV over USB/UART
              v
     laserping_serial_node
        |              |
        |              +--> sensor_msgs/JointState
        |              +--> sensor_msgs/Range
        |              +--> CSV log file
        v
   robot_state_publisher + URDF/Xacro
              |
              v
       TF base_link -> laser_frame
              |
              v
    laserping_cloud_node
              |
              v
 sensor_msgs/PointCloud2 -> RViz 2
```

The STM32 supplies raw measurements. ROS 2 is responsible for parsing, timestamp conversion, calibration, robot transforms, point generation, recording, and visualization.

## Required logged values

Each measurement record must contain:

```text
timestamp_ms,q1_raw,q2_raw,q3_raw,q4_raw,distance_mm,status
```

Definitions:

- `timestamp_ms`: monotonic STM32 time from boot, preferably captured as close as possible to the measurement set.
- `q1_raw` through `q4_raw`: measured/present Dynamixel positions, not commanded goal positions.
- `distance_mm`: measured LaserPING distance.
- `status`: `0` for a valid synchronized sample; use nonzero values for laser timeout, out-of-range, Dynamixel read failure, or another invalid condition.

Recommended prototype line format:

```text
LP,12540,512,315,487,530,428,0
```

Field order:

```text
LP,timestamp_ms,q1_raw,q2_raw,q3_raw,q4_raw,distance_mm,status
```

Requirements for the serial protocol:

- One complete record per line, terminated with `\n` (accept `\r\n` on the receiver).
- ASCII only for the first prototype.
- Prefix data records with `LP,` so diagnostic text can be ignored safely.
- Do not include units, degree symbols, labels, or spaces in data records.
- Emit at a configurable rate, initially 5-10 Hz.
- A record is valid only when all four present-position reads and the laser measurement succeeded.
- Do not output stale laser distance as a valid new sample.
- Keep human-readable startup messages distinct from `LP,` records.

If firmware changes are in scope in the target repository, implement this output there. If they are not in scope, build the ROS side against a mock serial source and document the exact firmware record that is still required.

## ROS 2 workspace deliverables

Create or extend a ROS 2 package named `laserping_mapper`. Python (`ament_python` and `rclpy`) is acceptable and preferred for fast prototyping unless the target repository already establishes a C++ convention.

Expected files, adapted to the existing workspace layout:

```text
laserping_mapper/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/laserping_mapper
├── laserping_mapper/
│   ├── __init__.py
│   ├── serial_node.py
│   └── cloud_node.py
├── config/
│   ├── mapper.yaml
│   └── joint_calibration.yaml
├── launch/
│   └── mapper.launch.py
├── rviz/
│   └── laserping_mapper.rviz
├── urdf/
│   └── prototype_arm.urdf.xacro
├── test/
│   ├── test_parser.py
│   └── test_geometry.py
└── README.md
```

If a robot-description package already exists, put the URDF/Xacro there instead of duplicating it.

## `laserping_serial_node`

Implement a node that:

1. Opens a configurable serial port with configurable baud rate; default to `115200` baud.
2. Reads newline-delimited records without blocking the ROS executor indefinitely.
3. Ignores diagnostic lines that do not begin with `LP,`.
4. Strictly validates field count, integer conversion, raw-position bounds, distance bounds, and status.
5. Converts raw Dynamixel positions to joint angles in radians using per-joint calibration.
6. Publishes `sensor_msgs/msg/JointState` on `/joint_states`.
7. Publishes `sensor_msgs/msg/Range` on `/laserping/range` for every valid laser sample.
8. Records accepted source samples to a CSV file when logging is enabled.
9. Publishes diagnostics or throttled warnings for malformed records and invalid samples without crashing.
10. Supports a `mock_mode` parameter so the complete ROS pipeline can be demonstrated without hardware.

Use these parameters:

```yaml
serial_port: /dev/ttyUSB0
baud_rate: 115200
serial_timeout_s: 0.05
mock_mode: false
publish_rate_hz: 10.0
base_frame: base_link
laser_frame: laser_frame
range_min_m: 0.02
range_max_m: 3.0
field_of_view_rad: 0.01
log_enabled: true
log_directory: ""
```

An empty `log_directory` should resolve to a documented, user-writable default. Do not write logs inside an installed ROS package.

### Time handling

Use the ROS receipt time for message headers in the first prototype. Also retain `timestamp_ms` in the CSV. Track the STM32 timestamp for dropped/out-of-order records and handle its 32-bit wraparound safely. Do not pretend the STM32 boot clock is Unix or ROS wall time.

### Joint calibration

Do not use an absolute difference from home; doing so destroys the sign of the joint angle. Use:

```text
q_rad = direction * (raw_position - home_raw) * radians_per_tick
```

For an AX-series actuator whose 0-1023 range represents 0-300 degrees:

```text
radians_per_tick = radians(300.0 / 1023.0)
```

Store these values per joint in `joint_calibration.yaml`:

```yaml
joints:
  - name: joint_1
    home_raw: 512
    direction: 1
    min_raw: 0
    max_raw: 1023
  - name: joint_2
    home_raw: 512
    direction: 1
    min_raw: 0
    max_raw: 1023
  - name: joint_3
    home_raw: 512
    direction: 1
    min_raw: 0
    max_raw: 1023
  - name: joint_4
    home_raw: 512
    direction: 1
    min_raw: 0
    max_raw: 1023
ticks_per_range: 1023.0
range_degrees: 300.0
```

Treat all example home positions and directions as placeholders until verified on the physical arm.

## Robot description and transforms

Create or reuse a URDF/Xacro model containing:

- `base_link`
- Four revolute joints with names matching `/joint_states`
- Corresponding arm links
- A fixed transform from the final arm link to `laser_frame`

The fixed laser transform must include both translation and rotation. Its positive X axis should represent the emitted laser ray unless the existing robot convention specifies another axis. Document the chosen ray axis.

Put link lengths, joint axes, joint origins, joint limits, and the laser mounting transform in Xacro properties or YAML-accessible configuration. Clearly label unmeasured geometry as placeholder values.

Launch `robot_state_publisher` with the resulting `robot_description`. Do not manually reimplement the complete forward-kinematics chain in the cloud node if TF already supplies `base_link -> laser_frame`.

## `laserping_cloud_node`

Implement a node that converts each valid range observation into a point in `base_link`.

If the laser ray is the positive X axis of `laser_frame`, the local hit is:

```text
p_laser = [distance_m, 0, 0]
```

Transform that point into `base_link` using TF at the measurement header timestamp:

```text
p_base = T_base_laser * p_laser
```

This is equivalent to:

```text
hit_position = laser_origin + distance * laser_direction
```

Important: laser origin `(x, y, z)` plus distance alone is insufficient. The orientation/direction from TF is required.

Node behavior:

- Subscribe to `/laserping/range`.
- Look up `base_link -> laser_frame` from `tf2_ros` using the range message timestamp.
- Reject samples cleanly when TF is unavailable or distance is invalid.
- Append accepted hits to an in-memory point collection.
- Publish the accumulated points as `sensor_msgs/msg/PointCloud2` on `/laserping/points`.
- Use `base_link` as the point-cloud frame.
- Make maximum retained points configurable, for example `max_points: 20000`, dropping the oldest points when full.
- Provide a reset mechanism, preferably a `std_srvs/srv/Empty` service at `/laserping/clear_map`.
- Optionally publish the newest hit separately on `/laserping/latest_point`.
- Do not use LaserScan unless the motion pattern actually guarantees an ordered planar angular scan.

For this prototype, XYZ-only `float32` PointCloud2 fields are sufficient.

## Data logging

The CSV log should contain both raw measurements and useful ROS-side results:

```text
ros_time_ns,stm32_timestamp_ms,q1_raw,q2_raw,q3_raw,q4_raw,q1_rad,q2_rad,q3_rad,q4_rad,distance_mm,status,hit_x_m,hit_y_m,hit_z_m
```

If a transform is temporarily unavailable, preserve the valid raw sample and leave hit coordinates empty, or place it in a separate raw CSV. Document the chosen behavior.

Flush periodically and close the file cleanly. Include a timestamp in each log filename so previous sessions are not overwritten.

## Launch and RViz

Provide one launch command that starts:

- `laserping_serial_node`
- `robot_state_publisher`
- `laserping_cloud_node`
- RViz 2 optionally through a launch argument

Example expected command:

```bash
ros2 launch laserping_mapper mapper.launch.py \
  serial_port:=/dev/ttyUSB0 \
  mock_mode:=false \
  use_rviz:=true
```

The RViz configuration should include:

- Fixed Frame: `base_link`
- RobotModel
- TF
- PointCloud2 display for `/laserping/points`
- Range display for `/laserping/range`, if supported and useful

## Mock mode

Mock mode is required so the software can be tested before connecting the arm. It should generate a slow, safe synthetic four-joint motion and plausible range measurements. It must use the same parser/publication path as real input wherever practical.

Provide at least one deterministic mock sequence that generates a visibly recognizable arc, plane, or box-like surface in RViz. Mark mock records clearly in logs or diagnostics.

## Dependencies

Declare all used dependencies correctly in `package.xml` and packaging metadata. Likely dependencies include:

- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `std_srvs`
- `tf2_ros`
- `tf2_geometry_msgs`
- `robot_state_publisher`
- `xacro`
- Python `pyserial`

Use repository/environment conventions for dependency management. Document any required `rosdep` and serial-port permission commands; do not silently modify system groups or permissions.

## Tests

At minimum, add automated tests for:

1. Parsing a valid `LP,` line.
2. Rejecting wrong prefixes, field counts, non-integer fields, invalid status, out-of-bounds encoder values, and invalid distances.
3. Signed encoder-to-radian calibration on both sides of the home position.
4. Laser hit geometry for identity transform.
5. Laser hit geometry for a known translation and rotation.
6. Maximum point-buffer behavior.
7. STM32 timestamp wraparound/out-of-order handling if that logic is separated enough to test.

Run the appropriate checks, such as:

```bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Adapt shell syntax for the actual operating system and ROS environment.

## Prototype acceptance criteria

The prototype is complete when:

1. The ROS workspace builds cleanly.
2. Parser and geometry tests pass.
3. Mock mode launches without hardware and publishes all expected topics.
4. `/joint_states` contains four correctly named joints in radians.
5. TF provides a connected chain from `base_link` to `laser_frame`.
6. `/laserping/range` publishes valid range messages in metres.
7. `/laserping/points` accumulates visible XYZ hits in RViz 2.
8. `/laserping/clear_map` clears the accumulated points.
9. A timestamped CSV log is created without overwriting earlier runs.
10. Malformed serial input produces a throttled warning rather than terminating a node.
11. The README explains setup, configuration, launch, calibration, logging, and known prototype limitations.

## Safety and limitations

- Begin with torque disabled or the arm physically supported while verifying encoder signs and zero positions.
- Verify joint IDs and mechanical limits before commanding motion.
- Use low speed and conservative motion limits for initial scans.
- Keep an accessible actuator power cutoff.
- Reject distances outside the configured valid sensor range.
- A moving base requires an additional localization transform; this prototype maps only relative to a stationary `base_link`.
- Accuracy depends on link dimensions, joint zero calibration, backlash, arm flex, laser mounting alignment, sensor noise, and timestamp skew.
- Four joints can position and orient the laser only within the arm's actual kinematic constraints; this is not equivalent to a general-purpose 6-DOF scanner.
- Do not claim metric map accuracy until geometry and laser extrinsics have been physically calibrated.

## Implementation handoff

When finished, provide:

1. A concise summary of created and modified files.
2. Exact build, test, and launch commands.
3. Test results.
4. Assumed ROS 2 distribution and operating system.
5. A table of every placeholder requiring physical measurement or verification, especially:
   - serial device name;
   - Dynamixel IDs;
   - home encoder values;
   - joint directions;
   - joint axes and limits;
   - link lengths and joint origins;
   - laser mounting translation and rotation;
   - laser valid range;
   - scan motion or sampling rate.
6. Any remaining firmware work needed to emit the specified `LP,` serial records.

