# R1 Pro DiMOS Integration — Setup & Connection Guide

## Overview

This directory contains test scripts for validating DiMOS connectivity to the
Galaxea R1 Pro humanoid robot over ethernet. The robot runs ROS2 Humble on a
Jetson Orin (Ubuntu 22.04 / L4T). The laptop runs Ubuntu 24.04 with ROS2 Jazzy.

**Current status**: Chassis movement, arm control, and keyboard teleop all
working end-to-end through DiMOS adapters. Dual-arm manipulation planning is
in progress.

---

## Network Setup

### Physical Connection
- Connect laptop to robot via ethernet cable
- Robot ethernet port: `eth1` on the robot

### Robot IP (persistent after netplan config)
- Robot `eth1`: `192.168.123.150/24`
- Laptop ethernet (`enxf8e43bb7046c`): `192.168.123.100/24`

### Set laptop ethernet IP (if not already set)
```bash
sudo ip addr add 192.168.123.100/24 dev enxf8e43bb7046c
```

### SSH into robot
```bash
ssh nvidia@192.168.123.150
# password: nvidia
```

### Make robot IP persistent across reboots (already done)
Edit `/etc/netplan/50-cloud-init.yaml` on the robot, add `192.168.123.150/24`
to eth1 addresses:
```yaml
eth1:
  dhcp4: true
  addresses: [192.168.2.150/24, 192.168.123.150/24]
```
Then: `sudo netplan apply`

---

## Robot Startup Procedure

Run these commands on the robot via SSH every session:

```bash
# Step 1: Start CAN bus driver
bash ~/can.sh

# Step 2: Launch full robot stack (ros2_discovery, mobiman, hdas, tools)
cd ~/galaxea/install/startup_config/share/startup_config/script
./robot_startup.sh boot ../sessions.d/ATCStandard/R1PROBody.d/

# Step 3: Wait ~30 seconds for HDAS to fully init (arms open/close = healthy)

# Step 4: Start chassis gatekeeper (required for chassis control from laptop)
source ~/galaxea/install/setup.bash
export ROS_DOMAIN_ID=41
python3 ~/chassis_gatekeeper.py
```

```bash
# Step 5: Verify all topics are up (use --no-daemon, the daemon is unreliable)
source ~/galaxea/install/setup.bash
export ROS_DOMAIN_ID=41
ros2 topic list --no-daemon | grep hdas | head -5
# Expected: /hdas/feedback_arm_left, /hdas/feedback_arm_right, etc.
```

### Robot tmux sessions
| Session | Purpose |
|---|---|
| `ros_discovery` | FastDDS discovery server on port 11811 (for VR/WiFi, not needed for ethernet) |
| `mobiman` | Main motion control stack |
| `hdas` | Hardware abstraction — arms, chassis, torso, grippers |
| `tools` | Utilities |

Check session health: `tmux attach -t hdas` (Ctrl+B D to detach)

---

## Laptop Setup (every session)

```bash
cd ~/Downloads/dimos

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=41
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/scripts/r1pro_test/fastdds_r1pro.xml
```

Tip: add these to a shell script `scripts/r1pro_test/env.sh` and `source` it.

---

## Chassis Gatekeeper (Key Concept)

The R1 Pro `chassis_control_node` has **three internal gates** that all must be
unlocked simultaneously for chassis movement to work. The gatekeeper runs on the
robot and handles all three, exposing a simple `/cmd_vel` topic for the laptop.

### The 3 Gates

| Gate | What blocks it | How gatekeeper fixes it |
|---|---|---|
| **Gate 1**: Subscriber count | Node skips IK if nobody subscribes to `/motion_control/chassis_speed` | Subscribes to the topic |
| **Gate 2**: `breaking_mode_` flag | HDAS publishes `mode=2` at 200Hz on `/controller`, setting `breaking_mode_=1` | Launch file remaps `/controller` → `/controller_unused`; gatekeeper publishes `mode=5` on `/controller_unused` |
| **Gate 3**: `acc_limit` defaults to zero | `calculateNextVelocity` uses `acc_limit * dt` which stays 0 | Publishes nonzero `TwistStamped` on `/motion_target/chassis_acc_limit` |

### Prerequisites (one-time on robot)
1. Edit `~/galaxea/src/mobiman/launch/r1_pro_chassis_control_launch.py`
2. Uncomment/add: `remappings=[('/controller', '/controller_unused')]`
3. Rebuild and restart mobiman

### Running
```bash
# On robot:
source ~/galaxea/install/setup.bash && export ROS_DOMAIN_ID=41
python3 ~/chassis_gatekeeper.py

# From laptop (test):
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 20
```

---

## Verification Tests

Run in order after startup:

```bash
# Test 1: Topic discovery (70 topics expected, ~10s)
python3 scripts/r1pro_test/test_01_topic_discovery.py

# Test 2: Read live arm joint data (safe, read-only)
python3 scripts/r1pro_test/test_02_read_arm_feedback.py

# Test 4: Arm movement (moves joint 0 by 0.3 rad, then returns home)
python3 scripts/r1pro_test/test_04_arm_joint_command.py

# Test 3: Chassis movement — requires chassis_gatekeeper on robot
python3 scripts/r1pro_test/test_03_chassis_command.py

# Test 5: DiMOS ROS layer integration
python3 scripts/r1pro_test/test_05_dimos_ros_layer.py
```

### Test status
| Test | Status | Notes |
|---|---|---|
| 01 topic discovery | PASS | 70 topics visible |
| 02 arm feedback | PASS | 7-joint positions/velocities/efforts streaming |
| 03 chassis command | PASS | Works via chassis_gatekeeper → `/cmd_vel` |
| 04 arm movement | PASS | Joint 0 moves 0.3 rad and returns home |
| 05 DiMOS ROS layer | PASS | DiMOS adapters communicate with robot |

**Important**: Do NOT run tests individually back-to-back with separate
`rclpy.init()`/`rclpy.shutdown()` cycles. FastDDS 3.x (Jazzy) creates new DDS
participants each cycle, which corrupts the robot's Humble DDS nodes. Use
`run_all_tests.py` for sequential testing, or wait 30+ seconds between runs.

---

## DiMOS Integration Architecture

### Adapters
| Component | File | Pattern |
|---|---|---|
| Chassis | `dimos/hardware/drive_trains/r1pro/adapter.py` | `TwistBaseAdapter` — publishes `Twist` to `/cmd_vel` (via gatekeeper) |
| Arms | `dimos/hardware/manipulators/r1pro/adapter.py` | `ManipulatorAdapter` — parameterized by side (left/right) |
| ROS env | `dimos/hardware/r1pro_ros_env.py` | Sets ROS_DOMAIN_ID=41, FastDDS, rmw_fastrtps_cpp |

### Blueprints
| Blueprint | File | Components |
|---|---|---|
| `coordinator_r1pro` | `dimos/control/blueprints/r1pro.py` | Arms + chassis |
| `coordinator_r1pro_arms` | `dimos/control/blueprints/r1pro.py` | Arms only |

### Keyboard Teleop
`dimos/robot/galaxea/r1pro/blueprints/r1pro_keyboard_teleop.py` — keyboard
control of chassis and arms through DiMOS.

---

## Key Topics

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | laptop → gatekeeper (RELIABLE QoS) |
| `/hdas/feedback_arm_left` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_arm_right` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_chassis` | `sensor_msgs/JointState` | robot → laptop |
| `/hdas/feedback_torso` | `sensor_msgs/JointState` | robot → laptop |
| `/motion_target/target_speed_chassis` | `geometry_msgs/TwistStamped` | gatekeeper → chassis_control_node |
| `/motion_target/target_joint_state_arm_left` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_joint_state_arm_right` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_joint_state_torso` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_position_gripper_left` | `sensor_msgs/JointState` | laptop → robot |
| `/motion_target/target_position_gripper_right` | `sensor_msgs/JointState` | laptop → robot |

---

## Challenges & How We Solved Them

### 1. Finding the robot's IP
Robot had no known IP when connected via ethernet. Used `tcpdump` and `arp -a`
to discover it. Robot's `eth1` had no IPv4 assigned by default — manually
assigned `192.168.123.150/24` with `sudo ip addr add`, then made it persistent
via netplan.

### 2. ROS2 topic discovery failing across machines
**Root causes found (in order):**

**a) `ROS_LOCALHOST_ONLY=1` set in robot's `~/.bashrc`**
The robot was configured to only accept local DDS connections. Changed to
`ROS_LOCALHOST_ONLY=0` in `~/.bashrc` so tmux sessions (which source bashrc)
inherit the correct setting.

**b) CycloneDDS ↔ FastDDS EDP incompatibility**
Tried CycloneDDS on the laptop (ROS2 Jazzy default) thinking it would
interoperate with FastDDS on the robot (ROS2 Humble). Peer discovery (PDP)
worked — tcpdump confirmed packets flowing both ways — but endpoint discovery
(EDP) failed silently. Topics never appeared.

Fix: switch laptop to FastDDS (`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) to match
the robot.

**c) FastDDS using wrong network interface on laptop**
Laptop has WiFi (`192.168.1.68`), ethernet (`192.168.123.100`), and Tailscale
(`100.78.x.x`). FastDDS multicast was going out the wrong interface.

Fix: `fastdds_r1pro.xml` — a FastDDS profile that binds to `192.168.123.100`
(ethernet) and sets `192.168.123.150:17650` as explicit unicast peer. This
bypasses multicast entirely.

**d) `interfaceWhiteList` renamed in FastDDS 3.x (Jazzy)**
The original XML used `<interfaceWhiteList>` which is FastDDS 2.x syntax.
FastDDS 3.x (shipped with Jazzy) renamed it to `allowlist`. The element was
silently ignored, so interface restriction never applied.

Fix: switched from transport-level interface restriction to locator-based
config (`metatrafficUnicastLocatorList`, `defaultUnicastLocatorList`,
`initialPeersList`) which works in both FastDDS 2.x and 3.x.

**e) Robot's FastDDS discovery server (port 11811)**
The robot runs a FastDDS discovery server via `start_discover_server.sh`.
Initially thought we needed to use `ROS_DISCOVERY_SERVER` to connect to it.
Investigation revealed the mobiman/hdas nodes do NOT connect to the discovery
server — they use standard multicast. The discovery server is for VR/WiFi
remote control only. Using `ROS_DISCOVERY_SERVER` on either side broke topic
visibility.

**f) HDAS process crashing (exit code -9)**
After restarting the robot stack, HDAS sometimes crashes on startup. Cause:
HDAS needs ~30 seconds to initialize and communicate with the arm motors over
CAN. If you check topics too early, only chassis topics appear. The arm
open/close cycle during boot confirms hardware is healthy. Always wait for this
before checking topics.

### 3. FastDDS 2.x/3.x DDS participant corruption
Running test scripts back-to-back with separate `rclpy.init()`/`rclpy.shutdown()`
cycles created new FastDDS 3.x participants each time. The `ParticipantEntitiesInfo`
wire format differs between FastDDS 2.x (Humble) and 3.x (Jazzy), corrupting the
robot's DDS participant state and causing topics to disappear.

Fix: `run_all_tests.py` calls `rclpy.init()` once, runs all tests, then calls
`rclpy.shutdown()` once. Each test exposes a `main() -> bool` function that
assumes rclpy is already initialized.

### 4. Chassis control node ignoring commands (the 3-gate problem)
Publishing `TwistStamped` to `/motion_target/target_speed_chassis` had no effect.
Binary analysis of `chassis_control_node` revealed three independent gates that
all block motion when unsatisfied. See the "Chassis Gatekeeper" section above
for the full solution.

This was the hardest problem — took multiple sessions of investigation including
binary disassembly of the node to identify the three gates.

### 5. ROS2 daemon unreliable on robot
The ros2 daemon on the robot has slow discovery and often shows only 2 topics
(`/parameter_events`, `/rosout`) even when 70+ topics are active. Always use
`ros2 topic list --no-daemon` on the robot for accurate results.

---

## Robot Architecture Notes

- **Platform**: Jetson Orin (aarch64), Ubuntu 22.04, L4T (Jetpack)
- **ROS2**: Humble, FastDDS (rmw_fastrtps_cpp)
- **ROS_DOMAIN_ID**: 41
- **CAN bus**: arms and torso communicate via CAN (`can.sh` starts the driver)
- **HDAS**: Hardware abstraction layer — publishes all sensor feedback, receives
  all motion commands
- **mobiman**: Motion manager — handles kinematics, IK, safety limits
- **Custom message package**: `hdas_msg` — used for motor control, BMS, LED,
  version info. Standard ROS2 types used for joint states and geometry
- **Chassis type**: W1 (3-wheel swerve drive), from `/opt/galaxea/body/hardware.json`

---

## Next Steps

- [x] Topic discovery and DDS connectivity over ethernet
- [x] Arm feedback reading
- [x] Arm joint movement
- [x] Chassis movement (via gatekeeper)
- [x] DiMOS adapters (chassis + arms)
- [x] Keyboard teleop through DiMOS
- [x] Sensor stream integration (wrist cameras, chassis cameras, LiDAR, IMUs)
- [x] Full ControlCoordinator integration with dual-arm + chassis blueprint
- [x] Whole-body adapter
- [ ] Sensor dropout under coordinator load — root cause still open (see below)
- [ ] Torso control adapter (4-DOF, deferred)

---

## Session Log — Sensor Streams & Dual-Arm Coordinator Integration

### What was built

**Sensor streams on adapters** (`dimos/hardware/manipulators/r1pro/adapter.py`,
`dimos/hardware/drive_trains/r1pro/adapter.py`)

Each adapter now subscribes to all sensors physically attached to its hardware
and publishes decoded frames to independent LCM transports on `connect()`.
No changes to `ControlCoordinator` — it remains fully generic.

| Adapter | Sensors → LCM transports |
|---|---|
| `R1ProArmAdapter` (left) | `/r1pro/left_arm/wrist_color`, `/r1pro/left_arm/wrist_depth` |
| `R1ProArmAdapter` (right) | `/r1pro/right_arm/wrist_color`, `/r1pro/right_arm/wrist_depth` |
| `R1ProChassisAdapter` | `/r1pro/chassis/head`, `/r1pro/chassis/chassis_front_left`, `/r1pro/chassis/chassis_front_right`, `/r1pro/chassis/chassis_left`, `/r1pro/chassis/chassis_right`, `/r1pro/chassis/chassis_rear`, `/r1pro/chassis/head_depth`, `/r1pro/chassis/lidar`, `/r1pro/chassis/imu_chassis`, `/r1pro/chassis/imu_torso` |

**Async worker pattern** (prevents blocking the ROS spin thread):

1. ROS spin thread callback → enqueue raw `msg` object (zero-copy, no GIL pressure)
2. Dedicated worker thread per sensor → `bytes(msg.data)` + decode + `transport.broadcast()`
3. All queues are `maxsize=1` (latest-frame semantics — stale frames are replaced)

**Separate rclpy context for sensor subscriptions** (isolated DDS participant):

Sensor subscriptions use a completely separate `rclpy.Context` with its own
`MultiThreadedExecutor` and DDS participant. This prevents control traffic
(arm commands at ~100 Hz) from saturating the shared DDS receive threads and
dropping large camera frames that require UDP fragmentation.

**Crash-resilient spin loop**:

```python
# spin_once in a loop instead of spin() so any callback exception is
# logged and recovered from rather than killing the entire spin thread.
while not sensor_stop.is_set():
    try:
        sensor_executor.spin_once(timeout_sec=0.1)
    except Exception as exc:
        log.warning("sensor executor exception (continuing): %s", exc)
```

**Callback counters in every worker log line** (every 5 seconds):
```
R1 Pro left wrist_color: 150 callbacks, 148 frames broadcast in last 5.0s
```
When sensors drop: `0 callbacks` = DDS stopped delivering; `N callbacks, 0 frames` = decode/broadcast failing.

**Blueprints added** (`dimos/robot/humanoids/r1pro/blueprints.py`,
`dimos/robot/catalog/galaxea.py`, `dimos/robot/all_blueprints.py`):

- `r1pro-dual-mock` — dual-arm + chassis with mock adapters (runs offline)
- `r1pro-full` — dual-arm + chassis with real R1Pro adapters

**Whole-body adapter** — created during this session to unify all robot
subsystems (arms + torso + chassis + sensors) behind a single interface.

---

### The sensor dropout problem (unresolved)

**Symptom**: Sensor LCM topics (`/r1pro/*/wrist_color`, `/r1pro/chassis/head`,
etc.) stop publishing as soon as the ControlCoordinator tick loop starts writing
joint commands (~100 Hz). IMU topics keep working. Happens after 5–30 seconds.
Sometimes fails immediately on the second launch.

**What works**: IMU (small messages, single UDP packet). **What stops**: all
cameras and LiDAR (large messages, require UDP fragment reassembly).

**Fixes tried — none resolved it**:

| Fix | Rationale | Result |
|---|---|---|
| Move `bytes(msg.data)` copy off spin thread | Reduce GIL contention on spin thread | No change |
| Separate `rclpy.Context` for sensors | Independent DDS participant, own UDP receive threads | No change |
| Lambda wrappers for callback signatures | Fixed `TypeError: missing argument '_topic'` that was crashing spin thread | Partial — fixed crash, sensor dropout persists |
| `spin_once` loop with try/except | Any remaining exception survives instead of killing spin thread | Not yet confirmed on hardware |
| Set `_sensor_stop` before `executor.shutdown()` | Clean shutdown ordering | Correctness fix, not related to dropout |

**Current hypothesis**: The spin thread may still be dying due to an exception
in `spin_once()` that originates in rclpy/FastDDS internals (not in user
callbacks). The crash-resilient `spin_once` loop should surface this via
`log.warning("sensor executor exception...")` lines in the logs.

**How to diagnose on next run**: Watch for these log patterns after sensors stop:
- `"sensor spin thread stopped"` → spin thread exited (look for exception above it)
- `"sensor executor exception (continuing): ..."` → exception being swallowed, spinning continues
- Worker log `"0 callbacks, 0 frames"` → DDS not delivering (likely spin thread died)
- Worker log `"N callbacks, 0 frames"` → DDS alive, broadcast failing

**Other candidates not yet ruled out**:
- FastDDS UDP receive buffer overflow (OS socket buffer ~212KB default, camera
  JPEGs ~100KB each, 8 cameras × 30 Hz = 24 MB/s — buffer fills and silently drops)
- FastDDS participant internal state corruption after prolonged mixed-rate traffic
- LCM transport `broadcast()` threading issue under concurrent coordinator writes
