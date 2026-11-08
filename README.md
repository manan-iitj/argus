# ARGUS

ROS 2 workspace sources for ARGUS — a drone simulation stack that combines a
PX4 SITL flight simulator in a custom industrial-zone Gazebo world with a
command controller driven over UDP (e.g. from an external gesture/voice
recognition program).

## Packages

| Package | Description |
|---------|-------------|
| [`argus_controller`](argus_controller/) | C++ ROS 2 node. Bridges a UDP command socket into PX4 offboard control. |
| [`spawn_drone`](spawn_drone/) | Launch files, custom Gazebo worlds, and world-generation tooling. |

## Architecture

```
gesture/voice app ──UDP :5005──▶ UdpCommandReceiver ──/argus/command──▶ ArgusCommandController
                                                                         │
                                                          PX4 topics (/fmu/in/*, /fmu/out/*)
                                                                         │
                                                                    PX4 SITL
                                                                         │
                                                        Gazebo (industrial-zone world)
```

1. **`spawn_drone`** starts PX4 SITL (`gz_x500_gimbal`), the MicroXRCE Agent
   (`udp4 -p 8888`), and Gazebo with a custom world.
2. An external program sends plain-text commands over **UDP port 5005**.
3. **`argus_controller`** receives them, republishes on `/argus/command`, and
   translates them into PX4 offboard setpoints and vehicle commands.

### `argus_controller`

Two nodes in one executable, spun by a `MultiThreadedExecutor`:

- **`udp_command_receiver`** — binds `0.0.0.0:<udp_port>` (default `5005`),
  forwards each command string to `/argus/command`. A cooldown
  (default `1.0 s`) collapses the continuous stream produced by a held
  gesture into single discrete steps; a different command always passes
  immediately.
- **`argus_command_controller`** — state machine
  (`DISCONNECTED → IDLE → ARMING → TAKING_OFF → FLYING → LANDING`) that
  publishes `OffboardControlMode`, `TrajectorySetpoint`, and `VehicleCommand`
  at `control_rate` (default 20 Hz), while consuming `VehicleStatus`,
  `VehicleOdometry`, and `VehicleCommandAck`.

Recognised commands:

| Command | Effect |
|---------|--------|
| `LAUNCH` | Request arming + offboard + takeoff to `takeoff_altitude` |
| `LAND` | Land from `FLYING` / `ARMING` / `TAKING_OFF` |
| `FRONT` / `BACK` | Move `movement_distance` along current yaw |
| `LEFT` / `RIGHT` | Strafe `movement_distance` perpendicular to yaw |
| `UP` / `DOWN` | Change altitude by `altitude_step` |
| `INSPECT` | Log-only inspection marker (requires `FLYING`) |

Parameters are set in
[`argus_controller/launch/argus_controller.launch.py`](argus_controller/launch/argus_controller.launch.py).

### `spawn_drone`

[`spawn_drone/launch/spawn_drone.launch.py`](spawn_drone/launch/spawn_drone.launch.py)
kills leftover sim processes, pre-warms the Gazebo world (required for worlds
that pull many models from Gazebo Fuel, since PX4 only waits ~30 s), then
launches PX4 SITL and the MicroXRCE Agent.

| Launch argument | Default | Description |
|-----------------|---------|-------------|
| `px4_dir` | *(local path)* | Path to the `PX4-Autopilot` checkout |
| `world` | `industrial_zone_real` | World name (without `.sdf`) |
| `model_target` | `gz_x500_gimbal` | PX4 SITL Gazebo target |
| `gcs_host` | `host.docker.internal` | Ground-control station host |

**Worlds** — see [`spawn_drone/README.md`](spawn_drone/README.md) for details:

- `industrial_zone` — procedural, lightweight, fully offline.
- `industrial_zone_real` — same layout with real 3D models from
  [Gazebo Fuel](https://app.gazebosim.org); needs internet on first launch.

Both are produced by [`spawn_drone/tools/gen_industrial_zone.py`](spawn_drone/tools/gen_industrial_zone.py).

## Requirements

- ROS 2 Jazzy
- [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) (for `spawn_drone`)
- [Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
- Gazebo (Harmonic or newer)
- [`px4_msgs`](https://github.com/PX4/px4_msgs) — **not included in this
  repository**; clone it into your workspace:

  ```bash
  cd <your_ws>/src
  git clone https://github.com/PX4/px4_msgs.git
  ```

## Build & run

```bash
cd <your_ws>
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs spawn_drone argus_controller
source install/setup.bash

# Terminal 1 — simulation + world
ros2 launch spawn_drone spawn_drone.launch.py px4_dir:=/path/to/PX4-Autopilot

# Terminal 2 — command controller
ros2 launch argus_controller argus_controller.launch.py
```

Send commands over UDP:

```bash
echo -n "LAUNCH" | nc -u -w1 127.0.0.1 5005
echo -n "FRONT"  | nc -u -w1 127.0.0.1 5005
echo -n "LAND"   | nc -u -w1 127.0.0.1 5005
```

## Repository layout

```
src/
├── argus_controller/          # C++ UDP → PX4 offboard bridge
│   ├── include/argus_controller/
│   ├── src/
│   └── launch/
├── spawn_drone/               # Simulation launch + worlds
│   ├── launch/
│   ├── worlds/                # SDF worlds + materials/textures
│   └── tools/                 # gen_industrial_zone.py
└── README.md
```

## License

- `argus_controller` — MIT
- `spawn_drone` — Apache-2.0
