# Impedance Implementation Cobot

6-DOF cobot simulation and execution stack with impedance control, using MuJoCo + CycloneDDS.

---

## 1. Clone the Repository

```bash
git clone --recursive git@github.com:shreehank22/impedance-implementation-cobot.git
cd impedance-implementation-cobot
```

---

## 2. Initialise Third-Party Libraries

All dependencies are tracked as git submodules. Initialise them with:

```bash
git submodule update --init --recursive
```

This clones the following libraries into `src/third_party/` at their pinned commits:

| Library | Version |
|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) | 3.1.3 |
| [Pinocchio](https://github.com/stack-of-tasks/pinocchio) | v2.7.0 |
| [Eigen](https://gitlab.com/libeigen/eigen) | nightly |
| [yaml-cpp](https://github.com/jbeder/yaml-cpp) | 0.9.0 |
| [iir1](https://github.com/berndporr/iir1) | 1.10.0 |
| [CycloneDDS](https://github.com/eclipse-cyclonedds/cyclonedds) | 0.5.1 |
| [CycloneDDS-CXX](https://github.com/eclipse-cyclonedds/cyclonedds-cxx) | 11.0.1 |
| [urdfdom_headers](https://github.com/ros/urdfdom_headers) | 3.0.1 |

You also need GLFW installed as a system package (used by the MuJoCo viewer):

```bash
sudo apt install libglfw3-dev
```

---

## 3. Build

Run the setup script to compile and install all third-party libraries:

```bash
./setup_libraries.sh
```

Then compile the full project:

```bash
./compile.sh
```

All binaries are installed to `custom/install/`.

---

## 4. Running the Simulation

The simulation stack has three components that each run in a separate terminal.
All commands are run from `custom/install/` unless noted.

```bash
cd custom/install
```

### Terminal 1 — Physics Simulation (MuJoCo)

Launches the MuJoCo viewer with the cobot mounted on a work table.
Defaults to `scene.xml` (table + robot, no platform).

```bash
./simulate_pv
```

Optional flags:

| Flag | Description |
|---|---|
| `--model <path>` | Override model XML (default: `scene.xml`) |
| `--robot-only` | Load `cobot_c1.xml` only (no table/scene) |
| `--postfix sim\|hw` | DDS topic postfix (default: `sim`) |
| `--robot-name <name>` | Robot name for DDS topics (default: `cobot_c1`) |

The sim publishes sensor data on `rt/cobot_c1/sim/sensor_data` and subscribes
to joint commands on `rt/cobot_c1/sim/joint_command`.

### Terminal 2 — Executor

Runs the FSM, planner, estimator, and controller (including the impedance
control law). Reads config from `src/execution/config/config_c1.yaml`.

Must be launched from `custom/install/` (two levels below project root):

```bash
cd custom/install
./run
```

Key config options in `config_c1.yaml`:

```yaml
comm_postfix: "sim"   # "sim" for MuJoCo, "hw" for hardware
pec_rate: 500         # control loop rate (Hz)
```

FSM states:

| Mode | Key | Description |
|---|---|---|
| SLEEP | `1` | Hold current joint position |
| TELEOP | `2` | Keyboard velocity control of end-effector |
| P2P | `3` | Point-to-point trajectory to a target EE pose |

### Terminal 3 — Keyboard Interface

Unified keyboard controller for all FSM modes.

```bash
./keyboard_interface_dds
```

Optional flags:

| Flag | Description |
|---|---|
| `--postfix sim\|hw` | DDS topic postfix (default: `sim`) |
| `--robot <name>` | Robot name (default: `cobot_c1`) |
| `--test` | Test mode — skips `tcgetattr` (for piped input) |

**Controls:**

| Key | Action |
|---|---|
| `1` | SLEEP |
| `2` | TELEOP |
| `3` | P2P — prompts for target EE pose |
| `W` / `S` | EE forward / backward |
| `A` / `D` | EE up / down |
| `I` / `K` | Wrist pitch +/− |
| `J` / `L` | Wrist roll +/− |
| `Q` / `E` | Base yaw left / right |
| `Z` / `X` | Jaw open / close |
| `Space` | Stop all axes |
| `Ctrl+C` | Quit |

**P2P pose format:**

```
x  y  z  pitch  roll  yaw   (metres / radians)
```

Workspace limits (enforced before trajectory starts):

```
x: 0.22 – 0.52 m
y: ±0.28 m
z: 0.05 – 0.55 m   (5 cm table clearance enforced)
```

All P2P targets are checked before execution:
1. Table clearance — `z ≥ 0.05 m`
2. Workspace box limits
3. Radial distance from base
4. IK reachability — joint angles within `±π`

If any check fails, `REJECTED` is printed and the robot does **not** move.

---

## 5. Optional: P2P File Interface

Run pre-defined waypoint sequences from a YAML file:

```bash
./p2p_interface_dds --file waypoints.yaml --postfix sim
```

Waypoints YAML format:

```yaml
waypoints:
  - pose: [0.35, 0.0, 0.20, 0.0, 0.0, 0.0]
    duration: 3.0
  - pose: [0.30, 0.10, 0.15, 0.0, 0.0, 0.0]
    duration: 4.0
```

Optional flags: `--postfix sim|hw`, `--robot <name>`, `--no-sleep`.

---

## 6. DDS Topics

| Topic | Publisher | Subscriber | Message |
|---|---|---|---|
| `rt/cobot_c1/sim/sensor_data` | `simulate_pv` | `run` | `SensorData_` |
| `rt/cobot_c1/sim/joint_command` | `run` | `simulate_pv` | `JointData_` |
| `rt/c1/joystick_data` | `keyboard_interface_dds` | `run` | `JoyData_` |
| `rt/cobot_c1/sim/cli_data` | `keyboard_interface_dds` / `p2p_interface_dds` | `run` | `CliData_` |

Replace `sim` with `hw` for hardware mode.

---

## 7. Project Structure

```
impedance-implementation-cobot/
├── compile.sh                  — build script
├── setup_libraries.sh          — third-party library setup
├── custom/install/             — compiled binaries and libraries
│   ├── simulate_pv             — MuJoCo physics simulation + DDS PLANT
│   ├── run                     — FSM executor (planner / controller / estimator)
│   ├── keyboard_interface_dds  — keyboard controller (TELEOP + P2P)
│   └── p2p_interface_dds       — file-based P2P waypoint player
├── src/
│   ├── communication/          — DDS layer (QuadDDSComm, keyboard, P2P interfaces)
│   ├── dynamics/               — Cobot kinematics (FK, IK, Jacobian, isReachable)
│   ├── execution/              — FSM, Planner, Controller (impedance control), Estimator
│   │   └── config/config_c1.yaml
│   ├── robot_simulation/       — simulate_pv MuJoCo simulation
│   └── robots/cobot_c1_description/
│       ├── mujoco/scene.xml    — default scene (table + robot)
│       ├── mujoco/cobot_c1.xml — robot model only
│       └── urdf/               — URDF for kinematics
└── src/utils/                  — shared types (memory_types.hpp, utils.hpp)
```
