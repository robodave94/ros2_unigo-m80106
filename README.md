# ros2-unigo-m80106

ROS 2 Jazzy packages for driving **Unitree GO-M8010-6** motors over RS-485 on the
**Unitree Unigo M80106** robot. The repository ships two packages:
a header-only library (`m80106_lib`) that bundles the Unitree SDK shared object for
x86\_64 and ARM64, and a set of CLI executables (`m80106_execs`) for motor
discovery, configuration, live telemetry monitoring, and motion verification.

---

## Package Overview

| Package | Type | Purpose |
|---------|------|---------|
| `m80106_lib` | `ament_cmake` INTERFACE | Header-only library + bundled Unitree SDK `.so` (x86\_64 & arm64). No external SDK download required. |
| `m80106_execs` | `ament_cmake` | CLI executables for motor discovery, ID management, telemetry, and motion testing. |

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| ROS 2 Jazzy | [Installation guide](https://docs.ros.org/en/jazzy/Installation.html) |
| `serial` (ROS 2) | Use the [robodave94/serial-ros2](https://github.com/robodave94/serial-ros2) fork. Clone into your workspace `src/`. |
| gcc ≥ 9 | Standard on Ubuntu 24.04 (Jazzy default). |
| USB RS-485 adapter | Default PID:VID `0403:6011` (FT4232H). Configurable via ROS parameter. |

---

## Getting the Source

This repository uses a Git submodule for the Unitree actuator SDK.
Clone with `--recurse-submodules` to populate it automatically:

```bash
git clone --recurse-submodules <repo-url>
```

If you have already cloned without it:

```bash
git submodule update --init --recursive
```

---

## Build

```bash
cd /path/to/ros2_ws
colcon build --packages-select m80106_lib m80106_execs
source install/setup.bash
```

---

## `m80106_lib` — Library Package

A header-only ament CMake package. Downstream packages depend on it via:

```cmake
find_package(m80106_lib REQUIRED)
target_link_libraries(my_target m80106_lib::m80106_lib)
```

### User-facing headers (`include/m80106_lib/`)

| Header | Purpose |
|--------|---------|
| `motor_types.hpp` | Constants, enums (motor IDs, modes), and unit-conversion helpers |
| `motor_bus.hpp` | RS-485 adapter discovery by USB PID:VID |
| `motor_driver.hpp` | Typed `SerialPort` wrapper + `scanMotors()` |
| `control_modes.hpp` | Scalable control-mode abstraction (PD, PI, impedance, direct torque, CRTP extension base) |

### Bundled SDK

`lib/libUnitreeMotorSDK_M80106_Linux64.so` (x86\_64) and
`lib/libUnitreeMotorSDK_M80106_Arm64.so` (aarch64) are selected automatically by
CMake based on `CMAKE_SYSTEM_PROCESSOR`. No manual download is needed.

---

## `m80106_execs` — CLI Executables

All executables accept the common `pidvid` parameter to target a specific RS-485
adapter (default `"0403:6011"`).

### `actuator_ping`

Discovers all USB-serial adapters matching the given PID:VID and pings every
Unitree GO-M8010-6 actuator on each bus (IDs 0–14). Prints a per-port summary
of responding motor IDs.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pidvid` | string | `"0403:6011"` | USB PID:VID of the RS-485 adapter |

```bash
ros2 run m80106_execs actuator_ping
ros2 run m80106_execs actuator_ping --ros-args -p pidvid:=0403:6011
```

---

### `scan_motor_info`

Live, htop-style terminal monitor for GO-M8010-6 motor telemetry. Continuously
polls each detected motor via brake commands and renders position, velocity,
torque, and temperature in a colour-coded UI. Press **`q`** to quit.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pidvid` | string | `"0403:6011"` | USB PID:VID of the RS-485 adapter |
| `target_id` | int | `-1` | If ≥ 0, monitor only this motor ID |
| `rate_hz` | double | `20.0` | Poll / refresh rate (Hz) |

```bash
ros2 run m80106_execs scan_motor_info
ros2 run m80106_execs scan_motor_info --ros-args -p target_id:=0 -p rate_hz:=10.0
```

---

### `motor_operation`

Preprogrammed ~30 s motion-verification routine that exercises every major
control mode in sequence:

| Step | Mode | Action |
|------|------|--------|
| 1/5 | BRAKE | Read initial position and telemetry |
| 2/5 | VELOCITY | +5 rad/s for 3 s, then −5 rad/s for 3 s (output side) |
| 3/5 | POSITION | Move to +0.5 rad, then −0.5 rad, then return to start |
| 4/5 | TORQUE | +1.5 N·m for 2 s, then −1.5 N·m for 2 s |
| 5/5 | BRAKE | Safe final state |

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `pidvid` | string | `"0403:6011"` | USB PID:VID of the RS-485 adapter |
| `motor_id` | int | `0` | Motor ID to operate (0–14) |

```bash
ros2 run m80106_execs motor_operation --ros-args -p motor_id:=0
ros2 run m80106_execs motor_operation --ros-args -p motor_id:=2 -p pidvid:=0403:6011
```

---

## Tools Submodule (`m80106_lib/tools`)

**Source:** [`unitreerobotics/unitree_actuator_sdk`](https://github.com/unitreerobotics/unitree_actuator_sdk)

This submodule is located at `m80106_lib/tools/` and contains the upstream
Unitree actuator SDK with C++ and Python examples for A1, B1, and GO-M8010-6
motors, plus pybind11 Python bindings and the [Motor Tools](#motor-tools-m80106_libtoolsmotor_tools)
pre-compiled utilities.

> **Important:** This submodule is **not required** to build `m80106_lib` or
> `m80106_execs` via `colcon`. The `m80106_lib` package bundles its own copies of
> the required `.so` files. The submodule is useful for running standalone SDK
> examples and for the `motor_tools` configuration utilities.

### Standalone SDK build (optional)

```bash
cd m80106_lib/tools
mkdir build && cd build
cmake ..
make
# Run examples with sudo:
sudo ./example_goM8010_6_motor
```

Requirements: gcc ≥ 5.4.0 (x86) or ≥ 7.5.0 (ARM). Check with `gcc --version`.

Python examples are in `m80106_lib/tools/python/`:

```bash
cd m80106_lib/tools/python
sudo python3 example_goM8010_6_motor.py
```

---

## Motor Tools (`m80106_lib/tools/motor_tools`)

The `motor_tools` directory contains **pre-compiled RS-485 configuration utilities**
for Linux (x86\_64, arm64, arm32). These tools communicate with the motor over
RS-485 to modify internal configuration parameters — operations such as changing
the motor ID, switching firmware modes, and upgrading firmware. They do **not**
require a ROS installation.

Full upstream documentation: [`m80106_lib/tools/motor_tools/README.md`](m80106_lib/tools/motor_tools/README.md)

### Tools

| Tool | Function |
|------|---------|
| `changeid` | Change the motor's RS-485 communication ID |
| `swboot` | Switch motor to **Bootloader mode** (required before firmware upgrade) |
| `swmotor` | Switch motor to **Motor mode** (default operating mode; configuration cannot be modified in this state) |
| `unisp` | Upgrade motor firmware |
| `cancelboot` | Rescue/recover a motor stuck in Bootloader mode (e.g. after a failed upgrade) |

### Platform Downloads (v1.2.4)

Pre-compiled binaries are bundled in this repository under the following directories:

| Platform | Directory |
|----------|-----------|
| x86\_64 Linux | `m80106_lib/tools/motor_tools/Unitree_MotorTools_v1.2.4_x86_64_Linux/` |
| ARM64 Linux | `m80106_lib/tools/motor_tools/Unitree_MotorTools_v1.2.4_arm64_Linux/` |
| ARM32 Linux | `m80106_lib/tools/motor_tools/Unitree_MotorTools_v1.2.4_arm32_Linux/` |

Each directory contains: `unisp`, `changeid`, `swboot`, `swmotor`, `cancelboot`.

### Setup

Downloaded binaries do not have execute permission by default. Grant it before first use:

```bash
cd m80106_lib/tools/motor_tools/Unitree_MotorTools_v1.2.4_x86_64_Linux/
sudo chmod 777 ./unisp ./changeid ./swboot ./swmotor ./cancelboot
```

The tools do not require root themselves, but the `/dev/ttyUSB*` device must be
accessible. Either run with `sudo`, or grant device permissions:

```bash
sudo chmod 777 /dev/ttyUSB0
```

### Usage Examples

**Change a motor's ID** (current ID `0` → new ID `2`):

```bash
sudo ./changeid /dev/ttyUSB0 0 2
```

**Switch to Bootloader mode** (required before `unisp`):

```bash
sudo ./swboot /dev/ttyUSB0
```

**Switch back to Motor mode:**

```bash
sudo ./swmotor /dev/ttyUSB0
```

**Upgrade firmware:**

```bash
sudo ./unisp /dev/ttyUSB0 <firmware_file>
```

**Rescue a bricked motor:**

```bash
sudo ./cancelboot /dev/ttyUSB0
```

### Identifying the Current Motor Mode (LED)

The green LED on the back of the motor indicates the current mode:

| Mode | LED Behaviour |
|------|--------------|
| **Motor mode** (default) | Slow blink |
| **Bootloader mode** | Fast blink × 3 |

> If you need to change a motor ID, use the `changeid` tool from `motor_tools`
> — see the [Motor Tools](#motor-tools-m80106_libtoolsmotor_tools) section below.

---

## RS-485 Bus Notes

- Up to **15 motors** can share a single RS-485 bus (IDs 0–14).
- Every motor on a shared bus **must have a unique ID**. Use `actuator_ping` or
  `scan_motor_info` to verify the current ID assignments before adding a motor.
- The default USB RS-485 adapter PID:VID is `0403:6011` (FT4232H). Pass a
  different value via the `pidvid` ROS parameter if your adapter differs.

---

## kp / kd Rotor-Side Conversion

All SDK commands are specified on the **rotor** side of the gearbox.
Given a gear ratio $r$, convert output-side gains to rotor-side gains by:

$$kp_{\text{rotor}} = \frac{kp_{\text{output}}}{r^2}$$

$$kd_{\text{rotor}} = \frac{kd_{\text{output}}}{r^2}$$

The GO-M8010-6 gear ratio and any motor-specific magic numbers are documented
in the SDK examples (`m80106_lib/tools/example/`).

---

## License

BSD-3-Clause. See individual package `package.xml` files for details.
The bundled Unitree SDK shared objects are subject to Unitree's own license.
For questions about the SDK, contact support@unitree.com.

