# cubemars_driver

ROS 2 driver for **CubeMars AK series** actuators on **SocketCAN**, using the
CubeMars *servo mode* protocol (extended 29-bit CAN frames).

Any number of motors on one CAN bus are driven from a single node. Each motor is
exposed as a named joint, commanded and reported through `sensor_msgs/JointState`.

## Features

- Four control modes per joint: **current**, **velocity**, **position**, **position+speed**
- Per-joint acceleration ramping, torque limit and encoder offset
- **Command timeout watchdog** — motors are stopped if commands stop arriving
- Fault decoding (over-temperature, over-current, over/under-voltage, encoder, stall)
- `read_only` joints, for motors driven by something else that you only want to observe
- Protocol layer (`libcubemars`) is plain C++ with no ROS dependency, so it can be
  reused from a test harness or a non-ROS program

## Build

```bash
cd ~/asgard/workspace
colcon build --packages-select cubemars_driver
source install/setup.bash
```

## Bring up the CAN interface

The driver does **not** configure the bus; do it before launching. CubeMars ships
at 1 Mbit:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 listen-only off loopback off
```

> [!WARNING]
> `ip link set ... up type can bitrate ...` does **not** clear control-mode flags
> left over from a previous session. If `listen-only` is still set the driver will
> open the socket, transmit nothing, receive nothing, and report no error. Always
> pass `listen-only off` explicitly. Check with `ip -details link show can0` —
> the output must not contain `LISTEN-ONLY`.

Verify motors are talking before you launch anything:

```bash
candump -ta can0        # expect 0x29XX status frames, one burst per motor
```

## Run

```bash
ros2 launch cubemars_driver cubemars.launch.py \
    robot_name:=freya robot_number:=1 config:=manipulator.yaml
```

### Launch arguments

| Argument       | Default            | Description                                        |
| -------------- | ------------------ | -------------------------------------------------- |
| `use_sim_time` | `False`            | Use simulation clock if true.                      |
| `robot_name`   | —                  | Robot name (`freya`). Also picks `config/<robot_name>/`. |
| `robot_number` | `''`               | Appended to the robot name (`freya_1`).            |
| `config`       | `manipulator.yaml` | Parameter file inside `config/<robot_name>/`.      |
| `state`        | `~/state`          | Output `JointState` topic.                         |
| `command`      | `~/command`        | Input `JointState` topic.                          |

### Topics

| Topic       | Type                       | Direction | Notes |
| ----------- | -------------------------- | --------- | ----- |
| `~/state`   | `sensor_msgs/JointState`   | published | Measured position, velocity, effort per joint, at `update_rate`. |
| `~/command` | `sensor_msgs/JointState`   | subscribed| Field read depends on each joint's `control_mode`. |

Which field of the command message is used depends on the joint's `control_mode`:
`velocity` reads `velocity[]`, `position`/`position_speed` read `position[]`, and
`current` reads `effort[]`. Joints are matched **by name**, so a command message may
address any subset of the configured joints, in any order.

> [!IMPORTANT]
> Commands must keep arriving. If nothing is received for `command_timeout_ms`
> the node sends a zero-speed command to every joint and logs a warning. This is
> a safety feature — publish continuously, not once.

## Parameters

Global:

| Parameter            | Type     | Default  | Description |
| -------------------- | -------- | -------- | ----------- |
| `can_interface`      | string   | `can0`   | SocketCAN interface name. |
| `update_rate`        | double   | `100.0`  | Hz. Feedback read, command send and state publish rate. |
| `command_timeout_ms` | double   | `100.0`  | Stop motors if no command arrives within this window. |
| `joints`             | string[] | `[]`     | Joint names. Each needs its own parameter block. Required. |

Per joint (nested under the joint name):

| Parameter          | Type   | Default    | Description |
| ------------------ | ------ | ---------- | ----------- |
| `can_id`           | int    | —          | Motor CAN id. Required. |
| `kt`               | double | —          | Torque constant, Nm/A. Required. |
| `pole_pairs`       | int    | —          | Motor pole pairs, for the ERPM conversion. Required. |
| `gear_ratio`       | int    | —          | External gear reduction. Required. |
| `control_mode`     | string | `velocity` | `current`, `velocity` (alias `speed`), `position`, `position_speed`. |
| `enc_off`          | double | `0.0`      | Encoder offset in rad, subtracted from reported position. |
| `pos_vel_limit`    | int    | `0`        | Speed limit used in `position_speed` mode. |
| `pos_acc_limit`    | int    | `0`        | Acceleration limit used in `position_speed` mode. |
| `trq_limit`        | double | `0.0`      | Nm. Exceeding it stops all motors. `0` disables. |
| `max_acceleration` | double | `0.0`      | rad/s², velocity ramp limit. `0` disables ramping. |
| `read_only`        | bool   | `false`    | Never transmit to this motor, only decode its feedback. |
| `position_feedback`| string | `output`   | Where position is measured: `output` or `rotor`. See below. |

### `position_feedback` — getting position into the right frame

CubeMars actuators differ in where the position sensor sits, and the driver cannot
detect which kind it is talking to. Set this per joint:

| Value    | Use for | Meaning |
| -------- | ------- | ------- |
| `output` | actuators with an output-shaft encoder, e.g. **AK60-39** | Reported angle is already post-reduction. Used as-is. |
| `rotor`  | actuators without one, e.g. **AK40-10** | Angle is measured before the reduction and is divided by `gear_ratio`. |

Both directions are handled: reported position is scaled into output radians, and
position *commands* have the inverse applied, so commanded and measured angles are
always in the same frame.

Getting this wrong scales position by `gear_ratio` — a 10x or 39x error, not a
subtle one. If you are unsure, rotate the joint by a known amount and check that
`~/state` reports the same angle.

> [!NOTE]
> Output-side encoders are typically **single-turn and lose their reference across
> a power cycle**, so treat their absolute value as valid only within one output
> revolution and re-home after every restart. Rotor-side feedback is `int16` at
> 0.1 deg, so it wraps past ±3276.7 deg *before* the reduction — with a 10:1 that
> is only about ±0.9 turns of output travel. Neither is suitable for a
> continuously rotating joint without unwrapping on top.

`velocity` and `effort` are always derived through `gear_ratio` regardless of this
setting, because the speed field is electrical RPM measured at the rotor in both
cases.

## Protocol notes

CubeMars servo mode, extended (29-bit) identifiers throughout.

**Commands** — `can_id | (mode << 8)`, 8 data bytes, big-endian `int32` payload:

| Mode             | Value | ID       | Payload |
| ---------------- | ----- | -------- | ------- |
| `current`        | 1     | `0x01xx` | mA |
| `speed`          | 3     | `0x03xx` | ERPM |
| `position`       | 4     | `0x04xx` | 1e-4 deg |
| `set_origin`     | 5     | `0x05xx` | — |
| `position_speed` | 6     | `0x06xx` | 1e-4 deg, then int16 speed and accel limits |

**Status** — the motor replies on `0x2900 | can_id` (packet type 41):

| Bytes | Field    | Scale |
| ----- | -------- | ----- |
| 0–1   | position | int16, 0.1 deg |
| 2–3   | speed    | int16, 10 ERPM |
| 4–5   | current  | int16, 0.01 A |
| 6     | temperature | int8, °C |
| 7     | fault    | uint8 |

`ERPM = rad/s × pole_pairs × gear_ratio × 60 / 2π`.

The receive filter matches on the **low byte only** (`mask 0xFF`), so a motor is
identified by id regardless of which packet type it sends. Only status type `0x29`
is decoded; if a motor is configured to emit other status packets they will be
parsed with the type-1 layout and produce nonsense.

Whether the reported position already includes the gear reduction depends on the
actuator, so it is selected per joint with
[`position_feedback`](#position_feedback--getting-position-into-the-right-frame).

## Identifying which motor is which joint

Motors do not broadcast unsolicited — a silent bus does not mean a broken one. To
map CAN ids to physical joints, command one motor to hold its **current** position
and see which joint goes stiff. Engage only while that joint is still: the target
is captured at engagement, so engaging against a moving joint makes the motor drive
back to a stale pose. Bound the effort with a current limit, and damp to a stop
before releasing rather than dropping straight to zero torque — a loaded joint
released at zero torque will swing.

## License

MIT — see [LICENSE](LICENSE).
