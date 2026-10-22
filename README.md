# cubemars_driver

**CubeMars AK series** actuators on **SocketCAN** (servo mode, extended 29-bit
CAN frames) as a ros2_control hardware plugin, `cubemars_driver/CubeMarsSystem`:
one system per CAN bus, any number of motors on it. On Freya it drives the
steering (`canturn`); the arm (`canarm`) follows.

```
controller (chassis_controller, ...) ──► CubeMarsSystem ──► libcubemars ──► CAN ──► motors
                                          position / velocity commands, position and velocity states
```

- The protocol layer (`libcubemars`) is plain C++ without ROS.
- Fault decoding (over-temperature, over-current, over/under-voltage, encoder, stall), logged.
- Per-joint acceleration ramp, encoder offset, direction, soft position limits.
- `tools/calibrate_zero.py`: the zero reference (`enc_off`) from the live bus.

## Usage

The plugin is configured in the robot's URDF, which models generates from the
robot's hardware config (`models/ugvs/<robot>/hardware.yaml`, section
`ros2_control`); the controller manager of the subsystem loads it (see
[chassis](https://github.com/BrnoMarsRover/chassis#readme)):

```yaml
ros2_control:
  steering:
    plugin: cubemars_driver/CubeMarsSystem
    params: {can_interface: canturn, position_kp: 1.6, position_ki: 0.1}
    joints:
      wheel_base_front_l: {can_id: 104, kt: 0.199, pole_pairs: 21, gear_ratio: 8, enc_off: -2.342, direction: -1.0}
```

The CAN bus has to be up at 1 Mbit before the controller manager starts; on the
robot systemd-networkd brings it up (bmr_system, `robots/<robot>/`). Check that
the motors talk:

```bash
candump -ta canturn      # 0x29XX status frames, one per motor and command
```

> [!WARNING]
> Set a bus up by hand with `listen-only off` (`ip link set canturn up type can
> bitrate 1000000 listen-only off`): a flag left from an earlier session stays,
> and the driver then sends and receives nothing without an error.

## CubeMarsSystem

| Joint command interface | |
|---|---|
| `velocity` | passed to the motor [rad/s at the output] |
| `position` | a PI loop in the plugin turns the position error into a velocity (`position_kp`, `position_ki`, integral limited to 1 rad s); the motors always run in velocity mode |

States: `position` and `velocity` at the output shaft. On activation a position
joint's target is where it is, so it does not jump; deactivation sends zero
speed to every motor. The command timeout is the controller's.

## Parameters

Hardware (`<hardware><param>`):

| Parameter | Default | |
|---|---|---|
| `can_interface` | - | SocketCAN interface, required |
| `position_kp`, `position_ki` | `1.6`, `0.1` | [1/s], [1/s²] the loop of position joints |

Joint (`<joint><param>`):

| Parameter          | Type   | Default    | Description |
| ------------------ | ------ | ---------- | ----------- |
| `can_id`           | int    | —          | Motor CAN id. Required. |
| `kt`               | double | —          | Torque constant, Nm/A. Required. |
| `pole_pairs`       | int    | —          | Motor pole pairs, for the ERPM conversion. Required. |
| `gear_ratio`       | int    | —          | External gear reduction. Required. |
| `enc_off`          | double | `0.0`      | Encoder offset in rad, subtracted from reported position. |
| `max_acceleration` | double | `0.0`      | rad/s², velocity ramp limit. `0` disables ramping. |
| `min_position`, `max_position` | double | `0.0` | Soft travel limits in rad; off while `max <= min`. |
| `position_feedback`| string | `output`   | Where position is measured: `output` or `rotor`. See below. |
| `direction`        | double | `1.0`      | `+1.0` or `-1.0` only. Flips this joint's axis. See below. |

### `direction` — flipping a joint without rewiring it

Mechanically mirrored joints (opposite sides of a chassis, a reversed linkage)
turn the "wrong" way relative to the motor's own sense. `direction: -1.0` inverts
that joint as ROS sees it:

- **feedback** — position, velocity and effort are multiplied by it, applied last,
  after all unit conversion and after `enc_off`
- **commands** — velocity, position and effort targets are multiplied by the same
  factor on the way in

Because it is applied to both paths, the two remain exact inverses and closed-loop
control is unaffected — but **only for ±1**. Feedback computes
`rad = (raw − enc_off)·d` and a command computes `raw = rad·d + enc_off`, which
round-trips solely because `1/d == d` when `d = ±1`. Any other value is rejected
at startup rather than silently corrupting control.

`direction` and `enc_off` are independent and compose: `enc_off` moves the zero,
`direction` chooses which way is positive from that zero.

### Setting the zero reference

`enc_off` is the raw angle at your chosen zero, subtracted *before* `direction`.
[`tools/calibrate_zero.py`](tools/calibrate_zero.py) captures it from the live bus
so you do not have to work it out by hand:

```bash
ros2 run cubemars_driver calibrate_zero.py \
    --config <bmr_system>/workspace/src/bmr/models/ugvs/freya/hardware.yaml \
    --system steering --service bmr_chassis
```

It reads the interface, joints and CAN ids of the system out of the hardware
config, so it works for any robot using this driver. Position the joints where zero should be, press
Enter, and it writes `enc_off` for each — replacing the existing value or adding
one if the config has never been calibrated. Comments and layout are preserved,
and it refuses to write at all unless every configured joint reported.

`--service` stops the given unit first: a joint the controller manager is
holding cannot be positioned by hand. `--install-copy` writes an installed copy
too, for a workspace built without `--symlink-install`.

> [!NOTE]
> An actuator with a single-turn output encoder loses its reference across a
> power cycle, so `enc_off` captured now will be wrong after one. Re-run this
> after power-up rather than trusting a value committed months ago.

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
`joint_states` reports the same angle.

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

The plugin uses `speed` only.

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

## Authors

Standa Svědiroh wrote the driver and the protocol layer (`libcubemars`); Milos
Cihlar moved it to ros2_control (`CubeMarsSystem`).

## Development

```
bmr_build --packages-select cubemars_driver
```

## License

MIT — see [LICENSE](LICENSE).
