#!/usr/bin/env python3
"""Set the zero reference (enc_off) for CubeMars joints, from the live bus.

Reads the joints, CAN ids and interface straight out of a cubemars_driver config
file, captures where each motor currently is, and writes that back as enc_off --
so the pose the arm is in when you run this becomes 0.000 rad.

The driver computes:      position = (raw * positionScale - enc_off) * direction
so enc_off is the RAW angle at your chosen zero, before direction is applied.
That is what this captures, which is why it works for both a joint with
direction +1 and one with -1.

  ./calibrate_zero.py --config <path/to/config.yaml> [options]

Options:
  --install-copy PATH   also write the installed copy (colcon copies configs, so
                        editing only the source has no effect until a rebuild)
  --service NAME        stop this systemd unit before capturing and start it
                        afterwards. Use it when the driver is actively holding
                        the joints -- a damped joint cannot be positioned by hand.
  --duration SEC        how long to sample the bus (default 1.5)
  --yes                 do not ask before writing

Offsets are captured exactly as the driver reads them, unwrapped. Actuators with
a single-turn output encoder lose their reference on power cycle, so expect to
re-run this after one.
"""

import argparse, math, os, re, struct, subprocess, sys, time

try:
    import yaml
except ImportError:
    sys.exit("PyYAML is required (it ships with ROS 2). Try: pip3 install pyyaml")

STATUS_PREFIX = 0x29  # CubeMars servo-mode status frames are 0x2900 | can_id


def load_config(path):
    """Pull interface + {joint: (can_id, direction)} out of a driver config."""
    with open(path) as f:
        doc = yaml.safe_load(f)
    # The node key is normally the '/**:' wildcard, but accept any single root.
    for root in doc.values():
        params = root.get("ros__parameters")
        if not params:
            continue
        iface = params.get("can_interface", "can0")
        joints = {}
        for name in params.get("joints", []):
            j = params.get(name, {})
            if "can_id" not in j:
                sys.exit(f"config: joint '{name}' has no can_id")
            joints[name] = (int(j["can_id"]), float(j.get("direction", 1.0)))
        if joints:
            return iface, joints
    sys.exit(f"config: no joints found in {path}")


def capture(iface, joints, duration):
    print(f"Sampling {duration}s of {iface} ...")
    out = subprocess.run(["timeout", str(duration), "candump", iface],
                         capture_output=True, text=True).stdout
    pat = re.compile(r"^\s*\S+\s+0000(\w{4})\s+\[8\]\s+([0-9A-Fa-f]{2})\s+([0-9A-Fa-f]{2})")
    latest = {}
    for line in out.splitlines():
        m = pat.match(line)
        if not m:
            continue
        full = int(m.group(1), 16)
        if (full >> 8) != STATUS_PREFIX:
            continue
        raw = struct.unpack(">h", bytes([int(m.group(2), 16), int(m.group(3), 16)]))[0]
        latest[full & 0xFF] = raw

    offsets, missing = {}, []
    for name, (can_id, direction) in joints.items():
        if can_id not in latest:
            missing.append((name, can_id))
            continue
        # NOT wrapped to [-pi, pi). The driver does not wrap either -- it computes
        # position = raw * positionScale - enc_off straight from the int16 field,
        # which spans +-3276.7 deg. Wrapping here but not there puts the two out of
        # step by a whole turn for any joint sitting outside +-180 deg: it produced
        # exactly 2*pi on a joint whose true reading was +263.7 deg. The captured
        # offset must mirror the driver's arithmetic exactly.
        raw_rad = latest[can_id] * 0.1 * math.pi / 180.0
        offsets[name] = raw_rad
        print(f"  {name:8s} CAN {can_id:<4d} dir {direction:+.0f}   "
              f"{raw_rad:+.4f} rad ({math.degrees(raw_rad):+7.2f} deg)")
    if missing:
        print()
        for name, can_id in missing:
            print(f"  ! no status frames from {name} (CAN {can_id})")
        return None
    return offsets


def update_yaml(path, offsets):
    """Rewrite enc_off in place, preserving comments and layout.

    Replaces an existing enc_off under each joint, or inserts one if the joint
    has none yet -- a config that has never been calibrated has no such line.

    Deliberately line-based rather than load-and-dump: round-tripping through
    PyYAML would discard every comment in the file, and these configs carry the
    notes explaining where the CAN ids and gear ratios came from.

    Refuses to write unless every joint was located. An earlier version appended
    the missing ones as fresh blocks at the end, which is silently catastrophic:
    YAML keeps the LAST of duplicate keys, so the joint would have ended up with
    only enc_off and no can_id.
    """
    with open(path) as f:
        lines = f.readlines()

    out, wrote = [], set()
    cur, cur_indent, blanks = None, 0, []

    def emit_enc_off():
        out.append(" " * (cur_indent + 2) + f"enc_off: {offsets[cur]}\n")
        wrote.add(cur)

    for line in lines:
        stripped = line.strip()
        indent = len(line) - len(line.lstrip())

        if cur is not None:
            if stripped.startswith("enc_off:"):
                out.extend(blanks); blanks = []
                emit_enc_off()
                cur = None
                continue
            if not stripped:
                blanks.append(line)       # hold: the block may end here
                continue
            if indent > cur_indent:
                out.extend(blanks); blanks = []
                out.append(line)
                continue
            # dedent => end of this joint's block; enc_off goes at its foot
            emit_enc_off()
            cur = None
            out.extend(blanks); blanks = []
            # fall through and handle this line normally

        out.extend(blanks); blanks = []
        key = stripped[:-1] if stripped.endswith(":") else None
        if key in offsets and key not in wrote:
            cur, cur_indent = key, indent
        out.append(line)

    if cur is not None:
        emit_enc_off()
    out.extend(blanks)

    missing = set(offsets) - wrote
    if missing:
        raise RuntimeError(f"{path}: could not locate joint block(s): {', '.join(sorted(missing))} "
                           "- file left untouched")

    with open(path, "w") as f:
        f.writelines(out)


def systemctl(action, unit):
    print(f"{action}ing {unit} ...")
    if subprocess.run(["sudo", "systemctl", action, unit]).returncode != 0:
        sys.exit(f"  ! systemctl {action} {unit} failed")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True)
    ap.add_argument("--install-copy")
    ap.add_argument("--service")
    ap.add_argument("--duration", type=float, default=1.5)
    ap.add_argument("--yes", action="store_true")
    a = ap.parse_args()

    iface, joints = load_config(a.config)
    print(f"=== CubeMars zero calibration ===\n")
    print(f"config    : {a.config}")
    print(f"interface : {iface}")
    print(f"joints    : {', '.join(joints)}\n")

    stopped = False
    try:
        if a.service:
            systemctl("stop", a.service)
            for _ in range(20):
                if subprocess.run(["pgrep", "-f", "cubemars_node"],
                                  capture_output=True).returncode != 0:
                    break
                time.sleep(0.5)
            stopped = True
            print("  driver stopped, joints are free\n")

        input("Position the joints at their zero pose, then press Enter to capture. ")
        offsets = capture(iface, joints, a.duration)
        if offsets is None:
            print("\nAborted - not every joint reported. Nothing written.")
            return

        if not a.yes:
            if input("\nWrite these as enc_off? [y/N] ").strip().lower() not in ("y", "yes"):
                print("Skipped writing.")
                return

        for path in filter(None, (a.config, a.install_copy)):
            update_yaml(path, offsets)
            print(f"  wrote {path}")
    finally:
        if stopped:
            print()
            systemctl("start", a.service)
            print("\nDone. The state topic should now read ~0 for every joint.")


if __name__ == "__main__":
    main()
