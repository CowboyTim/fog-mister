#!/usr/bin/env python3
"""Build, flash and monitor the fogbox boards.

Boards are listed in boards.toml (MAC, chip, role). The serial port of a
board is found through its MAC: native-USB ESP32-C3/S3 boards report the MAC
as their USB serial number, so swapped /dev/ttyACM* ports do not matter and
nothing is reset just to look. esptool is used to identify new boards and to
flash; it refuses to write when the chip type does not match.

Each role has its own sketch directory: MISTER -> fog_mister/,
TRIGGER -> fog_trigger/.

  python3 fog.py list                        ports, MACs and the board on each
  python3 fog.py identify /dev/ttyACM2       chip + MAC of a new board (boards.toml entry)
  python3 fog.py build   mister|trigger|all
  python3 fog.py flash   mister trigger      flash the last build
  python3 fog.py deploy  all --monitor       build + flash, then serial output
  python3 fog.py monitor mister trigger      serial output, prefixed with the board name

Needs Python 3.11+ (tomllib), pyserial, arduino-cli with the esp32:esp32 core.
"""

import argparse
import glob
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import tomllib

HERE = os.path.dirname(os.path.abspath(__file__))
BOARDS_FILE = os.path.join(HERE, "boards.toml")
DIST = os.path.join(HERE, "dist")
BUILD_CACHE = os.path.join(HERE, ".build")
PLATFORM = "esp32:esp32"
CORE_VERSION = "3.3.12"
SKETCHES = {"MISTER": "fog_mister", "TRIGGER": "fog_trigger"}
ROLES = tuple(SKETCHES)

# Per chip: arduino-cli FQBN, esptool --chip, and how the core derives the soft-AP MAC from the
# base MAC: the C3 is built with 4 universal MACs (AP = base + 1), the S3 with 2 (AP = base with
# the locally administered bit set, e.g. 9c:.. -> 9e:..).
CHIPS = {
    "esp32c3": {"fqbn": "esp32:esp32:esp32c3", "esptool": "esp32c3", "ap_mac": "plus1"},
    "esp32s3": {"fqbn": "esp32:esp32:esp32s3", "esptool": "esp32s3", "ap_mac": "local"},
}
# no_ota partition table: bootloader at 0x0, table at 0x8000, NVS at 0x9000 (the mister's
# settings, never written here), otadata at 0xe000 (boot_app0 selects app0), app at 0x10000.
PARTITIONS = "no_ota"
APP_MAX = 2097152
FLASH_LAYOUT = (
    ("0x0", "{sketch}.ino.bootloader.bin"),
    ("0x8000", "{sketch}.ino.partitions.bin"),
    ("0xe000", "boot_app0.bin"),
    ("0x10000", "{sketch}.ino.bin"),
)
FLASH_BAUD = 921600

# Serial runs over the native USB-Serial/JTAG port.
BASE_FLAGS = "-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1"


def die(msg):
    sys.exit(f"fog: {msg}")


def norm_mac(mac):
    """'E4:B3:23:F9:43:F0', 'e4-b3-...' or 'e4b323f943f0' -> 'e4:b3:23:f9:43:f0' (None if not a MAC)."""
    h = re.sub(r"[^0-9a-fA-F]", "", mac or "")
    if len(h) != 12:
        return None
    return ":".join(h[i:i + 2] for i in range(0, 12, 2)).lower()


def ap_mac(board):
    """The soft-AP MAC of a board, derived from its base MAC the way the core does it."""
    b = [int(x, 16) for x in board["mac"].split(":")]
    if CHIPS[board["chip"]]["ap_mac"] == "plus1":
        n = (int("".join(f"{x:02x}" for x in b), 16) + 1) & 0xFFFFFFFFFFFF
        b = [(n >> (40 - 8 * i)) & 0xFF for i in range(6)]
    else:
        # esp_derive_local_mac(): set the locally administered bit, flip bit 2 if that changes nothing
        first = b[0] | 0x02
        b[0] = first if first != b[0] else first ^ 0x04
    return ":".join(f"{x:02x}" for x in b)


# ---- boards.toml ------------------------------------------------------------

def load_boards():
    with open(BOARDS_FILE, "rb") as f:
        cfg = tomllib.load(f)
    boards = {}
    for name, b in cfg.get("boards", {}).items():
        role = str(b.get("role", "")).upper()
        chip = str(b.get("chip", "")).lower()
        if role not in ROLES:
            die(f"boards.toml: [boards.{name}] role must be one of {', '.join(ROLES)}")
        if chip not in CHIPS:
            die(f"boards.toml: [boards.{name}] chip must be one of {', '.join(CHIPS)}")
        mac = b.get("mac", "")
        if mac and not norm_mac(mac):
            die(f"boards.toml: [boards.{name}] mac '{mac}' is not a MAC address")
        boards[name] = {"name": name, "role": role, "chip": chip,
                        "mac": norm_mac(mac), "mister": b.get("mister")}
    macs = [b["mac"] for b in boards.values() if b["mac"]]
    if len(macs) != len(set(macs)):
        die("boards.toml: the same MAC is listed twice")
    return boards


def resolve(boards, n):
    """One selector (board name, role in any case, or 'all') -> (boards, must be connected).
    A name that is both a board and a role (a board called 'trigger') means the whole role as soon as the
    role has more than one board (a spare), so 'deploy trigger' reaches the spare too."""
    if n.lower() == "all":
        return list(boards.values()), False
    role_hits = [b for b in boards.values() if b["role"] == n.upper()]
    if n in boards and not (len(role_hits) > 1 and n.upper() in ROLES):
        return [boards[n]], True
    return role_hits, len(role_hits) == 1


def select(boards, names):
    """Board names, roles (any case) or 'all' -> list of boards, in boards.toml order."""
    if not names:
        die("name a board, a role or 'all' (see 'fog.py list')")
    picked = []
    for n in names:
        hits, _ = resolve(boards, n)
        if not hits:
            die(f"no board or role '{n}' in boards.toml (have: {', '.join(boards)})")
        picked += [b for b in hits if b not in picked]
    return picked


def select_req(boards, names):
    """Like select(), plus the names of the boards that must be connected: a board named exactly, or the
    only board of a role. A board picked as one of several (a role with a spare, or 'all') is optional."""
    picked = select(boards, names)
    required = set()
    for n in names:
        hits, req = resolve(boards, n)
        if req:
            required.update(b["name"] for b in hits)
    return picked, required


def present(picked, required, port, wait_for_required=False):
    """The boards of a selection that can be used now. Optional boards that have no mac or are not plugged
    in are skipped with a note; a required one is an error (or, for monitor, waited for)."""
    if port:
        return picked
    plugged = {mac for _, mac, _ in list_ports() if mac}
    out = []
    for b in picked:
        why = None
        if not b["mac"]:
            why = "no mac in boards.toml"
        elif b["mac"] not in plugged:
            why = "not connected"
        if not why:
            out.append(b)
        elif b["name"] in required:
            if why == "no mac in boards.toml":
                die(f"{b['name']}: no mac in boards.toml yet; run 'fog.py identify <port>' and fill it in, or pass --port")
            if wait_for_required:
                out.append(b)
            else:
                die(f"{b['name']} ({b['mac']}) is not connected (see 'fog.py list')")
        else:
            print(f"(skipping {b['name']}: {why})")
    if not out:
        die("none of the selected boards is connected (see 'fog.py list')")
    return out


def mister_mac(boards, board):
    """The MISTER board a TRIGGER talks to: the board's 'mister' key, else the only MISTER.
    None (the trigger then finds the mister by broadcast) when it has no mac yet."""
    if board["mister"]:
        peer = boards.get(board["mister"])
        if not peer or peer["role"] != "MISTER":
            die(f"[boards.{board['name']}] mister = '{board['mister']}' is not a MISTER board")
        misters = [peer]
    else:
        misters = [b for b in boards.values() if b["role"] == "MISTER"]
    if len(misters) != 1:
        die(f"{board['name']}: {len(misters)} MISTER boards in boards.toml, "
            f"set mister = \"<name>\" in [boards.{board['name']}]")
    if not misters[0]["mac"]:
        print(f"({board['name']}: mister '{misters[0]['name']}' has no mac in boards.toml, "
              f"the trigger will find it by broadcast)")
    return misters[0] if misters[0]["mac"] else None


# ---- ports ------------------------------------------------------------------

def list_ports():
    """[(device, mac or None, description)] for all USB serial ports."""
    try:
        from serial.tools import list_ports as lp
    except ImportError:
        die("pyserial is missing: pip install pyserial (or apt install python3-serial)")
    out = []
    for p in sorted(lp.comports(), key=lambda p: p.device):
        if p.vid is None:
            continue
        out.append((p.device, norm_mac(p.serial_number), p.description or ""))
    return out


def find_port(board, override=None):
    if override:
        return override
    if not board["mac"]:
        die(f"{board['name']}: no mac in boards.toml yet; run 'fog.py identify <port>' "
            f"and fill it in, or pass --port")
    hits = [dev for dev, mac, _ in list_ports() if mac == board["mac"]]
    if not hits:
        die(f"{board['name']} ({board['mac']}) is not connected (see 'fog.py list')")
    return hits[0]


# ---- tools ------------------------------------------------------------------

def arduino_data_dir():
    """$ARDUINO_DIRECTORIES_DATA, else ./.arduino15 if it exists, else arduino-cli's own
    default (~/.arduino15)."""
    d = os.environ.get("ARDUINO_DIRECTORIES_DATA")
    if not d and os.path.isdir(os.path.join(HERE, ".arduino15")):
        d = os.path.join(HERE, ".arduino15")
    if d:
        os.environ["ARDUINO_DIRECTORIES_DATA"] = d
        return d
    try:
        r = subprocess.run(["arduino-cli", "config", "get", "directories.data"],
                           capture_output=True, text=True, check=True)
        return r.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return os.path.expanduser("~/.arduino15")


def version_key(path):
    return [int(x) if x.isdigit() else 0 for x in re.split(r"[.\-]", path)]


def core_file(pattern):
    """Newest match of a glob under the installed esp32 core's package directory."""
    found = glob.glob(os.path.join(arduino_data_dir(), "packages/esp32", pattern))
    return max(found, key=version_key) if found else None


def esptool_cmd():
    """The esptool of the installed esp32 core, else one on PATH, else 'python3 -m esptool'."""
    found = core_file("tools/esptool_py/*/esptool")
    if found:
        return [found]
    for name in ("esptool", "esptool.py"):
        if shutil.which(name):
            return [name]
    return [sys.executable, "-m", "esptool"]


def esptool_v5(cmd):
    """esptool 5 spells commands write-flash/chip-id, older ones write_flash/chip_id."""
    try:
        r = subprocess.run(cmd + ["version"], capture_output=True, text=True, timeout=30)
        m = re.search(r"(\d+)\.\d+", r.stdout + r.stderr)
        return bool(m) and int(m.group(1)) >= 5
    except (OSError, subprocess.TimeoutExpired):
        return True


V4_NAMES = {"write-flash": "write_flash", "chip-id": "chip_id",
            "default-reset": "default_reset", "hard-reset": "hard_reset",
            "no-reset": "no_reset", "watchdog-reset": "watchdog_reset"}


def run_esptool(args):
    """Run esptool, echo its output, return (exit code, output)."""
    cmd = esptool_cmd()
    if not esptool_v5(cmd):
        args = [V4_NAMES.get(a, a) for a in args]
    print("+ " + " ".join(cmd + args), flush=True)
    try:
        p = subprocess.Popen(cmd + args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    except OSError as e:
        die(f"cannot run esptool: {e}")
    out = []
    for line in p.stdout:
        sys.stdout.write(line)
        out.append(line)
    return p.wait(), "".join(out)


def parse_esptool(out):
    """Chip ('esp32c3') and MAC from esptool output."""
    chip = re.search(r"Detecting chip type\.\.\. *(ESP32[-\w]*)", out) or \
        re.search(r"Chip (?:is|type:) *(ESP32[-\w]*)", out)
    mac = re.search(r"MAC: *([0-9a-fA-F:]{17})", out)
    return (chip.group(1).replace("-", "").lower() if chip else None,
            norm_mac(mac.group(1)) if mac else None)


# ---- commands ---------------------------------------------------------------

def sketch_name(board):
    return SKETCHES[board["role"]]


def out_dir(board):
    return os.path.join(DIST, board["chip"], sketch_name(board))


def cmd_list(args, boards):
    by_mac = {b["mac"]: b for b in boards.values() if b["mac"]}
    seen = set()
    print(f"{'PORT':14} {'MAC':18} {'BOARD':16} {'ROLE':11} CHIP")
    for dev, mac, desc in list_ports():
        b = by_mac.get(mac)
        if b:
            seen.add(b["name"])
            print(f"{dev:14} {mac:18} {b['name']:16} {b['role']:11} {b['chip']}")
        else:
            what = "unknown: fog.py identify " + dev if mac else "(no MAC: " + desc + ")"
            print(f"{dev:14} {mac or '-':18} {what}")
    for b in boards.values():
        if b["name"] not in seen:
            state = "not connected" if b["mac"] else "no mac in boards.toml"
            print(f"{'-':14} {b['mac'] or '-':18} {b['name']:16} {b['role']:11} {b['chip']}  ({state})")


def cmd_identify(args, boards):
    rc, out = run_esptool(["--port", args.port, "chip-id"])
    chip, mac = parse_esptool(out)
    if rc or not mac:
        die(f"esptool could not read {args.port}")
    known = [b for b in boards.values() if b["mac"] == mac]
    print()
    if known:
        b = known[0]
        print(f"{args.port}: {mac} is [boards.{b['name']}] ({b['role']}, {b['chip']})")
        if chip and chip != b["chip"]:
            print(f"WARNING: boards.toml says chip = \"{b['chip']}\", esptool found {chip}")
    else:
        print("New board, add to boards.toml (pick a name and role):\n")
        print(f"[boards.NAME]\nmac  = \"{mac}\"\nchip = \"{chip or '?'}\"\nrole = \"TRIGGER\"")


def mac_flag(name, mac):
    return f"-D{name}=" + ",".join("0x" + x for x in mac.split(":"))


def build_flags(board, boards, args):
    flags = [BASE_FLAGS]
    if board["role"] == "TRIGGER":
        peer = mister_mac(boards, board)
        if peer:
            # ESP-NOW goes to the mister's soft-AP interface, not its base (station) MAC
            flags.append(mac_flag("MISTER_MAC", ap_mac(peer)))
    if os.environ.get("DEBUG") == "1":
        flags.append("-DDEBUG")
    flags += (os.environ.get("EXTRA_FLAGS", "") + " " + " ".join(args.flags or [])).split()
    return " ".join(flags)


def build(board, flags, args):
    chip = CHIPS[board["chip"]]
    sketch = sketch_name(board)
    dst = out_dir(board)
    os.makedirs(dst, exist_ok=True)
    print(f"== build {sketch} for {board['chip']} -> {os.path.relpath(dst, HERE)}")
    print(f"   flags: {flags}", flush=True)
    arduino_data_dir()
    cmd = ["arduino-cli", "compile", "-b", chip["fqbn"],
           "--output-dir", dst,
           "--build-path", os.path.join(BUILD_CACHE, f"{board['chip']}-{sketch}"),
           "--build-property", f"compiler.cpp.extra_flags={flags}",
           "--build-property", f"compiler.c.extra_flags={flags}",
           "--build-property", f"build.extra_flags={flags}",
           "--board-options", f"PartitionScheme={PARTITIONS}",
           os.path.join(HERE, sketch)]
    if args.verbose:
        cmd[2:2] = ["-v"]
    if subprocess.run(cmd).returncode:
        die(f"build of {sketch} for {board['chip']} failed")
    boot_app0 = core_file("hardware/esp32/*/tools/partitions/boot_app0.bin")
    if not boot_app0:
        die("boot_app0.bin not found in the esp32 core, run 'fog.py update'")
    shutil.copy(boot_app0, dst)
    with open(os.path.join(dst, "build.json"), "w") as f:
        json.dump({"sketch": sketch, "role": board["role"], "chip": board["chip"], "flags": flags,
                   "built": time.strftime("%Y-%m-%d %H:%M:%S")}, f, indent=1)


def build_boards(picked, boards, args):
    done = {}      # one build per chip + role, also for a spare board
    for b in picked:
        key = (b["chip"], b["role"])
        flags = build_flags(b, boards, args)
        if key not in done:
            build(b, flags, args)
            done[key] = flags
        elif done[key] != flags:
            # dist/ has one build per chip + sketch; two triggers for two misters don't fit that
            die(f"{b['name']} needs a different {sketch_name(b)} build than another selected board, "
                f"deploy them one at a time")


def cmd_build(args, boards):
    build_boards(select(boards, args.boards), boards, args)


def flash(board, args):
    src = out_dir(board)
    sketch = sketch_name(board)
    bins = [(addr, os.path.join(src, name.format(sketch=sketch))) for addr, name in FLASH_LAYOUT]
    missing = [p for _, p in bins if not os.path.exists(p)]
    if missing:
        die(f"{board['name']}: no build in {os.path.relpath(src, HERE)}, run 'fog.py deploy {board['name']}'")
    try:
        with open(os.path.join(src, "build.json")) as f:
            info = json.load(f)
    except (OSError, ValueError):
        info = {}
    newest_src = max(os.path.getmtime(p) for p in glob.glob(os.path.join(HERE, sketch, "*.*")))
    if os.path.getmtime(bins[-1][1]) < newest_src:
        print(f"WARNING: {board['name']}: the build is older than the sketch sources")
    port = find_port(board, args.port)
    print(f"== flash {board['name']} ({board['role']}, {board['chip']}) on {port}"
          f"  built {info.get('built', '?')}  flags: {info.get('flags', '?')}", flush=True)
    # --no-reset: the board is already in download mode (BOOT held while plugging in or pressing
    # RESET); the RTS/DTR reset is skipped and a watchdog reset starts the new firmware
    before, after = ("no-reset", "watchdog-reset") if args.no_reset else ("default-reset", "hard-reset")
    rc, out = run_esptool(
        ["--chip", CHIPS[board["chip"]]["esptool"], "--port", port, "--baud", str(FLASH_BAUD),
         "--before", before, "--after", after,
         "write-flash", "-z", "--flash-mode", "keep", "--flash-freq", "keep", "--flash-size", "keep"]
        + [x for addr, path in bins for x in (addr, path)])
    chip, mac = parse_esptool(out)
    if rc:
        die(f"flashing {board['name']} on {port} failed (esptool exit {rc}); "
            f"is a monitor still open on {port}?")
    if board["mac"] and mac and mac != board["mac"]:
        die(f"{port} turned out to be {mac}, not {board['name']} ({board['mac']})")
    print(f"== {board['name']} flashed OK ({chip or '?'}, {mac or '?'})")


def cmd_flash(args, boards):
    picked, required = select_req(boards, args.boards)
    use = present(picked, required, args.port)
    if args.port and len(use) > 1:
        die("--port works with one board only")
    for b in use:
        flash(b, args)


def cmd_deploy(args, boards):
    picked, required = select_req(boards, args.boards)
    use = present(picked, required, args.port)      # fails before building when a required board is not there
    if args.port and len(use) > 1:
        die("--port works with one board only")
    build_boards(use, boards, args)
    for b in use:
        flash(b, args)
    if args.monitor:
        cmd_monitor(args, boards)


_say_lock = threading.Lock()


def say(line):
    """One whole line per write, so the lines of two boards never interleave."""
    with _say_lock:
        sys.stdout.write(line + "\n")
        sys.stdout.flush()


def monitor_one(board, port_override, prefix, stop):
    import serial
    tag = f"[{board['name']}] " if prefix else ""
    while not stop.is_set():
        try:
            port = find_port(board, port_override)
        except SystemExit:
            time.sleep(0.5)          # gone (reset, replugged): wait for it to come back
            continue
        try:
            s = serial.Serial(port, 115200, timeout=0.2)
            # Opening raises DTR+RTS (no reset). Drop RTS first: on the USB-Serial/JTAG
            # port RTS set with DTR clear means reset, which pyserial's own order hits.
            s.rts = False
            s.dtr = False
        except (serial.SerialException, OSError):
            time.sleep(0.5)
            continue
        say(f"{tag}--- connected on {port} ---")
        buf = b""
        try:
            while not stop.is_set():
                buf += s.read(4096)
                *lines, buf = buf.split(b"\n")
                for line in lines:
                    say(tag + line.decode("utf-8", "replace").rstrip("\r"))
        except (serial.SerialException, OSError):
            say(f"{tag}--- disconnected ---")
        finally:
            s.close()


def cmd_monitor(args, boards):
    picked, required = select_req(boards, args.boards)
    picked = present(picked, required, args.port, wait_for_required=True)   # a required board may be plugged in later
    if args.port and len(picked) > 1:
        die("--port works with one board only")
    stop = threading.Event()
    threads = [threading.Thread(target=monitor_one, args=(b, args.port, len(picked) > 1, stop),
                                daemon=True) for b in picked]
    for t in threads:
        t.start()
    print("--- monitor: Ctrl-C to quit ---", flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        stop.set()


def cmd_update(args, boards):
    urls = os.environ.get("DEV_URLS", "https://espressif.github.io/arduino-esp32/package_esp32_index.json")
    core = PLATFORM + "@" + os.environ.get("CORE_VERSION", CORE_VERSION)
    arduino_data_dir()
    for c in (["--additional-urls", urls, "update"],
              ["--additional-urls", urls, "core", "install", core],
              ["--additional-urls", urls, "board", "list"]):
        if subprocess.run(["arduino-cli"] + c).returncode:
            die("arduino-cli " + " ".join(c) + " failed")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0],
                                 epilog=__doc__.split("\n", 2)[2],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list", help="serial ports, their MAC and board")
    p = sub.add_parser("identify", help="chip and MAC of the board on a port (resets it)")
    p.add_argument("port")
    sub.add_parser("update", help=f"install the esp32 core ({CORE_VERSION}, or $CORE_VERSION)")
    for name, hlp in (("build", "compile"), ("flash", "flash the last build"),
                      ("deploy", "build + flash"), ("monitor", "serial output")):
        p = sub.add_parser(name, help=hlp)
        p.add_argument("boards", nargs="+", help="board name, role or 'all'")
        p.add_argument("--port", help="use this port instead of finding it by MAC (one board only)")
        if name in ("build", "deploy"):
            p.add_argument("--flags", action="append", help="extra compiler flags, written as --flags='-DX -UY' (also $EXTRA_FLAGS)")
            p.add_argument("-v", "--verbose", action="store_true", help="verbose arduino-cli compile")
        if name in ("flash", "deploy"):
            p.add_argument("--no-reset", action="store_true",
                           help="board is already in download mode (BOOT + RESET): don't reset it first")
        if name == "deploy":
            p.add_argument("--monitor", action="store_true", help="monitor the boards afterwards")
    args = ap.parse_args()
    boards = load_boards()
    {"list": cmd_list, "identify": cmd_identify, "update": cmd_update, "build": cmd_build,
     "flash": cmd_flash, "deploy": cmd_deploy, "monitor": cmd_monitor}[args.cmd](args, boards)


if __name__ == "__main__":
    main()
