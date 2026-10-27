# AGENTS.md

Guidance for coding agents working in this repository. See
[README.md](README.md) for what the fogbox is and how it is used.

## Layout

- `fog_mister/main.cpp` – mister box firmware (ESP32-S3 SuperMini): DMX fog
  machine, SSR-switched ultrasonic mister, PWM blower, web UI, cycle timing,
  ESP-NOW receiver. "Mister" is the box/board; `MISTER_PIN` / `mister_on` are
  the ultrasonic mister output inside it.
- `fog_trigger/main.cpp` – trigger node firmware (ESP32-S3 SuperMini): button / PIR,
  ESP-NOW sender.
- `fog_*/fog_*.ino` – empty stubs. Keep them empty; all code goes in
  `main.cpp` so the Arduino IDE doesn't generate prototypes.
- `docs/gen_schematics.py` – generates `docs/fog_mister.svg` and
  `docs/fog_trigger.svg`. Never hand-edit the SVGs.
- `deploy.py` – build / flash / monitor tool; `boards.toml` lists the physical
  boards (MAC, chip, role `MISTER` or `TRIGGER`). Each role keeps its own
  sketch directory.
- Branch `esp-at-ble-config` holds the old AT-style BLE config firmware; it
  is parked, don't merge it or build on it.

## Build and check

There is no test suite and no hardware in CI. The minimum check for any
firmware change is a clean compile of both sketches:

```sh
python3 deploy.py build all
```

`deploy.py` uses `$ARDUINO_DIRECTORIES_DATA`, else `./.arduino15` if it exists,
else arduino-cli's default. If the ESP32 core is installed elsewhere, point at
it:

```sh
ARDUINO_DIRECTORIES_DATA=$HOME/.arduino15 python3 deploy.py build all
```

Don't flash boards unless asked: `deploy.py flash/deploy` writes to whatever
board in `boards.toml` is plugged in.

Target: Espressif Arduino core 3.3.x, chip from `boards.toml` (`esp32s3` now,
`esp32c3` must keep compiling), `no_ota` partitions, USB CDC on boot. Code that
differs per chip goes under `#if CONFIG_IDF_TARGET_ESP32S3` (see `LED_PIN`). Use the core 3.x APIs (`ledcAttach`/`ledcWrite` by pin,
`esp_now_recv_info_t` receive callback).

After changing `docs/gen_schematics.py`, regenerate and look at the result
(e.g. `rsvg-convert docs/fog_mister.svg -o /tmp/c.png`) for overlapping
labels or wires:

```sh
python3 docs/gen_schematics.py docs
```

## Things that must stay in sync

- **ESP-NOW protocol:** `FogMsg`, `FOG_MAGIC` and the `MSG_*` types are
  duplicated in both sketches. Change both together; bump `FOG_MAGIC` if the
  format changes incompatibly.
- **`ESPNOW_CHANNEL`** must be equal on both boards and match the
  mister box's AP channel. `deploy.py` passes the trigger the mister box's
  soft-AP MAC as `-DMISTER_MAC`, derived per chip in `ap_mac()` (S3: locally
  administered bit set, C3: base + 1). The mister prints its AP MAC at boot.
- **Pin assignments:** a pin change touches the `#define`s in `main.cpp`, the
  schematic generator (then regenerate the SVGs) and the pin tables in
  `README.md`.
- **Defaults and behaviour:** cycle timings, settings defaults, LED patterns
  and web UI buttons are documented in `README.md`; update it with the code.

## Safety rules (don't relax without being asked)

The mister box drives a 1200 W fog machine and mains-powered gear outdoors
near people. Keep these in `fog_mister`:

- No fog during the warm-up (`WARMUP_MS`, 4 min after power-on).
- Bursts capped at `MAX_BURST_MS` (15 s), at least `MIN_GAP_MS` (10 s)
  between bursts.
- STOP (GPIO7 switch or web) turns fog, ultrasonic mister and blower off immediately.
- Fallback cycles stop after `FALLBACK_MAX` in a row without a real trigger.
- Outputs default off at boot; the 10 kΩ pull-downs on the MOSFET triggers
  are part of that.
- The ultrasonic mister's DC side stays isolated from the 12 V ground (SSR in its
  positive lead).

## Code style

- C++ in Arduino style, 4-space indent, `snake_case` functions and
  variables, `UPPER_CASE` defines, short `/* ---- section ---- */` headers.
- No dynamic allocation in the main loop; fixed-size arrays and `snprintf`
  for JSON.
- Everything time-based uses `millis()` with wrap-safe comparisons
  (`elapsed()` / `(int32_t)(now - t)`).
- Work done in the ESP-NOW receive callback stays minimal: copy the message
  into a queue or volatile fields and handle it in `loop()`.
- The web UI is a single inline HTML page in `PAGE`; keep it dependency-free
  (no external scripts or fonts, the phone has no internet on `fogbox`).

## Git

- Default branch is `main`. Keep commits focused and describe the hardware
  impact (pins, timings, safety) in the message.
- Don't commit build output (`dist/`, `.build/`, `build/`, `.arduino15/`).
