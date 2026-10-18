# fog-mister

Firmware for the fogbox: a chilled low-fog box along a wooded path, fogging
each group just before it arrives.

- `fog_controller/` runs the box: BeamZ fog machine over DMX (GPIO3 via a
  MAX3485), the ultrasonic mister via an SSR (GPIO5), the blower via PWM
  (GPIO4), a STOP toggle (GPIO7) and a group button (GPIO10). It serves a
  web UI on Wi-Fi `fogbox` at http://192.168.4.1.
- `fog_trigger/` sits 100–200 m upstream and sends "group coming" over
  ESP-NOW from a button (GPIO3) or an optional PIR sensor (GPIO4).

Both target an ESP32-C3 on the Espressif Arduino core 3.3. The code is in each
folder's `main.cpp`; the `.ino` files are stubs.

```
./build.sh build                           # both, into dist/<module>
MODULE=fog_controller ./build.sh deploy    # build, upload, monitor
MODULE=fog_trigger ./build.sh deploy
```

The earlier AT-style BLE config firmware is on the `esp-at-ble-config` branch.
