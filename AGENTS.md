# AGENTS.md: set up VL53L5CX skin sensors

You are helping someone get VL53L5CX time-of-flight sensors (on ESP32 boards)
streaming to their PC. Work through the steps in order. **Each step has a
check. Don't move on until it passes.** [`README.md`](README.md) has the
human-oriented detail.

## Facts you need

- Sensor: VL53L5CX, 8×8 zones, 45° FOV, ≤ 4 m, ≤ 15 Hz. Every unit sits at I2C `0x29`.
  More than one per board needs a mux (PCA9546A/TCA9548A, default `0x70`).
- Board: ESP32 (tested ESP32-C6, core `esp32:esp32` 3.3.x), library
  "SparkFun VL53L5CX Arduino Library". The sketch is `firmware/skin_board/skin_board.ino`.
- Transport: one UDP datagram per sensor frame, to multicast `239.0.0.1`
  (or unicast) on one port per board. Default ports are 5004–5016. Alternatively
  USB serial (`SEND_SERIAL 1`), read with `--serial <port>`. Serial needs no
  Wi-Fi, firewall or credentials, so it's the fastest way to prove the hardware works.
- Packet: `[u8 sensor][64×u16 filtered mm][64×u16 raw mm][u32 board µs][u16 counter]`,
  little-endian, 263 bytes. Filtered 4000 = no target. In `tools/tof.py`, no target = 0.
- The firmware auto-detects I2C pins and sensors by default (`I2C_SDA -1`,
  `NUM_SENSORS 0`). The boot log says what it found.
- Native-USB ESP32s (C6, S3) need `CDCOnBoot=cdc` in the FQBN, or the serial
  port stays silent.
- PC tool: `tools/tof.py` with `status`, `view`, `record`, `timing` and `fake`.
  It needs numpy, plus matplotlib for `view`/`timing --save` and pyserial for `--serial`.

## Rules

- **Never commit `firmware/skin_board/config.h`.** It holds Wi-Fi credentials
  and is git-ignored. Ask the user for the SSID and password. Don't guess them,
  and don't echo the password back.
- Prefer `status --seconds N --json` over `view`. It's headless and machine-readable.
  Use `view --save shot.png` if you need to see the heatmaps yourself.
- Hardware steps (plugging cables, covering sensors) need the user. Tell them
  exactly what to do, then verify with a command.
- Firewall rules and driver installs change the user's machine. Ask first.
- Flashing overwrites whatever the board runs now. Ask first, and offer a backup:
  `esptool --port <PORT> read-flash 0 <size> backup.bin`. Read the partition table
  at `0x8000` to see how much is in use. On native-USB C6 boards, big reads can
  stall with "Packet content transfer stopped": read in 64 KB pieces and retry
  failures in smaller ones.

## Step 0: PC side works (no hardware)

```bash
pip install -r requirements.txt
python tests/test_tof.py
```
✅ Prints `all tests passed`. That test runs a fake board over loopback, so
Python, numpy and UDP receive all work.

## Step 1: Collect the hardware facts

Ask the user, or read their project, for:
1. Board model and its FQBN. `arduino-cli board list` identifies the chip,
   and a native-USB C6 shows as VID `303a`.
2. Wiring (pins, mux, channels) is optional. Auto-detect finds it. Ask only if
   they need a fixed sensor order.
3. Transport. Start with **serial** (`SEND_UDP 0`, `SEND_SERIAL 1`) to prove
   the hardware. Then switch to UDP, which needs a 2.4 GHz SSID/password the PC
   shares (not a guest network that isolates clients) and one UDP port per board.

## Step 2: Configure and build the firmware

```bash
cp firmware/skin_board/config.example.h firmware/skin_board/config.h
# edit config.h: SEND_UDP/SEND_SERIAL, and for UDP: WIFI_SSID, WIFI_PASS, DEST_IP, DEST_PORT
# (pins/mux/sensors auto-detect; set NUM_SENSORS + SENSOR_MUX_PORT[] only to fix the order)
arduino-cli core install esp32:esp32
arduino-cli lib install "SparkFun VL53L5CX Arduino Library"
arduino-cli compile --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc firmware/skin_board
```
✅ Ends with `Sketch uses ... bytes`. Use the FQBN of the user's board if it isn't
a generic C6 (`arduino-cli board listall esp32c6`). On Windows, `arduino-cli`
also ships inside the Arduino IDE at
`%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`.

## Step 3: Flash it and read the boot log

```bash
arduino-cli board list                    # find the port (COMx / /dev/ttyACM0)
arduino-cli upload --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc -p <PORT> firmware/skin_board
python tools/tof.py status --serial <PORT> --log --seconds 15   # serial mode: log + frames
# UDP mode: arduino-cli monitor -p <PORT> -c baudrate=115200
```
✅ The log contains `[auto] N sensor(s)` and `[ready] N/N sensors running`,
then `[rate Hz] s0=15.0 ... read_ms avg=3.3` every 5 s. In serial mode, `status` also lists
every sensor.

| Log line | Meaning → action |
|---|---|
| stuck at `[wifi] connecting` | Wrong SSID/password, or a 5 GHz-only network |
| nothing at all | Missing `CDCOnBoot=cdc`, or the wrong port |
| `[i2c] ... (nothing answers)` on every pin pair | No power to the mux/sensors, or unusual pins: set `I2C_SDA`/`I2C_SCL` |
| `[auto] 0 sensor(s)` but the mux is found | No sensor answers on any channel: check cables |
| `no mux at 0x70` but `[scan] bus` shows `0x7X` | Set `MUX_ADDR` to the address shown |
| `[scan] mux port P: 0x70` (no `0x29`) then `sensor i: no VL53L5CX` | Nothing on that channel: fix the cable or `SENSOR_MUX_PORT[i]` |
| `[ready] 3/4` | One sensor failed. The others still stream. Fix it and re-flash |

## Step 4: Receive on the PC (UDP)

Skip this step if you're staying on serial.

```bash
python tools/tof.py status --ports <DEST_PORT> --seconds 5 --json
```
✅ Exit code 0. `sensors` lists every index `0..N-1`, each with `hz` ≥ 8
(typically 12–15).

If `sensors` is empty (exit 1), but Step 3 passed:
1. Check the PC is on the same Wi-Fi/LAN as the board, not a guest network.
2. Windows firewall (ask first, needs admin):
   `netsh advfirewall firewall add rule name="tof skin" dir=in action=allow protocol=UDP localport=5004-5016`
3. Multicast join on the right adapter: add `--iface <PC's IP on that network>`.
4. Still nothing: set `DEST_IP` in `config.h` to the PC's IP (unicast) and re-flash.

## Step 5: Map the sensors (with the user)

Ask the user to cover one sensor at a time with their hand, about 5 cm away,
while you watch:

```bash
python tools/tof.py status --ports <DEST_PORT>
```
✅ The covered sensor's `nearest` drops to roughly 30–80 mm. Write down
physical position → sensor index. Do it once per skin. The index order is
the order of `SENSOR_MUX_PORT[]`, which may not match the physical layout.

Also: zones that read under ~60 mm with nothing in front mean the skin's own
shell is in view. Record those zone indices so the user's code can mask them.

## Step 6: Timing and data over time

```bash
python tools/tof.py timing --seconds 20 --save timing.png --json   # add --serial <PORT> or --ports <P>
```
✅ JSON `clock` is `"board"` (the packets carry board time). Each sensor has
`rate_hz` around 15 (or whatever `RANGING_HZ` is), `interval_ms.std` of a few
ms, and `lost` 0 on serial. Report each sensor's rate, interval spread,
phase and drift, and show the user `timing.png`. Sensors are not synchronised:
non-zero `drift_ms_per_s` is normal (a reference board measured -5 to +9 ms/s).
If rates sit near 13 Hz with many `gaps` but `lost` 0, the board's reads are too slow:
check that `build_opt.h` is in the sketch folder and the heartbeat's `read_ms` is about 3 ms
(about 19 ms without it).

## Step 7: Visualise / record

```bash
python tools/tof.py view --ports <DEST_PORT>            # user-facing live window
python tools/tof.py view --ports <DEST_PORT> --save shot.png --seconds 5   # for you
python tools/tof.py record session.npz --ports <DEST_PORT> --seconds 60
```
✅ `view --save` prints `N sensor panel(s) -> shot.png`. `record` prints
`saved K frames`.

## Integrating into another codebase

- Copy `parse_packet` and `zone_rays` from `tools/tof.py`, or use
  `examples/minimal_receiver.py`. Both are dependency-light.
- Point in the sensor frame: `zone_rays()[k] * mm[k] / 1000` (metres, z out of the
  sensor). To place it on the robot, apply the sensor's mounting transform
  (from the skin's CAD/URDF) and the link's forward kinematics.
- With several boards, tell them apart by UDP port and keep a
  `port → skin` table in config.
- The hardware can't tell the robot's own body from an obstacle. Filter
  self-returns against the robot model.

## Done when

- [ ] `python tests/test_tof.py` passes
- [ ] The boot log shows `[ready] N/N`
- [ ] `status --json` shows every sensor at ≥ 8 Hz
- [ ] `timing` report and figure shared with the user
- [ ] Sensor index ↔ physical position recorded (Step 5)
- [ ] The user has seen `tof.py view`
