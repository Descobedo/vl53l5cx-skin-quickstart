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
  (or unicast) on one port per board. Default ports are 5004–5016.
- Packet: `[u8 sensor][64×u16 filtered mm][64×u16 raw mm]`, little-endian,
  257 bytes. Filtered 4000 = no target. In `tools/tof.py`, no target = 0.
- PC tool: `tools/tof.py` with `status`, `view`, `record` and `fake`. It needs
  only numpy (plus matplotlib for `view`).

## Rules

- **Never commit `firmware/skin_board/config.h`.** It holds Wi-Fi credentials
  and is git-ignored. Ask the user for the SSID and password. Don't guess them,
  and don't echo the password back.
- Prefer `status --seconds N --json` over `view`. It's headless and machine-readable.
  Use `view --save shot.png` if you need to see the heatmaps yourself.
- Hardware steps (plugging cables, covering sensors) need the user. Tell them
  exactly what to do, then verify with a command.
- Firewall rules and driver installs change the user's machine. Ask first.

## Step 0: PC side works (no hardware)

```bash
pip install -r requirements.txt
python tests/test_tof.py
```
✅ Prints `all tests passed`. That test runs a fake board over loopback, so
Python, numpy and UDP receive all work.

## Step 1: Collect the hardware facts

Ask the user, or read their project, for:
1. Board model → I2C pins (Qwiic ESP32-C6: SDA 6 / SCL 7; generic C6 dev module: 23 / 22).
2. Mux or not, its type, and its address.
3. How many sensors, and which mux channel each is on.
4. Wi-Fi SSID/password (2.4 GHz), and whether multicast works on that
   network. If unsure, multicast first, unicast as the fallback.
5. Which UDP port this board gets. Every board on the network needs its own.

## Step 2: Configure and build the firmware

```bash
cp firmware/skin_board/config.example.h firmware/skin_board/config.h
# edit config.h: WIFI_SSID, WIFI_PASS, DEST_IP, DEST_PORT, I2C_SDA, I2C_SCL,
#                USE_MUX, MUX_ADDR, NUM_SENSORS, SENSOR_MUX_PORT[]
arduino-cli core install esp32:esp32
arduino-cli lib install "SparkFun VL53L5CX Arduino Library"
arduino-cli compile --fqbn esp32:esp32:esp32c6 firmware/skin_board
```
✅ Ends with `Sketch uses ... bytes`. Use the FQBN of the user's board if it isn't
a generic C6 (`arduino-cli board listall esp32c6`). On Windows, `arduino-cli`
also ships inside the Arduino IDE at
`%LOCALAPPDATA%\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe`.

## Step 3: Flash it and read the boot log

```bash
arduino-cli board list                    # find the port (COMx / /dev/ttyACM0)
arduino-cli upload --fqbn esp32:esp32:esp32c6 -p <PORT> firmware/skin_board
arduino-cli monitor -p <PORT> -c baudrate=115200
```
✅ The log contains `[ready] N/N sensors running`, then
`[rate Hz] s0=12.x ...` every 5 s.

| Log line | Meaning → action |
|---|---|
| stuck at `[wifi] connecting` | Wrong SSID/password, or a 5 GHz-only network |
| `[scan] bus: (none)` | Wrong `I2C_SDA`/`I2C_SCL`, or no power to the mux |
| `no mux at 0x70` but `[scan] bus` shows `0x7X` | Set `MUX_ADDR` to the address shown |
| `[scan] mux port P: 0x70` (no `0x29`) then `sensor i: no VL53L5CX` | Nothing on that channel: fix the cable or `SENSOR_MUX_PORT[i]` |
| `[ready] 3/4` | One sensor failed. The others still stream. Fix it and re-flash |

## Step 4: Receive on the PC

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

## Step 6: Visualise / record

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
- [ ] Sensor index ↔ physical position recorded (Step 5)
- [ ] The user has seen `tof.py view`
