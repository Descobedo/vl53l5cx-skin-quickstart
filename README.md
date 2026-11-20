# VL53L5CX skin quickstart

Get VL53L5CX time-of-flight sensors on a robot skin streaming to your PC
and see them live.

```
VL53L5CX x N ──I2C──> mux (PCA9546A / TCA9548A) ──> ESP32-C6 ──Wi-Fi UDP──> PC: tools/tof.py
```

![Live viewer: one 8x8 depth panel per sensor](docs/viewer.png)

**Pointing an AI agent at this repo?** Send it to [`AGENTS.md`](AGENTS.md).
It has the same steps, each with a check that proves the step worked.

---

## 1. What you need

| Part | Notes |
|---|---|
| VL53L5CX breakouts | 8×8 zones, 45° field of view, up to ~4 m, up to 15 Hz. Each one sits at I2C address `0x29`. |
| I2C mux | PCA9546A (4 channels) or TCA9548A (8). It's needed because every sensor has the same address. Skip it for a single sensor. |
| ESP32-C6 board | e.g. a SparkFun Qwiic ESP32-C6. Other ESP32s work if you set the I2C pins. |
| 2.4 GHz Wi-Fi | The PC and the boards must be on the same network. |
| PC | Python 3.9+ with `numpy` and `matplotlib`. |

## 2. Wiring

```
ESP32-C6            mux (0x70)              sensors
 3V3 ───────────────  VIN ─────────────────  VIN   (every sensor)
 GND ───────────────  GND ─────────────────  GND   (every sensor)
 SDA (GPIO 6) ──────  SDA      SD0/SC0 ────  sensor 0 SDA/SCL
 SCL (GPIO 7) ──────  SCL      SD1/SC1 ────  sensor 1 SDA/SCL
                               SD2/SC2 ────  sensor 2 ...
```

- With Qwiic cables, plug them in and you're done. Qwiic carries 3.3 V, GND, SDA and SCL.
- Sensor *i* in the firmware is whatever is plugged into `SENSOR_MUX_PORT[i]`.
  That index is the ID in every packet.
- Long or daisy-chained cables: lower `I2C_HZ` to `400000`.

## 3. Flash the firmware

1. Install the **Arduino IDE** (or `arduino-cli`), the **esp32** core by
   Espressif (3.x), and the **SparkFun VL53L5CX Arduino Library** from the
   Library Manager.
2. Copy `firmware/skin_board/config.example.h` to `config.h` in the same folder.
   Set your Wi-Fi, the port (one per board), the I2C pins, the number of
   sensors and their mux channels. `config.h` is git-ignored.
3. Board: **ESP32C6 Dev Module** (or your exact board). Upload. With
   `arduino-cli`:

   ```bash
   arduino-cli core install esp32:esp32
   arduino-cli lib install "SparkFun VL53L5CX Arduino Library"
   arduino-cli compile --fqbn esp32:esp32:esp32c6 firmware/skin_board
   arduino-cli upload  --fqbn esp32:esp32:esp32c6 -p COM5 firmware/skin_board   # your port
   ```

4. Open the serial monitor at **115200**. A healthy boot looks like this:

   ```
   [wifi] ip=192.168.1.42 rssi=-51 dBm
   [udp] sending to 239.0.0.1:5005
   [scan] bus: 0x70
   [scan] mux port 0: 0x29 0x70
   [sensor 0] ok
   ...
   [ready] 4/4 sensors running
   [rate Hz] s0=12.4 s1=12.4 s2=12.4 s3=12.4  wifi=up
   ```

   Each sensor takes a few seconds to initialise: the library uploads ~90 KB of
   firmware to every VL53L5CX at boot.

## 4. See the data

```bash
pip install -r requirements.txt

python tools/tof.py status            # text: rate, nearest distance, % invalid zones per sensor
python tools/tof.py view              # live heatmaps (near = red, grey = no target)
python tools/tof.py view --numbers    # with the mm value in each zone
python tools/tof.py record run1.npz --seconds 60
```

By default it listens on ports **5004–5016**, multicast and unicast. Narrow
that with `--ports 5005`.

**No hardware yet?** Run a fake board, then `status` or `view` in a second
terminal:

```bash
python tools/tof.py fake --sensors 4 --port 5005
```

## 5. Use it in your own code

[`examples/minimal_receiver.py`](examples/minimal_receiver.py) is about 20 lines
and needs only numpy. Or import the helpers:

```python
import sys; sys.path.insert(0, 'path/to/vl53l5cx-skin-quickstart/tools')
from tof import parse_packet, zone_rays
sensor, mm, raw = parse_packet(datagram)       # mm: (64,) uint16, 0 = no target
points_m = zone_rays() * (mm[:, None] / 1000)  # (64, 3) in the sensor frame, z out of the sensor
```

### Wire format

One UDP datagram per sensor frame, little-endian, 257 bytes:

| Bytes | Content |
|---|---|
| 0 | sensor index (0…N−1 on that board) |
| 1–128 | 64 × `uint16` filtered distance, mm. **4000 = no target** (the sensor's `target_status == 255`). |
| 129–256 | 64 × `uint16` raw distance, mm, unfiltered |

- Zone *k* is row `k // 8`, column `k % 8`. Check which way is "up" on your
  mounting by waving a hand across the sensor.
- Older firmware sends only the first 129 bytes. `tof.py` accepts both.
- Boards are told apart by **port**, sensors by the **index byte**.

### `record` output (`.npz`)

| Array | Shape | Meaning |
|---|---|---|
| `timestamps` | (N,) uint64 | receive time, ns since epoch |
| `ports`, `sensor_ids` | (N,) | which board, which sensor |
| `readings_mm` | (N, 64) uint16 | filtered mm, 0 = no target |
| `raw_mm` | (N, 64) uint16 | raw mm (zeros if the board sends short packets) |
| `sources` | (N,) str | sender IP |

## 6. Troubleshooting

| Symptom | Fix |
|---|---|
| Serial: `no mux at 0x70` / `[scan] bus: (none)` | Wrong pins: check `I2C_SDA`/`I2C_SCL` (Qwiic C6: 6/7; generic C6: 23/22). Check power and cables. If an address jumper is set, the mux sits at 0x71–0x77. |
| Serial: `sensor N: no VL53L5CX at 0x29` | Look at that line's `[scan] mux port P:`. With no `0x29` there, the cable or the mux channel is wrong. |
| Serial stops at `[wifi] connecting...` | The network must be 2.4 GHz. Check the SSID and password. |
| `tof.py status`: "no frames yet" while the board says `[ready]` | (1) The PC is on a different network or a guest Wi-Fi that isolates clients. (2) The **Windows firewall** blocks inbound UDP. Allow it once from an admin shell: `netsh advfirewall firewall add rule name="tof skin" dir=in action=allow protocol=UDP localport=5004-5016`. (3) Multicast is blocked: pass `--iface <your PC's IP>`, or set `DEST_IP` in `config.h` to your PC's IP (unicast). |
| `[warn] port ...: Address already in use` | Another receiver holds the port. Close it. |
| Rate well below 12 Hz | Wi-Fi is weak (RSSI below −75 dBm) or the I2C bus is overloaded. Each board polls its sensors one after another. |
| Some zones always read 1–60 mm | The skin's shell is in that zone's view. Mask them in your code. |
| Many zones invalid (grey) | Normal past ~2–4 m, on dark or angled surfaces, and in sunlight. |

## Layout

```
firmware/skin_board/   ESP32 sketch + config.example.h
tools/tof.py           status / view / record / fake (single file)
examples/              minimal receiver
tests/                 python tests/test_tof.py  (no hardware needed)
AGENTS.md              step-by-step setup for AI agents
```

The wire format matches the firmware used on HIRO Lab's proximity skins, so
existing recordings and tools that expect it will work.
