// Copy this file to config.h (same folder) and fill it in. config.h is git-ignored.
#pragma once

// ---- Wi-Fi ---------------------------------------------------------------
#define WIFI_SSID "your-network"
#define WIFI_PASS "your-password"

// ---- Where frames go -------------------------------------------------------
// Multicast (default): every PC on the network can listen, no IP to configure.
// Unicast: set DEST_IP to the receiving PC's address, e.g. "192.168.1.50".
#define DEST_IP   "239.0.0.1"
// One UDP port per board/skin. Give every board on the network its own port.
#define DEST_PORT 5005

// ---- I2C -------------------------------------------------------------------
// SparkFun Qwiic ESP32-C6 boards: 6 / 7. Generic ESP32-C6 dev module: 23 / 22.
#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_HZ  1000000   // VL53L5CX supports 1 MHz; drop to 400000 on long cables

// ---- Sensors ---------------------------------------------------------------
// USE_MUX 1: sensors sit behind a PCA9546A / TCA9548A mux (all VL53L5CX share 0x29).
// USE_MUX 0: exactly one sensor wired straight to the board.
#define USE_MUX     1
#define MUX_ADDR    0x70
#define NUM_SENSORS 4
// SENSOR_MUX_PORT[i] = mux channel of sensor i. Sensor i is the ID in each packet.
static const uint8_t SENSOR_MUX_PORT[NUM_SENSORS] = {0, 1, 2, 3};

#define RANGING_HZ 15     // 8x8 mode tops out at 15 Hz

// ---- Status LED ------------------------------------------------------------
// -1 disables it. On: running. Blinking: a sensor or the mux failed (see serial).
#define LED_PIN -1
