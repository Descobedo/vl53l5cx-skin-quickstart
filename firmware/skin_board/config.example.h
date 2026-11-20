// Copy this file to config.h (same folder) and fill it in. config.h is git-ignored.
#pragma once

// ---- Wi-Fi ---------------------------------------------------------------
#define WIFI_SSID "your-network"
#define WIFI_PASS "your-password"

// ---- Transports ------------------------------------------------------------
// SEND_UDP: Wi-Fi (the normal setup). SEND_SERIAL: binary frames over the USB cable
// (bench tests, timing, networks that block UDP). Either or both. With SEND_UDP 0
// the board never touches Wi-Fi.
#define SEND_UDP    1
#define SEND_SERIAL 0
#define SERIAL_BAUD 921600  // native-USB boards (ESP32-C6/S3) ignore it

// ---- Where frames go (UDP) -------------------------------------------------
// Multicast (default): every PC on the network can listen, no IP to configure.
// Unicast: set DEST_IP to the receiving PC's address, e.g. "192.168.1.50".
#define DEST_IP   "239.0.0.1"
// One UDP port per board/skin. Give every board on the network its own port.
#define DEST_PORT 5005

// ---- I2C -------------------------------------------------------------------
// -1 = auto: try 6/7 (SparkFun Qwiic ESP32-C6), 23/22 (generic C6), the board default, ...
#define I2C_SDA -1
#define I2C_SCL -1
#define I2C_HZ  1000000   // VL53L5CX supports 1 MHz; drop to 400000 on long cables

// ---- Sensors ---------------------------------------------------------------
// NUM_SENSORS 0 = auto: use every mux channel (PCA9546A / TCA9548A, all VL53L5CX share
// 0x29) that has a sensor, in channel order; or one sensor straight on the bus if no mux.
// To fix the order instead: set NUM_SENSORS, USE_MUX and SENSOR_MUX_PORT[i] = mux
// channel of sensor i (the ID in each packet).
#define NUM_SENSORS 0
#define USE_MUX     1
#define MUX_ADDR    0x70     // auto-searched in 0x70-0x77 if nothing answers here
static const uint8_t SENSOR_MUX_PORT[] = {0, 1, 2, 3};

#define RANGING_HZ 15     // 8x8 mode tops out at 15 Hz

// ---- Status LED ------------------------------------------------------------
// -1 disables it. On: running. Blinking: a sensor or the mux failed (see serial).
#define LED_PIN -1
