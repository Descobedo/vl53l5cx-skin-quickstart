// VL53L5CX skin board: N sensors (8x8, up to 15 Hz) -> one packet per frame.
//
// Wire format (little-endian, 263 bytes):
//   [uint8 sensor index][64 x uint16 filtered mm][64 x uint16 raw mm]
//   [uint32 board time, us, when the frame was read][uint16 per-sensor frame counter]
// "filtered" holds 4000 where the sensor reports no target (target_status 255).
// The first 257 bytes are the classic HIRO format; readers that stop there still work.
//
// Transports (config.h): SEND_UDP over Wi-Fi, SEND_SERIAL over the USB cable. Serial
// frames are wrapped as [0xA5 0x5A][uint16 length][packet]; log lines are plain text.
//
// Needs: ESP32 Arduino core (tested on 3.3.x, ESP32-C6) and
//        "SparkFun VL53L5CX Arduino Library" from the Library Manager.
// Native-USB boards (ESP32-C6/S3): build with "USB CDC On Boot: Enabled"
// (arduino-cli: --fqbn esp32:esp32:esp32c6:CDCOnBoot=cdc) or the log never shows.
// Fill in config.h first (copy config.example.h).
//
// build_opt.h (next to this file) turns off the sensor outputs this sketch doesn't send
// (ambient, signal, sigma, reflectance, ...). Each frame read then moves a fraction of
// the bytes over I2C. That matters: the board reads its sensors one after another, so
// N sensors x read time must stay under the 66.7 ms frame period, or every sensor
// drops below 15 Hz. The heartbeat prints the read time (read_ms) to check it.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"

#ifndef SEND_UDP
#define SEND_UDP 1
#endif
#ifndef SEND_SERIAL
#define SEND_SERIAL 0
#endif
#ifndef SERIAL_BAUD
#define SERIAL_BAUD 921600   // native-USB boards ignore it; USB-UART bridges need it high
#endif

static const int ZONES = 64;
static const uint16_t NO_TARGET_MM = 4000;
static const uint8_t SENSOR_ADDR = 0x29;
static const int MAX_SENSORS = 8;

SparkFun_VL53L5CX imagers[MAX_SENSORS];
uint8_t muxPort[MAX_SENSORS];    // mux channel of sensor i (unused without a mux)
bool alive[MAX_SENSORS];
uint32_t frames[MAX_SENSORS];
uint16_t seq[MAX_SENSORS];
uint32_t readUsSum = 0, readUsMax = 0, readCount = 0;
int nSensors = 0;
bool haveMux = false;
uint8_t muxAddr = MUX_ADDR;

VL53L5CX_ResultsData results;
WiFiUDP udp;
IPAddress destIp;
static const int PKT_LEN = 1 + 4 * ZONES + 4 + 2;
uint8_t pkt[PKT_LEN];

void led(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
}

bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Select one mux channel (PCA9546A: 0-3, TCA9548A: 0-7).
bool selectChannel(uint8_t ch) {
  Wire.beginTransmission(muxAddr);
  Wire.write(uint8_t(1u << ch));
  return Wire.endTransmission() == 0;
}

bool selectSensor(uint8_t i) {
  return haveMux ? selectChannel(muxPort[i]) : true;
}

void scanBus(const char *label) {
  Serial.printf("[scan] %s:", label);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cAck(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " (none)");
}

// Start I2C on the configured pins, or (I2C_SDA = -1) try common pairs until a mux or sensor answers.
void startI2C() {
  int pins[][2] = {{I2C_SDA, I2C_SCL}, {6, 7}, {23, 22}, {SDA, SCL}, {8, 9}, {21, 22}};
  int first = (I2C_SDA >= 0) ? 0 : 1, last = (I2C_SDA >= 0) ? 1 : 6;
  for (int k = first; k < last; k++) {
    Wire.end();
    Wire.begin(pins[k][0], pins[k][1]);
    Wire.setClock(I2C_HZ);
    bool any = i2cAck(SENSOR_ADDR);
    for (uint8_t a = 0x70; a <= 0x77 && !any; a++) any = i2cAck(a);
    Serial.printf("[i2c] sda=%d scl=%d %d Hz%s\n", pins[k][0], pins[k][1], I2C_HZ, any ? "" : "  (nothing answers)");
    if (any || I2C_SDA >= 0) return;
  }
  Serial.println("[error] no mux or sensor on any common I2C pins: check power and wiring, or set I2C_SDA/I2C_SCL");
}

// Fill nSensors / muxPort[]. NUM_SENSORS > 0 uses the config; 0 finds them.
void findSensors() {
  if (!i2cAck(muxAddr)) {                       // mux somewhere else in 0x70-0x77?
    for (uint8_t a = 0x70; a <= 0x77; a++) if (i2cAck(a)) { muxAddr = a; break; }
  }
#if NUM_SENSORS > 0
  haveMux = USE_MUX;
  nSensors = NUM_SENSORS > MAX_SENSORS ? MAX_SENSORS : NUM_SENSORS;
  for (int i = 0; i < nSensors; i++) muxPort[i] = SENSOR_MUX_PORT[i];
  if (haveMux && !i2cAck(muxAddr))
    Serial.printf("[error] no mux at 0x%02X: check wiring, I2C pins, MUX_ADDR\n", MUX_ADDR);
#else
  haveMux = i2cAck(muxAddr);
  if (haveMux) {
    Serial.printf("[auto] mux at 0x%02X; looking for sensors on channels 0-7\n", muxAddr);
    for (uint8_t ch = 0; ch < 8 && nSensors < MAX_SENSORS; ch++) {
      if (selectChannel(ch) && i2cAck(SENSOR_ADDR)) muxPort[nSensors++] = ch;
    }
    selectChannel(muxPort[0]);
  } else if (i2cAck(SENSOR_ADDR)) {
    Serial.println("[auto] no mux; one sensor directly on the bus");
    nSensors = 1;
  }
  Serial.printf("[auto] %d sensor(s)", nSensors);
  for (int i = 0; i < nSensors && haveMux; i++) Serial.printf("  s%d=ch%d", i, muxPort[i]);
  Serial.println();
#endif
}

void setup() {
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  Serial.begin(SERIAL_BAUD);
  delay(2000);
  Serial.println("\n[boot] vl53l5cx skin board");

#if SEND_UDP
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  WiFi.setSleep(false);  // power save adds 100+ ms latency spikes
  Serial.printf("\n[wifi] ip=%s rssi=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  destIp.fromString(DEST_IP);
  Serial.printf("[udp] sending to %s:%d\n", DEST_IP, DEST_PORT);
#endif
#if SEND_SERIAL
  Serial.println("[serial] binary frames on this port (A5 5A len packet)");
#endif

  startI2C();
  scanBus("bus");
  findSensors();

  int ok = 0;
  for (int i = 0; i < nSensors; i++) {
    alive[i] = false;
    frames[i] = 0;
    seq[i] = 0;
    if (!selectSensor(i)) { Serial.printf("[error] sensor %d: mux select failed\n", i); continue; }
    if (haveMux) {
      char label[24];
      snprintf(label, sizeof label, "mux port %d", muxPort[i]);
      scanBus(label);  // expect 0x29 (plus the mux itself)
    }
    Serial.printf("[sensor %d] init (uploads ~90 KB firmware, a few seconds)...\n", i);
    if (!imagers[i].begin()) { Serial.printf("[error] sensor %d: no VL53L5CX at 0x29\n", i); continue; }
    imagers[i].setResolution(ZONES);
    imagers[i].setRangingFrequency(RANGING_HZ);
    imagers[i].startRanging();
    alive[i] = true;
    ok++;
    Serial.printf("[sensor %d] ok\n", i);
  }
  Serial.printf("[ready] %d/%d sensors running\n", ok, nSensors);
  led(ok > 0 && ok == nSensors);
}

void put16(int at, uint16_t v) { pkt[at] = v & 0xFF; pkt[at + 1] = v >> 8; }

void sendFrame(uint8_t id, uint32_t t_us) {
  pkt[0] = id;
  for (int z = 0; z < ZONES; z++) {
    uint16_t raw = uint16_t(results.distance_mm[z]);
    put16(1 + 2 * z, (results.target_status[z] == 255) ? NO_TARGET_MM : raw);
    put16(1 + 2 * ZONES + 2 * z, raw);
  }
  put16(1 + 4 * ZONES, t_us & 0xFFFF);
  put16(3 + 4 * ZONES, t_us >> 16);
  put16(5 + 4 * ZONES, seq[id]++);
#if SEND_UDP
  udp.beginPacket(destIp, DEST_PORT);
  udp.write(pkt, PKT_LEN);
  udp.endPacket();
#endif
#if SEND_SERIAL
  const uint8_t hdr[4] = {0xA5, 0x5A, PKT_LEN & 0xFF, PKT_LEN >> 8};
  Serial.write(hdr, 4);
  Serial.write(pkt, PKT_LEN);
#endif
}

uint32_t lastReport = 0;
bool blink = false;

void loop() {
  // Poll every sensor; send whichever has a new frame. Never blocks on one sensor.
  for (int i = 0; i < nSensors; i++) {
    if (!alive[i] || !selectSensor(i)) continue;
    if (imagers[i].isDataReady()) {
      uint32_t t_us = micros();  // board time the frame was found ready (poll granularity)
      if (!imagers[i].getRangingData(&results)) continue;
      uint32_t took = micros() - t_us;
      readUsSum += took; readCount++; if (took > readUsMax) readUsMax = took;
      sendFrame(i, t_us);
      frames[i]++;
    }
  }

  uint32_t now = millis();
  if (now - lastReport >= 5000) {  // a heartbeat you can read on the serial monitor
    Serial.print("[rate Hz]");
    for (int i = 0; i < nSensors; i++) {
      Serial.printf(" s%d=%.1f", i, alive[i] ? frames[i] * 1000.0f / (now - lastReport) : 0.0f);
      frames[i] = 0;
    }
    if (readCount)
      Serial.printf("  read_ms avg=%.1f max=%.1f", readUsSum / 1000.0f / readCount, readUsMax / 1000.0f);
    readUsSum = readUsMax = readCount = 0;
#if SEND_UDP
    Serial.printf("  wifi=%s", WiFi.status() == WL_CONNECTED ? "up" : "DOWN");
#endif
    Serial.println();
    lastReport = now;
    bool allOk = nSensors > 0;
    for (int i = 0; i < nSensors; i++) allOk &= alive[i];
    if (!allOk) { blink = !blink; led(blink); }
  }
  delay(1);
}
