// VL53L5CX skin board: N sensors (8x8, up to 15 Hz) -> one UDP datagram per frame.
//
// Wire format (little-endian, 257 bytes):
//   [uint8 sensor index][64 x uint16 filtered mm][64 x uint16 raw mm]
// "filtered" holds 4000 where the sensor reports no target (target_status 255).
//
// Needs: ESP32 Arduino core (tested on 3.3.x, ESP32-C6) and
//        "SparkFun VL53L5CX Arduino Library" from the Library Manager.
// Fill in config.h first (copy config.example.h).

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SparkFun_VL53L5CX_Library.h>
#include "config.h"

static const int ZONES = 64;
static const uint16_t NO_TARGET_MM = 4000;

SparkFun_VL53L5CX imagers[NUM_SENSORS];
bool alive[NUM_SENSORS];
uint32_t frames[NUM_SENSORS];
VL53L5CX_ResultsData results;
WiFiUDP udp;
IPAddress destIp;
uint8_t pkt[1 + 4 * ZONES];

void led(bool on) {
  if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW);
}

bool i2cAck(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Select one mux channel. Works for PCA9546A (4 ch) and TCA9548A (8 ch).
bool selectSensor(uint8_t i) {
#if USE_MUX
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(uint8_t(1u << SENSOR_MUX_PORT[i]));
  return Wire.endTransmission() == 0;
#else
  return true;
#endif
}

void scanBus(const char *label) {
  Serial.printf("[scan] %s:", label);
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cAck(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " (none)");
}

void setup() {
  if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n[boot] vl53l5cx skin board");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[wifi] connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  WiFi.setSleep(false);  // power save adds 100+ ms latency spikes
  Serial.printf("\n[wifi] ip=%s rssi=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
  destIp.fromString(DEST_IP);
  Serial.printf("[udp] sending to %s:%d\n", DEST_IP, DEST_PORT);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Serial.printf("[i2c] sda=%d scl=%d %d Hz\n", I2C_SDA, I2C_SCL, I2C_HZ);
  scanBus("bus");

#if USE_MUX
  if (!i2cAck(MUX_ADDR)) {
    Serial.printf("[error] no mux at 0x%02X: check wiring, I2C_SDA/I2C_SCL, MUX_ADDR\n", MUX_ADDR);
  }
#endif

  int ok = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    alive[i] = false;
    frames[i] = 0;
    if (!selectSensor(i)) { Serial.printf("[error] sensor %d: mux select failed\n", i); continue; }
#if USE_MUX
    char label[24];
    snprintf(label, sizeof label, "mux port %d", SENSOR_MUX_PORT[i]);
    scanBus(label);  // expect 0x29 (plus the mux itself)
#endif
    Serial.printf("[sensor %d] init (uploads ~90 KB firmware, a few seconds)...\n", i);
    if (!imagers[i].begin()) { Serial.printf("[error] sensor %d: no VL53L5CX at 0x29\n", i); continue; }
    imagers[i].setResolution(ZONES);
    imagers[i].setRangingFrequency(RANGING_HZ);
    imagers[i].startRanging();
    alive[i] = true;
    ok++;
    Serial.printf("[sensor %d] ok\n", i);
  }
  Serial.printf("[ready] %d/%d sensors running\n", ok, NUM_SENSORS);
  led(ok == NUM_SENSORS);
}

void sendFrame(uint8_t id) {
  pkt[0] = id;
  for (int z = 0; z < ZONES; z++) {
    uint16_t raw = uint16_t(results.distance_mm[z]);
    uint16_t filt = (results.target_status[z] == 255) ? NO_TARGET_MM : raw;
    pkt[1 + 2 * z] = filt & 0xFF;
    pkt[2 + 2 * z] = filt >> 8;
    pkt[1 + 2 * ZONES + 2 * z] = raw & 0xFF;
    pkt[2 + 2 * ZONES + 2 * z] = raw >> 8;
  }
  udp.beginPacket(destIp, DEST_PORT);
  udp.write(pkt, sizeof pkt);
  udp.endPacket();
}

uint32_t lastReport = 0;
bool blink = false;

void loop() {
  // Poll every sensor; send whichever has a new frame. Never blocks on one sensor.
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!alive[i] || !selectSensor(i)) continue;
    if (imagers[i].isDataReady() && imagers[i].getRangingData(&results)) {
      sendFrame(i);
      frames[i]++;
    }
  }

  uint32_t now = millis();
  if (now - lastReport >= 5000) {  // a heartbeat you can read on the serial monitor
    Serial.print("[rate Hz]");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      Serial.printf(" s%d=%.1f", i, alive[i] ? frames[i] * 1000.0f / (now - lastReport) : 0.0f);
      frames[i] = 0;
    }
    Serial.printf("  wifi=%s\n", WiFi.status() == WL_CONNECTED ? "up" : "DOWN");
    lastReport = now;
    bool allOk = true;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) allOk &= alive[i];
    if (!allOk) { blink = !blink; led(blink); }
  }
  delay(1);
}
