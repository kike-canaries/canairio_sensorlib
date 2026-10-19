/**
 * ESP32 (MASTER) - I2C scan and read of the noise sensor node
 *
 * Low-level example: talks to the node directly, without Sensors.hpp, using the
 * protocol definitions of drivers/NoiseSlave.h. For normal use, let the library
 * do it (see examples/test_noise_max4466).
 *
 * - SDA = GPIO 8, SCL = GPIO 9 (change below for your board)
 * - Scans 0x08-0x0F, identifies the node and reads its metadata
 * - Only prints a frame that passed every check: status 1 before and after the
 *   read, CRC correct (firmware >= 3.3.1) and `cycles` changed. Nothing is ever
 *   printed from stale or uninitialised memory.
 */

#include <Arduino.h>
#include <Wire.h>

#include "drivers/NoiseSlave.h"

static constexpr uint8_t SDA_PIN = 8;
static constexpr uint8_t SCL_PIN = 9;
static constexpr uint32_t I2C_FREQ_HZ = 100000;  // 100 kHz

static uint8_t sensorAddress = 0;
static bool sensorDetected = false;
static uint32_t fwVersion = 0;  // 0 = node without metadata (legacy protocol)
static uint32_t lastCycles = 0;
static bool haveCycles = false;

static bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return (Wire.endTransmission() == 0);
}

// Drains the reply and returns how many bytes arrived. The ESP32 master pads a
// short reply up to the requested length, so the count alone proves nothing.
static size_t readReply(uint8_t *buf, size_t len) {
  size_t n = 0;
  while (Wire.available() && n < len) buf[n++] = Wire.read();
  while (Wire.available()) Wire.read();
  return n;
}

static bool readStatus(uint8_t address, uint8_t &status) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)(fwVersion != 0 ? CMD_GET_STATUS : CMD_GET_STATUS_LEGACY));
  if (Wire.endTransmission(true) != 0) return false;
  delay(10);
  Wire.requestFrom(address, (uint8_t)1);
  return readReply(&status, 1) == 1;
}

static bool readIdentity(uint8_t address, SensorIdentity &id) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)CMD_IDENTIFY);
  if (Wire.endTransmission(true) != 0) return false;
  delay(5);
  Wire.requestFrom(address, (uint8_t)sizeof(SensorIdentity));
  uint8_t b[sizeof(SensorIdentity)] = {0};
  if (readReply(b, sizeof(b)) != sizeof(b)) return false;
  memcpy(&id, b, sizeof(id));
  return true;
}

static bool readMetadata(uint8_t address, NodeMetadata &meta) {
  Wire.beginTransmission(address);
  Wire.write((uint8_t)CMD_GET_METADATA);
  if (Wire.endTransmission(true) != 0) return false;
  delay(5);
  Wire.requestFrom(address, (uint8_t)sizeof(NodeMetadata));
  uint8_t b[sizeof(NodeMetadata)] = {0};
  if (readReply(b, sizeof(b)) < 7) return false;
  if (b[0] == 0) return false;  // a node without the command answers a single 0
  size_t len = noiseMetadataLength(noiseFwVersion(b[0], b[1], b[2]));
  memset(&meta, 0, sizeof(meta));
  memcpy(&meta, b, len);
  return true;
}

// Status, data, status again; publishable only if every check passes.
static bool readFrame(uint8_t address, SensorData &out) {
  const bool full = fwVersion >= NOISE_FW_FULL_PROTOCOL;
  const size_t want = full ? sizeof(SensorData) : NOISE_LEGACY_DATA_LEN;

  uint8_t status = 0;
  if (!readStatus(address, status)) {
    Serial.println("Status read failed (I2C error)");
    return false;
  }
  if (status != 1) {
    Serial.println("Status 0: node booting, invalid second or sampling stalled - frame not read");
    return false;
  }

  Wire.beginTransmission(address);
  Wire.write((uint8_t)CMD_GET_DATA);
  if (Wire.endTransmission(true) != 0) return false;
  delay(20);
  Wire.requestFrom(address, (uint8_t)want);
  uint8_t buf[sizeof(SensorData)] = {0};
  size_t got = readReply(buf, want);

  uint8_t statusAfter = 0;
  bool afterOk = readStatus(address, statusAfter);

  if (got != want) {
    Serial.printf("Incomplete frame: expected %u bytes, got %u\n", (unsigned)want, (unsigned)got);
    return false;
  }
  SensorData frame{};
  memcpy(&frame, buf, want);

  if (full && noiseCrc16(buf, NOISE_CRC_LEN) != frame.crc16) {
    Serial.println("CRC mismatch: frame discarded");
    return false;
  }
  if (!afterOk || statusAfter != 1) {
    Serial.println("Status fell to 0 during the read: frame discarded");
    return false;
  }
  if (haveCycles && frame.cycles == lastCycles) {
    Serial.println("cycles not advancing: same second or frozen node, frame skipped");
    return false;
  }
  lastCycles = frame.cycles;
  haveCycles = true;
  out = frame;
  return true;
}

static void scanAndReport() {
  sensorDetected = false;
  sensorAddress = 0;
  fwVersion = 0;
  haveCycles = false;

  Serial.println("=== Searching for the noise node (I2C) ===");
  Serial.printf("SDA=%u SCL=%u Freq=%luHz\n", SDA_PIN, SCL_PIN, (unsigned long)I2C_FREQ_HZ);
  Serial.printf("Range: 0x%02X - 0x%02X\n", NOISE_MIN_SCAN_ADDR, NOISE_MAX_SCAN_ADDR);

  for (uint8_t addr = NOISE_MIN_SCAN_ADDR; addr <= NOISE_MAX_SCAN_ADDR; addr++) {
    if (!i2cDevicePresent(addr)) continue;

    SensorIdentity id;
    if (!readIdentity(addr, id)) continue;
    if (id.sensorType != NOISE_SENSOR_TYPE_ID || id.i2cAddress != addr) continue;

    sensorDetected = true;
    sensorAddress = addr;

    NodeMetadata meta{};
    bool hasMeta = readMetadata(addr, meta);
    if (hasMeta) fwVersion = noiseFwVersion(meta.fw_major, meta.fw_minor, meta.fw_patch);

    Serial.println("=====================================");
    Serial.println("NOISE NODE CONNECTED");
    Serial.printf("  I2C address: 0x%02X\n", sensorAddress);
    if (hasMeta) {
      Serial.printf("  Firmware %u.%u.%u, node type %s, clock %s\n", meta.fw_major, meta.fw_minor,
                    meta.fw_patch,
                    meta.node_type == NOISE_NODE_I2S
                        ? "I2S (ICS-43434)"
                        : (meta.node_type == NOISE_NODE_ADC ? "ADC (MAX4466)" : "unknown"),
                    meta.time_synced ? "set" : "NOT set (Ld/Le/Ln/Lden = 0)");
    } else {
      Serial.println("  No metadata: legacy node (68-byte protocol, no CRC)");
    }
    Serial.println("=====================================");
    return;
  }

  Serial.println("=====================================");
  Serial.println("NOISE NODE NOT FOUND");
  Serial.println("Check:");
  Serial.println("  - SDA/SCL wiring and a common GND");
  Serial.println("  - Pull-ups (4.7k) on SDA and SCL to 3V3 if the bus hangs");
  Serial.println("  - Power of the node");
  Serial.println("=====================================");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);
  Wire.setTimeOut(100);  // ms: a hung bus must not lock the master
#if defined(ARDUINO_ARCH_ESP32)
  Wire.setBufferSize(128);  // must hold the 88-byte frame: never set it below 88
#endif

  Serial.println();
  Serial.println("=== ESP32 MASTER I2C - Noise node ===");
  scanAndReport();
}

void loop() {
  static unsigned long lastActionMs = 0;
  const unsigned long now = millis();

  if (!sensorDetected) {
    if (now - lastActionMs >= 5000) {
      lastActionMs = now;
      scanAndReport();
    }
    delay(50);
    return;
  }

  if (now - lastActionMs < 5000) {
    delay(10);
    return;
  }
  lastActionMs = now;

  if (!i2cDevicePresent(sensorAddress)) {
    Serial.println("Node disconnected (no I2C answer)");
    sensorDetected = false;
    return;
  }

  SensorData d{};
  if (!readFrame(sensorAddress, d)) return;  // nothing is printed from a bad frame

  Serial.printf("LAeq=%.1f dB  LAFmax=%.1f dB  L10=%.1f dB  L90=%u dB\n", d.noiseAvgDb,
                d.noisePeakDb, d.noiseAvgLegalDb, (unsigned)d.lowNoiseLevel);
  if (fwVersion >= NOISE_FW_FULL_PROTOCOL) {
    Serial.printf("LASmax(since last read)=%.1f dB  LCpeak(since last read)=%.1f dB(C)\n",
                  d.noiseLASmaxHoldDb, d.noiseLCpeakHoldDb);
  }
  Serial.printf("Ld=%.1f Le=%.1f Ln=%.1f Lden=%.1f  cycles=%lu\n", d.Ld, d.Le, d.Ln, d.noiseLden,
                (unsigned long)d.cycles);
}
