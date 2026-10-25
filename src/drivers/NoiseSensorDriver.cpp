// I2C noise node protocol (LAeq, L10, L90, Ld/Le/Ln/Lden).
#include "NoiseSensorDriver.h"

#include <math.h>
#include <string.h>
#include <time.h>

#include "../Sensors.hpp"

bool NoiseSensorDriver::begin() {
  autoDetect();
  return enabled();
}

bool NoiseSensorDriver::read() {
  collect();
  return enabled();
}

bool NoiseSensorDriver::enabled() const { return _sensors.isSensorRegistered(SENSORS::SNOISE); }

void NoiseSensorDriver::service() {
  if (_sensors.noiseSensorEnabled || _sensors.noiseScanDone) return;
  autoDetect();
}

bool NoiseSensorDriver::sendTime(uint32_t unixTime) { return sendTimeImpl(unixTime); }

bool NoiseSensorDriver::syncTime() { return syncTimeImpl(); }

void NoiseSensorDriver::setTimeSyncInterval(uint32_t intervalMs) {
  _sensors.noiseTimeSyncIntervalMs = intervalMs;
}

bool NoiseSensorDriver::sendTimeImpl(uint32_t unixTime) {
  if (!_sensors.noiseTimeSyncEnabled) return false;

  if (!_sensors.noiseSensorEnabled) return false;

  if (!_sensors.noiseWireReady) {
    initWire();
  }
  if (_sensors.noiseWire == nullptr || _sensors.noiseSensorAddress == 0) return false;

  if (!devicePresent(*_sensors.noiseWire, _sensors.noiseSensorAddress)) {
    _sensors.noiseSensorEnabled = false;
    _sensors.noiseSensorAddress = 0;
    return false;
  }

  uint8_t payload[1 + sizeof(uint32_t)];
  payload[0] = CMD_SET_TIME;
  memcpy(&payload[1], &unixTime, sizeof(uint32_t));

  _sensors.noiseWire->beginTransmission(_sensors.noiseSensorAddress);
  size_t wrote = _sensors.noiseWire->write(payload, sizeof(payload));
  uint8_t err = _sensors.noiseWire->endTransmission();
  if (err == 0 && wrote == sizeof(payload)) {
    _sensors.noiseLastTimeSyncMs = millis();
    return true;
  }
  return false;
}

/**
 * @brief LOCAL epoch for the noise node: UTC plus the local offset.
 * The node applies no time zone of its own and reads the day (07-19 h),
 * evening (19-23 h) and night (23-07 h) bands straight off the epoch it is
 * given, so sending UTC shifts every Ld/Le/Ln/Lden by the zone offset. The
 * offset is the one of the system time zone (TZ), which the application owns:
 * the CanAirIO firmware sets it from the "tzone" setting, and without any TZ it
 * is 0 (UTC).
 */
uint32_t NoiseSensorDriver::localEpoch(time_t utcNow) const {
  struct tm lt, gt;
  localtime_r(&utcNow, &lt);
  gmtime_r(&utcNow, &gt);
  int days = lt.tm_yday - gt.tm_yday;
  if (lt.tm_year > gt.tm_year) {
    days = 1;
  } else if (lt.tm_year < gt.tm_year) {
    days = -1;
  }
  int32_t offset = days * 86400 + (lt.tm_hour - gt.tm_hour) * 3600 + (lt.tm_min - gt.tm_min) * 60;
  return static_cast<uint32_t>(static_cast<int64_t>(utcNow) + offset);
}

bool NoiseSensorDriver::syncTimeImpl() {
  if (!_sensors.noiseTimeSyncEnabled || !_sensors.noiseSensorEnabled) return false;

  // The firmware calls this on every pass of loop(), so throttle before any
  // clock work (time() and the zone lookup take a lock).
  uint32_t now_ms = millis();
  if (_sensors.noiseLastTimeSyncMs == 0) {
    // Never synced, or the node lost its clock: retry at a fixed spacing.
    if ((now_ms - _sensors.noiseLastSyncAttemptMs) < Sensors::NOISE_SYNC_RETRY_MS) return false;
    _sensors.noiseLastSyncAttemptMs = now_ms;
  } else {
    // Synced: look at the clock once a minute, to catch a DST change.
    if ((now_ms - _sensors.noiseLastZoneCheckMs) < Sensors::NOISE_ZONE_CHECK_MS) return false;
    _sensors.noiseLastZoneCheckMs = now_ms;
  }

  time_t now = time(nullptr);
  if (!(now > 1609459200)) return false;  // isTimeValid(): NTP has not set the clock yet

  uint32_t localEpochValue = localEpoch(now);
  int32_t offset =
      static_cast<int32_t>(static_cast<int64_t>(localEpochValue) - static_cast<int64_t>(now));

  if (_sensors.noiseLastTimeSyncMs != 0) {
    // The zone offset changes twice a year with DST: resend at once instead of
    // leaving the node an hour off until the next scheduled sync.
    bool offsetChanged = offset != _sensors.noiseLastSentOffset;
    bool due = (now_ms - _sensors.noiseLastTimeSyncMs) >= _sensors.noiseTimeSyncIntervalMs;
    if (!offsetChanged && !due) return false;
    _sensors.noiseLastSyncAttemptMs = now_ms;
  }

  if (!sendTimeImpl(localEpochValue)) return false;
  _sensors.noiseLastSentOffset = offset;
  return true;
}

bool NoiseSensorDriver::autoDetect() {
  if (_sensors.noiseSensorEnabled) return true;
  if (_sensors.noiseScanDone) return false;
  _sensors.noiseScanDone = true;

  initWire();
  if (_sensors.noiseWire == nullptr) {
    _sensors.DEBUG("-->[SLIB] Noise sensor detect\t: no i2c");
    return false;
  }

  for (uint8_t addr = NOISE_MIN_SCAN_ADDR; addr <= NOISE_MAX_SCAN_ADDR; addr++) {
    if (!devicePresent(*_sensors.noiseWire, addr)) continue;
    if (_sensors.devmode)
      Serial.printf("-->[SLIB] Found device at: 0x%02X, reading identity...\r\n", addr);

    SensorIdentity identity;
    if (!readIdentity(*_sensors.noiseWire, addr, identity)) {
      if (_sensors.devmode) Serial.printf("-->[SLIB] No identity response at: 0x%02X\r\n", addr);
      continue;
    }
    if (identity.sensorType != NOISE_SENSOR_TYPE_ID) {
      if (_sensors.devmode)
        Serial.printf("-->[SLIB] Wrong sensor type at: 0x%02X (0x%02X)\r\n", addr,
                      identity.sensorType);
      continue;
    }
    if (identity.i2cAddress != addr) {
      if (_sensors.devmode)
        Serial.printf("-->[SLIB] Addr mismatch at: 0x%02X (reported 0x%02X)\r\n", addr,
                      identity.i2cAddress);
      continue;
    }

    _sensors.noiseSensorEnabled = true;
    _sensors.noiseSensorAddress = addr;
    _sensors.noiseHaveCycles = false;
    // The real firmware version decides the frame layout (88 bytes with CRC
    // from 3.3.1, 68 before) and which status command to use. A node that does
    // not answer the metadata command is treated as legacy.
    _sensors.noiseHasMeta = readMetadata(*_sensors.noiseWire, addr, _sensors.noiseMeta);
    _sensors.noiseFwVer = _sensors.noiseHasMeta ? noiseFwVersion(_sensors.noiseMeta.fw_major,
                                                                 _sensors.noiseMeta.fw_minor,
                                                                 _sensors.noiseMeta.fw_patch)
                                                : 0;
    if (_sensors.devmode) {
      if (_sensors.noiseHasMeta) {
        Serial.printf("-->[SLIB] Noise node fw %u.%u.%u type=%s\r\n", _sensors.noiseMeta.fw_major,
                      _sensors.noiseMeta.fw_minor, _sensors.noiseMeta.fw_patch,
                      _sensors.noiseMeta.node_type == NOISE_NODE_I2S
                          ? "I2S"
                          : (_sensors.noiseMeta.node_type == NOISE_NODE_ADC ? "ADC" : "?"));
      } else {
        Serial.println("-->[SLIB] Noise node without metadata: legacy 68-byte protocol");
      }
    }
    _sensors.sensorAnnounce(SENSORS::SNOISE);
    _sensors.sensorRegister(SENSORS::SNOISE);
    _sensors.DEBUG("-->[SLIB] Noise sensor detect\t: ok");
    return true;
  }

  _sensors.DEBUG("-->[SLIB] Noise sensor detect\t: not found");
  return false;
}

void NoiseSensorDriver::collect() {
  if (!_sensors.noiseSensorEnabled) return;

  if (_sensors.noiseWire == nullptr || !_sensors.noiseSensorEnabled) return;

  if (_sensors.noiseSensorAddress == 0) return;
  if (!devicePresent(*_sensors.noiseWire, _sensors.noiseSensorAddress)) {
    _sensors.noiseSensorEnabled = false;
    _sensors.noiseSensorAddress = 0;
    return;
  }
  // Node metadata: non-destructive, and it follows the node through a reflash
  // (the firmware version decides the frame layout) and a reboot (the node
  // loses its clock). Nodes older than 3.2.0 do not answer it.
  NodeMetadata meta{};
  if (readMetadata(*_sensors.noiseWire, _sensors.noiseSensorAddress, meta)) {
    uint32_t ver = noiseFwVersion(meta.fw_major, meta.fw_minor, meta.fw_patch);
    if (ver != _sensors.noiseFwVer)
      _sensors.noiseHaveCycles = false;  // reflashed: forget the heartbeat
    _sensors.noiseMeta = meta;
    _sensors.noiseHasMeta = true;
    _sensors.noiseFwVer = ver;
    if (!meta.time_synced) {
      // Rebooted or never set: without the clock Ld/Le/Ln/Lden stay at 0, and
      // the next scheduled sync could be a day away. syncTime()
      // keeps its own 30 s retry spacing.
      _sensors.noiseLastTimeSyncMs = 0;
      syncTime();
    }
  }

  // Never publish a frame that did not pass every check: no flat lines from
  // stale data, no values from a corrupted or half-served frame.
  if (!readData(*_sensors.noiseWire, _sensors.noiseSensorAddress, _sensors.noiseSensorData)) return;

  const SensorData &d = _sensors.noiseSensorData;
  const bool fullFrame = _sensors.noiseFwVer >= NOISE_FW_FULL_PROTOCOL;

  _sensors.noiseInstant = d.noiseAvgDb;             // LAeq,1s
  _sensors.noisePeakValue = d.noisePeakDb;          // LAFmax
  _sensors.noiseAvgLegalValue = d.noiseAvgLegalDb;  // L10
  _sensors.noiseL90Value = (float)d.lowNoiseLevel;  // L90
  if (fullFrame) {
    // Maxima since the previous read, so no event is lost between readings.
    _sensors.noiseLASmaxValue = d.noiseLASmaxHoldDb;
    _sensors.noiseLCpeakValue = d.noiseLCpeakHoldDb;
  }

  // Ld/Le/Ln/Lden: sensor returns 0 when period has no samples (e.g. Ld/Le=0 at night).
  // Preserve last valid value so UI shows recent data instead of 0.
  static constexpr float NOISE_PERIOD_MIN_VALID_DB = 15.0f;  // below this, 0 = "no data"
  auto updateIfValid = [](float newVal, float &stored) {
    if (!isnan(newVal) && newVal >= NOISE_PERIOD_MIN_VALID_DB) {
      stored = newVal;
    }
  };
  updateIfValid(d.Ld, _sensors.noiseLdValue);
  updateIfValid(d.Le, _sensors.noiseLeValue);
  updateIfValid(d.Ln, _sensors.noiseLnValue);
  updateIfValid(d.noiseLden, _sensors.noiseLdenValue);

  _sensors.unitRegister(UNIT::NOISE);
  _sensors.dataReady = true;

  _sensors.unitRegister(UNIT::NOISEPEAK);
  _sensors.unitRegister(UNIT::NOISEAVGLEGAL);
  _sensors.unitRegister(UNIT::NOISEL90);
  if (fullFrame) {
    _sensors.unitRegister(UNIT::NOISELASMAX);
    _sensors.unitRegister(UNIT::NOISELCPEAK);
  }
  _sensors.unitRegister(UNIT::NOISELD);
  _sensors.unitRegister(UNIT::NOISELE);
  _sensors.unitRegister(UNIT::NOISELN);
  _sensors.unitRegister(UNIT::NOISELDEN);
}

bool NoiseSensorDriver::readIdentity(TwoWire &wire, uint8_t address, SensorIdentity &identity) {
  wire.beginTransmission(address);
  wire.write(CMD_IDENTIFY);
  if (wire.endTransmission(true) != 0) return false;
  delay(5);
  uint8_t got = wire.requestFrom(address, (uint8_t)sizeof(SensorIdentity));
  if (got != sizeof(SensorIdentity)) return false;
  uint8_t buffer[sizeof(SensorIdentity)];
  wire.readBytes(buffer, sizeof(SensorIdentity));
  memcpy(&identity, buffer, sizeof(SensorIdentity));
  return true;
}

/**
 * @brief Node status: 1 = the last second is valid, 0 = do not publish.
 * Status 0 means booting, invalid second (clipping, microphone fault, too
 * quiet) or sampling stalled. Nodes of unknown (legacy) firmware only know
 * the legacy code 0x00, which every version accepts.
 */
// Drains what the last requestFrom() left and returns how many bytes it got.
// The count alone proves nothing on ESP32: its I2C master pads a short slave
// reply up to the requested length, so frames are validated by CRC instead.
static size_t noiseReadReply(TwoWire &wire, uint8_t *buf, size_t len) {
  size_t n = 0;
  while (wire.available() && n < len) buf[n++] = wire.read();
  while (wire.available()) wire.read();
  return n;
}

/**
 * @brief Node status: 1 = the last second is valid, 0 = do not publish.
 * Status 0 means booting, invalid second (clipping, microphone fault, too
 * quiet) or sampling stalled. Nodes of unknown (legacy) firmware only know
 * the legacy code 0x00, which every version accepts.
 */
bool NoiseSensorDriver::readStatus(TwoWire &wire, uint8_t address, uint8_t &status) {
  wire.beginTransmission(address);
  wire.write((uint8_t)(_sensors.noiseFwVer != 0 ? CMD_GET_STATUS : CMD_GET_STATUS_LEGACY));
  if (wire.endTransmission(true) != 0) return false;
  delay(10);
  wire.requestFrom(address, (uint8_t)1);
  return noiseReadReply(wire, &status, 1) == 1;
}

/**
 * @brief Node metadata (firmware version, node type, clock state...).
 * A node that does not know the command answers a single 0, and no release is
 * 0.x, so a leading 0 means "legacy node". Only the bytes the node's version
 * says it sent are kept: the rest of the reply is padding.
 */
bool NoiseSensorDriver::readMetadata(TwoWire &wire, uint8_t address, NodeMetadata &meta) {
  wire.beginTransmission(address);
  wire.write((uint8_t)CMD_GET_METADATA);
  if (wire.endTransmission(true) != 0) return false;
  delay(5);
  uint8_t b[sizeof(NodeMetadata)] = {0};
  wire.requestFrom(address, (uint8_t)sizeof(NodeMetadata));
  if (noiseReadReply(wire, b, sizeof(b)) < 7) return false;
  if (b[0] == 0) return false;
  size_t len = noiseMetadataLength(noiseFwVersion(b[0], b[1], b[2]));
  memset(&meta, 0, sizeof(meta));
  memcpy(&meta, b, len);
  return true;
}

/**
 * @brief Reads one SensorData frame, or fails: `out` is only written with a
 * frame that passed every check, so the caller never sees stale or corrupt data.
 *
 * Sequence (per the node's protocol): status, data, status again. A node
 * second can end between those transactions; if that second was invalid the
 * frame is served with status 0 and carries the previous second's values, so
 * it must not be published. On top of that:
 *  - the CRC (nodes >= 3.3.1) discards a frame corrupted in transit;
 *  - `cycles` must change, not grow (it restarts after a node reboot): equal
 *    means a frozen node or two reads inside the same node second.
 * Older nodes (< 3.3.1) send the 68-byte frame without CRC or hold fields.
 */
bool NoiseSensorDriver::readData(TwoWire &wire, uint8_t address, SensorData &out) {
  const bool full = _sensors.noiseFwVer >= NOISE_FW_FULL_PROTOCOL;
  const size_t want = full ? sizeof(SensorData) : NOISE_LEGACY_DATA_LEN;

  int retries = 3;
  while (retries-- > 0) {
    uint8_t status = 0;
    if (!readStatus(wire, address, status)) {
      delay(100);
      continue;  // bus error: worth another try
    }
    if (status != 1) {
      if (_sensors.devmode) Serial.println("-->[SLIB] Noise node status 0: nothing to publish");
      return false;  // booting, invalid second or stalled: a retry would not change it
    }

    wire.beginTransmission(address);
    wire.write((uint8_t)CMD_GET_DATA);
    if (wire.endTransmission(true) != 0) {
      delay(100);
      continue;
    }
    delay(20);
    uint8_t buffer[sizeof(SensorData)] = {0};
    wire.requestFrom(address, (uint8_t)want);
    size_t got = noiseReadReply(wire, buffer, want);

    uint8_t statusAfter = 0;
    bool afterOk = readStatus(wire, address, statusAfter);

    if (got != want) {
      delay(100);
      continue;
    }

    SensorData frame{};
    memcpy(&frame, buffer, want);

    if (full && noiseCrc16(buffer, NOISE_CRC_LEN) != frame.crc16) {
      if (_sensors.devmode) Serial.println("-->[SLIB] Noise frame CRC mismatch, discarded");
      delay(100);
      continue;
    }
    if (!afterOk || statusAfter != 1) {
      if (_sensors.devmode) Serial.println("-->[SLIB] Noise status fell to 0 during the read");
      return false;
    }
    if (_sensors.noiseHaveCycles && frame.cycles == _sensors.noiseLastCycles) {
      if (_sensors.devmode)
        Serial.println("-->[SLIB] Noise node cycles not advancing: frame skipped");
      return false;
    }

    _sensors.noiseLastCycles = frame.cycles;
    _sensors.noiseHaveCycles = true;
    out = frame;
    return true;
  }
  return false;
}

bool NoiseSensorDriver::devicePresent(TwoWire &wire, uint8_t address) {
  wire.beginTransmission(address);
  return (wire.endTransmission() == 0);
}

void NoiseSensorDriver::initWire() {
  if (_sensors.noiseWireReady) return;
  _sensors.noiseWire = &Wire;
  _sensors.noiseWireReady = true;
}
