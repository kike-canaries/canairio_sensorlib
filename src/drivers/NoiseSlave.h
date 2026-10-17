/*
 * Noise Monitor I2C Slave - Communication Protocol
 *
 * Master-side (sensorlib) view of the protocol spoken by the noise node
 * firmware: https://github.com/roberbike/noise_UNE-EN_ISO_1996-2-2009
 * (ESP32-C3 + MAX4466 or XIAO ESP32-S3 + ICS-43434, I2C slave, address 0x08).
 *
 * Keep this file in step with the firmware's src/I2C_Comm.h and
 * src/DSP_Engine.h. The wire format only ever grows at the END of a frame, so
 * a master built against an older definition keeps working, but every offset
 * below is part of the protocol: the static_asserts at the bottom break the
 * build if a field drifts, instead of letting the master read wrong values
 * silently (that is exactly what happened with firmware 3.3.0).
 */

#ifndef NOISE_SLAVE_H
#define NOISE_SLAVE_H

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// I2C commands (one command byte written by the master, then a read)
// ---------------------------------------------------------------------------
// CMD_SET_TIME and CMD_IDENTIFY share the code 0x09 on purpose: the node tells
// them apart by length. 0x09 alone = identify (reply: SensorIdentity);
// 0x09 + 4 bytes = set the clock (no reply).
enum I2CCommand {
  CMD_GET_STATUS_LEGACY = 0x00,  // status, accepted by every firmware version
  CMD_GET_DATA = 0x01,           // SensorData (88 bytes; 68 on firmware < 3.3.1)
  CMD_IDENTIFY = 0x09,           // SensorIdentity (5 bytes)
  CMD_SET_TIME = 0x09,           // + uint32 little-endian LOCAL epoch, no reply
  CMD_SET_CALIB = 0x0A,          // + int16 little-endian, hundredths of dB, no reply
  CMD_GET_STATUS = 0x20,         // 1 byte: 1 = last second valid, 0 = do not publish
  CMD_GET_METADATA = 0x50        // NodeMetadata (16 bytes; 7 before 3.3.1)
};

// ---------------------------------------------------------------------------
// CMD_GET_DATA frame
// ---------------------------------------------------------------------------
// Several names are historical and do not describe what the node really puts
// in them. The layout is frozen, so the comments here are the truth.
struct SensorData {
  uint32_t noise;            // RMS of the last second (mV on ADC nodes, uFS on I2S nodes)
  float noiseAvg;            // same RMS, unrounded
  float noiseAvgDb;          // LAeq,1s in dB(A)
  float noisePeak;           // LAFmax, linear units (NOT a peak)
  float noisePeakDb;         // LAFmax in dB(A)       (NOT a peak, see noiseLCpeakDb)
  float noiseMin;            // DEPRECATED: duplicate of noiseAvg
  float noiseMinDb;          // DEPRECATED: duplicate of noiseAvgDb
  float noiseAvgLegal;       // L10 in dB(A) (despite the name, NOT mV)
  float noiseAvgLegalDb;     // L10 in dB(A)
  float noiseAvgLegalMax;    // DEPRECATED: duplicate of noisePeak (LAFmax, linear)
  float noiseAvgLegalMaxDb;  // DEPRECATED: duplicate of noisePeakDb (LAFmax, dB)
  uint16_t lowNoiseLevel;    // L90 in dB(A), rounded
  uint32_t cycles;           // seconds aggregated (valid or not): the node's heartbeat
  float Ld;                  // day index   (07-19 h), dB(A), 0 until the period has data
  float Le;                  // evening     (19-23 h)
  float Ln;                  // night       (23-07 h)
  float noiseLden;           // day-evening-night index, dB
  // --- appended in firmware 3.3.1; absent in the 68-byte frame of older nodes ---
  float noiseLASmaxDb;      // LASmax of the last second (Slow weighting), dB(A)
  float noiseLCpeakDb;      // LCpeak of the last second (true peak, C weighting), dB(C)
  float noiseLASmaxHoldDb;  // max LASmax since the previous CMD_GET_DATA, dB(A)
  float noiseLCpeakHoldDb;  // max LCpeak since the previous CMD_GET_DATA, dB(C)
  uint16_t crc16;           // CRC-16/CCITT-FALSE over the 84 bytes before it
  uint16_t reserved;        // always 0, NOT covered by the CRC: do not interpret
};

// Bytes the CRC covers: everything ahead of the crc16 field.
static constexpr size_t NOISE_CRC_LEN = offsetof(SensorData, crc16);
// Frame length of nodes older than 3.3.1 (layout 3.2.x): no hold fields, no CRC.
static constexpr size_t NOISE_LEGACY_DATA_LEN = offsetof(SensorData, noiseLASmaxDb);

// ---------------------------------------------------------------------------
// CMD_IDENTIFY reply (5 bytes)
// ---------------------------------------------------------------------------
// Only sensorType and i2cAddress are reliable. The firmware answers a fixed
// version (2.1) and a fixed status here whatever it really is, so use
// CMD_GET_METADATA for the real firmware version and node type.
struct SensorIdentity {
  uint8_t sensorType;    // 0x01 = noise sensor
  uint8_t versionMajor;  // fixed by the firmware, not its real version
  uint8_t versionMinor;
  uint8_t status;      // fixed by the firmware
  uint8_t i2cAddress;  // I2C address the node answers on
} __attribute__((packed));

// ---------------------------------------------------------------------------
// CMD_GET_METADATA reply (16 bytes from 3.3.1, 9 in 3.3.0, 7 before that)
// ---------------------------------------------------------------------------
// An ESP32 master pads a short reply up to the length it asked for, so the
// number of bytes received says nothing: the real length is deduced from the
// firmware version (see noiseMetadataLength()).
struct NodeMetadata {
  uint8_t fw_major;
  uint8_t fw_minor;
  uint8_t fw_patch;
  uint8_t node_type;      // NOISE_NODE_ADC or NOISE_NODE_I2S
  uint8_t time_synced;    // 1 once a master has set the node's clock
  uint16_t clip_count;    // clipped samples in the last second
  int16_t calib_offset;   // persistent calibration offset, hundredths of dB
  uint16_t window_fill;   // valid seconds currently in the L10/L90 window...
  uint16_t window_size;   // ...out of this many
  uint8_t lden_periods;   // bit 0 day has data, bit 1 evening, bit 2 night
  uint16_t lden_minutes;  // minutes accumulated across the populated periods
} __attribute__((packed));

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
static constexpr uint8_t DEFAULT_NOISE_ADDR = 0x08;
static constexpr uint8_t NOISE_MIN_SCAN_ADDR = 0x08;
static constexpr uint8_t NOISE_MAX_SCAN_ADDR = 0x0F;

static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;
static constexpr uint8_t NOISE_SENSOR_TYPE_ID = SENSOR_TYPE_NOISE;

static constexpr uint8_t NOISE_NODE_ADC = 0x01;  // ESP32-C3 + MAX4466
static constexpr uint8_t NOISE_NODE_I2S = 0x02;  // XIAO ESP32-S3 + ICS-43434

// First firmware with the 88-byte CRC-protected frame and the full metadata.
static constexpr uint32_t NOISE_FW_FULL_PROTOCOL = 0x030301;  // 3.3.1
// First firmware that answers CMD_GET_METADATA at all.
static constexpr uint32_t NOISE_FW_METADATA = 0x030200;  // 3.2.0

static inline uint32_t noiseFwVersion(uint8_t major, uint8_t minor, uint8_t patch) {
  return ((uint32_t)major << 16) | ((uint32_t)minor << 8) | patch;
}

// Bytes of CMD_GET_METADATA the node really sent, from its firmware version.
// 3.3.0 reported itself as 3.2.1, so below 3.3.1 only 7 bytes are certain.
static inline size_t noiseMetadataLength(uint32_t fwVersion) {
  return (fwVersion >= NOISE_FW_FULL_PROTOCOL) ? sizeof(NodeMetadata) : 7;
}

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF), the same the node uses.
// Check value of "123456789" is 0x29B1.
static inline uint16_t noiseCrc16(const void *data, size_t len) {
  const uint8_t *p = (const uint8_t *)data;
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)p[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// Wire-format guards: if one fails, fix the struct, never the number.
// ---------------------------------------------------------------------------
static_assert(offsetof(SensorData, noiseAvgDb) == 8, "wire format: noiseAvgDb moved");
static_assert(offsetof(SensorData, noisePeakDb) == 16, "wire format: noisePeakDb moved");
static_assert(offsetof(SensorData, noiseAvgLegalDb) == 32, "wire format: noiseAvgLegalDb moved");
static_assert(offsetof(SensorData, lowNoiseLevel) == 44, "wire format: lowNoiseLevel moved");
static_assert(offsetof(SensorData, cycles) == 48, "wire format: cycles moved");
static_assert(offsetof(SensorData, Ld) == 52, "wire format: Ld moved");
static_assert(offsetof(SensorData, noiseLden) == 64, "wire format: noiseLden moved");
static_assert(offsetof(SensorData, noiseLASmaxDb) == 68, "wire format: noiseLASmaxDb moved");
static_assert(offsetof(SensorData, noiseLCpeakDb) == 72, "wire format: noiseLCpeakDb moved");
static_assert(offsetof(SensorData, noiseLASmaxHoldDb) == 76,
              "wire format: noiseLASmaxHoldDb moved");
static_assert(offsetof(SensorData, noiseLCpeakHoldDb) == 80,
              "wire format: noiseLCpeakHoldDb moved");
static_assert(offsetof(SensorData, crc16) == 84, "wire format: crc16 moved");
static_assert(sizeof(SensorData) == 88, "wire format: unexpected SensorData size");
static_assert(NOISE_LEGACY_DATA_LEN == 68, "wire format: legacy frame length changed");
static_assert(sizeof(SensorIdentity) == 5, "wire format: unexpected SensorIdentity size");
static_assert(sizeof(NodeMetadata) == 16, "wire format: unexpected NodeMetadata size");

#endif
