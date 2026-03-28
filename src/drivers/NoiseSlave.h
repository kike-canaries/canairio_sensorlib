/*
 * Noise Monitor I2C Slave - Communication Protocol
 * Consolidated protocol + slave declarations.
 */

#ifndef NOISE_SLAVE_H
#define NOISE_SLAVE_H

#include <Arduino.h>
#include <stdint.h>

// I2C Commands
enum I2CCommand {
  CMD_GET_STATUS = 0x00,  // Request status
  CMD_GET_DATA = 0x01,    // Request all sensor data
  CMD_RESET = 0x08,       // Reset cycle
  CMD_SET_TIME = 0x09,    // Send UNIX timestamp
  CMD_IDENTIFY = 0x09,    // Identify sensor type and version
  CMD_PING = 0x09         // Alias for identify
};

// Sensor data structure (aligned to 68 bytes)
struct SensorData {
  uint32_t noise;            // Current noise level (mV)
  float noiseAvg;            // Average (mV)
  float noiseAvgDb;          // Average (dB)
  float noisePeak;           // Peak (mV)
  float noisePeakDb;         // Peak (dB)
  float noiseMin;            // Minimum (mV)
  float noiseMinDb;          // Minimum (dB)
  float noiseAvgLegal;       // Legal average (mV)
  float noiseAvgLegalDb;     // Legal average (dB)
  float noiseAvgLegalMax;    // Legal max average (mV)
  float noiseAvgLegalMaxDb;  // Legal max average (dB)
  uint16_t lowNoiseLevel;    // Dynamic base noise level
  uint32_t cycles;           // Number of cycles completed
  float Ld;                  // Day index
  float Le;                  // Evening index
  float Ln;                  // Night index
  float noiseLden;           // Global day-evening-night index
};

// Sensor identity structure (5 bytes)
struct SensorIdentity {
  uint8_t sensorType;    // 0x01 = Noise Sensor
  uint8_t versionMajor;  // Firmware version major
  uint8_t versionMinor;  // Firmware version minor
  uint8_t status;        // Bitmask: bit0=initialized, bit1=adc_active, bit2=data_ready
  uint8_t i2cAddress;    // Current I2C address
} __attribute__((packed));

// I2C Addresses
static constexpr uint8_t DEFAULT_NOISE_ADDR = 0x08;
static constexpr uint8_t MIN_I2C_ADDRESS = 0x08;
static constexpr uint8_t MAX_I2C_ADDRESS = 0x77;

static constexpr uint8_t SENSOR_TYPE_NOISE = 0x01;
static constexpr uint8_t NOISE_SENSOR_TYPE_ID = SENSOR_TYPE_NOISE;

#endif
