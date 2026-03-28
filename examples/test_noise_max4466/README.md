# MAX4466 Noise Sensor Test Example

Test example for the MAX4466 noise sensor integrated with CanAirIO SensorLib.

## Features

This example demonstrates:
- Auto-detection of MAX4466 I2C noise sensor (address 0x08)
- Reading real-time noise measurements (LAeq,1s)
- Accessing regulatory indicators (Ld, Le, Ln, Lden)
- Time synchronization for period-based calculations

## Hardware Required

- **Master Device**: ESP32-S3 / ESP32-S2 (or ESP32)
- **Slave Device**: ESP32-C3 with MAX4466 microphone running the noise sensor firmware
- I2C connections (SDA, SCL, GND)

## Wiring

| Master ESP32-S3/ESP32-S2 | Slave ESP32-C3 |
|:-------------------------|:---------------|
| SDA (GPIO 8)             | SDA (GPIO 8)   |
| SCL (GPIO 9)             | SCL (GPIO 9)   |
| GND                      | GND            |

For Wemos LOLIN S2 Mini, set `SLIB_I2C_SDA=8` and `SLIB_I2C_SCL=9` in `build_flags`.

## Usage

### 1. Flash the Slave Device

First upload the noise sensor firmware to the ESP32-C3:

```bash
cd g:\mediciones_ruido\une-en-iso\noise_UNE-EN_ISO_1996-2-2009
pio run -e lolin_c3_mini --target upload
```

### 2. Compile and Upload Master

```bash
cd C:\Users\house\Documents\platformio\Projects\sensorlib\examples\test_noise_max4466
pio run -e esp32s2 --target upload
pio device monitor
```

### 3. Expected Output

```
╔══════════════════════════════════════╗
║  MAX4466 Noise Sensor Test          ║
║  CanAirIO SensorLib Integration     ║
╚══════════════════════════════════════╝

[INFO] Initializing sensors...
-->[SLIB] Noise sensor detect: ok
-->[SLIB] Noise sensor address: 0x08

[INFO] Synchronizing time with sensor...
[OK] Time synchronized successfully!

=================================
  NOISE SENSOR MEASUREMENTS
=================================
Instant Noise:  45.3 dB
Average:        44.8 dB
Peak:           52.1 dB
Minimum:        38.5 dB

--- Regulatory Indicators ---
Ld (Day):       44.2 dB  [07:00-19:00]
Le (Evening):   0.0 dB   [19:00-23:00]
Ln (Night):     0.0 dB   [23:00-07:00]
Lden (Global):  0.0 dB   [24h index]
=================================
```

## Notes

- **Time Sync**: Ld/Le/Ln/Lden require time synchronization. The example sends a sample timestamp.
- **Period Accumulation**: Indicators update as measurements accumulate in each period.
- **Zero Values**: If a period has no samples yet, it will show 0.0 dB.

## Troubleshooting

If the sensor is not detected:
1. Check I2C wiring (SDA, SCL, GND)
2. Verify slave firmware is running (`pio device monitor` on slave)
3. Use I2C scanner to verify address 0x08 is present
4. Check for I2C pull-up resistors (4.7kΩ recommended)

## License

MIT License - Same as CanAirIO SensorLib



