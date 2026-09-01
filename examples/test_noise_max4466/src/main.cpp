/*
 * Test Example: MAX4466 Noise Sensor with CanAirIO SensorLib
 *
 * This example demonstrates how to read noise data from the MAX4466
 * I2C slave device using the CanAirIO SensorLib framework.
 *
 * Hardware connections:
 * - Master SDA -> Slave GPIO 8
 * - Master SCL -> Slave GPIO 10
 * - GND -> GND
 *
 * Platform: ESP32-S3 or ESP32 (master)
 */

#include <Arduino.h>

#include <Sensors.hpp>

// Callback cuando los datos del sensor están listos
void onSensorDataOk() {
  Serial.println("\n=================================");
  Serial.println("  NOISE SENSOR MEASUREMENTS");
  Serial.println("=================================");

  // Mediciones instantáneas
  Serial.print("Instant Noise:  ");
  Serial.print(sensors.getNoise());
  Serial.println(" dB");

  Serial.print("Average:        ");
  Serial.print(sensors.getNoiseAverage());
  Serial.println(" dB");

  Serial.print("Peak:           ");
  Serial.print(sensors.getNoisePeak());
  Serial.println(" dB");

  Serial.print("Minimum:        ");
  Serial.print(sensors.getNoiseMin());
  Serial.println(" dB");

  Serial.println("\n--- Regulatory Indicators ---");

  // Indicadores normativos
  Serial.print("Ld (Day):       ");
  Serial.print(sensors.getNoiseLd());
  Serial.println(" dB  [07:00-19:00]");

  Serial.print("Le (Evening):   ");
  Serial.print(sensors.getNoiseLe());
  Serial.println(" dB  [19:00-23:00]");

  Serial.print("Ln (Night):     ");
  Serial.print(sensors.getNoiseLn());
  Serial.println(" dB  [23:00-07:00]");

  Serial.print("Lden (Global):  ");
  Serial.print(sensors.getNoiseLden());
  Serial.println(" dB  [24h index]");

  Serial.println("=================================\n");
}

// Callback de error
void onSensorDataError(const char *msg) {
  Serial.print("[ERROR] Sensor error: ");
  Serial.println(msg);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n\n");
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║  MAX4466 Noise Sensor Test          ║");
  Serial.println("║  CanAirIO SensorLib Integration     ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.println();

  // Configurar callbacks
  sensors.setOnDataCallBack(&onSensorDataOk);
  sensors.setOnErrorCallBack(&onSensorDataError);

  // Configurar modo debug
  sensors.setDebugMode(true);

  // Forzar detección solo I2C (no UART)
  sensors.detectI2COnly(true);

  // Sample time en segundos
  sensors.setSampleTime(5);

  Serial.println("[INFO] Initializing sensors...");
  sensors.init();

  delay(2000);

  // Sincronizar tiempo (opcional pero recomendado para Ld/Le/Ln/Lden)
  // Aquí usamos un timestamp de ejemplo (2024-02-11 12:00:00 UTC)
  // En producción, obtener de RTC o NTP
  uint32_t testTimestamp = 1707652800;

  Serial.println("\n[INFO] Synchronizing time with sensor...");
  Serial.print("[INFO] Sending timestamp: ");
  Serial.println(testTimestamp);

  if (sensors.sendNoiseSensorTime(testTimestamp)) {
    Serial.println("[OK] Time synchronized successfully!");
    Serial.println("[INFO] Sensor will now calculate Ld, Le, Ln based on time periods");
  } else {
    Serial.println("[WARNING] Time sync failed - Ld/Le/Ln/Lden will remain at 0");
  }

  Serial.println("\n[INFO] Starting data acquisition...\n");
}

void loop() {
  sensors.loop();  // Leer y procesar datos de sensores
}
