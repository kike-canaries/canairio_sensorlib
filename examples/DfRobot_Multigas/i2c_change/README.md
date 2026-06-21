# DFRobot MultiGas Sensor - Auto-Configuration

## Descripción General

Este sketch permite detectar automáticamente sensores de gas DFRobot conectados a tu microcontrolador y configura todos con los mismos parámetros:

- ✅ **Detección automática** del tipo de sensor (NH3, CO, NO2, O3)
- ✅ **Cambio de dirección I2C** al grupo 7 (0x78-0x7B)
- ✅ **Activación de compensación de temperatura**
- ✅ **Activación del modo pasivo** (lectura bajo demanda)

No necesitas hardcodear direcciones ni conocer los tipos de sensores de antemano.

---

## Requisitos de Hardware

### Microcontrolador Soportado
- ESP32, ESP8266, Arduino, o cualquier board con soporte I2C

### Sensores DFRobot Multigas
Cualquier combinación de estos sensores (hasta 4):
- **SEN0466** - Gas NH3
- **SEN0467** - Gas CO
- **SEN0468** - Gas NO2
- **SEN0469** - Gas O3

### Conexiones I2C
```
Sensor Pin 1 (GND) → GND del microcontrolador
Sensor Pin 2 (VCC) → 3.3V o 5V
Sensor Pin 3 (SDA) → Pin SDA del microcontrolador (GPIO 21 en ESP32)
Sensor Pin 4 (SCL) → Pin SCL del microcontrolador (GPIO 22 en ESP32)
```

> **Nota:** Los sensores deben estar en modo I2C (DIP switch SEL = 0)

---

## Cómo Funciona

### Pasos que el Sketch Realiza

#### 1️⃣ **Inicialización** (setup)
```
Serial begins → Scanning for sensors...
```

#### 2️⃣ **Escaneo de Sensores**
- Explora las direcciones del **grupo 6 por defecto** (0x74, 0x75, 0x76, 0x77)
- Identifica cada sensor encontrado con su tipo de gas
- Crea un registro interno para cada sensor

#### 3️⃣ **Configuración de Cada Sensor**
Para cada sensor detectado:
1. Cambia la dirección I2C al **grupo 7**
2. Activa **compensación de temperatura**
3. Activa **modo pasivo** (necesarias peticiones para datos)

#### 4️⃣ **Lectura Continua** (loop)
- Solicita datos a cada sensor cada 2 segundos
- Muestra concentración en PPM o %vol (según el gas)

---

## Pasos para Usar

### Paso 1: Preparar el Hardware
1. Conecta los sensores DFRobot al microcontrolador por I2C
2. Verifica que todos los sensores estén en modo I2C (DIP: SEL = 0)
3. Asegúrate de que cada sensor está en el **grupo 6 por defecto** (ver tabla de direcciones)

### Paso 2: Cargar el Sketch
1. Abre PlatformIO o Arduino IDE
2. Abre el archivo `src/main.cpp` en esta carpeta
3. Conecta tu microcontrolador por USB
4. Selecciona la placa y puerto correcto
5. Compila y carga el sketch: `Upload` o `Ctrl+Alt+U`

### Paso 3: Monitorear la Consola Serial
1. Abre el Monitor Serial (`Ctrl+J` en PlatformIO)
2. Configura la velocidad en **115200 baud**
3. Presiona el botón RESET del microcontrolador (o desconecta/reconecta USB)

### Paso 4: Verificar la Detección
Deberías ver en consola:
```
========================================
DFRobot MultiGas Sensor Auto-Configure
========================================

Scanning for sensors...

Detected sensor at 0x74: NH3
Detected sensor at 0x75: CO
Detected sensor at 0x76: NO2
Detected sensor at 0x77: O3

Found 4 sensor(s)
```

### Paso 5: Verificar la Configuración
El sketch continuará mostrando:
```
========================================
Configuring sensors...
========================================

Configuring NH3 sensor at 0x74
  - I2C address group changed to 7
  - Temperature compensation enabled
  - Passive mode enabled
  - Configuration complete for NH3 sensor

[Similar para los otros sensores...]

========================================
Configuration complete!
========================================
```

### Paso 6: Lectura de Datos
Una vez completada la configuración, verás lecturas periódicas:
```
NH3 (0x74): 45.23 PPM
CO (0x75): 12.56 PPM
NO2 (0x76): 8.90 PPM
O3 (0x77): 5.34 PPM
```

---

## Salida Esperada en la Consola

### Caso de Éxito ✅
```
========================================
DFRobot MultiGas Sensor Auto-Configure
========================================

Scanning for sensors...

Detected sensor at 0x74: NH3
Detected sensor at 0x75: CO

Found 2 sensor(s)

========================================
Configuring sensors...
========================================

Configuring NH3 sensor at 0x74
  - I2C address group changed to 7
  - Temperature compensation enabled
  - Passive mode enabled
  - Configuration complete for NH3 sensor

Configuring CO sensor at 0x75
  - I2C address group changed to 7
  - Temperature compensation enabled
  - Passive mode enabled
  - Configuration complete for CO sensor

========================================
Configuration complete!
========================================

NH3 (0x74): 23.45 PPM
CO (0x75): 5.67 PPM

NH3 (0x74): 23.50 PPM
CO (0x75): 5.68 PPM
```

### Caso de Error ❌
```
ERROR: No sensors detected!
Please check I2C connections and sensor power.
```

---

## Solución de Problemas

### "No sensors detected!"

**Causas posibles:**
1. ❌ Sensores no conectados correctamente
2. ❌ Sensores sin alimentación
3. ❌ Sensores en modo UART (DIP: SEL = 1)
4. ❌ Cables I2C defectuosos
5. ❌ Pines SDA/SCL incorrectos en el código

**Soluciones:**
- Verifica las conexiones físicas
- Asegúrate de que los sensores tengan alimentación (verde LED encendido)
- Cambia el DIP switch SEL a 0 (modo I2C)
- Prueba con un cable USB diferente
- Consulta el datasheet de tu microcontrolador para los pines I2C correctos

### "IIC address change attempt failed, retrying..."

**Causas posibles:**
1. ❌ Sensor recién conectado (necesita tiempo de inicialización)
2. ❌ Comunicación I2C inestable
3. ❌ Sensor con dirección bloqueada

**Soluciones:**
- Espera 10 segundos después de conectar los sensores antes de cargar el sketch
- Reduce la velocidad I2C si es posible (por defecto 100kHz)
- Intenta reinicar el sketch (botón RESET)
- Si falla 5 veces consecutivas, revisa el sensor físicamente

---

## Notas Técnicas

### Tabla de Direcciones I2C por Grupo

| Grupo | A0=0, A1=0 | A0=1, A1=0 | A0=0, A1=1 | A0=1, A1=1 |
|-------|----------|----------|----------|----------|
| 6 (Default) | 0x74 | 0x76 | 0x75 | 0x77 |
| 7 (Este sketch) | 0x78 | 0x7A | 0x79 | 0x7B |

### Parámetros Configurados

| Parámetro | Valor |
|-----------|-------|
| Grupo I2C | 7 |
| Compensación de Temperatura | ON |
| Modo de Adquisición | PASSIVITY (bajo demanda) |
| Intervalo de Lectura | 2 segundos |

### Modificar los Parámetros

Si necesitas cambiar algo, edita estas líneas en `src/main.cpp`:

```cpp
// Cambiar grupo I2C (línea 72)
while (!info.sensor->changeI2cAddrGroup(7) && attempts < MAX_ATTEMPTS) {
  // Cambia el 7 al grupo deseado (1-8)
}

// Cambiar intervalo de lectura (línea 164)
delay(2000);  // Cambia 2000 por el intervalo en milisegundos
```

---

## Librería Requerida

Este sketch requiere la librería oficial de DFRobot:
```
DFRobot_MultiGasSensor
```

Si usas PlatformIO, se instalará automáticamente desde `platformio.ini`.

---

## Preguntas Frecuentes

**P: ¿Puedo usar sensores de diferentes grupos a la vez?**
R: No, este script cambia todos al grupo 7. Si necesitas grupos diferentes, modifica la función `configureSensor()`.

**P: ¿Cuántos sensores puedo conectar?**
R: Hasta 4 sensores diferentes (uno de cada tipo de gas) en las 4 direcciones del grupo 7.

**P: ¿El script daña los sensores?**
R: No, solo cambia parámetros de configuración almacenados en EEPROM del sensor. Son reversibles.

**P: ¿Necesito volver a subir el sketch cada vez?**
R: No, la configuración se guarda en el sensor. Solo necesitas hacerlo una vez.

---

## Versión y Autor

- **Versión:** 2.0
- **Fecha:** 2026
- **Base:** DFRobot MultiGasSensor Library
- **Librería Original:** https://github.com/DFRobot/DFRobot_MultiGasSensor

