# Simulador de IncuNest en Wokwi

Este documento describe cómo trasladar el firmware principal de IncuNest al simulador Wokwi, replicando el hardware del módulo `motherBoard` y los sensores que utiliza el proyecto. Se detallan los ajustes de firmware necesarios, la configuración del proyecto en Wokwi y las estrategias para emular cada periférico.

## 1. Preparar el firmware base

1. Clona el repositorio y abre la carpeta `Firmware/motherBoard`, que contiene el firmware que controla sensores, actuadores y la interfaz de usuario del incubador.【F:Firmware/motherBoard/src/main.cpp†L28-L120】
2. Mantén la configuración de PlatformIO, ya que define que la placa objetivo es una ESP32 FireBeetle (`board = firebeetle32`) y lista todas las dependencias que también tendrás que cargar en Wokwi.【F:Firmware/motherBoard/platformio.ini†L1-L57】
3. Identifica los archivos que necesitarás portar al proyecto Wokwi:
   - Código fuente en `src/` y cabeceras en `include/` (por ejemplo `main.cpp`, `sensors.cpp`, `initHardware.cpp`, `main.h`).
   - Recursos adicionales como las librerías locales (`lib/`) y los ficheros de configuración (`board.h`, `in3ator_humidifier.h`).

## 2. Crear el proyecto en Wokwi

1. Inicia sesión en [wokwi.com](https://wokwi.com) y crea un nuevo proyecto de tipo "ESP32".
2. Sustituye el contenido inicial por la estructura del firmware de IncuNest. Para mantener la compatibilidad con Wokwi, puedes usar un `sketch.ino` que incluya `#include "main.cpp"` o dividir el código en ficheros `.h/.cpp` dentro del editor de Wokwi.
3. Añade un archivo `wokwi.toml` y define la placa FireBeetle32 para que el simulador utilice el pinout correcto:
   ```toml
   [wokwi]
   version = 1
   elf = "firmware.elf"

   [board]
   name = "firebeetle32"
   ```
   El firmware se compilará dentro de Wokwi usando el backend de Arduino, pero también puedes subir un `firmware.elf` generado con PlatformIO.
4. Declara las dependencias externas en la sección `libraries` de `diagram.json` o mediante la interfaz de "Libraries" de Wokwi. Copia la lista desde `platformio.ini` (PID Library, RotaryEncoder, INA3221, Adafruit GFX, SHTC3, ThingsBoard, TCA9555, SHT4x, Arduino-Filters, AFE4490, ArduinoMqttClient, TFT_eSPI, HttpClient, BQ25792, STS3x, ArduinoJson, TBPubSubClient).【F:Firmware/motherBoard/platformio.ini†L36-L57】

## 3. Mapear el hardware del proyecto

Usa `board.h` para replicar el cableado de la revisión de hardware 14 (la predeterminada en el repositorio).【F:Firmware/motherBoard/include/board.h†L26-L114】 En Wokwi, conecta los periféricos a los mismos pines:

| Componente | Pines ESP32 (FireBeetle v14) | Notas |
|------------|-------------------------------|-------|
| Bus I2C principal | SDA GPIO21, SCL GPIO22 | Todos los sensores digitales comparten el bus.【F:Firmware/motherBoard/include/board.h†L96-L106】|
| Encoder rotativo | A GPIO25, B GPIO34, pulsador GPIO4 | Configuración usada por `RotaryEncoder` en el firmware.【F:Firmware/motherBoard/include/board.h†L90-L105】|
| TFT (`TFT_eSPI`) | CS GPIO15, DC GPIO0, retroiluminación GPIO33 | Mantén el SPI por defecto para depurar solo la lógica de sensores si no necesitas la pantalla.【F:Firmware/motherBoard/include/board.h†L90-L107】|
| NTC piel bebé | ADC GPIO39 | Se lee como entrada analógica en `measureNTCTemperature()`.【F:Firmware/motherBoard/include/board.h†L96-L108】【F:Firmware/motherBoard/src/sensors.cpp†L332-L380】|
| Ventilador y relés | GPIO12/13/14/27 | Opcional si solo simulas sensores.【F:Firmware/motherBoard/include/board.h†L92-L103】|

## 4. Sensores usados por el firmware

El firmware inicializa los siguientes sensores y dispositivos en el bus I2C compartido y los usa durante el ciclo principal:

| Sensor / Dispositivo | Dirección I2C | Uso en el firmware |
|----------------------|---------------|--------------------|
| SHTC3 (Sensirion) | `0x70` | Temperatura y humedad de la cúpula.【F:Firmware/motherBoard/include/main.h†L415-L431】【F:Firmware/motherBoard/src/sensors.cpp†L307-L403】|
| STS35 (Sensirion, principal y redundante) | `0x4A` y `0x4B` | Temperatura redundante de aire; se promedia cuando ambos sensores responden.【F:Firmware/motherBoard/include/main.h†L415-L431】【F:Firmware/motherBoard/src/sensors.cpp†L323-L397】|
| SHT4x (ambiental) | `0x44` | Temperatura y humedad ambiente, se inicializa aparte.【F:Firmware/motherBoard/include/main.h†L408-L411】【F:Firmware/motherBoard/src/sensors.cpp†L405-L417】|
| INA3221 (corriente principal) | `0x41` | Monitoriza corrientes de actuadores.【F:Firmware/motherBoard/include/main.h†L405-L410】【F:Firmware/motherBoard/src/initHardware.cpp†L381-L414】|
| INA3221 (corriente secundaria) | `0x40` | Monitoriza cargador, batería y calefactor.【F:Firmware/motherBoard/include/main.h†L405-L410】【F:Firmware/motherBoard/src/initHardware.cpp†L381-L414】|
| BQ25792 | `0x6B` (definido en la librería) | Gestión de carga de batería.【F:Firmware/motherBoard/src/main.cpp†L32-L43】|
| Humidificador I2C propio | `0x02` | Control binario de humidificador.【F:Firmware/motherBoard/src/main.cpp†L32-L43】【F:Firmware/motherBoard/include/in3ator_humidifier.h†L37-L76】|
| NTC piel bebé | Entrada analógica | Convierte a °C con `adcToCelsius`.【F:Firmware/motherBoard/src/sensors.cpp†L332-L399】|

Durante el arranque, `initSensors()` escanea cada dirección y marca qué dispositivos están presentes. Si un sensor no responde, el firmware reintenta y genera alarmas, por lo que en la simulación es necesario devolver ACK en cada dirección esperada.【F:Firmware/motherBoard/src/initHardware.cpp†L316-L429】

## 5. Estrategias de simulación en Wokwi

### 5.1 SHTC3 (temperatura y humedad de la cúpula)

- Usa un componente personalizado de Wokwi (`parts/shtc3` o un `custom` I2C). Implementa un script que reconozca los comandos de medida y devuelva tramas con temperatura/humedad estáticas o controladas por sliders.
- Asegúrate de responder con `ACK` al escaneo inicial y de actualizar las lecturas periódicamente, porque `updateRoomSensor()` espera valores dentro del rango válido (`DIG_TEMP_TO_DISCARD_MIN/MAX`).【F:Firmware/motherBoard/src/sensors.cpp†L323-L380】

### 5.2 STS35 redundante

- Crea dos instancias del mismo dispositivo personalizado, fijando las direcciones `0x4A` y `0x4B`. El firmware usa `measureSingleShot` y acepta la media de ambos sensores cuando responden correctamente.【F:Firmware/motherBoard/src/sensors.cpp†L323-L397】
- Devuelve temperaturas coherentes para evitar que la verificación reinicie los sensores (`initRoomSensor()` se relanza si fallan).【F:Firmware/motherBoard/src/initHardware.cpp†L320-L365】

### 5.3 SHT4x ambiental

- Wokwi incluye un modelo para la familia SHT4x. Conéctalo a SDA/SCL y selecciona la dirección `0x44`. El firmware lee `sht4.getEvent()` para obtener temperatura y humedad ambiente.【F:Firmware/motherBoard/src/sensors.cpp†L405-L417】

### 5.4 Monitores de corriente INA3221

- No existe un modelo oficial en Wokwi, así que emplea un componente personalizado que responda a lecturas de tensión/corriente. Basta con confirmar la presencia (ACK a `0x41` y `0x40`) y devolver registros estáticos para que `initCurrentSensor()` complete la inicialización.【F:Firmware/motherBoard/src/initHardware.cpp†L381-L414】
- Si quieres mostrar valores dinámicos, implementa los registros básicos del INA3221 (voltaje de bus y shunt) y actualízalos con scripts.

### 5.5 Controlador de carga BQ25792

- El firmware solo instancia el objeto y utiliza la librería para gestionar la batería, por lo que en la simulación puedes reemplazarlo por un stub que devuelva estados fijos o un componente personalizado que responda a lecturas del bus I2C.【F:Firmware/motherBoard/src/main.cpp†L32-L43】

### 5.6 Humidificador I2C

- Implementa un dispositivo simple que acepte comandos en la dirección `0x02` y active un LED virtual o un mensaje en consola para visualizar cuándo `turn()` enciende el humidificador.【F:Firmware/motherBoard/include/in3ator_humidifier.h†L37-L76】

### 5.7 Sensor NTC analógico

- Sustituye el NTC físico por un potenciómetro de Wokwi conectado a GPIO39 (entrada analógica). Girar el potenciómetro cambia la tensión y, a través de `adcToCelsius()`, la temperatura estimada.【F:Firmware/motherBoard/include/board.h†L90-L108】【F:Firmware/motherBoard/src/sensors.cpp†L332-L380】

## 6. Ajustes opcionales del firmware para el modo simulador

1. Define un macro (`WOKWI_SIM`) en la configuración de compilación para ejecutar rutas específicas cuando no haya hardware real. Puedes añadirlo en la lista de `build_flags` de PlatformIO antes de exportar el firmware al simulador.
2. Dentro de `initSensors()` y `updateRoomSensor()` añade bloques `#ifdef WOKWI_SIM` que inserten valores por defecto si un sensor no está presente, evitando reinicios continuos durante las pruebas.【F:Firmware/motherBoard/src/initHardware.cpp†L316-L429】【F:Firmware/motherBoard/src/sensors.cpp†L307-L403】
3. Activa la salida serie (`Serial.begin(115200)`) para inspeccionar en Wokwi la información de depuración que ya genera el firmware (mensajes `logI`, `logE`).

## 7. Ejecutar la simulación

1. Compila el proyecto en Wokwi o sube el `firmware.elf` construido con PlatformIO (`pio run`). Usa la misma partición que en el proyecto original (`ESP32_OTA_partition_16MB.csv`).【F:Firmware/motherBoard/platformio.ini†L1-L35】
2. Inicia la simulación. Comprueba en el monitor serie que todos los sensores se detectan correctamente (`[HW] -> Room sensor found ...`, `[HW] ->digital sensor detected`).【F:Firmware/motherBoard/src/initHardware.cpp†L320-L414】
3. Modifica los valores de los sensores virtuales (potenciómetros, sliders o scripts) para observar cómo cambian las lecturas y las alarmas gestionadas por `updateRoomSensor()` y `measureNTCTemperature()`.【F:Firmware/motherBoard/src/sensors.cpp†L307-L403】
4. Si necesitas validar la respuesta de los actuadores, conecta LEDs o relés virtuales a los pines correspondientes y vigila el consumo reportado por los INA3221 simulados.

Con esta configuración tendrás un entorno reproducible para validar la lógica del firmware de IncuNest directamente en Wokwi, reutilizando el código original y controlando los sensores críticos desde el simulador.
