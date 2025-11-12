# Arquitectura integral de IncuNest

## Resumen de la solución
IncuNest es una incubadora neonatal de bajo coste cuyo objetivo es proporcionar control térmico y de humedad en entornos con recursos limitados, manteniendo un diseño replicable a partir de documentación abierta de hardware, electrónica y firmware.【F:README.md†L1-L52】

La plataforma se divide en tres dominios principales:

1. **Diseño físico**: planos mecánicos y electrónicos publicados como CAD, PCB y listas de materiales replicables.
2. **Firmware embebido**: dos firmwares independientes (placa base y HMI) construidos con PlatformIO/Arduino para microcontroladores ESP32.
3. **Servicios conectados**: telemetría y actualizaciones OTA mediante WiFi y GPRS hacia servidores ThingsBoard u otras integraciones.

## Componentes físicos y repositorios de diseño
El directorio `Hardware/` agrupa todo el material de ingeniería: esquemáticos y PCB de la placa base, sensores auxiliares y controladoras de iluminación en `Hardware/Electronics/`, junto con los modelos 3D en `Hardware/Mechanical/`. Esto permite fabricar localmente tanto la electrónica como la envolvente mecánica.【F:README.md†L21-L48】

## Arquitectura del firmware de la placa base
El firmware principal (`Firmware/motherBoard/`) se ejecuta sobre un ESP32 que concentra la sensórica, el control y la conectividad. Toda la información operacional se almacena en la estructura `in3ator_parameters`, que mantiene lecturas de sensores, estado de actuadores, telemetría eléctrica, alarmas y parámetros de idioma, sirviendo como núcleo de estado compartido entre módulos.【F:Firmware/motherBoard/include/main.h†L554-L616】

### Inicialización de hardware y servicios
En `setup()` se inicializan periféricos (I2C, sensores SHTC3/STS3x/SHT4, encoder, display TFT, sensores de corriente INA3221, cargador BQ25792) y se habilita la conectividad WiFi cuando está disponible.【F:Firmware/motherBoard/src/main.cpp†L32-L144】【F:Firmware/motherBoard/src/main.cpp†L291-L320】

### Modelo de concurrencia
El firmware usa FreeRTOS para repartir el trabajo en tareas cooperativas:

- **`GPRS_Task`** y su monitor vigilan la conectividad celular y eliminan la tarea si queda bloqueada, reintentando la provisión o publicación de datos.【F:Firmware/motherBoard/src/main.cpp†L154-L196】
- **`sensors_Task`** toma muestras periódicas de todos los sensores (termistores, capacitivos, STS3x redundantes, SHTC3/SHT4, INA3221, ventilador) y actualiza el estado compartido.【F:Firmware/motherBoard/src/main.cpp†L205-L241】【F:Firmware/motherBoard/src/sensors.cpp†L200-L360】
- **`OTA_WIFI_Task`**, **`buzzer_Task`**, **`security_Task`**, **`Backlight_Task`**, **`TimeTrack_Task`** y **`UI_Task`** encapsulan respectivamente la lógica OTA WiFi, avisos acústicos, comprobaciones de seguridad, ahorro energético del display, telemetría temporal y rendering de la interfaz local.【F:Firmware/motherBoard/src/main.cpp†L244-L383】

La función `loop()` se reduce a refrescar el watchdog y consolidar datos (`updateData()`), manteniendo la mayor parte del trabajo en tareas concurrentes.【F:Firmware/motherBoard/src/main.cpp†L394-L397】

### Adquisición de sensores y filtrado
`updateRoomSensor()` agrega lecturas de los sensores digitales de temperatura/humedad (SHTC3, STS3x redundante) validando rangos y registrando errores, mientras que `measureNTCTemperature()` convierte el termistor de piel con filtrado Butterworth de sexto orden para suavizar ruido. Los INA3221 aportan corriente y tensión de actuadores, y se calcula la velocidad del ventilador a partir de un codificador de pulsos.【F:Firmware/motherBoard/src/sensors.cpp†L230-L360】

### Control y actuadores
El control térmico y de humedad se implementa mediante tres controladores PID (aire, piel y humedad) que comparten un canal PWM para la resistencia y un control discreto del humidificador. El módulo ajusta automáticamente el límite de potencia segura del calefactor según el consumo medido y aplica anti-windup para evitar integraciones excesivas.【F:Firmware/motherBoard/src/PID.cpp†L31-L194】

### Seguridad y alarmas
El parámetro `in3ator_parameters` mantiene banderas de alarmas y consumos fuera de rango, mientras que `security_Task` invoca `securityCheck()` para evaluar límites críticos (temperatura, corriente, ventilador) con capacidad de silenciar o reactivar el zumbador según la configuración.【F:Firmware/motherBoard/include/main.h†L554-L616】【F:Firmware/motherBoard/src/main.cpp†L259-L266】

## Arquitectura del firmware HMI
El firmware `Firmware/Display_HMI/` corre sobre un ESP32-S3 con interfaz RGB para un panel 800×480 y táctil GT911. Usa LovyanGFX para la capa de hardware, LVGL para la lógica de UI y SquareLine Studio para la generación de pantallas (`ui_*.c`). La clase `LGFX` configura el bus RGB y el controlador del panel, mientras que `my_disp_flush` y `my_touchpad_read` integran LVGL con el driver gráfico y el controlador táctil.【F:Firmware/Display_HMI/src/main.cpp†L1-L152】

La lógica de interacción gestiona modos de control (aire/piel), interruptores y botones de ajuste, activando o desactivando elementos táctiles según el estado del sistema para guiar al personal clínico.【F:Firmware/Display_HMI/src/main.cpp†L154-L200】

## Comunicación, telemetría y OTA
La placa base expone dos canales de conectividad hacia ThingsBoard u otras plataformas MQTT:

- **WiFi**: `wifiInit()` configura el ESP32 en modo estación, abre un servidor HTTP autenticado para actualizaciones OTA y reutiliza un cliente MQTT para publicar telemetrías y recibir comandos.【F:Firmware/motherBoard/src/Wifi_OTA.cpp†L29-L200】
- **GPRS**: el módulo SIM800 se gestiona con TinyGSM; el firmware maneja provisión dinámica, lectura de ubicación por triangulación y descargas OTA vía MQTT, monitorizando buffers y la señal RSSI para asegurar estabilidad en redes débiles.【F:Firmware/motherBoard/src/GPRS.cpp†L33-L200】

Ambos canales comparten la misma lógica de callbacks OTA (`progressCallback`, `updatedCallback`) y publican telemetrías empaquetadas en documentos JSON estáticos optimizados para los recursos del microcontrolador.【F:Firmware/motherBoard/src/GPRS.cpp†L69-L94】【F:Firmware/motherBoard/src/Wifi_OTA.cpp†L41-L56】

## Flujo operacional de extremo a extremo
1. Los sensores físicos (piel, aire, humedad, corriente, ventilador) se muestrean en `sensors_Task`, se filtran y se escriben en `in3ator_parameters`.
2. `PIDHandler()` ajusta potencia de calefactor, ventilador y humidificador según objetivos clínicos, respetando límites de seguridad dinámicos.【F:Firmware/motherBoard/src/PID.cpp†L66-L160】
3. `UI_Task` renderiza los valores y estados en la pantalla TFT, permitiendo al operario seleccionar modos y consignas; la HMI replica el mismo modelo lógico con LVGL para proporcionar interacción táctil intuitiva.【F:Firmware/motherBoard/src/main.cpp†L268-L282】【F:Firmware/Display_HMI/src/main.cpp†L154-L200】
4. `Wifi_OTA` y `GPRS` empaquetan la telemetría eléctrica, ambiental y de alarmas para ThingsBoard, habilitando monitorización remota y despliegues de firmware sin intervención física.【F:Firmware/motherBoard/src/Wifi_OTA.cpp†L29-L200】【F:Firmware/motherBoard/src/GPRS.cpp†L45-L200】

## Construcción y despliegue
El README detalla cómo compilar cada firmware con PlatformIO y subrayar que hay dos placas a programar (placa base y display). Para habilitar conectividad es necesario crear `Firmware/motherBoard/include/Credentials.h` con las credenciales de WiFi y ThingsBoard antes del despliegue.【F:README.md†L54-L98】

## Consideraciones para extensiones
- **Integración de nuevos sensores**: reutilizar el pipeline de filtrado en `sensors.cpp` y extender `in3ator_parameters` con los campos necesarios.
- **Nuevos modos de control**: ajustar los controladores PID o añadir nuevas tareas FreeRTOS siguiendo el patrón de `setup()` para encapsular funcionalidad y evitar bloqueo del bucle principal.
- **Personalización de la UI**: modificar los archivos `ui_*.c` generados por SquareLine y ampliar los callbacks en `main.cpp` de la HMI para nuevas interacciones.

Con esta arquitectura modular, IncuNest puede adaptarse a diferentes entornos hospitalarios manteniendo telemetría y actualizaciones OTA seguras incluso con conectividad limitada.
