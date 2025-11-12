# IncuNest x IASA Business Technology Strategy

---

## Propósito del Proyecto
- IncuNest es una incubadora neonatal de bajo coste orientada a hospitales con recursos limitados.【F:README.md†L1-L33】
- Objetivo social: reducir la mortalidad neonatal con un dispositivo replicable y accesible (≈ €350).【F:README.md†L11-L19】
- Impacto probado con más de 200 incubadoras desplegadas en 30 países.【F:README.md†L63-L76】

---

## Business Technology Strategy (BTS)
- **Valor para el negocio**: reemplazo asequible frente a incubadoras comerciales de €35.000, habilitando nuevos mercados de salud pública y ONG.【F:README.md†L11-L26】
- **Modelo operativo**: documentación abierta que permite fabricación local y reducción de costes logísticos.【F:README.md†L17-L19】【F:README.md†L83-L96】
- **Estrategia tecnológica**: combinación de hardware abierto, firmware modular y telemetría remota para habilitar mantenimiento predictivo y servicios de soporte.
- **Roadmap**: facilidad para extender sensores/controladores siguiendo patrones definidos en la arquitectura para evolucionar funcionalidades clínicas.【F:docs/arquitectura.md†L83-L104】

---

## Human Dynamics
- **Comunidad y voluntariado**: mantenido por la ONG Medical Open World con apoyo de centros formativos y empresas colaboradoras.【F:README.md†L67-L76】【F:README.md†L108-L131】
- **Empoderamiento local**: documentación abierta (CAD, PCB, firmware) fomenta capacidades técnicas en regiones de despliegue.【F:README.md†L21-L48】
- **Habilitación de equipos clínicos**: HMI táctil guiada y alarmas auditivas que facilitan el uso por personal sanitario diverso.【F:Firmware/Display_HMI/src/main.cpp†L1-L200】【F:Firmware/motherBoard/src/main.cpp†L244-L283】
- **Gestión del cambio**: telemetría y OTA reducen la necesidad de soporte presencial, facilitando adopción progresiva en hospitales con limitada asistencia técnica.【F:Firmware/motherBoard/src/Wifi_OTA.cpp†L29-L200】【F:Firmware/motherBoard/src/GPRS.cpp†L33-L200】

---

## Design
- **Arquitectura integral**: separación en dominios de hardware mecánico/electrónico, firmware embebido y servicios conectados.【F:docs/arquitectura.md†L9-L24】
- **Firmware modular**: tareas FreeRTOS para sensórica, UI, comunicaciones y seguridad, reduciendo acoplamiento.【F:docs/arquitectura.md†L26-L60】
- **Control adaptativo**: múltiples PIDs gestionan temperatura de aire/piel y humedad con anti-windup y límites dinámicos.【F:docs/arquitectura.md†L67-L80】
- **UI diseñada para contexto clínico**: LVGL con control táctil, perfiles de interacción y elementos bloqueables según estado.【F:docs/arquitectura.md†L92-L108】

---

## IT Environment
- **Plataforma embebida**: doble ESP32 (placa base e HMI) con PlatformIO y Arduino framework para asegurar reproducibilidad del entorno de desarrollo.【F:README.md†L21-L36】【F:README.md†L54-L94】
- **Conectividad resiliente**: WiFi y GPRS duales con provisión dinámica, monitorización de RSSI y recuperación ante fallos.【F:docs/arquitectura.md†L112-L138】
- **Operación continua**: tareas dedicadas a watchdog, seguridad y backlight conservan estabilidad incluso con recursos limitados.【F:Firmware/motherBoard/src/main.cpp†L244-L383】
- **Pipeline de datos**: telemetría JSON optimizada y OTA vía MQTT soportan integración con plataformas IoT como ThingsBoard.【F:docs/arquitectura.md†L112-L138】

---

## Quality Attributes
- **Confiabilidad**: redundancia de sensores (STS3x + SHTC3/SHT4), filtrado avanzado y tareas de seguridad reducen fallos críticos.【F:docs/arquitectura.md†L59-L74】【F:Firmware/motherBoard/src/sensors.cpp†L200-L360】
- **Mantenibilidad**: estructura `in3ator_parameters` centraliza el estado y facilita la ampliación del firmware con nuevos módulos.【F:docs/arquitectura.md†L32-L46】
- **Escalabilidad operativa**: documentación abierta y OTA permiten escalar despliegues globales sin dependencias de proveedores propietarios.【F:README.md†L17-L48】【F:docs/arquitectura.md†L112-L138】
- **Seguridad y cumplimiento**: alarmas, watchdog y control de potencia segura con anti-windup priorizan la integridad del paciente.【F:docs/arquitectura.md†L67-L80】【F:Firmware/motherBoard/src/PID.cpp†L31-L160】

---

## Próximos Pasos Recomendados
- Formalizar indicadores de calidad de servicio (tiempo medio entre fallos, tasa de alarmas) para medir impacto clínico.
- Desarrollar kits de capacitación estandarizados para personal hospitalario basados en la interfaz HMI existente.
- Evaluar integración con plataformas de salud pública para reporte automático de métricas neonatales.

