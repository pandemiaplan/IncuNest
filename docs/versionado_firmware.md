# Guía de versionado del firmware

Este documento explica cómo versionar el código de los firmware del proyecto y cómo vincular cada versión con una release en GitHub.

## Esquema de versiones

- Usa `MAJOR.MINOR.PATCH` (SemVer). Ejemplo: `14.12.1`.
- El valor **MAJOR** debe reflejar la revisión de hardware soportada (`HW_NUM` en el código). Si el hardware cambia de forma incompatible, incrementa MAJOR.
- **MINOR**: nuevas funcionalidades compatibles con la misma revisión de hardware.
- **PATCH**: correcciones sin cambios de comportamiento visibles.
- Para versiones de prueba, añade sufijos pre-release (`-rc.1`, `-beta.1`) y marca la release como *Pre-release* en GitHub.

Para la incubadora **versión A** hay **dos firmwares** que se publican juntos:
- **Placa madre** (`Firmware/motherBoard`).
- **HMI/display** (`Firmware/Display_HMI`).

Ambos deben compartir el mismo esquema de versión y quedar enlazados en la misma release (mismo tag), incluso si en un ciclo solo cambia uno de los binarios.

## Fuente de verdad dentro del código

1. **Firmware motherBoard**  
   El número de firmware que se reporta por OTA, WiFi y telemetría se define en `Firmware/motherBoard/include/board.h` con la macro `FWversion`.  
   Actualiza ese valor antes de cortar una release para que coincida con el tag que vas a publicar.
2. **Firmware Display_HMI**  
   Mantén el mismo esquema de numeración en el código fuente del HMI (por ejemplo, texto mostrado en pantalla o constantes internas) y sincronízalo con el tag que publiques. Aunque solo cambie el HMI, usa el mismo tag de release y documenta en las notas qué binario se actualiza.

## Flujo recomendado para cortar una release

1. **Bump de versión en código**  
   - Edita `FWversion` en `Firmware/motherBoard/include/board.h`.  
   - Ajusta la versión equivalente en el firmware del HMI si aplica. Mantén ambos alineados en la incubadora versión A.
2. **Builds reproducibles**  
   - Ejecuta `pio run -e in3ator_UP_TO_V14` dentro de `Firmware/motherBoard` para generar `firmware.bin` (ruta: `.pio/build/in3ator_UP_TO_V14/firmware.bin`).  
   - Construye el firmware del HMI desde `Firmware/Display_HMI` con su entorno PlatformIO correspondiente.
3. **Tag de Git**  
   - Crea un tag anotado con el formato `vMAJOR.MINOR.PATCH` (ej: `v14.12.1`) en el commit que contiene los cambios de versión.  
   - Si solo afecta a una placa, indícalo en el mensaje del tag (ej: "motherBoard v14.12.1 compatible HW 14").
4. **Release en GitHub**  
   - Genera una release usando el mismo tag.  
   - Adjunta los binarios resultantes de **ambos firmwares**: placa (`firmware.bin`) y HMI. Incluye, si cambia, la tabla de particiones OTA usada (`ESP32_OTA_partition_16MB.csv`).  
   - Incluye en las notas: alcance, cambios relevantes, qué binarios se actualizaron, revisión de hardware soportada y hashes de build si los tienes.
5. **Coherencia en OTA/telemetría**  
   - Verifica que el valor de `FWversion` visible en monitor serie y en los envíos ThingsBoard coincida con el tag publicado.  
   - Si el HMI muestra la versión en pantalla, comprueba que coincide con la release.

### Ejemplo de flujo con GitHub Actions (build + assets en tags)

Agrega un workflow (ej. `.github/workflows/build-firmware.yml`) que compile ambos firmwares cuando se cree un tag `v*` y suba los binarios como artefactos; estos se pueden anexar manualmente a la release o con otro job:

```yaml
name: Build firmware

on:
  push:
    tags:
      - "v*"

jobs:
  build-motherboard:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.x"
      - name: Install PlatformIO
        run: pip install platformio
      - name: Build motherBoard
        working-directory: Firmware/motherBoard
        run: pio run -e in3ator_UP_TO_V14
      - name: Upload motherboard bin
        uses: actions/upload-artifact@v4
        with:
          name: firmware-motherboard
          path: Firmware/motherBoard/.pio/build/in3ator_UP_TO_V14/firmware.bin

  build-hmi:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
        with:
          fetch-depth: 0
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: "3.x"
      - name: Install PlatformIO
        run: pip install platformio
      - name: Build HMI
        working-directory: Firmware/Display_HMI
        run: pio run
      - name: Upload HMI bin
        uses: actions/upload-artifact@v4
        with:
          name: firmware-hmi
          path: Firmware/Display_HMI/.pio/build/**/firmware.bin
```

Puntos clave:
- Se dispara solo con tags `v*` (emparejado con el formato SemVer usado en código).
- Cada job sube su binario como artefacto; puedes anexarlos a la release asociada al tag.
- Si usas particiones personalizadas, incluye la CSV en los artefactos o en la release.

## Buenas prácticas

- Un único origen de verdad: no edites binarios a mano después de actualizar `FWversion`.
- Mantén incrementos monotónicos (no reutilices versiones ni regreses a números anteriores).
- Documenta breaking changes de hardware (pines, particiones) en la nota de la release y en el mensaje del tag.
- Usa ramas de hotfix para corregir bugs críticos y vuelve a `main` con un tag de `PATCH`.
- Guarda artefactos compilados junto al tag para que puedan reproducirse las imágenes OTA históricas.
