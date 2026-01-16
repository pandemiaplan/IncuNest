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
          # Ajusta el nombre de entorno si difiere; esto evita capturar múltiples bins.
          path: Firmware/Display_HMI/.pio/build/default/firmware.bin
```

Puntos clave:
- Se dispara solo con tags `v*` (emparejado con el formato SemVer usado en código).
- Cada job sube su binario como artefacto; puedes anexarlos a la release asociada al tag.
- Si usas particiones personalizadas, incluye la CSV en los artefactos o en la release.

## Paso a paso: ejemplo práctico

1) **Crear rama de trabajo**  
   - Crea una rama descriptiva, ej.: `release/v14.12.2`.  
   - Objetivo: ajustar `FWversion` (placa) y la versión equivalente en el HMI, más cualquier fix asociado.

2) **Actualizar versiones en código**  
   - Edita `Firmware/motherBoard/include/board.h` → `#define FWversion "14.12.2"`.  
   - Ajusta la versión en el HMI si aplica (mismos números para la incubadora A).

3) **Ejecutar pruebas automáticas y build local**  
   - Desde `Firmware/motherBoard`: `pio run -e in3ator_UP_TO_V14`.  
   - Desde `Firmware/Display_HMI`: `pio run` (o el entorno que uses).  
   - Opcional: correr linters/tests existentes; verifica logs de CI si el repo tiene Actions configuradas.

4) **Validación manual rápida**  
   - Revisa `firmware.bin` generado y, si es posible, carga en un banco de pruebas o simulador.  
   - Verifica que el monitor serie muestre la versión correcta (coherente con `FWversion`/tag).

5) **Crear tag anotado**  
   - En el commit que contiene los bumps de versión:  
     ```bash
     git tag -a v14.12.2 -m "motherBoard + HMI v14.12.2 (HW 14A)"
     git push origin v14.12.2
     ```

6) **Generar release en GitHub**  
   - Usa el tag `v14.12.2`.  
   - Adjunta artefactos de ambos firmwares:  
     - `Firmware/motherBoard/.pio/build/in3ator_UP_TO_V14/firmware.bin`  
     - `Firmware/Display_HMI/.pio/build/default/firmware.bin` (ajusta si usas otro entorno)  
     - Tabla de particiones `ESP32_OTA_partition_16MB.csv` si aplica.  
   - Nota de la release: alcance, hardware soportado, qué binarios cambiaron, hashes opcionales.

7) **Verificar publicación**  
   - Confirma que la release muestra los dos binarios y el tag correcto.  
   - Si usas OTA/telemetría, valida que `FWversion` reportado coincide con la release publicada.

## Política de ramas, main y releases

- **main**: rama estable; debe contener solo código ya validado y listo para producción. Los tags `v*` se crean siempre sobre commits presentes en `main` para que las releases se basen en el estado estable.
- **Ramas de trabajo**: usa `feature/` para desarrollo y `release/` para preparar una versión (ej.: `release/v14.12.2`). Los bumps de versión y artefactos de build se validan aquí antes de fusionar a `main`.
- **Releases y almacenamiento**: el tag `vMAJOR.MINOR.PATCH` se publica en `main` y enlaza una GitHub Release donde se adjuntan los binarios (placa + HMI) y, opcionalmente, la tabla de particiones. Los artefactos generados por Actions en builds de tag sirven como origen reproducible para esas descargas.

## Buenas prácticas

- Un único origen de verdad: no edites binarios a mano después de actualizar `FWversion`.
- Mantén incrementos monotónicos (no reutilices versiones ni regreses a números anteriores).
- Documenta breaking changes de hardware (pines, particiones) en la nota de la release y en el mensaje del tag.
- Usa ramas de hotfix para corregir bugs críticos y vuelve a `main` con un tag de `PATCH`.
- Guarda artefactos compilados junto al tag para que puedan reproducirse las imágenes OTA históricas.
