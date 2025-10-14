#include "Arduino.h"
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "usb/cdc_acm_host.h"
#include "usb/usb_host.h"
#include "usb/vcp.hpp"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"

#include "board.h"
#include "display_comms.h"


/* ===== Config de ejemplo ===== */
#define SENSOR_TEMP_QTY 4
#define SENSOR_HUM_QTY 2
#define NUM_ALARMS 8

/* Simulación de variables */
static double temperature[SENSOR_TEMP_QTY];
static double humidity[SENSOR_HUM_QTY];
static double system_current = 0.5;
static double system_voltage = 12.1;
static float fan_rpm = 1200.0f;

/* Setpoints/mandos recibidos del Display */
static int actuation = 0;
static double desiredControlTemperature = 36.5;
static double desiredControlHumidity = 60.0;
static bool temperatureControl = true;
static bool humidityControl = false;
static bool phototherapy = false;

/* Flags para actuar desde loop() si se desea */
static volatile bool photo_cmd_pending =
    false; // se pone a true cuando se detecta "photo0/1"
static volatile bool photo_cmd_value = false; // true -> photo1, false -> photo0

using namespace esp_usb;

// Change these values to match your needs
#define EXAMPLE_BAUDRATE (115200)
#define EXAMPLE_STOP_BITS (0) // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define EXAMPLE_PARITY (0) // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define EXAMPLE_DATA_BITS (8)

/* Driver */
static dc_handle_t dc;

/* === Callbacks: Motherboard recibe COMMAND del Display === */
static void on_command_tlv(uint16_t t, const uint8_t *v, uint16_t l) {
  switch (t) {
  case DC_T_ACTUATION:
    if (l == sizeof(int32_t)) {
      int32_t x;
      memcpy(&x, v, l);
      actuation = (int)x;
    }
    break;
  case DC_T_DESIRED_TEMP:
    if (l == sizeof(double)) {
      memcpy(&desiredControlTemperature, v, l);
    }
    break;
  case DC_T_DESIRED_HUM:
    if (l == sizeof(double)) {
      memcpy(&desiredControlHumidity, v, l);
    }
    break;
  case DC_T_TEMP_CONTROL_EN:
    if (l == 1) {
      temperatureControl = v[0] != 0;
    }
    break;
  case DC_T_HUM_CONTROL_EN:
    if (l == 1) {
      humidityControl = v[0] != 0;
    }
    break;
  case DC_T_PHOTOTHERAPY_EN:
    if (l == 1) {
      phototherapy = v[0] != 0;
    }
    break;
  default:
    break;
  }
}

static void on_request_full_state(void) {
  // Cuando el display pida estado completo, lo enviamos inmediatamente
  if (dc_begin_frame(DC_MSG_TELEMETRY)) {
    dc_add_tlv_blob(DC_T_TEMPERATURES, temperature, sizeof(temperature));
    dc_add_tlv_blob(DC_T_HUMIDITIES, humidity, sizeof(humidity));
    dc_add_tlv_double(DC_T_SYSTEM_CURRENT, system_current);
    dc_add_tlv_double(DC_T_SYSTEM_VOLTAGE, system_voltage);
    dc_add_tlv_float(DC_T_FAN_RPM, fan_rpm);
    dc_end_frame(&dc);
  }
}

static void on_link_up(void) { /* opcional: LED, estado, etc. */
}
static void on_link_down(void) { /* opcional: LED, estado, etc. */
}
/* Motherboard NO necesita on_telemetry_tlv */

namespace {
static const char *TAG = "IncuNest_main";
static SemaphoreHandle_t device_disconnected_sem;

/**
 * @brief Data received callback
 *
 * Detecta "photo1"/"photo0" aunque llegue fragmentado o junto con otros textos,
 * usando una ventana deslizante de 6 bytes. Mantiene el printf para trazar.
 *
 * @param[in] data     Puntero a datos recibidos
 * @param[in] data_len Longitud de datos en bytes
 * @param[in] arg      Argumento pasado al open (no usado)
 * @return true (procesado)
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
  // Ventana deslizante para buscar "photo0"/"photo1"
  static uint8_t win[6];
  static size_t wlen = 0;

  // (opcional) seguir mostrando por consola todo lo recibido
  printf("%.*s", (int)data_len, (const char *)data);

  for (size_t i = 0; i < data_len; ++i) {
    // Añade byte a la ventana (máx 6)
    if (wlen < sizeof(win)) {
      win[wlen++] = data[i];
    } else {
      memmove(win, win + 1, sizeof(win) - 1);
      win[sizeof(win) - 1] = data[i];
    }

    // Compara cuando haya al menos 6 bytes en ventana
    if (wlen >= 6) {
      if (memcmp(win, "photo1", 6) == 0) {
        phototherapy = true; // actualiza estado global
        photo_cmd_value = true;
        photo_cmd_pending = true;
        ESP_LOGI(TAG, "CMD: phototherapy ON (photo1)");
      } else if (memcmp(win, "photo0", 6) == 0) {
        phototherapy = false;
        photo_cmd_value = false;
        photo_cmd_pending = true;
        ESP_LOGI(TAG, "CMD: phototherapy OFF (photo0)");
      }
    }
  }

  return true; // hemos procesado los datos
}

/**
 * @brief Device event callback
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event,
                         void *user_ctx) {
  switch (event->type) {
  case CDC_ACM_HOST_ERROR:
    ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %d", event->data.error);
    break;
  case CDC_ACM_HOST_DEVICE_DISCONNECTED:
    ESP_LOGI(TAG, "Device suddenly disconnected");
    xSemaphoreGive(device_disconnected_sem);
    break;
  case CDC_ACM_HOST_SERIAL_STATE:
    ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
    break;
  case CDC_ACM_HOST_NETWORK_CONNECTION:
  default:
    break;
  }
}

/**
 * @brief USB Host library handling task
 */
static void usb_lib_task(void *arg) {
  while (1) {
    uint32_t event_flags;
    usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
      ESP_ERROR_CHECK(usb_host_device_free_all());
    }
    if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
      ESP_LOGI(TAG, "USB: All devices freed");
      // Continuar manejando eventos para permitir reconexión
    }
  }
}
} // namespace

/**
 * @brief Main application
 *
 * Este ejemplo usa los drivers Virtual COM Port (VCP).
 */
void setup(void) {
  device_disconnected_sem = xSemaphoreCreateBinary();
  assert(device_disconnected_sem);

  // Install USB Host driver. Debe llamarse una única vez
  ESP_LOGI(TAG, "Installing USB Host");
  const usb_host_config_t host_config = {
      .skip_phy_setup = false,
      .intr_flags = ESP_INTR_FLAG_LEVEL1,
  };
  ESP_ERROR_CHECK(usb_host_install(&host_config));

  // Tarea que maneja eventos de la librería USB
  BaseType_t task_created =
      xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
  assert(task_created == pdTRUE);

  ESP_LOGI(TAG, "Installing CDC-ACM driver");
  ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

  // Registrar drivers VCP
  VCP::register_driver<FT23x>();
  VCP::register_driver<CP210x>();
  VCP::register_driver<CH34x>();

  // Bucle para permitir reconexiones del dispositivo
  while (true) {
    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = 5000, // 5 s para conectar o probar timeout
        .out_buffer_size = 512,
        .in_buffer_size = 512,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = NULL,
    };

    ESP_LOGI(TAG, "Opening any VCP device...");
    auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

    if (vcp == nullptr) {
      ESP_LOGI(TAG, "Failed to open VCP device");
      continue;
    }
    vTaskDelay(10);

    ESP_LOGI(TAG, "Setting up line coding");
    cdc_acm_line_coding_t line_coding = {
        .dwDTERate = EXAMPLE_BAUDRATE,
        .bCharFormat = EXAMPLE_STOP_BITS,
        .bParityType = EXAMPLE_PARITY,
        .bDataBits = EXAMPLE_DATA_BITS,
    };
    ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

    // Ejemplo: envío de prueba
    ESP_LOGI(TAG, "Sending data through CdcAcmDevice");
    uint8_t data[] = "test_string";
    ESP_ERROR_CHECK(vcp->tx_blocking(data, sizeof(data)));
    ESP_ERROR_CHECK(vcp->set_control_line_state(true, true));

    // Esperar desconexión y reiniciar ciclo
    ESP_LOGI(TAG, "Done. You can reconnect the VCP device to run again.");
    xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
  }

  pinMode(ENC_SWITCH, INPUT_PULLUP);
  pinMode(TFT_CS, OUTPUT);
  pinMode(PHOTOTHERAPY, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(HEATER, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(SCREENBACKLIGHT, OUTPUT);
  pinMode(ACTUATORS_EN, OUTPUT);
  digitalWrite(ACTUATORS_EN, HIGH);
  digitalWrite(PHOTOTHERAPY,HIGH);

  // GPIOWrite(FAN, LOW);
  //  pinMode(ON_OFF_SWITCH, INPUT);
}

static uint32_t t_last_snapshot = 0;
static uint32_t t_last_delta = 0;

void loop() {
  // (El driver gestiona RX/ACK/Heartbeats en sus tasks)
  // Si prefieres actuar fuera del callback, usa el flag:
  if (photo_cmd_pending) {
    // Captura el valor y limpia el flag de forma atómica simple
    bool value = photo_cmd_value;
    photo_cmd_pending = false;

    // Aquí puedes enviar un ACK, actualizar salidas, etc.
    // Por ejemplo:
    // send_ack_phototherapy(value);

    // Log informativo
    if (value) {
      ESP_LOGI(TAG, "loop(): phototherapy ON aplicado");
    } else {
      ESP_LOGI(TAG, "loop(): phototherapy OFF aplicado");
    }
  }

  vTaskDelay(1);
}
