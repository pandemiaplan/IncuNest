#include "display_comms.h"
#include <string.h>
#include <math.h>

/* ===== CRC16-CCITT (X25): poly 0x1021, init 0xFFFF, xorout 0xFFFF ===== */
static uint16_t crc16_ccitt(const uint8_t* d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; ++i) {
    crc ^= (uint16_t)d[i] << 8;
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc <<= 1;
    }
  }
  return crc ^ 0xFFFF;
}

/* ===== Internal TX frame builder ===== */
typedef struct {
  uint8_t  msgType;
  uint16_t payloadLen;
  uint8_t  buf[DC_MAX_FRAME_SIZE];
} dc_txbuf_t;

static dc_txbuf_t g_tx;

static inline void wr_u16_le(uint8_t* p, uint16_t v){ p[0]=(uint8_t)(v); p[1]=(uint8_t)(v>>8); }
static inline void wr_u32_le(uint8_t* p, uint32_t v){ p[0]=(uint8_t)(v); p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

void dc_init_stream(dc_handle_t* dc, Stream* port, dc_callbacks_t* cbs) {
  memset(dc, 0, sizeof(*dc));
  dc->io = port;
  if (cbs) dc->cbs = *cbs;
  dc->txSeq = 1;
  dc->rxLastSeq = 0;
  dc->txQueue = xQueueCreate(DC_TX_QUEUE_LEN, sizeof(uint16_t));
  dc->ioMtx = xSemaphoreCreateMutex();
  dc->lastRxMs = millis();
}

void dc_init(dc_handle_t* dc, HardwareSerial* port, uint32_t baud, dc_callbacks_t* cbs) {
  if (port) port->begin(baud);
  dc_init_stream(dc, (Stream*)port, cbs);
}

typedef struct {
  uint16_t seq;
  uint16_t len;
  uint8_t  data[DC_MAX_FRAME_SIZE];
} dc_frame_t;

static void dc_rxTaskFn(void* arg);
static void dc_hbTaskFn(void* arg);

void dc_start(dc_handle_t* dc) {
  xTaskCreatePinnedToCore(dc_rxTaskFn, "dc_rx", 4096, dc, 2, &dc->rxTask, 0);
  xTaskCreatePinnedToCore(dc_hbTaskFn, "dc_hb", 2048, dc, 1, &dc->hbTask, 0);
}

/* ===== Frame Build API ===== */
bool dc_begin_frame(uint8_t msgType) {
  g_tx.msgType = msgType;
  g_tx.payloadLen = 0;
  return true;
}

static bool add_tlv_hdr(uint16_t t, uint16_t l) {
  if (8 + g_tx.payloadLen + 4 + l + 2 > DC_MAX_FRAME_SIZE) return false;
  uint8_t* p = &g_tx.buf[8 + g_tx.payloadLen];
  wr_u16_le(p+0, t);
  wr_u16_le(p+2, l);
  g_tx.payloadLen += 4;
  return true;
}

bool dc_add_tlv_blob(uint16_t t, const void* data, uint16_t len) {
  if (!add_tlv_hdr(t, len)) return false;
  memcpy(&g_tx.buf[8 + g_tx.payloadLen], data, len);
  g_tx.payloadLen += len;
  return true;
}
bool dc_add_tlv_u8(uint16_t t, uint8_t v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }
bool dc_add_tlv_u16(uint16_t t, uint16_t v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }
bool dc_add_tlv_u32(uint16_t t, uint32_t v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }
bool dc_add_tlv_i32(uint16_t t, int32_t v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }
bool dc_add_tlv_float(uint16_t t, float v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }
bool dc_add_tlv_double(uint16_t t, double v){ return dc_add_tlv_blob(t, &v, sizeof(v)); }

/* ===== TX with ACK/NACK ===== */
static bool write_and_wait_ack(dc_handle_t* dc, const uint8_t* frame, uint16_t len, uint16_t seq) {
  for (int attempt = 0; attempt < DC_MAX_RETRIES; ++attempt) {
    xSemaphoreTake(dc->ioMtx, portMAX_DELAY);
    dc->io->write(frame, len);
    dc->io->flush();
    xSemaphoreGive(dc->ioMtx);

    uint32_t t0 = millis();
    while ((millis() - t0) < DC_ACK_TIMEOUT_MS) {
      if (dc->io->available() >= 8) {
        int a = dc->io->peek();
        if (a == DC_PREAMBLE_A) {
          uint8_t hdr[8];
          size_t r = dc->io->readBytes(hdr, 8);
          if (r == 8 && hdr[0]==DC_PREAMBLE_A && hdr[1]==DC_PREAMBLE_B) {
            uint8_t ver=hdr[2], type=hdr[3];
            uint16_t rseq = (uint16_t)hdr[4] | ((uint16_t)hdr[5]<<8);
            uint16_t plen = (uint16_t)hdr[6] | ((uint16_t)hdr[7]<<8);
            if (ver==DC_PROTO_VERSION && (type==DC_MSG_ACK || type==DC_MSG_NACK)) {
              uint8_t tail[2 + DC_MAX_PAYLOAD_SIZE];
              size_t rr = dc->io->readBytes(tail, plen + 2);
              if (rr == (size_t)(plen+2)) {
                uint16_t crc = crc16_ccitt(&hdr[2], 6 + plen);
                uint16_t rcrc = (uint16_t)tail[plen] | ((uint16_t)tail[plen+1]<<8);
                if (crc==rcrc && rseq==seq) {
                  return (type==DC_MSG_ACK);
                }
              }
            } else {
              // No es ACK/NACK; podríamos reenfiletarlo en un buffer si quisieras
            }
          } else {
            // cabecera incompleta o no válida
          }
        } else {
          dc->io->read(); // descarta basura hasta preámbulo
        }
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
  return false;
}

bool dc_end_frame(dc_handle_t* dc) {
  // Construye cabecera y CRC; envía y espera ACK
  uint8_t* b = g_tx.buf;
  b[0] = DC_PREAMBLE_A;
  b[1] = DC_PREAMBLE_B;
  b[2] = DC_PROTO_VERSION;
  b[3] = g_tx.msgType;

  uint16_t seq = dc->txSeq++;
  wr_u16_le(&b[4], seq);
  wr_u16_le(&b[6], g_tx.payloadLen);

  uint16_t crc = crc16_ccitt(&b[2], 6 + g_tx.payloadLen);
  wr_u16_le(&b[8 + g_tx.payloadLen], crc);

  uint16_t total = 10 + g_tx.payloadLen;

  bool ok = write_and_wait_ack(dc, b, total, seq);
  return ok;
}

bool dc_send_heartbeat(dc_handle_t* dc) {
  if (!dc_begin_frame(DC_MSG_HEARTBEAT)) return false;
  return dc_end_frame(dc);
}
bool dc_send_request_full(dc_handle_t* dc) {
  if (!dc_begin_frame(DC_MSG_REQUEST_FULL)) return false;
  return dc_end_frame(dc);
}

/* ===== RX Parser State ===== */
typedef enum { ST_SYNC_A, ST_SYNC_B, ST_HDR, ST_PAYLOAD, ST_CRC } st_t;
typedef struct {
  st_t st;
  uint8_t hdr[6]; // [ver,type,seqL,seqH,lenL,lenH]
  uint16_t plen;
  uint16_t idx;
  uint8_t payload[DC_MAX_PAYLOAD_SIZE];
  uint8_t crcBytes[2];
} rxp_t;

static void send_ack(dc_handle_t* dc, uint16_t seq, uint8_t type) {
  uint8_t b[10];
  b[0]=DC_PREAMBLE_A; b[1]=DC_PREAMBLE_B;
  b[2]=DC_PROTO_VERSION; b[3]=type;
  wr_u16_le(&b[4], seq);
  wr_u16_le(&b[6], 0);
  uint16_t crc = crc16_ccitt(&b[2], 6);
  wr_u16_le(&b[8], crc);
  xSemaphoreTake(dc->ioMtx, portMAX_DELAY);
  dc->io->write(b, 10);
  dc->io->flush();
  xSemaphoreGive(dc->ioMtx);
}

static void handle_frame(dc_handle_t* dc, uint8_t ver, uint8_t type, uint16_t seq,
                         const uint8_t* payload, uint16_t plen)
{
  (void)ver; (void)seq;
  dc->lastRxMs = millis();
  if (!dc->linkUp) { dc->linkUp = true; if (dc->cbs.on_link_up) dc->cbs.on_link_up(); }

  switch (type) {
    case DC_MSG_COMMAND: {
      uint16_t i = 0;
      while (i + 4 <= plen) {
        uint16_t t = (uint16_t)payload[i] | ((uint16_t)payload[i+1]<<8);
        uint16_t l = (uint16_t)payload[i+2] | ((uint16_t)payload[i+3]<<8);
        i += 4;
        if (i + l > plen) break;
        if (dc->cbs.on_command_tlv) dc->cbs.on_command_tlv(t, &payload[i], l);
        i += l;
      }
    } break;

    case DC_MSG_TELEMETRY: {
      uint16_t i = 0;
      while (i + 4 <= plen) {
        uint16_t t = (uint16_t)payload[i] | ((uint16_t)payload[i+1]<<8);
        uint16_t l = (uint16_t)payload[i+2] | ((uint16_t)payload[i+3]<<8);
        i += 4;
        if (i + l > plen) break;
        if (dc->cbs.on_telemetry_tlv) dc->cbs.on_telemetry_tlv(t, &payload[i], l);
        i += l;
      }
    } break;

    case DC_MSG_REQUEST_FULL:
      if (dc->cbs.on_request_full_state) dc->cbs.on_request_full_state();
      break;

    default:
      // Otros tipos (ACK/NACK/HEARTBEAT) ya se gestionan en TX o son no-op aquí
      break;
  }
}

static void dc_rxTaskFn(void* arg) {
  dc_handle_t* dc = (dc_handle_t*)arg;
  rxp_t rx; memset(&rx, 0, sizeof(rx)); rx.st = ST_SYNC_A;

  for (;;) {
    while (dc->io && dc->io->available()) {
      int ch = dc->io->read();
      if (ch < 0) break;
      uint8_t c = (uint8_t)ch;

      switch (rx.st) {
        case ST_SYNC_A:
          if (c == DC_PREAMBLE_A) rx.st = ST_SYNC_B;
          break;

        case ST_SYNC_B:
          rx.st = (c == DC_PREAMBLE_B) ? ST_HDR : ST_SYNC_A;
          rx.idx = 0;
          break;

        case ST_HDR:
          rx.hdr[rx.idx++] = c;
          if (rx.idx == 6) {
            uint8_t ver = rx.hdr[0];
            uint8_t /*type = rx.hdr[1],*/ dummy = 0; (void)dummy;
            rx.plen = (uint16_t)rx.hdr[4] | ((uint16_t)rx.hdr[5]<<8);
            if (ver != DC_PROTO_VERSION || rx.plen > DC_MAX_PAYLOAD_SIZE) {
              rx.st = ST_SYNC_A; // descartar
              break;
            }
            rx.idx = 0;
            rx.st = (rx.plen ? ST_PAYLOAD : ST_CRC);
          }
          break;

        case ST_PAYLOAD:
          rx.payload[rx.idx++] = c;
          if (rx.idx == rx.plen) { rx.idx = 0; rx.st = ST_CRC; }
          break;

        case ST_CRC:
          rx.crcBytes[rx.idx++] = c;
          if (rx.idx == 2) {
            uint16_t seq = (uint16_t)rx.hdr[2] | ((uint16_t)rx.hdr[3]<<8);
            uint8_t type = rx.hdr[1];
            uint8_t ver  = rx.hdr[0];
            uint16_t rcrc = (uint16_t)rx.crcBytes[0] | ((uint16_t)rx.crcBytes[1]<<8);

            uint8_t tmp[6 + DC_MAX_PAYLOAD_SIZE];
            memcpy(tmp, rx.hdr, 6);
            memcpy(tmp+6, rx.payload, rx.plen);
            uint16_t crc = crc16_ccitt(tmp, 6 + rx.plen);

            if (crc == rcrc) {
              dc->rxLastSeq = seq;
              send_ack(dc, seq, DC_MSG_ACK);
              handle_frame(dc, ver, type, seq, rx.payload, rx.plen);
            } else {
              send_ack(dc, seq, DC_MSG_NACK);
            }
            rx.st = ST_SYNC_A;
            rx.idx = 0;
          }
          break;
      }
    }

    // Timeout de inactividad -> link down
    if (dc->linkUp && (millis() - dc->lastRxMs) > (DC_HEARTBEAT_PERIOD_MS * 3)) {
      dc->linkUp = false;
      if (dc->cbs.on_link_down) dc->cbs.on_link_down();
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void dc_hbTaskFn(void* arg) {
  dc_handle_t* dc = (dc_handle_t*)arg;
  for (;;) {
    dc_send_heartbeat(dc);
    vTaskDelay(pdMS_TO_TICKS(DC_HEARTBEAT_PERIOD_MS));
  }
}
