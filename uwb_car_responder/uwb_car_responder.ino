/*
 * 車上 ESP32-UWB = DS-TWR Responder
 * 收 POLL / 回 RESP / 收 FINAL → 算距離
 * Serial2 TX GPIO17 → UNO D10（9600）
 * USB 除錯：115200
 */
#include "dw3000.h"
#include <algorithm>

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

#define TX_ANT_DLY 16400
#define RX_ANT_DLY 16400

#define POLL_RX_TO_RESP_TX_DLY_UUS  10000
#define RESP_TX_TO_FINAL_RX_DLY_UUS 0
#define FINAL_RX_TIMEOUT_UUS        20000

static uint8_t rx_poll_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t tx_resp_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x22, 0, 0};
static uint8_t rx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0, 0};

#define ALL_MSG_COMMON_LEN 10
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

static uint8_t frame_seq_nb = 0;
static uint8_t rx_buffer[32];
static uint32_t status_reg = 0;

static dwt_config_t config = {
  5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8,
  DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
  DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

#define WINDOW_SIZE 5
float dist_buffer[WINDOW_SIZE] = {0};
uint8_t buf_idx = 0;
bool buf_full = false;

bool apply_smart_filter(float raw_dist, float *out_dist) {
  if (raw_dist <= 0.05f || raw_dist > 40.0f) return false;
  dist_buffer[buf_idx] = raw_dist;
  buf_idx = (buf_idx + 1) % WINDOW_SIZE;
  if (buf_idx == 0) buf_full = true;
  uint8_t count = buf_full ? WINDOW_SIZE : buf_idx;
  float temp[WINDOW_SIZE];
  for (uint8_t i = 0; i < count; i++) temp[i] = dist_buffer[i];
  std::sort(temp, temp + count);
  *out_dist = temp[count / 2];
  return true;
}

void msg_get_ts(const uint8_t *ts_field, uint32_t *ts) {
  *ts = 0;
  for (int i = 0; i < 4; i++) {
    *ts |= ((uint32_t)ts_field[i]) << (i * 8);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);
  delay(1000);
  Serial.println(F("--- 車上 Anchor = DS-TWR Responder ---"));

  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
  delay(2);
  while (!dwt_checkidlerc()) delay(10);

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println(F("DW3000 初始化失敗"));
    while (1) {}
  }

  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  dwt_configure(&config);
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  Serial.println(F("車上就緒，等待身上 POLL"));
}

void loop() {
  uint64_t poll_rx_ts, resp_tx_ts, final_rx_ts;
  uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
  uint32_t resp_tx_time;

  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
  dwt_setrxtimeout(0);
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  uint32_t guard = millis();
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
    if (millis() - guard > 50) return;
  }

  if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) return;

  dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
  uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frame_len > sizeof(rx_buffer)) return;

  dwt_readrxdata(rx_buffer, frame_len, 0);
  rx_buffer[ALL_MSG_SN_IDX] = 0;
  if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) != 0) return;

  poll_rx_ts = get_rx_timestamp_u64();
  resp_tx_time = (uint32_t)((poll_rx_ts + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
  resp_tx_time &= 0xFFFFFFFEUL;
  dwt_setdelayedtrxtime(resp_tx_time);
  resp_tx_ts = (((uint64_t)resp_tx_time) << 8) + TX_ANT_DLY;

  tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
  dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);
  dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
  dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

  if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    frame_seq_nb++;
    return;
  }

  guard = millis();
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    if (millis() - guard > 30) break;
  }

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0) {
        final_rx_ts = get_rx_timestamp_u64();
        msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
        msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
        msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

        uint32_t poll_rx_ts_32  = (uint32_t)poll_rx_ts;
        uint32_t resp_tx_ts_32  = (uint32_t)resp_tx_ts;
        uint32_t final_rx_ts_32 = (uint32_t)final_rx_ts;

        int32_t Ra = (int32_t)(resp_rx_ts - poll_tx_ts);
        int32_t Rb = (int32_t)(final_rx_ts_32 - resp_tx_ts_32);
        int32_t Da = (int32_t)(final_tx_ts - resp_rx_ts);
        int32_t Db = (int32_t)(resp_tx_ts_32 - poll_rx_ts_32);

        double denom = (double)Ra + (double)Rb + (double)Da + (double)Db;
        if (denom != 0) {
          double tof = ((double)Ra * (double)Rb - (double)Da * (double)Db) / denom;
          float raw = (float)(tof * DWT_TIME_UNITS * SPEED_OF_LIGHT);
          float distance;
          if (apply_smart_filter(raw, &distance)) {
            Serial.printf("[CAR] %.2f m\n", distance);
            Serial2.println(distance, 2);
          }
        }
      }
    }
  }

  frame_seq_nb++;
}
