/*
 * 人身上 ESP32-UWB = DS-TWR Initiator
 * 發 POLL → 收 RESP → 發 FINAL（距離由車上 Responder 計算）
 * USB 除錯：115200
 */
#include "dw3000.h"

const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 4;

#define TX_ANT_DLY 16400
#define RX_ANT_DLY 16400

#define POLL_TX_TO_RESP_RX_DLY_UUS  0
#define RESP_RX_TIMEOUT_UUS         15000
#define RESP_RX_TO_FINAL_TX_DLY_UUS 10000
#define RNG_DELAY_MS                80

static uint8_t tx_poll_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8_t rx_resp_msg[]  = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0x22, 0, 0};
static uint8_t tx_final_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0, 0};

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

void msg_set_ts(uint8_t *ts_field, uint64_t ts) {
  for (int i = 0; i < 4; i++) {
    ts_field[i] = (uint8_t)(ts & 0xFF);
    ts >>= 8;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("--- 身上 Tag = DS-TWR Initiator ---"));

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
  Serial.println(F("身上就緒，開始 POLL"));
}

void loop() {
  uint64_t poll_tx_ts, resp_rx_ts, final_tx_ts;
  uint32_t final_tx_time;

  dwt_forcetrxoff();
  dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
  dwt_setrxtimeout(0);

  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
  dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
  dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  uint32_t guard = millis();
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
           (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
    if (millis() - guard > 25) break;
  }

  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer)) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0) {
        poll_tx_ts = get_tx_timestamp_u64();
        resp_rx_ts = get_rx_timestamp_u64();

        final_tx_time = (uint32_t)((resp_rx_ts + ((uint64_t)RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
        final_tx_time &= 0xFFFFFFFEUL;
        dwt_setdelayedtrxtime(final_tx_time);
        final_tx_ts = (((uint64_t)final_tx_time) << 8) + TX_ANT_DLY;

        msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
        msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
        msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
        tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
        dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);
        dwt_starttx(DWT_START_TX_DELAYED);
      }
    }
  }

  frame_seq_nb++;
  delay(RNG_DELAY_MS);
}
