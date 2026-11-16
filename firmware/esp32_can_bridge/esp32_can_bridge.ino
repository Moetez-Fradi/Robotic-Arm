#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN   GPIO_NUM_5
#define CAN_RX_PIN   GPIO_NUM_4
#define STATUS_LED   2
#define SERIAL_BAUD  921600

// 15-byte USB packet: [0xAA, 0x55, ID_H, ID_L, DLC, D0..D7, CSUM, 0x0A]
uint8_t rx_buf[15];
uint8_t rx_idx = 0;
uint32_t last_led_ms = 0;
bool led_state = false;

uint8_t calc_csum(const uint8_t *buf) {
  uint8_t cs = 0;
  for (int i = 2; i <= 12; i++) cs ^= buf[i];
  return cs;
}

void startTwai() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 64;
  g_config.tx_queue_len = 64;
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }
}

void setup() {
  pinMode(STATUS_LED, OUTPUT);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  startTwai();
}

void loop() {
  // 1. Monitor CAN Controller Health & Auto-Recover from Bus-Off
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    if (status.state == TWAI_STATE_BUS_OFF) {
      twai_initiate_recovery();
    } else if (status.state == TWAI_STATE_STOPPED) {
      twai_start();
    }

    // LED Indicator: fast blink (100ms) on CAN fault, slow heartbeat (500ms) when healthy
    uint32_t blink_period = (status.state == TWAI_STATE_RUNNING) ? 500 : 100;
    if (millis() - last_led_ms >= blink_period) {
      last_led_ms = millis();
      led_state = !led_state;
      digitalWrite(STATUS_LED, led_state);
    }
  }

  // 2. Host PC (USB Serial) -> CAN Bus
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (rx_idx == 0 && b != 0xAA) continue;
    if (rx_idx == 1 && b != 0x55) {
      rx_idx = (b == 0xAA) ? 1 : 0;
      continue;
    }

    rx_buf[rx_idx++] = b;
    if (rx_idx == 15) {
      rx_idx = 0;
      if (rx_buf[14] == 0x0A && calc_csum(rx_buf) == rx_buf[13]) {
        twai_message_t tx_msg = {};
        tx_msg.identifier = ((uint16_t)rx_buf[2] << 8) | rx_buf[3];
        tx_msg.data_length_code = min((uint8_t)8, rx_buf[4]);
        memcpy(tx_msg.data, &rx_buf[5], 8);
        twai_transmit(&tx_msg, pdMS_TO_TICKS(1));
      }
    }
  }

  // 3. CAN Bus -> Host PC (USB Serial)
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (!rx_msg.extd && !rx_msg.rtr) {
      uint8_t tx_pkt[15] = {0};
      tx_pkt[0] = 0xAA;
      tx_pkt[1] = 0x55;
      tx_pkt[2] = (rx_msg.identifier >> 8) & 0xFF;
      tx_pkt[3] = rx_msg.identifier & 0xFF;
      tx_pkt[4] = rx_msg.data_length_code;
      memcpy(&tx_pkt[5], rx_msg.data, rx_msg.data_length_code);
      tx_pkt[13] = calc_csum(tx_pkt);
      tx_pkt[14] = 0x0A;
      Serial.write(tx_pkt, 15);
    }
  }
}