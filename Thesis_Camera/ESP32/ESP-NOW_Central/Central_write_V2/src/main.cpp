#include "comm.h"

uint8_t recv_cmd_buf[20] = {0};
size_t  recv_cmd_count   =  0 ;

uint8_t LED_TX_PIN = 43;
uint8_t LED_RX_PIN = 44;

void setup() {
  Serial.begin(921600);

  while (!Serial) {
    delay(1); yield();
  }

  pinMode(LED_TX_PIN, OUTPUT);

  digitalWrite(LED_TX_PIN, LOW);

  comm_init();
}

void loop() {
  update_comm(recv_cmd_buf, recv_cmd_count);

  if (recv_cmd_count > 0) {
    digitalWrite(LED_TX_PIN, LOW);
    handle_cmd(recv_cmd_buf, recv_cmd_count);
    recv_cmd_count = 0;
  }

  if (received_ack_flag) {
      uint8_t local_ack[250];
      int local_len;

      noInterrupts();
      local_len = ack_length;
      memcpy(local_ack, ack_buffer, local_len);
      received_ack_flag = false;
      interrupts();

      Serial.write(local_ack, local_len);
      Serial.flush();
      digitalWrite(LED_TX_PIN, HIGH);
  }
}


