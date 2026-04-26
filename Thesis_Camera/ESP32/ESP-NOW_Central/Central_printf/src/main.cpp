#include "comm.h"

uint8_t recv_cmd_buf[20] = {0};
size_t  recv_cmd_count   =  0 ;

void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }

  comm_init();
}

void loop() {
  update_comm(recv_cmd_buf, recv_cmd_count);

  if (recv_cmd_count > 0) {
    handle_cmd(recv_cmd_buf, recv_cmd_count);
    recv_cmd_count = 0;
  }
}


