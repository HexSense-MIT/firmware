#include "comm.h"

uint8_t recv_cmd_buf[20] = {0};
size_t  recv_cmd_count   =  0 ;

void setup() {
  Serial.begin(921600);
  while (!Serial) { delay(10); }

  comm_init();
}

  // CommandPacket cmd = {};
  // cmd.header[0] = 0xEB;
  // cmd.header[1] = 0x90;
  // cmd.cmd = 0x01;
  // cmd.camera_index = 0x00;
  // memcpy(cmd.mac, rxMac, 6);
  // cmd.crc = 0;
  // esp_now_send(rxMac, (uint8_t*)&cmd, sizeof(cmd));

void loop() {
  update_comm(recv_cmd_buf, recv_cmd_count);

  if (recv_cmd_count > 0) {
    handle_cmd(recv_cmd_buf, recv_cmd_count);
    recv_cmd_count = 0;
  }
}


