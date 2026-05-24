#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "comm.h"

extern SPI_HandleTypeDef hspi2;

comm_cmd_t g_received_cmd = {0}; /* global variable to hold the received command */
comm_ack_t g_last_ack     = {0}; /* global variable to hold the ACK to be sent */
comm_data_t g_last_data   = {0}; /* global variable to hold the data packet to be sent */

volatile bool g_new_cmd_received = false; /* flag to indicate a new command has been received */

static void read_received_cmd(void) {
  uint8_t buf[sizeof(comm_cmd_t)];
  uint8_t rx_len = 0;
  if (LR2021_LoRa_Receive(&hspi2, buf, &rx_len, 0) == HAL_OK &&
      rx_len == sizeof(comm_cmd_t)) {
    memcpy(&g_received_cmd, buf, sizeof(comm_cmd_t));
  }
}

void update_comm(void) {
  if (g_new_cmd_received) {
    g_new_cmd_received = false;
    read_received_cmd();

    // only handle commands addressed to this node (LOCAL_NODE_ID)
    if (g_received_cmd.node_id == LOCAL_NODE_ID) {
      handle_cmd(&g_received_cmd);
      send_ack(g_received_cmd.cmd_id, g_received_cmd.cmd_param, g_received_cmd.next_radio);
      send_data(1, 0, 100, (uint8_t *)"Hello, World!", 13);
    }
  }
}

void handle_cmd(comm_cmd_t *cmd) {
  /* For demo: just print the received command */
  printf("Received CMD: node=%u seq=%u id=%u param=%lu next_radio=%u\r\n",
         (unsigned)cmd->node_id, (unsigned)cmd->seq_num, (unsigned)cmd->cmd_id,
         (unsigned long)cmd->cmd_param, (unsigned)cmd->next_radio);
}

void send_ack(comm_cmd_id_t ack_id, uint32_t cmd_param, comm_radio_t next_radio) {
  static uint8_t seq_num = 0;

  g_last_ack.node_id = LOCAL_NODE_ID;
  g_last_ack.seq_num = seq_num++;
  g_last_ack.ack_id = ack_id;  /* ACK for the received command ID */
  g_last_ack.cmd_param = cmd_param;  /* Echo back the command param in ACK */
  g_last_ack.next_radio = next_radio;
  g_last_ack.checksum = (g_last_ack.node_id + g_last_ack.seq_num + g_last_ack.ack_id +
                  (g_last_ack.cmd_param & 0xFF) + ((g_last_ack.cmd_param >> 8) & 0xFF) +
                  ((g_last_ack.cmd_param >> 16) & 0xFF) + ((g_last_ack.cmd_param >> 24) & 0xFF) +
                  g_last_ack.next_radio) % 256;

  /* For demo: print the ACK instead of sending over SPI */
  printf("ACK: node=%u seq=%u id=%u param=%lu next_radio=%u\r\n",
         (unsigned)g_last_ack.node_id, (unsigned)g_last_ack.seq_num, (unsigned)g_last_ack.ack_id,
         (unsigned long)g_last_ack.cmd_param, (unsigned)g_last_ack.next_radio);
}

void send_data(uint16_t frame_seq_num, uint16_t frame_left, uint32_t bytes_left,
               const uint8_t *payload, uint16_t payload_len) {
  comm_data_t data;
  data.node_id = LOCAL_NODE_ID;
  data.frame_seq_num = frame_seq_num;
  data.frame_left = frame_left;
  data.bytes_left = bytes_left;
  if (payload_len > sizeof(data.payload)) payload_len = sizeof(data.payload);
  memcpy(data.payload, payload, payload_len);

  /* Simple checksum: sum of all bytes modulo 256 */
  uint32_t sum = data.node_id + (data.frame_seq_num >> 8) + (data.frame_seq_num & 0xFF) +
                 (data.frame_left >> 8) + (data.frame_left & 0xFF) +
                 ((data.bytes_left >> 24) & 0xFF) + ((data.bytes_left >> 16) & 0xFF) +
                 ((data.bytes_left >> 8) & 0xFF) + (data.bytes_left & 0xFF);
  for (uint16_t i = 0; i < payload_len; i++) sum += data.payload[i];
  data.checksum = sum % 256;

  /* For demo: print the data packet instead of sending over SPI */
  printf("DATA: node=%u seq=%u frame_left=%u bytes_left=%lu payload_len=%u checksum=%u\r\n",
         (unsigned)data.node_id, (unsigned)data.frame_seq_num, (unsigned)data.frame_left,
         (unsigned long)data.bytes_left, (unsigned)payload_len, (unsigned)data.checksum);
}