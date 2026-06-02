#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "comm.h"

extern SPI_HandleTypeDef hspi2;

comm_cmd_t g_received_cmd = {0}; /* global variable to hold the received command */
comm_ack_t g_last_ack     = {0}; /* global variable to hold the ACK to be sent */
comm_data_t g_last_data   = {0}; /* global variable to hold the data packet to be sent */

volatile bool g_new_cmd_received = false; /* flag to indicate a new command has been received */

static uint8_t checksum_cmd(const comm_cmd_t *cmd) {
  uint32_t sum = cmd->node_id + cmd->seq_num + cmd->cmd_id +
                 (cmd->cmd_param & 0xFFU) + ((cmd->cmd_param >> 8) & 0xFFU) +
                 ((cmd->cmd_param >> 16) & 0xFFU) + ((cmd->cmd_param >> 24) & 0xFFU) +
                 cmd->next_radio;
  return (uint8_t)(sum & 0xFFU);
}

static uint8_t checksum_ack(const comm_ack_t *ack) {
  uint32_t sum = ack->node_id + ack->seq_num + ack->ack_id +
                 (ack->cmd_param & 0xFFU) + ((ack->cmd_param >> 8) & 0xFFU) +
                 ((ack->cmd_param >> 16) & 0xFFU) + ((ack->cmd_param >> 24) & 0xFFU) +
                 ack->next_radio;
  return (uint8_t)(sum & 0xFFU);
}

bool comm_store_received_cmd(const uint8_t *payload, uint8_t payload_len) {
  comm_cmd_t cmd;

  if (!payload || payload_len != sizeof(cmd)) {
    return false;
  }

  memcpy(&cmd, payload, sizeof(cmd));
  if (cmd.checksum != checksum_cmd(&cmd)) {
    return false;
  }

  g_received_cmd = cmd;
  g_new_cmd_received = true;
  return true;
}

void update_comm(bool irq_flag) {
  bool restart_rx = false;

  if (irq_flag) {
    uint32_t irq = LR2021_IRQ_NONE;
    HAL_StatusTypeDef ret = LR2021_HandleIRQ(&hspi2, &irq);
    if (ret != HAL_OK) {
      printf("[LR2021 IRQ] read failed: %u\r\n", (unsigned)ret);
      return;
    }

    if (irq == LR2021_IRQ_NONE) {
      return;
    }

    printf("[LR2021 IRQ] status=0x%08lX\r\n", (unsigned long)irq);

    if (irq & LR2021_IRQ_RX_DONE) {
      uint8_t payload[LR2021_MAX_LORA_PAYLOAD];
      uint8_t payload_len = 0;

      ret = LR2021_LoRa_ReadPayload(&hspi2, payload, &payload_len);
      if (ret != HAL_OK) {
        printf("[LoRa RX] payload read failed: %u\r\n", (unsigned)ret);
      } else if (!comm_store_received_cmd(payload, payload_len)) {
        printf("[LoRa RX] ignored invalid command: %u bytes\r\n", (unsigned)payload_len);
      }
    }

    if (irq & (LR2021_IRQ_RX_DONE | LR2021_IRQ_TIMEOUT |
               LR2021_IRQ_CRC_ERROR | LR2021_IRQ_LEN_ERROR)) {
      restart_rx = true;
    }
  }

  if (g_new_cmd_received) {
    g_new_cmd_received = false;

    // only handle commands addressed to this node (LOCAL_NODE_ID)
    if (g_received_cmd.node_id == LOCAL_NODE_ID) {
      handle_cmd(&g_received_cmd);
      send_ack((comm_cmd_id_t)g_received_cmd.cmd_id, g_received_cmd.cmd_param,
               (comm_radio_t)g_received_cmd.next_radio);
      send_data(1, 0, 100, (uint8_t *)"Hello, World!", 13);
    }

    LR2021_SetMode(&hspi2, LR2021_MODE_LORA);
    LR2021_LoRa_StartReceive(&hspi2, 0);
    restart_rx = false;
  }

  if (restart_rx) {
    LR2021_LoRa_StartReceive(&hspi2, 0);
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
  g_last_ack.ack_id = (uint8_t)ack_id;  /* ACK for the received command ID */
  g_last_ack.cmd_param = cmd_param;  /* Echo back the command param in ACK */
  g_last_ack.next_radio = (uint8_t)next_radio;
  g_last_ack.checksum = checksum_ack(&g_last_ack);

  LR2021_SetMode(&hspi2, LR2021_MODE_LORA);
  HAL_StatusTypeDef tx_status =
      LR2021_LoRa_Send(&hspi2, (const uint8_t *)&g_last_ack, sizeof(g_last_ack));

  printf("ACK: node=%u seq=%u id=%u param=%lu next_radio=%u tx=%s\r\n",
         (unsigned)g_last_ack.node_id, (unsigned)g_last_ack.seq_num, (unsigned)g_last_ack.ack_id,
         (unsigned long)g_last_ack.cmd_param, (unsigned)g_last_ack.next_radio,
         tx_status == HAL_OK ? "OK" : "ERR");
}

void send_data(uint16_t frame_seq_num, uint16_t frame_left, uint32_t bytes_left,
               const uint8_t *payload, uint16_t payload_len) {
  if (!payload && payload_len > 0) return;

  memset(&g_last_data, 0, sizeof(g_last_data));
  g_last_data.node_id = LOCAL_NODE_ID;
  g_last_data.frame_seq_num = frame_seq_num;
  g_last_data.frame_left = frame_left;
  g_last_data.bytes_left = bytes_left;
  if (payload_len > sizeof(g_last_data.payload)) payload_len = sizeof(g_last_data.payload);
  memcpy(g_last_data.payload, payload, payload_len);

  /* Simple checksum: sum of all bytes modulo 256 */
  uint32_t sum = g_last_data.node_id + (g_last_data.frame_seq_num >> 8) + (g_last_data.frame_seq_num & 0xFF) +
                 (g_last_data.frame_left >> 8) + (g_last_data.frame_left & 0xFF) +
                 ((g_last_data.bytes_left >> 24) & 0xFF) + ((g_last_data.bytes_left >> 16) & 0xFF) +
                 ((g_last_data.bytes_left >> 8) & 0xFF) + (g_last_data.bytes_left & 0xFF);
  for (uint16_t i = 0; i < payload_len; i++) sum += g_last_data.payload[i];
  g_last_data.checksum = sum % 256;

  /* For demo: print the data packet instead of sending over SPI */
  printf("DATA: node=%u seq=%u frame_left=%u bytes_left=%lu payload_len=%u checksum=%u\r\n",
         (unsigned)g_last_data.node_id, (unsigned)g_last_data.frame_seq_num, (unsigned)g_last_data.frame_left,
         (unsigned long)g_last_data.bytes_left, (unsigned)payload_len, (unsigned)g_last_data.checksum);
}

void clear_irq_status(void) {
  clear_IRQ(&hspi2);
}
