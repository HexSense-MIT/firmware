#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "comm.h"

extern SPI_HandleTypeDef hspi2;

comm_cmd_t g_received_cmd = {0}; /* global variable to hold the received command */
comm_ack_t g_last_ack     = {0}; /* global variable to hold the ACK to be sent */
comm_data_t g_last_data   = {0}; /* global variable to hold the data packet to be sent */

volatile bool g_new_cmd_received = false; /* flag to indicate a new command has been received */

static uint16_t cobs_encode(const uint8_t* input, uint16_t length, uint8_t* output) {
  uint16_t read_index = 0;
  uint16_t write_index = 1;
  uint8_t code = 1;

  while (read_index < length) {
    if (input[read_index] == 0) {
      output[write_index - code] = code;
      code = 1;
      write_index++;
      read_index++;
    } else {
      output[write_index] = input[read_index];
      write_index++;
      read_index++;
      code++;
      if (code == 0xFF) {
        output[write_index - code] = code;
        code = 1;
        write_index++;
      }
    }
  }
  output[write_index - code] = code;
  return write_index;
}

static uint16_t cobs_decode(const uint8_t* input, uint16_t length,
                     uint8_t* output, uint16_t output_capacity) {
  uint16_t read_index = 0;
  uint16_t write_index = 0;

  while (read_index < length) {
    uint8_t code = input[read_index++];
    if (code == 0 || read_index + code - 1U > length) {
      return 0; // Invalid COBS data
    }

    for (uint8_t i = 1; i < code; i++) {
      if (write_index >= output_capacity) {
        return 0;
      }
      output[write_index] = input[read_index];
      write_index++;
      read_index++;
    }

    if (code != 0xFF && read_index < length) {
      if (write_index >= output_capacity) {
        return 0;
      }
      output[write_index] = 0;
      write_index++;
    }
  }
  return write_index;
}

static uint8_t checksum_cmd(const comm_cmd_t *cmd) {
  uint32_t sum = cmd->node_id + cmd->seq_num + cmd->cmd_id;
  return (uint8_t)(sum & 0xFFU);
}

static uint8_t checksum_ack(const comm_ack_t *ack) {
  uint32_t sum = ack->node_id + ack->seq_num + ack->ack_id;
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

void update_comm(volatile bool *irq_flag) {
  if (*irq_flag) {
    *irq_flag = false;

    uint32_t irq_status = LR2021_IRQ_NONE;
    LR2021_HandleIRQ(&hspi2, &irq_status);

    uint8_t rx_payload[LR2021_MAX_LORA_PAYLOAD];
    uint8_t rx_len = 0;

    // receiveed a command
    if (irq_status & LR2021_IRQ_RX_DONE) {
      HAL_StatusTypeDef rx_ret = LR2021_LoRa_ReadPayload(&hspi2, rx_payload, &rx_len);

      if (rx_ret == HAL_OK) {
        uint16_t decoded_len = cobs_decode(rx_payload, rx_len - 1, (uint8_t *)&g_received_cmd, sizeof(g_received_cmd));

        if (decoded_len > 0) {
          handle_cmd(&g_received_cmd);
        }
      }
    }
  }
}

void handle_cmd(comm_cmd_t *cmd) {
  /* For demo: just print the received command */
  if (cmd->node_id == LOCAL_NODE_ID) {
    HAL_Delay(200); /* simulate some processing delay */
    switch (cmd->cmd_id) {
      case COMM_CMD_ID_PING:
        send_ack(COMM_CMD_ID_PING, cmd->cmd_param[0], COMM_RADIO_LORA);
        break;
      case COMM_CMD_ID_TURN_ON_CAM:
        send_ack(COMM_CMD_ID_TURN_ON_CAM, cmd->cmd_param[0], COMM_RADIO_LORA);
        break;
      case COMM_CMD_ID_TURN_OFF_CAM:
        send_ack(COMM_CMD_ID_TURN_OFF_CAM, cmd->cmd_param[0], COMM_RADIO_LORA);
        break;
      case COMM_CMD_ID_TAKE_PHOTO:
        send_ack(COMM_CMD_ID_TAKE_PHOTO, cmd->cmd_param[0], COMM_RADIO_LORA);
        break;
      case COMM_CMD_ID_SEND_DATA:
        send_ack(COMM_CMD_ID_SEND_DATA, cmd->cmd_param[0], COMM_RADIO_LORA);
        break;
      default:
        printf("Unknown command ID: %u\r\n", (unsigned)cmd->cmd_id);
        break;
    }
    LR2021_LoRa_StartReceive(&hspi2, 0);
  }
}

void send_ack(comm_cmd_id_t ack_id, uint32_t cmd_param, comm_radio_t next_radio) {
  static uint8_t seq_num = 0;

  g_last_ack.node_id = LOCAL_NODE_ID;
  g_last_ack.seq_num = seq_num++;
  g_last_ack.ack_id = (uint8_t)ack_id;  /* ACK for the received command ID */
  // g_last_ack.cmd_param = cmd_param;  /* Echo back the command param in ACK */
  // g_last_ack.next_radio = (uint8_t)next_radio;
  g_last_ack.checksum = checksum_ack(&g_last_ack);

  LR2021_SetMode(&hspi2, LR2021_MODE_LORA);
  LR2021_LoRa_Send(&hspi2, (const uint8_t *)&g_last_ack, sizeof(g_last_ack));
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
