#include "comm.h"

extern SPI_HandleTypeDef hspi2;
extern I2C_HandleTypeDef hi2c1;

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
  uint32_t sum = 0;
  sum += cmd->node_id;
  sum += cmd->seq_num;
  sum += cmd->cmd_id;
  for (size_t i = 0; i < sizeof(cmd->cmd_param); i++) {
    sum += cmd->cmd_param[i];
  }
  return (uint8_t)(sum & 0xFFU);
}

static uint8_t checksum_ack(const comm_ack_t *ack) {
  uint32_t sum = 0;
  sum += ack->node_id;
  sum += ack->seq_num;
  sum += ack->ack_id;
  for (size_t i = 0; i < sizeof(ack->ack_param); i++) {
    sum += ack->ack_param[i];
  }

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

void handle_ping(uint8_t *ack_param) {
  ICM20948_Data_t sensor_data = {0};

  ICM20948_PowrOn();
  ICM20948_Init(&hi2c1);
  HAL_Delay(50);
  ICM20948_ReadAll(&hi2c1, &sensor_data);
  ICM20948_PowrOff();

  float gravity = sqrtf(sensor_data.accel_x * sensor_data.accel_x +
                        sensor_data.accel_y * sensor_data.accel_y +
                        sensor_data.accel_z * sensor_data.accel_z);

  float temperature = sensor_data.temp_c;

  memcpy(ack_param, (uint8_t *)&gravity, sizeof(float));
  memcpy(ack_param + sizeof(float), (uint8_t *)&temperature, sizeof(float));
}

void handle_cam_on_off(uint8_t *cam_param, uint8_t *ack_param, size_t n) {
  memcpy(ack_param, cam_param, n);
}

void handle_take_photo(uint8_t *cam_param, uint8_t *ack_param, size_t n) {
  memcpy(ack_param, cam_param, n);
}

void handle_cmd(comm_cmd_t *cmd) {
  uint8_t ack_param[11] = {0};

  if (cmd->node_id == LOCAL_NODE_ID) {
    HAL_Delay(50); /* simulate some processing delay */
    switch (cmd->cmd_id) {
      case COMM_CMD_ID_PING:
        handle_ping(ack_param);
        send_ack(cmd, ack_param);
        break;

      case COMM_CMD_ID_TURN_ON_CAM:
        handle_cam_on_off(cmd->cmd_param, ack_param, sizeof(cmd->cmd_param));
        send_ack(cmd, ack_param);
        break;

      case COMM_CMD_ID_TURN_OFF_CAM:
        handle_cam_on_off(cmd->cmd_param, ack_param, sizeof(cmd->cmd_param));
        send_ack(cmd, ack_param);
        break;

      case COMM_CMD_ID_TAKE_PHOTO:
        handle_take_photo(cmd->cmd_param, ack_param, sizeof(cmd->cmd_param));
        send_ack(cmd, ack_param);
        break;

      case COMM_CMD_ID_SEND_DATA:
        // send_data(cmd, ack_param);
        break;

      default:
        break;
    }
  }
  LR2021_LoRa_StartReceive(&hspi2, 0);
}

void send_ack(comm_cmd_t *cmd, uint8_t *ack_param) {
  g_last_ack.node_id   = LOCAL_NODE_ID;
  g_last_ack.seq_num   = cmd->seq_num;  /* ACK the same sequence number as the received command */
  g_last_ack.ack_id    = cmd->cmd_id;   /* ACK the same command ID as the received command */
  memcpy(g_last_ack.ack_param, ack_param, sizeof(g_last_ack.ack_param));
  g_last_ack.checksum  = checksum_ack(&g_last_ack);

  uint8_t  encoded_ack[sizeof(g_last_ack) + 2] = {0};
  uint16_t encoded_len = cobs_encode((const uint8_t *)&g_last_ack, sizeof(g_last_ack), encoded_ack);

  LR2021_LoRa_Send(&hspi2, encoded_ack, encoded_len);

  LR2021_SetMode(&hspi2, LR2021_MODE_LORA);
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
