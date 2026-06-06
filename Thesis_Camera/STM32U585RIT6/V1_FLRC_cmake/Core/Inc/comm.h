#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "comm_cmd.h"
#include "lr2021.h"
#include "icm20948.h"

#define LOCAL_NODE_ID 0x01U

extern comm_cmd_t g_received_cmd; /* global variable to hold the received command */
extern comm_ack_t g_last_ack;     /* global variable to hold the ACK to be sent */
extern comm_data_t g_last_data;   /* global variable to hold the data packet to be sent */

extern volatile bool g_new_cmd_received; /* flag to indicate a new command has been received */

extern void update_comm(volatile bool *irq_flag); /* function to be called in main loop to process commands and send ACK/data */
extern bool comm_store_received_cmd(const uint8_t *payload, uint8_t payload_len);
extern void handle_cmd(comm_cmd_t *cmd);
extern void send_ack(comm_cmd_t *cmd, uint8_t *ack_param);
extern void send_data(uint16_t frame_seq_num, uint16_t frame_left, uint32_t bytes_left,
                      const uint8_t *payload, uint16_t payload_len);
extern void clear_irq_status(void);
