#ifndef _USB_MSG_QUEUE_H_
#define _USB_MSG_QUEUE_H_

#include "usb_comm.h"

#define CHIP_PIN_MAX 16

int usb_msg_queue_init(void);
int usb_msg_queue_deinit(void);
int usb_msg_queue_block_get(cmd_packet *packet);
int usb_msg_queue_block_put(const cmd_packet *packet);

int msg_parse_exec(cmd_packet *packet);

#endif