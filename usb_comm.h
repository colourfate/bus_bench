#ifndef _USB_COMM_H_
#define _USB_COMM_H_

#include "mbed.h"

#define INTF_PROTOCOL_PACKET_MAX 64
#define USB_MSG_HEAD_LEN (sizeof(cmd_packet) - 1)
#define USB_MSG_DATA_LEN_MAX (INTF_PROTOCOL_PACKET_MAX - USB_MSG_HEAD_LEN)

typedef enum {
    INTF_CMD_MODE_CTRL = 0,
    INTF_CMD_MODE_CFG,
    INTF_CMD_MODE_INFO,
    INTF_CMD_MODE_MAX
} intf_cmd_mode;

typedef union {
    struct {
        uint8_t dir : 1;
        uint8_t mode : 2;
        uint8_t type : 5;
    } bit;

    uint8_t data;
} intf_cmd;

typedef union {
    struct {
        uint8_t pin : 4;
        uint8_t group : 4;
    } bit;

    uint8_t data;
} intf_gpio;

typedef struct {
    intf_cmd cmd;
    intf_gpio gpio;
    uint8_t data_len;
    uint8_t data[1];
} cmd_packet;

enum {
    USB_MSG_OK = 0,
    USB_MSG_RETRY,
    USB_MSG_REPEAT_OPT,
    USB_MSG_INVAILD_PARAM,
    USB_MSG_PROTOCOL_ERROR,
    USB_MSG_INTER_ERROR,
    USB_MSG_FAILED = -1
};

void usb_comm_init(void);
int usb_msg_queue_block_get(cmd_packet *packet);
int usb_msg_queue_block_put(const cmd_packet *packet);
void usb_printf(const char *format, ...);

#define DEBUG
#ifdef DEBUG
#define log_info(fmt, args...) usb_printf("%s[%d]: " fmt, __func__, __LINE__, ##args)
#define log_info_raw(fmt, args...) usb_printf(fmt, ##args);
#else
#define log_info(fmt, args...)
#define log_info_raw(fmt, args...)
#endif
#define log_err(fmt, args...) usb_printf("%s[%d]: " fmt, __func__, __LINE__, ##args)

#endif