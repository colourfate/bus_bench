#include "usb_comm.h"
#include "USBSerial.h"

#define MAX_INFO_LEN 128

/* defualt vendor_id=0x1f00, product_id=0x2012 */
USBSerial g_usb_serial;

void usb_comm_init(void)
{
    g_usb_serial.set_blocking(true);
}

int usb_msg_queue_block_get(cmd_packet *packet)
{
    uint8_t *p = (uint8_t *)packet;
    uint8_t len;

    if (packet == NULL) {
        log_err("packet is NULL\n");
        return USB_MSG_INVAILD_PARAM;
    }

    len = g_usb_serial.read(p, USB_MSG_HEAD_LEN);
    if (len != USB_MSG_HEAD_LEN) {
        log_err("read msg head(%d) error, expect(%d)\n", len, sizeof(cmd_packet));
        return USB_MSG_INTER_ERROR;
    }

    if (packet->data_len > USB_MSG_DATA_LEN_MAX || packet->data_len == 0) {
        log_err("msg data len is invalid: %d\n", packet->data_len);
        return USB_MSG_PROTOCOL_ERROR;
    }

    p = &packet->data[0];
    len = g_usb_serial.read(p, packet->data_len);
    if (len != packet->data_len) {
        log_err("read msg msg(%d) error, expect(%d)\n", len, packet->data_len);
        return USB_MSG_INTER_ERROR;
    }

    return USB_MSG_OK;
}

int usb_msg_queue_put(const cmd_packet *packet)
{
    uint8_t data_len;
    
    if (packet == NULL) {
        log_err("packet is NULL\n");
        return USB_MSG_INVAILD_PARAM;
    }

    if (packet->data_len > USB_MSG_DATA_LEN_MAX || packet->data_len == 0) {
        log_err("msg data len is invalid: %d\n", packet->data_len);
        return USB_MSG_INTER_ERROR;
    }
    
    data_len = USB_MSG_HEAD_LEN + packet->data_len;
    if (!g_usb_serial.send((uint8_t *)packet, data_len)) {
        log_err("serial send failed\n");
        return USB_MSG_INTER_ERROR;
    }

    return USB_MSG_OK;
}

uint8_t g_raw_data[MAX_INFO_LEN];

void usb_printf(const char *format, ...)
{
    cmd_packet *packet;
    va_list aptr;
    int len;
    uint32_t send_len;

    memset(g_raw_data, 0 ,sizeof(g_raw_data));
    packet = (cmd_packet *)g_raw_data;
    va_start(aptr, format);
    len = vsprintf((char *)packet->data, format, aptr);
    va_end(aptr);

    if (len > MAX_INFO_LEN - sizeof(cmd_packet) - 1) {
        len = MAX_INFO_LEN - sizeof(cmd_packet) - 1;
    }
    
    if (packet->data[len - 1] == '\n') {
        packet->data[len] = '\r';
        len++;
    }
    
    packet->cmd.bit.mode = INTF_CMD_MODE_INFO;
    packet->gpio.data = 0;
    packet->data_len = len;

    while (!g_usb_serial.send(g_raw_data, len + USB_MSG_HEAD_LEN));
}
