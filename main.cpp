#include "mbed.h"
#include "usb_msg_queue.h"
#include "port_hal.h"

uint8_t g_usb_msg[INTF_PROTOCOL_PACKET_MAX];

int main()
{
    cmd_packet *packet;
    int ret;

    usb_msg_queue_init();
    usb_comm_init();
    log_info("usb init ok\n");

    packet = (cmd_packet *)g_usb_msg;
    packet->data_len = USB_MSG_DATA_LEN_MAX;
    while (true) {
        ret = usb_msg_queue_block_get(packet);
        if (ret != USB_MSG_OK) {
            log_err("usb_msg_queue_get failed, exit\n");
            break;
        }

        log_info("get packet: [cmd: %d, dir: %d, mode: %d, group: %d, pin: %d, len: %d, value: %d]\n",
                packet->cmd.bit.type, packet->cmd.bit.dir, packet->cmd.bit.mode,
                packet->gpio.bit.group, packet->gpio.bit.pin,
                packet->data_len, packet->data[0]);

        ret = msg_parse_exec(packet);
        if (ret != USB_MSG_OK) {
            log_err("msg_parse_exec failed\n");
        }

        if (packet->cmd.bit.mode == INTF_CMD_MODE_CFG || packet->cmd.bit.dir == PORT_DIR_IN) {
            int i;
            log_info("put packet: [cmd: %d, dir: %d, mode: %d, group: %d, pin: %d, len: %d, value: %d]\n",
                packet->cmd.bit.type, packet->cmd.bit.dir, packet->cmd.bit.mode,
                packet->gpio.bit.group, packet->gpio.bit.pin,
                packet->data_len, packet->data[0]);

            for (i = 0; i < packet->data_len; i++) {
                log_info_raw("%c(%d) ", packet->data[i], i);
            }

            ret = usb_msg_queue_put(packet);
            if (ret != USB_MSG_OK) {
                log_err("usb_msg_queue_put failed\n");
            }
        }
    }
}