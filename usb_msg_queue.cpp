#include "usb_msg_queue.h"
#include "port_hal.h"
#include "common.h"
#include "usb_comm.h"

int usb_msg_queue_init(void)
{
    port_hal_init();

    return USB_MSG_OK;
}

int usb_msg_queue_deinit(void)
{
    port_hal_deinit();

    return USB_MSG_OK;
}

static int gpio_read_exec_func(cmd_packet *packet)
{
    int ret;
    uint8_t value;

    log_info("\n");
    if (packet->data_len != 1) {
        log_err("Invalid param len: %d\n", packet->data_len);
        return USB_MSG_FAILED;
    }

    ret = port_hal_gpio_read((port_group)packet->gpio.bit.group, packet->gpio.bit.pin, &value);
    if (ret != osOK) {
        log_err("port_hal_gpio_read failed\n");
        return USB_MSG_FAILED;
    }

    packet->data[0] = value;

    return USB_MSG_OK;
}

static int gpio_write_exec_func(cmd_packet *packet)
{
    int ret;

    log_info("\n");
    if (packet->data[0] != 0 && packet->data[0] != 1) {
        log_err("Invalid param data: %d\n", packet->data[0]);
        return USB_MSG_FAILED;
    }

    ret = port_hal_gpio_write((port_group)packet->gpio.bit.group, packet->gpio.bit.pin, packet->data[0]);
    if (ret != osOK) {
        log_err("port_hal_gpio_read failed\n");
        return USB_MSG_FAILED;
    }

    return USB_MSG_OK;
}

static int serial_in_exec_func(cmd_packet *packet)
{
    log_info("\n");
    return port_hal_serial_in((port_group)packet->gpio.bit.group, packet->gpio.bit.pin,
        packet->data, &packet->data_len);
}

static int serial_out_exec_func(cmd_packet *packet)
{
    log_info("\n");
    return port_hal_serial_out((port_group)packet->gpio.bit.group, packet->gpio.bit.pin,
        packet->data, packet->data_len);
}

static int pwm_write_exec_func(cmd_packet *packet)
{
    int ret;
    uint16_t value;
    log_info("\n");

    if (packet->data_len != 2) {
        log_err("Invalid param len: %d\n", packet->data_len);
        return USB_MSG_FAILED;
    }

    value = packet->data[0] | packet->data[1] << 8;
    if (value > HAL_PWM_MAX_PULSE) {
        log_err("Invalid param data: %d\n", packet->data[0]);
        return USB_MSG_FAILED;
    }

    ret = port_hal_pwm_write((port_group)packet->gpio.bit.group, packet->gpio.bit.pin, value);
    if (ret != osOK) {
        log_err("port_hal_gpio_read failed\n");
        return USB_MSG_FAILED;
    }

    return USB_MSG_OK;
}

static int gpio_cfg_exec_func(cmd_packet *packet)
{
    int ret;

    log_info("\n");

    ret = port_hal_gpio_config((port_group)packet->gpio.bit.group, packet->gpio.bit.pin, (gpio_config *)packet->data);
    packet->data_len = 1;
    packet->data[0] = ret;

    return ret;
}

static int serial_cfg_exec_func(cmd_packet *packet)
{
    int ret;
    log_info("\n");

    if (packet->data_len != sizeof(uart_config)) {
        log_err("invalid data len: %d\n", packet->data_len);
        return osError;
    }

    ret = port_hal_serial_config((port_group)packet->gpio.bit.group, packet->gpio.bit.pin,
        (const uart_config *)packet->data);
    packet->data_len = 1;
    packet->data[0] = ret;
    return ret;
}

static int pwm_cfg_exec_func(cmd_packet *packet)
{
    int ret;
    log_info("\n");

    if (packet->data_len != sizeof(pwm_config)) {
        log_err("invalid data len: %d\n", packet->data_len);
        return osError;
    }

    ret = port_hal_pwm_config((port_group)packet->gpio.bit.group, packet->gpio.bit.pin, (const pwm_config *)packet->data);
    packet->data_len = 1;
    packet->data[0] = ret;
    return ret;
}

typedef struct {
    port_type cmd_type;
    intf_cmd_mode cmd_mode;
    port_io cmd_dir;
    int (*entry)(cmd_packet *packet);
} cmd_exec_unit;

static const cmd_exec_unit g_cmd_exec_tab[] = {
    /* control function */
    { PORT_TYPE_GPIO, INTF_CMD_MODE_CTRL, PORT_DIR_IN, gpio_read_exec_func },
    { PORT_TYPE_GPIO, INTF_CMD_MODE_CTRL, PORT_DIR_OUT, gpio_write_exec_func },
    { PORT_TYPE_SERIAL, INTF_CMD_MODE_CTRL, PORT_DIR_IN, serial_in_exec_func },
    { PORT_TYPE_SERIAL, INTF_CMD_MODE_CTRL, PORT_DIR_OUT, serial_out_exec_func },
    { PORT_TYPE_PWM, INTF_CMD_MODE_CTRL, PORT_DIR_OUT, pwm_write_exec_func },
    /* config function */
    { PORT_TYPE_GPIO, INTF_CMD_MODE_CFG, PORT_DIR_MAX, gpio_cfg_exec_func },
    { PORT_TYPE_SERIAL, INTF_CMD_MODE_CFG, PORT_DIR_MAX, serial_cfg_exec_func },
    { PORT_TYPE_PWM, INTF_CMD_MODE_CFG, PORT_DIR_OUT, pwm_cfg_exec_func }
};

int msg_parse_exec(cmd_packet *packet)
{
    int ret = USB_MSG_FAILED;
    uint8_t i;

    if (packet == NULL) {
        log_err("packet is NULL\n");
        return USB_MSG_FAILED;
    }

    if (packet->data_len > USB_MSG_DATA_LEN_MAX) {
        log_err("packet is too large: %d\n", packet->data_len);
        return USB_MSG_FAILED;
    }

    for (i = 0; i < count_of(g_cmd_exec_tab); i++) {
        // TODO: 这个判断可能需要改
        if (g_cmd_exec_tab[i].cmd_type == packet->cmd.bit.type && g_cmd_exec_tab[i].cmd_mode == packet->cmd.bit.mode &&
            (g_cmd_exec_tab[i].cmd_dir == PORT_DIR_MAX || g_cmd_exec_tab[i].cmd_dir == packet->cmd.bit.dir)) {
            ret = g_cmd_exec_tab[i].entry(packet);
            break;
        }
    }

    if (ret != USB_MSG_OK) {
        log_err("Cmd exec failed, type: %d, mode: %d, dir: %d\n", packet->cmd.bit.type, packet->cmd.bit.mode,
            packet->cmd.bit.dir);
        return USB_MSG_FAILED;
    }

    return USB_MSG_OK;
}
