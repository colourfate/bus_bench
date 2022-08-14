#include "port_hal.h"
#include "mbed.h"
#include "common.h"
#include "usb_comm.h"

#define RESERVE_PIN 0xff
#define PIN_NAME_MAX PC_15

#define GPIO_PORT_MAX 48
#define PWM_PORT_MAX 36
#define ADC_PORT_MAX 16
#define I2C_PORT_MAX 3
#define SPI_PORT_MAX 3
#define UART_PORT_MAX 3

enum {
    I2C_SCL, I2C_SDA, I2C_MAX
};

enum {
    UART_TX, UART_RX, UART_CTS, UART_RTS, UART_MAX
};

enum {
    SPI_SLK, SPI_MISO, SPI_MOSI, SPI_SEL, SPI_MAX
};

typedef union {
    struct {
        uint8_t is_used : 1;
        uint8_t name : 7;
    };
    uint8_t data;
} pin_describe;

typedef struct {
    void *enforcer;
    pin_describe pin[0];
} port_cell;

typedef struct {
    uint8_t pin_cnt;
    uint8_t cell_cnt;
    port_cell *cell;
} port_describe;

static port_describe g_port_desc_tab[PORT_TYPE_MAX];

static inline uint8_t get_cell_size(uint8_t pin_cnt)
{
    return sizeof(port_cell) + pin_cnt * sizeof(pin_describe);
}

static inline port_cell *cell_at(port_describe *port_desc, uint8_t index)
{
    return (port_cell *)((uint8_t *)port_desc->cell + index * get_cell_size(port_desc->pin_cnt));
}

static void gpio_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    for (i = PA_0; i < PC_15; i++) {
        cell = cell_at(desc, i);
        cell->pin[0].name = i;
    }

    /* for usb connect */
    cell = cell_at(desc, PA_11);
    cell->pin[0].data = RESERVE_PIN;
    cell = cell_at(desc, PA_12);
    cell->pin[0].data = RESERVE_PIN;
}

static void pwm_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    for (i = PA_0; i < PB_15; i++) {
        cell = cell_at(desc, i);
        cell->pin[0].name = i;
    }
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_6;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_7;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_8;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_9;

    cell = cell_at(desc, PA_11);
    cell->pin[0].data = RESERVE_PIN;
    cell = cell_at(desc, PA_12);
    cell->pin[0].data = RESERVE_PIN;
}

static void adc_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    for (i = PA_0; i < PA_7; i++) {
        cell = cell_at(desc, i);
        cell->pin[0].name = i;
    }
    cell = cell_at(desc, i++);
    cell->pin[0].name = PB_0;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PB_1;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_0;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_1;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_2;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_3;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_4;
    cell = cell_at(desc, i++);
    cell->pin[0].name = PC_5;
}

static void i2c_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    cell = cell_at(desc, 0);
    cell->pin[I2C_SCL].name = PB_6;
    cell->pin[I2C_SDA].name = PB_7;

    cell = cell_at(desc, 1);
    cell->pin[I2C_SCL].name = PB_10;
    cell->pin[I2C_SDA].name = PB_3;

    cell = cell_at(desc, 2);
    cell->pin[I2C_SCL].name = PA_8;
    cell->pin[I2C_SDA].name = PC_9;
}

static void spi_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    cell = cell_at(desc, 0);
    cell->pin[SPI_SLK].name = PA_5;
    cell->pin[SPI_MISO].name = PA_6;
    cell->pin[SPI_MOSI].name = PA_7;
    cell->pin[SPI_SEL].name = PA_4;

    cell = cell_at(desc, 1);
    cell->pin[SPI_SLK].name = PB_13;
    cell->pin[SPI_MISO].name = PB_14;
    cell->pin[SPI_MOSI].name = PB_15;
    cell->pin[SPI_SEL].name = PB_12;

    cell = cell_at(desc, 2);
    cell->pin[SPI_SLK].name = PC_10;
    cell->pin[SPI_MISO].name = PC_11;
    cell->pin[SPI_MOSI].name = PC_12;
    cell->pin[SPI_SEL].name = PA_4;
}

static void uart_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    cell = cell_at(desc, 0);
    cell->pin[UART_TX].name = PA_9;
    cell->pin[UART_RX].name = PA_10;
    cell->pin[UART_CTS].name = PA_11;
    cell->pin[UART_RTS].name = PA_12;

    cell = cell_at(desc, 1);
    cell->pin[UART_TX].name = PA_2;
    cell->pin[UART_RX].name = PA_3;
    cell->pin[UART_CTS].name = PA_0;
    cell->pin[UART_RTS].name = PA_1;

    cell = cell_at(desc, 2);
    cell->pin[UART_TX].name = PC_6;
    cell->pin[UART_RX].name = PC_7;
    cell->pin[UART_CTS].data = RESERVE_PIN;
    cell->pin[UART_RTS].data = RESERVE_PIN;
}

void port_hal_init(void)
{
    uint8_t pin_cnt;

    pin_cnt = 1;
    g_port_desc_tab[PORT_TYPE_GPIO].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_GPIO].cell_cnt = GPIO_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_GPIO].cell = (port_cell *)malloc(GPIO_PORT_MAX * get_cell_size(pin_cnt));
    g_port_desc_tab[PORT_TYPE_PWM].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_PWM].cell_cnt = PWM_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_PWM].cell = (port_cell *)malloc(PWM_PORT_MAX * get_cell_size(pin_cnt));
    g_port_desc_tab[PORT_TYPE_ADC].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_ADC].cell_cnt = ADC_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_ADC].cell = (port_cell *)malloc(ADC_PORT_MAX * get_cell_size(pin_cnt));

    pin_cnt = 2;
    g_port_desc_tab[PORT_TYPE_I2C].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_I2C].cell_cnt = I2C_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_I2C].cell = (port_cell *)malloc(I2C_PORT_MAX * get_cell_size(pin_cnt));

    pin_cnt = 4;
    g_port_desc_tab[PORT_TYPE_SPI].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_SPI].cell_cnt = SPI_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_SPI].cell = (port_cell *)malloc(SPI_PORT_MAX * get_cell_size(pin_cnt));
    g_port_desc_tab[PORT_TYPE_SERIAL].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_SERIAL].cell_cnt = UART_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_SERIAL].cell = (port_cell *)malloc(UART_PORT_MAX * get_cell_size(pin_cnt));

    gpio_port_init(&g_port_desc_tab[PORT_TYPE_GPIO]);
    pwm_port_init(&g_port_desc_tab[PORT_TYPE_PWM]);
    adc_port_init(&g_port_desc_tab[PORT_TYPE_ADC]);
    i2c_port_init(&g_port_desc_tab[PORT_TYPE_I2C]);
    spi_port_init(&g_port_desc_tab[PORT_TYPE_SPI]);
    uart_port_init(&g_port_desc_tab[PORT_TYPE_SERIAL]);
}

void port_hal_deinit(void)
{
    return;
}

static inline PinName get_pin_name(port_group group, uint8_t pin)
{
    return (PinName)(group * 16 + pin);
}

static port_cell *get_port_cell(PinName name, port_type type)
{
    uint8_t i, j;

    for (i = 0; i < g_port_desc_tab[type].cell_cnt; i++) {
        port_cell *cell = cell_at(&g_port_desc_tab[type], i);
        for (j = 0; j < g_port_desc_tab[type].pin_cnt; j++) {
            if (cell->pin[j].name == name) {
                return cell;
            }
        }
    }

    return NULL;
}

static void reset_other_cell(PinName name, port_type type)
{
    uint8_t i, j;

    for (i = 0; i < count_of(g_port_desc_tab); i++) {
        if (i == type) {
            continue;
        }

        port_cell *cell = get_port_cell(name, type);
        if (cell == NULL || cell->enforcer == NULL) {
            continue;
        }

        log_info("start to reset %d port enforcer\n", type);
        if (type == PORT_TYPE_GPIO) {
            DigitalInOut *digital_io = (DigitalInOut *)cell->enforcer;
            delete digital_io;
        } else if (type == PORT_TYPE_PWM) {
            PwmOut *pwm_out = (PwmOut *)cell->enforcer;
            delete pwm_out;
        } else if (type == PORT_TYPE_ADC) {
            AnalogIn *adc_in = (AnalogIn *)cell->enforcer;
            delete adc_in;
        } else if (type == PORT_TYPE_SERIAL) {
            BufferedSerial *buf_serial = (BufferedSerial *)cell->enforcer;
            delete buf_serial;
        } else if (type == PORT_TYPE_I2C) {
            I2C *i2c = (I2C *)cell->enforcer;
            delete i2c;
        } else if (type == PORT_TYPE_SPI) {
            SPI *spi = (SPI *)cell->enforcer;
            delete spi;
        } else {
            log_err("Not support type %d to reset\n", type);
        }
        cell->enforcer = NULL;

        for (j = 0; j < g_port_desc_tab[type].pin_cnt; j++) {
            cell->pin[j].is_used = 0;
        }
    }
}

int port_hal_gpio_config(port_group group, uint8_t pin, gpio_config *attr)
{
    port_cell *cell;
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }
    
    cell = get_port_cell(name, PORT_TYPE_GPIO);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->pin[0].is_used) {
        log_info("The pin has been inited: %d\n", name);
        return PORT_CFG_OK;
    }

    reset_other_cell(name, PORT_TYPE_GPIO);
    cell->enforcer = new DigitalInOut(name);
    cell->pin[0].is_used = 1;

    return PORT_CFG_OK;
}

static int gpio_io_func(PinName name, bool is_output, uint8_t *value)
{
    port_cell *cell;
    DigitalInOut *digital_io;

    cell = get_port_cell(name, PORT_TYPE_GPIO);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (!cell->pin[0].is_used || cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    digital_io = (DigitalInOut *)cell->enforcer;
    if (digital_io->is_connected() == 0) {
        log_err("DigitalInOut %d NOT be connected\n", name);
        return PORT_CFG_INTER_ERR;
    }

    if (is_output) {
        digital_io->output();
        digital_io->write(*value);
    } else {
        digital_io->input();
        *value = digital_io->read();
    }
    
    return PORT_CFG_OK;
}

int port_hal_gpio_write(port_group group, uint8_t pin, uint8_t value)
{
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    return gpio_io_func(name, true, &value);
}

int port_hal_gpio_read(port_group group, uint8_t pin, uint8_t *value)
{
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (value == NULL) {
        log_err("invalid param\n");
        return PORT_CFG_INVALID_PARAM;
    }

    return gpio_io_func(name, false, value);
}

static const uint32_t g_baud_rate_tab[PORT_UART_BUAD_MAX] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200,
    230400, 460800, 921600, 1000000, 2000000, 4000000 };
static const uint32_t g_word_len_tab[PORT_UART_WORT_LEN_MAX] = {8, 9};
static const uint32_t g_stop_bit_tab[PORT_UART_STOP_BIT_MAX] = {1, 2};
static const BufferedSerial::Parity g_parity_tab[PORT_UART_PARITY_MAX] = 
    { BufferedSerial::None, BufferedSerial::Even, BufferedSerial::Odd };

int port_hal_serial_config(port_group group, uint8_t pin, const uart_config *config)
{
    port_cell *cell;
    BufferedSerial *buf_serial;
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }
    
    cell = get_port_cell(name, PORT_TYPE_SERIAL);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_SERIAL);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        buf_serial = (BufferedSerial *)cell->enforcer;
    } else {
        buf_serial = new BufferedSerial((PinName)cell->pin[UART_TX].name, (PinName)cell->pin[UART_RX].name);
        cell->enforcer = buf_serial;
    }

    buf_serial->set_baud(g_baud_rate_tab[config->buad_rate]);
    buf_serial->set_format(g_word_len_tab[config->word_len], g_parity_tab[config->parity],
        g_stop_bit_tab[config->stop_bit]);
    cell->pin[UART_TX].is_used = 1;
    cell->pin[UART_RX].is_used = 1;

    return PORT_CFG_OK;
}

static int serial_io_func(PinName name, uint8_t *data, uint8_t *len, bool is_output)
{
    port_cell *cell;
    BufferedSerial *buf_serial;

    cell = get_port_cell(name, PORT_TYPE_SERIAL);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (!cell->pin[0].is_used || cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    buf_serial = (BufferedSerial *)cell->enforcer;
    if (is_output) {
        buf_serial->write(data, *len);
    } else {
        *len = buf_serial->read(data, *len);
    }

    return PORT_CFG_OK;
}

int port_hal_serial_out(port_group group, uint8_t pin, uint8_t *data, uint8_t len)
{
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (data == NULL) {
        log_err("invalid param\n");
        return PORT_CFG_INVALID_PARAM;
    }

    return serial_io_func(name, data, &len, true);
}

int port_hal_serial_in(port_group group, uint8_t pin, uint8_t *data, uint8_t *len)
{
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (data == NULL || len == NULL) {
        log_err("invalid param\n");
        return PORT_CFG_INVALID_PARAM;
    }

    return serial_io_func(name, data, len, false);
}

int port_hal_pwm_config(port_group group, uint8_t pin, const pwm_config *config)
{
    return 0;
}

int port_hal_pwm_write(port_group group, uint8_t pin, uint16_t value)
{
    return 0;
}
