#include "port_hal.h"
#include "mbed.h"
#include "bus_common.h"
#include "usb_comm.h"

#define RESERVE_PIN 0xff
#define PIN_NAME_MAX PC_15

#define GPIO_PORT_MAX 48
#define INT_PORT_MAX 48
#define PWM_PORT_MAX 36
#define ADC_PORT_MAX 16
#define I2C_PORT_MAX 3
#define SPI_PORT_MAX 3
#define UART_PORT_MAX 3

enum {
    PORT_I2C_SCL, PORT_I2C_SDA, PORT_I2C_MAX
};

enum {
    PORT_UART_TX, PORT_UART_RX, PORT_UART_CTS, PORT_UART_RTS, PORT_UART_MAX
};

enum {
    PORT_SPI_SLK, PORT_SPI_MISO, PORT_SPI_MOSI, PORT_SPI_SEL, PORT_SPI_MAX
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

static void port_pin_reserve(port_describe *desc)
{
    port_cell *cell;

    /* for usb connect */
    cell = cell_at(desc, PA_11);
    cell->pin[0].data = RESERVE_PIN;
    cell = cell_at(desc, PA_12);
    cell->pin[0].data = RESERVE_PIN;
    /* for swd */
    cell = cell_at(desc, PA_13);
    cell->pin[0].data = RESERVE_PIN;
    cell = cell_at(desc, PA_14);
    cell->pin[0].data = RESERVE_PIN;
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

    port_pin_reserve(desc);
}

static void int_port_init(port_describe *desc)
{
    gpio_port_init(desc);
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

    port_pin_reserve(desc);
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
    cell->pin[PORT_I2C_SCL].name = PB_6;
    cell->pin[PORT_I2C_SDA].name = PB_7;

    cell = cell_at(desc, 1);
    cell->pin[PORT_I2C_SCL].name = PB_10;
    cell->pin[PORT_I2C_SDA].name = PB_3;

    cell = cell_at(desc, 2);
    cell->pin[PORT_I2C_SCL].name = PA_8;
    cell->pin[PORT_I2C_SDA].name = PC_9;
}

static void spi_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    cell = cell_at(desc, 0);
    cell->pin[PORT_SPI_SLK].name = PA_5;
    cell->pin[PORT_SPI_MISO].name = PA_6;
    cell->pin[PORT_SPI_MOSI].name = PA_7;
    cell->pin[PORT_SPI_SEL].name = PA_4;

    cell = cell_at(desc, 1);
    cell->pin[PORT_SPI_SLK].name = PB_13;
    cell->pin[PORT_SPI_MISO].name = PB_14;
    cell->pin[PORT_SPI_MOSI].name = PB_15;
    cell->pin[PORT_SPI_SEL].name = PB_12;

    cell = cell_at(desc, 2);
    cell->pin[PORT_SPI_SLK].name = PC_10;
    cell->pin[PORT_SPI_MISO].name = PC_11;
    cell->pin[PORT_SPI_MOSI].name = PC_12;
    cell->pin[PORT_SPI_SEL].name = PA_4;
}

static void uart_port_init(port_describe *desc)
{
    uint8_t i;
    port_cell *cell;

    memset((uint8_t *)desc->cell, 0, get_cell_size(desc->pin_cnt) * desc->cell_cnt);

    cell = cell_at(desc, 0);
    cell->pin[PORT_UART_TX].name = PA_9;
    cell->pin[PORT_UART_RX].name = PA_10;
    cell->pin[PORT_UART_CTS].name = RESERVE_PIN;
    cell->pin[PORT_UART_RTS].name = RESERVE_PIN;

    cell = cell_at(desc, 1);
    cell->pin[PORT_UART_TX].name = PA_2;
    cell->pin[PORT_UART_RX].name = PA_3;
    cell->pin[PORT_UART_CTS].name = PA_0;
    cell->pin[PORT_UART_RTS].name = PA_1;

    cell = cell_at(desc, 2);
    cell->pin[PORT_UART_TX].name = PC_6;
    cell->pin[PORT_UART_RX].name = PC_7;
    cell->pin[PORT_UART_CTS].data = RESERVE_PIN;
    cell->pin[PORT_UART_RTS].data = RESERVE_PIN;
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
    g_port_desc_tab[PORT_TYPE_INT].pin_cnt = pin_cnt;
    g_port_desc_tab[PORT_TYPE_INT].cell_cnt = INT_PORT_MAX;
    g_port_desc_tab[PORT_TYPE_INT].cell = (port_cell *)malloc(INT_PORT_MAX * get_cell_size(pin_cnt));

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
    int_port_init(&g_port_desc_tab[PORT_TYPE_INT]);
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

static port_cell *get_port_cell(PinName name, port_type type, uint8_t *pin_num)
{
    uint8_t i, j;

    for (i = 0; i < g_port_desc_tab[type].cell_cnt; i++) {
        port_cell *cell = cell_at(&g_port_desc_tab[type], i);
        for (j = 0; j < g_port_desc_tab[type].pin_cnt; j++) {
            if (cell->pin[j].name == name) {
                *pin_num = j;
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
        uint8_t num;
        if (i == type) {
            continue;
        }

        port_cell *cell = get_port_cell(name, (port_type)i, &num);
        if (cell == NULL || cell->enforcer == NULL || !cell->pin[num].is_used) {
            continue;
        }

        log_info("start to reset %d port enforcer\n", i);
        if (i == PORT_TYPE_GPIO) {
            DigitalInOut *digital_io = (DigitalInOut *)cell->enforcer;
            delete digital_io;
        } else if (i == PORT_TYPE_PWM) {
            PwmOut *pwm_out = (PwmOut *)cell->enforcer;
            delete pwm_out;
        } else if (i == PORT_TYPE_ADC) {
            AnalogIn *adc_in = (AnalogIn *)cell->enforcer;
            delete adc_in;
        } else if (i == PORT_TYPE_SERIAL) {
            BufferedSerial *buf_serial = (BufferedSerial *)cell->enforcer;
            delete buf_serial;
        } else if (i == PORT_TYPE_I2C) {
            I2C *i2c = (I2C *)cell->enforcer;
            delete i2c;
        } else if (i == PORT_TYPE_SPI) {
            SPI *spi = (SPI *)cell->enforcer;
            delete spi;
        } else {
            log_err("Not support type %d to reset\n", i);
        }
        cell->enforcer = NULL;

        for (j = 0; j < g_port_desc_tab[i].pin_cnt; j++) {
            cell->pin[j].is_used = 0;
        }
    }
}

/*********************************************************************************
 ************************************ GPIO ***************************************
 *********************************************************************************/
int port_hal_gpio_config(port_group group, uint8_t pin, gpio_config *attr)
{
    port_cell *cell;
    PinName name = get_pin_name(group, pin);
    uint8_t num;

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }
    
    cell = get_port_cell(name, PORT_TYPE_GPIO, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        return PORT_CFG_OK;
    }

    reset_other_cell(name, PORT_TYPE_GPIO);
    /* FIXME: 对new失败进行处理 */
    cell->enforcer = new DigitalInOut(name);
    cell->pin[0].is_used = 1;

    return PORT_CFG_OK;
}

static int gpio_io_func(PinName name, bool is_output, uint8_t *value)
{
    port_cell *cell;
    DigitalInOut *digital_io;
    uint8_t num;

    cell = get_port_cell(name, PORT_TYPE_GPIO, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer == NULL) {
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

/*********************************************************************************
 ************************************ UART ***************************************
 *********************************************************************************/

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
    uint8_t num;

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (config->buad_rate >= PORT_UART_BUAD_MAX || config->word_len >= PORT_UART_WORT_LEN_MAX ||
        config->parity >= PORT_UART_PARITY_MAX || config->stop_bit >= PORT_UART_STOP_BIT_MAX) {
        log_err("uart config invailed\n");
        return PORT_CFG_INVALID_PARAM;
    }
    
    cell = get_port_cell(name, PORT_TYPE_SERIAL, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_SERIAL);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        buf_serial = (BufferedSerial *)cell->enforcer;
    } else {
        buf_serial = new BufferedSerial((PinName)cell->pin[PORT_UART_TX].name, (PinName)cell->pin[PORT_UART_RX].name);
        cell->enforcer = buf_serial;
    }

    buf_serial->set_blocking(false);
    buf_serial->set_baud(g_baud_rate_tab[config->buad_rate]);
    buf_serial->set_format(g_word_len_tab[config->word_len], g_parity_tab[config->parity],
        g_stop_bit_tab[config->stop_bit]);
    cell->pin[PORT_UART_TX].is_used = 1;
    cell->pin[PORT_UART_RX].is_used = 1;

    return PORT_CFG_OK;
}

static int serial_io_func(PinName name, uint8_t *data, uint8_t *len, bool is_output)
{
    port_cell *cell;
    BufferedSerial *buf_serial;
    uint8_t num;

    cell = get_port_cell(name, PORT_TYPE_SERIAL, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    buf_serial = (BufferedSerial *)cell->enforcer;
    if (is_output) {
        buf_serial->write(data, *len);
    } else {
        ssize_t ret;
        ret = buf_serial->read(data, *len);
        *len = (ret == -EAGAIN ? 0 : ret);
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

/*********************************************************************************
 ************************************ PWM ****************************************
 *********************************************************************************/

int port_hal_pwm_config(port_group group, uint8_t pin, const pwm_config *config)
{
    port_cell *cell;
    PwmOut *pwm_out;
    PinName name = get_pin_name(group, pin);
    uint8_t num;

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (config == NULL || config->frequency > HAL_PWM_MAX_FREQ || config->frequency < HAL_PWM_MIN_FREQ) {
        log_err("pwm config invailed\n");
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_PWM, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_PWM);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        pwm_out = (PwmOut *)cell->enforcer;
    } else {
        pwm_out = new PwmOut(name);
        cell->enforcer = pwm_out;
    }

    log_info("freq: %d, period: %d\n", config->frequency, 1000000 / config->frequency);
    pwm_out->period_us(1000000 / config->frequency);
    cell->pin[0].is_used = 1;

    return PORT_CFG_OK;
}

int port_hal_pwm_write(port_group group, uint8_t pin, uint16_t value)
{
    port_cell *cell;
    PwmOut *pwm_out;
    uint8_t num;
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (value > HAL_PWM_MAX_DUTY_CYCLE) {
        log_err("duty cycle invalied: %d\n", value);
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_PWM, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    pwm_out = (PwmOut *)cell->enforcer;
    pwm_out->write(value / 1000.0f);

    return PORT_CFG_OK;
}

/*********************************************************************************
 ************************************ ADC ****************************************
 *********************************************************************************/
 /* TODO: 增加设置参考电压 */
#define PORT_HAL_ADC_REF 3.3f

int port_hal_adc_config(port_group group, uint8_t pin)
{
    port_cell *cell;
    AnalogIn *adc_in;
    PinName name = get_pin_name(group, pin);
    uint8_t num;

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_ADC, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_ADC);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        adc_in = (AnalogIn *)cell->enforcer;
    } else {
        adc_in = new AnalogIn(name);
        cell->enforcer = adc_in;
    }

    adc_in->set_reference_voltage(PORT_HAL_ADC_REF);
    cell->pin[0].is_used = 1;

    return PORT_CFG_OK;
}

int port_hal_adc_read(port_group group, uint8_t pin, uint16_t *value)
{
    port_cell *cell;
    AnalogIn *adc_in;
    uint8_t num;
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (value == NULL) {
        log_err("value is NULL\n", value);
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_ADC, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    adc_in = (AnalogIn *)cell->enforcer;
    *value = adc_in->read_u16();

    return PORT_CFG_OK;
}

/*********************************************************************************
 ************************************ INT ****************************************
 *********************************************************************************/
class ext_int_enforcer
{
private:
    using duration = std::chrono::duration<int, std::milli>;
    static Thread _event_thread;
    static EventQueue _queue;
    InterruptIn *_int_in;
    cmd_packet _packet;
    bool _is_rise;
    uint8_t _interval;

    void int_bottom_half(void);
public:
    ext_int_enforcer(port_type type, port_group group, uint8_t pin);
    void init(bool is_rise, uint8_t interval);
    void enable(bool enable);
    void entry(void);
};

Thread ext_int_enforcer::_event_thread;
EventQueue ext_int_enforcer::_queue(128 * EVENTS_EVENT_SIZE);

ext_int_enforcer::ext_int_enforcer(port_type type, port_group group, uint8_t pin)
{
    _packet.cmd.bit.dir = PORT_DIR_IN;
    _packet.cmd.bit.mode = INTF_CMD_MODE_CTRL;
    _packet.cmd.bit.type = type;
    _packet.gpio.bit.group = group;
    _packet.gpio.bit.pin = pin;
    _packet.data_len = 1;
    _packet.data[0] = 0;    // reserve

    if (_event_thread.get_state() == Thread::Deleted) {
        _event_thread.start(callback(&_queue, &EventQueue::dispatch_forever));
    }
}

void ext_int_enforcer::init(bool is_rise, uint8_t interval)
{
    PinName name = get_pin_name((port_group)_packet.gpio.bit.group, _packet.gpio.bit.pin);
    _int_in = new InterruptIn(name);
    _is_rise = is_rise;
    _interval = interval;

    log_info("is_rise: %d, interval: %d\n", _is_rise, _interval);
    if (is_rise) {
        _int_in->rise(callback(this, &ext_int_enforcer::entry));
        _int_in->mode(PullDown);
    } else {
        _int_in->fall(callback(this, &ext_int_enforcer::entry));
        _int_in->mode(PullUp);
    }
}

void ext_int_enforcer::enable(bool enable)
{
    if (enable) {
        _int_in->enable_irq();
    } else {
        _int_in->disable_irq();
    }
}

void ext_int_enforcer::entry(void)
{
    if (_interval != 0) {
        _queue.call_in(duration(_interval), this, &ext_int_enforcer::int_bottom_half);
    } else {
        _queue.call(this, &ext_int_enforcer::int_bottom_half);
    }
}

void ext_int_enforcer::int_bottom_half(void)
{
    bool confirm = true;

    if (_interval != 0) {
        int expect = _is_rise ? 1 : 0;
        confirm = (_int_in->read() == expect ? true : false);
    }

    if (confirm) {
        (void)usb_msg_queue_block_put(&_packet);
    }
}

int port_hal_int_config(port_group group, uint8_t pin, const interrupt_config *config)
{
    port_cell *cell;
    ext_int_enforcer *int_enforcer;
    PinName name = get_pin_name(group, pin);
    uint8_t num;

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_INT, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_INT);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        int_enforcer = (ext_int_enforcer *)cell->enforcer;
    } else {
        int_enforcer = new ext_int_enforcer(PORT_TYPE_INT, group, pin);
        int_enforcer->init(config->mode == INT_MODE_RISE_EDGE, config->interval);
        cell->enforcer = int_enforcer;
    }
    
    int_enforcer->enable(config->enable == 1);
    cell->pin[0].is_used = 1;

    return PORT_CFG_OK;
}

/*********************************************************************************
 ************************************ I2C ****************************************
 *********************************************************************************/
static const uint32_t i2c_freq_tab[I2C_FREQ_MAX] = {100000, 400000, 1000000, 3400000};

int port_hal_i2c_config(port_group group, uint8_t pin, const i2c_config *config)
{
    port_cell *cell;
    I2C *i2c;
    PinName name = get_pin_name(group, pin);
    uint8_t num;

    if (config == NULL || config->frequency > I2C_FREQ_HSM) {
        log_err("i2c config invalid\n");
        return PORT_CFG_INVALID_PARAM;
    }

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    cell = get_port_cell(name, PORT_TYPE_I2C, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    reset_other_cell(name, PORT_TYPE_I2C);
    if (cell->enforcer != NULL) {
        log_info("The pin has been inited: %d\n", name);
        i2c = (I2C *)cell->enforcer;
    } else {
        i2c = new I2C((PinName)cell->pin[PORT_I2C_SDA].name, (PinName)cell->pin[PORT_I2C_SCL].name);
        cell->enforcer = i2c;
    }

    i2c->frequency(i2c_freq_tab[config->frequency]);
    cell->pin[PORT_I2C_SCL].is_used = 1;
    cell->pin[PORT_I2C_SDA].is_used = 1;

    return PORT_CFG_OK;
}

static int i2c_io_func(PinName name, bool is_write, i2c_ctrl *ctrl, uint8_t data_len)
{
    port_cell *cell;
    I2C *i2c;
    uint8_t num;
    int ret, i;

    cell = get_port_cell(name, PORT_TYPE_I2C, &num);
    if (cell == NULL) {
        log_err("NOT support PinName: %d\n", name);
        return PORT_CFG_INVALID_PARAM;
    }

    if (cell->enforcer == NULL) {
        log_err("PinName: %d NOT inited\n", name);
        return PORT_CFG_NOT_INIT;
    }

    i2c = (I2C *)cell->enforcer;
    if (is_write) {
        ret = i2c->write(ctrl->addr, ctrl->data, data_len, ctrl->repeated);
    } else {
        ret = i2c->read(ctrl->addr, ctrl->data, data_len, ctrl->repeated);
    }

    if (ret != 0) {
        log_err("i2c ctrl(%d) failed\n", is_write);
    }
    
    return PORT_CFG_OK;
}

int port_hal_i2c_read(port_group group, uint8_t pin, i2c_ctrl *ctrl, uint8_t ctrl_size)
{
    uint8_t data_len = ctrl_size - sizeof(i2c_ctrl);
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (ctrl == NULL || data_len == 0) {
        log_err("i2c control invalid\n");
        return PORT_CFG_INVALID_PARAM;
    }

    return i2c_io_func(name, false, ctrl, data_len);
}

int port_hal_i2c_write(port_group group, uint8_t pin, i2c_ctrl *ctrl, uint8_t ctrl_size)
{
    uint8_t data_len = ctrl_size - sizeof(i2c_ctrl);
    PinName name = get_pin_name(group, pin);

    if (name > PIN_NAME_MAX) {
        log_err("NOT support group %d, pin %d\n", group, pin);
        return PORT_CFG_INVALID_PARAM;
    }

    if (ctrl == NULL || data_len == 0) {
        log_err("i2c control invalid\n");
        return PORT_CFG_INVALID_PARAM;
    }

    return i2c_io_func(name, true, ctrl, data_len);
}
