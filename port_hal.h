#ifndef _PORT_HAL_H_
#define _PORT_HAL_H_

#include "mbed.h"
#include "usb_comm.h"

#define HAL_PWM_MAX_FREQ 50000
#define HAL_PWM_MIN_FREQ 1
#define HAL_PWM_MAX_DUTY_CYCLE 1000
#define HAL_SPI_MAX_FREQ 18000
#define HAL_SPI_MIN_FREQ 1
#define HAL_SPI_MAX_BITS 32
#define HAL_SPI_MIN_BITS 4

enum {
    PORT_CFG_OK,
    PORT_CFG_INVALID_PARAM,
    PORT_CFG_FORBIDDENT,
    PORT_CFG_NOT_INIT,
    PORT_CFG_INTER_ERR = 0xff
};

typedef enum {
    PORT_TYPE_GPIO = 0,
    PORT_TYPE_PWM,
    PORT_TYPE_ADC,
    PORT_TYPE_SERIAL,
    PORT_TYPE_I2C,
    PORT_TYPE_SPI,
    PORT_TYPE_INT,
    //PORT_TYPE_IC,
    //PORT_TYPE_CAN,
    PORT_TYPE_MAX
} port_type;

typedef enum {
    PORT_DIR_IN = 0,
    PORT_DIR_OUT,
    PORT_DIR_MAX
} port_io;

typedef enum {
    PORT_GPIOA = 0,
    PORT_GPIOB,
    PORT_GPIOC,
    PORT_GPIOD,
    PORT_GPIOE,
    PORT_GPIOF,
    PORT_GPIOG,
    PORT_GPIOH,
    PORT_MUL_FUNC,
    PORT_GPIO_MAX
} port_group;

typedef enum {
    PORT_UART_BUAD_1200 = 0,
    PORT_UART_BUAD_2400,
    PORT_UART_BUAD_4800,
    PORT_UART_BUAD_9600,
    PORT_UART_BUAD_19200,
    PORT_UART_BUAD_38400,
    PORT_UART_BUAD_57600,
    PORT_UART_BUAD_115200,
    PORT_UART_BUAD_230400,
    PORT_UART_BUAD_460800,
    PORT_UART_BUAD_921600,
    PORT_UART_BUAD_1M,
    PORT_UART_BUAD_2M,
    PORT_UART_BUAD_4M,
    PORT_UART_BUAD_MAX
} port_uart_buad_type;

typedef enum {
    PORT_UART_WORT_LEN_8 = 0,
    PORT_UART_WORT_LEN_9,
    PORT_UART_WORT_LEN_MAX
} port_uart_word_len;

typedef enum {
    PORT_UART_STOP_BIT_1 = 0,
    PORT_UART_STOP_BIT_2,
    PORT_UART_STOP_BIT_MAX
} port_uart_stop_bit;

typedef enum {
    PORT_UART_PARITY_NONE = 0,
    PORT_UART_PARITY_EVEN,
    PORT_UART_PARITY_ODD,
    PORT_UART_PARITY_MAX
} port_uart_parity;

typedef enum {
    PORT_UART_HWCTL_NONE = 0,
    PORT_UART_HWCTL_RTS,
    PORT_UART_HWCTL_CTS,
    PORT_UART_HWCTL_RTS_CTS,
    PORT_UART_HWCTL_MAX
} port_uart_hwctl;

#define UART_NUM_MAX 8

typedef struct {
    uint8_t uart_num : 3;
    uint8_t buad_rate : 4;
    uint8_t word_len : 1;

    uint8_t stop_bit : 1;
    uint8_t parity : 2;
    uint8_t hwctl : 2;
} uart_config;

typedef enum {
    PORT_GPIO_SPEED_LOW = 0,
    PORT_GPIO_SPEED_MEDIUM,
    PORT_GPIO_SPEED_HIGH,
    PORT_GPIO_SPEED_VERY_HIGH
} gpio_speed;

typedef struct {
    uint8_t speed : 2;
    uint8_t reserve : 6;
} gpio_config;

typedef struct {
    uint16_t frequency;     // 1Hz~50KHz
} pwm_config;

typedef enum {
    INT_MODE_RISE_EDGE,
    INT_MODE_FALL_EDGE,
    INT_MODE_MAX
} interrupt_mode;

typedef struct {
    uint8_t enable : 1;
    uint8_t mode : 1;
    uint8_t reserve : 6;

    uint8_t interval;
} interrupt_config;

enum {
    I2C_FREQ_SM,        /* 100K */
    I2C_FREQ_FM,        /* 400K */
    I2C_FREQ_FM_PLUS,   /* 1M */
    I2C_FREQ_HSM,       /* 3.4M */
    I2C_FREQ_MAX
};

typedef struct {
    uint8_t frequency : 2;
    uint8_t reserve : 6;
} i2c_config;

typedef struct {
    uint8_t addr;
    uint8_t repeated;
    char data[0];
} i2c_ctrl;

typedef struct {
    uint16_t frequency;     /* 1K~18000KHz */
    uint8_t bits : 6;       /* 4~32bit */
    uint8_t pol : 1;        /* polarity */
    uint8_t pha : 1;        /* phase */
    intf_gpio csb;
} spi_config;

typedef struct {
    uint8_t tx_len;
    uint8_t rx_len;
    char data[0];
} spi_ctrl;

void port_hal_init(void);
void port_hal_deinit(void);
int port_hal_gpio_config(port_group group, uint8_t pin, gpio_config *attr);
int port_hal_serial_config(port_group group, uint8_t pin, const uart_config *config);
int port_hal_pwm_config(port_group group, uint8_t pin, const pwm_config *config);
int port_hal_adc_config(port_group group, uint8_t pin);
int port_hal_int_config(port_group group, uint8_t pin, const interrupt_config *config);
int port_hal_i2c_config(port_group group, uint8_t pin, const i2c_config *config);
int port_hal_spi_config(port_group group, uint8_t pin, const spi_config *config);

int port_hal_gpio_read(port_group group, uint8_t pin, uint8_t *value);
int port_hal_gpio_write(port_group group, uint8_t pin, uint8_t value);
int port_hal_serial_out(port_group group, uint8_t pin, uint8_t *data, uint8_t len);
int port_hal_serial_in(port_group group, uint8_t pin, uint8_t *data, uint8_t *len);
int port_hal_pwm_write(port_group group, uint8_t pin, uint16_t value);
int port_hal_adc_read(port_group group, uint8_t pin, uint16_t *value);
int port_hal_i2c_read(port_group group, uint8_t pin, i2c_ctrl *ctrl, uint8_t ctrl_size);
int port_hal_i2c_write(port_group group, uint8_t pin, i2c_ctrl *ctrl, uint8_t ctrl_size);
int port_hal_spi_transfer(port_group group, uint8_t pin, uint8_t *data, uint8_t data_len);

#endif