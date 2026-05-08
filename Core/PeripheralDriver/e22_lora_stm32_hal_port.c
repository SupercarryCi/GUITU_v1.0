#include "e22_lora_stm32_hal_port.h"
#include "e22_lora_port.h"

static e22_lora_stm32_hal_config_t stm32_cfg;
static uint32_t saved_primask;

static e22_lora_stm32_gpio_t get_gpio(e22_lora_pin_t pin)
{
    switch (pin)
    {
    case E22_LORA_PIN_NSS:
        return stm32_cfg.nss;
    case E22_LORA_PIN_RESET:
        return stm32_cfg.reset;
    case E22_LORA_PIN_BUSY:
        return stm32_cfg.busy;
    case E22_LORA_PIN_TXEN:
        return stm32_cfg.txen;
    case E22_LORA_PIN_RXEN:
        return stm32_cfg.rxen;
    case E22_LORA_PIN_DIO1:
        return stm32_cfg.dio1;
    default:
        break;
    }

    return stm32_cfg.nss;
}

static void stm32_delay_ms(void *user, uint32_t delay_ms)
{
    (void)user;
    HAL_Delay(delay_ms);
}

static uint32_t stm32_get_tick_ms(void *user)
{
    (void)user;
    return HAL_GetTick();
}

static void stm32_gpio_write(void *user, e22_lora_pin_t pin, bool level)
{
    e22_lora_stm32_gpio_t gpio;

    (void)user;
    gpio = get_gpio(pin);
    HAL_GPIO_WritePin(gpio.port, gpio.pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool stm32_gpio_read(void *user, e22_lora_pin_t pin)
{
    e22_lora_stm32_gpio_t gpio;

    (void)user;
    gpio = get_gpio(pin);
    return (HAL_GPIO_ReadPin(gpio.port, gpio.pin) == GPIO_PIN_SET);
}

static bool stm32_spi_write(void *user, const uint8_t *data, uint16_t length)
{
    (void)user;

    if ((data == 0) || (length == 0U))
    {
        return true;
    }

    return (HAL_SPI_Transmit(stm32_cfg.hspi, (uint8_t *)data, length, stm32_cfg.spi_timeout_ms) == HAL_OK);
}

static bool stm32_spi_read(void *user, uint8_t *data, uint16_t length)
{
    (void)user;

    if ((data == 0) || (length == 0U))
    {
        return true;
    }

    return (HAL_SPI_Receive(stm32_cfg.hspi, data, length, stm32_cfg.spi_timeout_ms) == HAL_OK);
}

static void stm32_enter_critical(void *user)
{
    (void)user;
    saved_primask = __get_PRIMASK();
    __disable_irq();
}

static void stm32_exit_critical(void *user)
{
    (void)user;
    if (saved_primask == 0U)
    {
        __enable_irq();
    }
}

bool e22_lora_stm32_hal_bind(const e22_lora_stm32_hal_config_t *config)
{
    e22_lora_port_t port;

    if ((config == 0) || (config->hspi == 0))
    {
        return false;
    }

    stm32_cfg = *config;
    if (stm32_cfg.spi_timeout_ms == 0U)
    {
        stm32_cfg.spi_timeout_ms = 1000U;
    }
    if (stm32_cfg.busy_timeout_ms == 0U)
    {
        stm32_cfg.busy_timeout_ms = 1000U;
    }

    HAL_GPIO_WritePin(stm32_cfg.nss.port, stm32_cfg.nss.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(stm32_cfg.reset.port, stm32_cfg.reset.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(stm32_cfg.txen.port, stm32_cfg.txen.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(stm32_cfg.rxen.port, stm32_cfg.rxen.pin, GPIO_PIN_RESET);

    port.user = 0;
    port.delay_ms = stm32_delay_ms;
    port.get_tick_ms = stm32_get_tick_ms;
    port.gpio_write = stm32_gpio_write;
    port.gpio_read = stm32_gpio_read;
    port.spi_write = stm32_spi_write;
    port.spi_read = stm32_spi_read;
    port.enter_critical = stm32_enter_critical;
    port.exit_critical = stm32_exit_critical;
    port.busy_timeout_ms = stm32_cfg.busy_timeout_ms;

    return e22_lora_port_bind(&port);
}
