#include "e22_lora_port.h"
#include "sx126x_hal.h"

static e22_lora_port_t e22_port;
static bool e22_port_ready;

static bool e22_lora_port_has_required_callbacks(void)
{
    return (e22_port.delay_ms != 0) &&
           (e22_port.get_tick_ms != 0) &&
           (e22_port.gpio_write != 0) &&
           (e22_port.gpio_read != 0) &&
           (e22_port.spi_write != 0) &&
           (e22_port.spi_read != 0);
}

bool e22_lora_port_bind(const e22_lora_port_t *port)
{
    if (port == 0)
    {
        e22_port_ready = false;
        return false;
    }

    e22_port = *port;
    if (e22_port.busy_timeout_ms == 0U)
    {
        e22_port.busy_timeout_ms = 1000U;
    }

    e22_port_ready = e22_lora_port_has_required_callbacks();
    return e22_port_ready;
}

bool e22_lora_port_is_ready(void)
{
    return e22_port_ready;
}

void e22_lora_port_delay_ms(uint32_t delay_ms)
{
    if ((e22_port_ready == true) && (e22_port.delay_ms != 0))
    {
        e22_port.delay_ms(e22_port.user, delay_ms);
    }
}

uint32_t e22_lora_port_get_tick_ms(void)
{
    if ((e22_port_ready == true) && (e22_port.get_tick_ms != 0))
    {
        return e22_port.get_tick_ms(e22_port.user);
    }
    return 0U;
}

void e22_lora_port_gpio_write(e22_lora_pin_t pin, bool level)
{
    if ((e22_port_ready == true) && (e22_port.gpio_write != 0))
    {
        e22_port.gpio_write(e22_port.user, pin, level);
    }
}

bool e22_lora_port_gpio_read(e22_lora_pin_t pin)
{
    if ((e22_port_ready == true) && (e22_port.gpio_read != 0))
    {
        return e22_port.gpio_read(e22_port.user, pin);
    }
    return false;
}

bool e22_lora_port_spi_write(const uint8_t *data, uint16_t length)
{
    if ((e22_port_ready == false) || (e22_port.spi_write == 0))
    {
        return false;
    }
    return e22_port.spi_write(e22_port.user, data, length);
}

bool e22_lora_port_spi_read(uint8_t *data, uint16_t length)
{
    if ((e22_port_ready == false) || (e22_port.spi_read == 0))
    {
        return false;
    }
    return e22_port.spi_read(e22_port.user, data, length);
}

void e22_lora_port_enter_critical(void)
{
    if ((e22_port_ready == true) && (e22_port.enter_critical != 0))
    {
        e22_port.enter_critical(e22_port.user);
    }
}

void e22_lora_port_exit_critical(void)
{
    if ((e22_port_ready == true) && (e22_port.exit_critical != 0))
    {
        e22_port.exit_critical(e22_port.user);
    }
}

static sx126x_hal_status_t e22_lora_wait_on_busy(void)
{
    uint32_t start_tick;

    if (e22_port_ready == false)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    start_tick = e22_lora_port_get_tick_ms();
    while (e22_lora_port_gpio_read(E22_LORA_PIN_BUSY) == true)
    {
        if ((e22_lora_port_get_tick_ms() - start_tick) > e22_port.busy_timeout_ms)
        {
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
    (void)context;

    if (e22_port_ready == false)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_RESET, false);
    e22_lora_port_delay_ms(1U);
    e22_lora_port_gpio_write(E22_LORA_PIN_RESET, true);
    e22_lora_port_delay_ms(10U);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context)
{
    (void)context;

    if (e22_port_ready == false)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, false);
    e22_lora_port_delay_ms(1U);
    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);

    return e22_lora_wait_on_busy();
}

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
                                      const uint8_t *data, const uint16_t data_length)
{
    (void)context;

    if ((e22_port_ready == false) || (e22_lora_wait_on_busy() != SX126X_HAL_STATUS_OK))
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, false);

    if ((command_length > 0U) && (e22_lora_port_spi_write(command, command_length) == false))
    {
        e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
        return SX126X_HAL_STATUS_ERROR;
    }

    if ((data_length > 0U) && (e22_lora_port_spi_write(data, data_length) == false))
    {
        e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
                                     uint8_t *data, const uint16_t data_length)
{
    (void)context;

    if ((e22_port_ready == false) || (e22_lora_wait_on_busy() != SX126X_HAL_STATUS_OK))
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, false);

    if ((command_length > 0U) && (e22_lora_port_spi_write(command, command_length) == false))
    {
        e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
        return SX126X_HAL_STATUS_ERROR;
    }

    if ((data_length > 0U) && (e22_lora_port_spi_read(data, data_length) == false))
    {
        e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_NSS, true);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_rf_switch_tx(void)
{
    if (e22_port_ready == false)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_RXEN, false);
    e22_lora_port_gpio_write(E22_LORA_PIN_TXEN, true);
    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_rf_switch_rx(void)
{
    if (e22_port_ready == false)
    {
        return SX126X_HAL_STATUS_ERROR;
    }

    e22_lora_port_gpio_write(E22_LORA_PIN_TXEN, false);
    e22_lora_port_gpio_write(E22_LORA_PIN_RXEN, true);
    return SX126X_HAL_STATUS_OK;
}
