#ifndef E22_LORA_PORT_H
#define E22_LORA_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    E22_LORA_PIN_NSS = 0,
    E22_LORA_PIN_RESET,
    E22_LORA_PIN_BUSY,
    E22_LORA_PIN_TXEN,
    E22_LORA_PIN_RXEN,
    E22_LORA_PIN_DIO1,
} e22_lora_pin_t;

typedef struct
{
    void *user;
    void (*delay_ms)(void *user, uint32_t delay_ms);
    uint32_t (*get_tick_ms)(void *user);
    void (*gpio_write)(void *user, e22_lora_pin_t pin, bool level);
    bool (*gpio_read)(void *user, e22_lora_pin_t pin);
    bool (*spi_write)(void *user, const uint8_t *data, uint16_t length);
    bool (*spi_read)(void *user, uint8_t *data, uint16_t length);
    void (*enter_critical)(void *user);
    void (*exit_critical)(void *user);
    uint32_t busy_timeout_ms;
} e22_lora_port_t;

bool e22_lora_port_bind(const e22_lora_port_t *port);
bool e22_lora_port_is_ready(void);
void e22_lora_port_delay_ms(uint32_t delay_ms);
uint32_t e22_lora_port_get_tick_ms(void);
void e22_lora_port_gpio_write(e22_lora_pin_t pin, bool level);
bool e22_lora_port_gpio_read(e22_lora_pin_t pin);
bool e22_lora_port_spi_write(const uint8_t *data, uint16_t length);
bool e22_lora_port_spi_read(uint8_t *data, uint16_t length);
void e22_lora_port_enter_critical(void);
void e22_lora_port_exit_critical(void);

#ifdef __cplusplus
}
#endif

#endif /* E22_LORA_PORT_H */
