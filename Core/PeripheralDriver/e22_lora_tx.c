#include "e22_lora_tx.h"
#include "e22_lora_port.h"
#include "sx126x_hal.h"
#include <stddef.h>

typedef struct
{
    bool is_tx;
} e22_lora_context_t;

static e22_lora_context_t e22_context;
static e22_lora_tx_config_t e22_config;
static e22_lora_tx_diag_t e22_diag;
static sx126x_pkt_params_lora_t pkt_cfg;
static uint32_t tx_start_tick;

void e22_lora_tx_get_default_config(e22_lora_tx_config_t *config)
{
    if (config == 0)
    {
        return;
    }

    config->rf_freq_hz = 915000000UL;
    config->cal_img_low_mhz = 850U;
    config->cal_img_high_mhz = 930U;
    config->sync_word = 0x14U;
    config->tx_power_dbm = 0;
    config->ramp_time = SX126X_RAMP_40_US;
    config->pa_cfg.pa_duty_cycle = 0x04;
    config->pa_cfg.hp_max = 0x07;
    config->pa_cfg.device_sel = 0x00;
    config->pa_cfg.pa_lut = 0x01;
    config->sf = SX126X_LORA_SF7;
    config->bw = SX126X_LORA_BW_500;
    config->cr = SX126X_LORA_CR_4_5;
    config->ldro = false;
    config->preamble_len = 8U;
    config->crc_on = true;
    config->invert_iq = false;
    config->tcxo_voltage = SX126X_TCXO_CTRL_2_2V;
    config->tcxo_startup_time = 320U;
    config->tx_timeout_ms = 3000U;
}

void e22_lora_tx_rf_switch_close(void)
{
    e22_lora_port_gpio_write(E22_LORA_PIN_TXEN, false);
    e22_lora_port_gpio_write(E22_LORA_PIN_RXEN, false);
}

static bool e22_lora_spi_check(void)
{
    uint8_t temp = 0xA5U;

    sx126x_write_register(&e22_context, 0x06BB, &temp, 1U);
    temp = 0U;
    sx126x_read_register(&e22_context, 0x06BB, &temp, 1U);

    return (temp == 0xA5U);
}

static bool e22_lora_xosc_check(void)
{
    sx126x_errors_mask_t error_info = 0;

    sx126x_clear_device_errors(&e22_context);
    sx126x_set_standby(&e22_context, SX126X_STANDBY_CFG_XOSC);
    sx126x_get_device_errors(&e22_context, &error_info);

    return ((error_info & SX126X_ERRORS_XOSC_START) == 0U);
}

bool e22_lora_tx_init(const e22_lora_tx_config_t *config)
{
    sx126x_mod_params_lora_t mod_cfg;

    if (e22_lora_port_is_ready() == false)
    {
        return false;
    }

    if (config == 0)
    {
        e22_lora_tx_get_default_config(&e22_config);
    }
    else
    {
        e22_config = *config;
        if (e22_config.tx_timeout_ms == 0U)
        {
            e22_config.tx_timeout_ms = 3000U;
        }
    }

    e22_context.is_tx = false;
    e22_diag.tx_start_count = 0U;
    e22_diag.tx_done_count = 0U;
    e22_diag.tx_timeout_count = 0U;
    e22_diag.last_irq = 0U;
    e22_diag.last_payload_length = 0U;
    e22_diag.is_busy = false;

    sx126x_reset(&e22_context);
    sx126x_wakeup(&e22_context);
    sx126x_set_standby(&e22_context, SX126X_STANDBY_CFG_RC);

    if (e22_lora_spi_check() == false)
    {
        return false;
    }

    sx126x_init_retention_list(&e22_context);
    sx126x_set_reg_mode(&e22_context, SX126X_REG_MODE_DCDC);
    sx126x_set_dio2_as_rf_sw_ctrl(&e22_context, false);
    sx126x_set_dio3_as_tcxo_ctrl(&e22_context, e22_config.tcxo_voltage, e22_config.tcxo_startup_time);
    sx126x_cal(&e22_context, SX126X_CAL_ALL);

    if (e22_lora_xosc_check() == false)
    {
        return false;
    }

    sx126x_cal_img_in_mhz(&e22_context, e22_config.cal_img_low_mhz, e22_config.cal_img_high_mhz);
    sx126x_set_pkt_type(&e22_context, SX126X_PKT_TYPE_LORA);
    sx126x_set_rf_freq(&e22_context, e22_config.rf_freq_hz);
    sx126x_set_pa_cfg(&e22_context, &e22_config.pa_cfg);
    sx126x_set_tx_params(&e22_context, e22_config.tx_power_dbm, e22_config.ramp_time);
    sx126x_set_rx_tx_fallback_mode(&e22_context, SX126X_FALLBACK_STDBY_RC);
    sx126x_cfg_rx_boosted(&e22_context, false);

    mod_cfg.sf = e22_config.sf;
    mod_cfg.bw = e22_config.bw;
    mod_cfg.cr = e22_config.cr;
    mod_cfg.ldro = e22_config.ldro;
    sx126x_set_lora_mod_params(&e22_context, &mod_cfg);

    pkt_cfg.preamble_len_in_symb = e22_config.preamble_len;
    pkt_cfg.header_type = SX126X_LORA_PKT_EXPLICIT;
    pkt_cfg.pld_len_in_bytes = 255U;
    pkt_cfg.crc_is_on = e22_config.crc_on;
    pkt_cfg.invert_iq_is_on = e22_config.invert_iq;
    sx126x_set_lora_pkt_params(&e22_context, &pkt_cfg);
    sx126x_set_lora_sync_word(&e22_context, e22_config.sync_word);
    sx126x_clear_irq_status(&e22_context, SX126X_IRQ_ALL);

    e22_lora_tx_rf_switch_close();
    return true;
}

bool e22_lora_tx_send(const uint8_t *payload, uint8_t length)
{
    if ((payload == 0) || (length == 0U) || (e22_lora_tx_is_busy() == true))
    {
        return false;
    }

    pkt_cfg.pld_len_in_bytes = length;
    sx126x_set_lora_pkt_params(&e22_context, &pkt_cfg);
    sx126x_write_buffer(&e22_context, 0x00, payload, length);

    sx126x_set_dio_irq_params(&e22_context,
                              SX126X_IRQ_TX_DONE,
                              SX126X_IRQ_TX_DONE,
                              SX126X_IRQ_NONE,
                              SX126X_IRQ_NONE);
    sx126x_clear_irq_status(&e22_context, SX126X_IRQ_ALL);

    sx126x_rf_switch_tx();
    tx_start_tick = e22_lora_port_get_tick_ms();
    e22_context.is_tx = true;
    e22_diag.is_busy = true;
    e22_diag.tx_start_count++;
    e22_diag.last_payload_length = length;
    sx126x_set_tx(&e22_context, 0x00);

    return true;
}

bool e22_lora_tx_is_busy(void)
{
    if (e22_context.is_tx == true)
    {
        e22_lora_tx_dio1_irq_handler();
    }

    if ((e22_context.is_tx == true) &&
        ((e22_lora_port_get_tick_ms() - tx_start_tick) > e22_config.tx_timeout_ms))
    {
        e22_context.is_tx = false;
        e22_diag.is_busy = false;
        e22_diag.tx_timeout_count++;
        e22_lora_tx_rf_switch_close();
    }

    return e22_context.is_tx;
}

void e22_lora_tx_dio1_irq_handler(void)
{
    sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

    sx126x_get_irq_status(&e22_context, &irq_mask);

    if (irq_mask != SX126X_IRQ_NONE)
    {
        e22_diag.last_irq = (uint16_t)irq_mask;
        sx126x_clear_irq_status(&e22_context, irq_mask);

        if ((irq_mask & SX126X_IRQ_TX_DONE) != 0U)
        {
            e22_context.is_tx = false;
            e22_diag.is_busy = false;
            e22_diag.tx_done_count++;
            e22_lora_tx_rf_switch_close();
        }
    }
}

void e22_lora_tx_get_diag(e22_lora_tx_diag_t *diag)
{
    if (diag != 0)
    {
        *diag = e22_diag;
        diag->is_busy = e22_lora_tx_is_busy();
    }
}
