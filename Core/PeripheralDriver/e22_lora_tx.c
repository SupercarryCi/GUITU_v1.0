#include "e22_lora_tx.h"
#include "e22_lora_port.h"
#include "sx126x_hal.h"
#include <stddef.h>
#include <string.h>

typedef struct
{
    bool is_initialized;
    bool is_tx;
    bool is_rx_enabled;
    bool resume_rx_after_tx;
    bool rx_ready;
    uint8_t rx_buffer[E22_LORA_MAX_PAYLOAD_LEN];
    uint8_t rx_length;
    int8_t rx_rssi_dbm;
    int8_t rx_snr_db;
    int8_t rx_signal_rssi_dbm;
} e22_lora_context_t;

static e22_lora_context_t e22_context;
static e22_lora_tx_config_t e22_config;
static e22_lora_tx_diag_t e22_diag;
static sx126x_pkt_params_lora_t pkt_cfg;
static uint32_t tx_start_tick;

static bool e22_lora_config_rx_continuous(void);
static void e22_lora_resume_rx_if_needed(void);
static void e22_lora_handle_rx_done(sx126x_irq_mask_t irq_mask);

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
    config->sf = SX126X_LORA_SF9;
    config->bw = SX126X_LORA_BW_125;
    config->cr = SX126X_LORA_CR_4_8;
    config->ldro = true;
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

    /* 0x06BB 是 SX126x 可读写寄存器，用来做基础 SPI 通信检查。 */
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

    memset(&e22_context, 0, sizeof(e22_context));
    memset(&e22_diag, 0, sizeof(e22_diag));

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
    pkt_cfg.pld_len_in_bytes = E22_LORA_MAX_PAYLOAD_LEN;
    pkt_cfg.crc_is_on = e22_config.crc_on;
    pkt_cfg.invert_iq_is_on = e22_config.invert_iq;
    sx126x_set_lora_pkt_params(&e22_context, &pkt_cfg);
    sx126x_set_lora_sync_word(&e22_context, e22_config.sync_word);
    sx126x_clear_irq_status(&e22_context, SX126X_IRQ_ALL);

    e22_lora_tx_rf_switch_close();
    e22_context.is_initialized = true;
    return true;
}

bool e22_lora_tx_send(const uint8_t *payload, uint8_t length)
{
    if ((e22_context.is_initialized == false) || (payload == 0) || (length == 0U))
    {
        return false;
    }

    /* 发送前先处理可能已经到来的 RX_DONE，避免清 IRQ 时丢掉刚收到的包。 */
    e22_lora_irq_handler();

    if (e22_lora_tx_is_busy() == true)
    {
        return false;
    }

    e22_context.resume_rx_after_tx = e22_context.is_rx_enabled;
    e22_context.is_rx_enabled = false;

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
    e22_diag.tx_start_count++;
    e22_diag.last_payload_length = length;
    sx126x_set_tx(&e22_context, 0x00);

    return true;
}

bool e22_lora_tx_is_busy(void)
{
    if (e22_context.is_initialized == false)
    {
        return false;
    }

    if (e22_context.is_tx == true)
    {
        e22_lora_irq_handler();
    }

    if ((e22_context.is_tx == true) &&
        ((e22_lora_port_get_tick_ms() - tx_start_tick) > e22_config.tx_timeout_ms))
    {
        e22_context.is_tx = false;
        e22_diag.tx_timeout_count++;
        e22_lora_resume_rx_if_needed();
    }

    return e22_context.is_tx;
}

static bool e22_lora_config_rx_continuous(void)
{
    pkt_cfg.pld_len_in_bytes = E22_LORA_MAX_PAYLOAD_LEN;
    sx126x_set_lora_pkt_params(&e22_context, &pkt_cfg);

    sx126x_set_dio_irq_params(&e22_context,
                              SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                                  SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_TIMEOUT,
                              SX126X_IRQ_RX_DONE | SX126X_IRQ_PREAMBLE_DETECTED |
                                  SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR | SX126X_IRQ_TIMEOUT,
                              SX126X_IRQ_NONE,
                              SX126X_IRQ_NONE);
    sx126x_clear_irq_status(&e22_context, SX126X_IRQ_ALL);

    sx126x_rf_switch_rx();
    sx126x_set_rx_with_timeout_in_rtc_step(&e22_context, SX126X_RX_CONTINUOUS);
    return true;
}

bool e22_lora_rx_start(void)
{
    if (e22_context.is_initialized == false)
    {
        return false;
    }

    if (e22_lora_tx_is_busy() == true)
    {
        return false;
    }

    e22_context.resume_rx_after_tx = false;
    e22_context.is_rx_enabled = true;
    return e22_lora_config_rx_continuous();
}

void e22_lora_rx_stop(void)
{
    if (e22_context.is_initialized == false)
    {
        return;
    }

    if (e22_context.is_tx == true)
    {
        e22_context.resume_rx_after_tx = false;
        return;
    }

    sx126x_set_standby(&e22_context, SX126X_STANDBY_CFG_RC);
    e22_context.is_rx_enabled = false;
    e22_context.resume_rx_after_tx = false;
    e22_lora_tx_rf_switch_close();
}

bool e22_lora_rx_available(void)
{
    if (e22_context.is_initialized == false)
    {
        return false;
    }

    e22_lora_irq_handler();
    return e22_context.rx_ready;
}

bool e22_lora_rx_read_packet(e22_lora_rx_packet_t *packet)
{
    if (packet == 0)
    {
        return false;
    }

    if (e22_context.is_initialized == false)
    {
        return false;
    }

    e22_lora_irq_handler();

    if (e22_context.rx_ready == false)
    {
        return false;
    }

    e22_lora_port_enter_critical();
    packet->length = e22_context.rx_length;
    packet->rssi_dbm = e22_context.rx_rssi_dbm;
    packet->snr_db = e22_context.rx_snr_db;
    packet->signal_rssi_dbm = e22_context.rx_signal_rssi_dbm;
    memcpy(packet->payload, e22_context.rx_buffer, e22_context.rx_length);
    e22_context.rx_ready = false;
    e22_context.rx_length = 0U;
    e22_lora_port_exit_critical();

    return true;
}

bool e22_lora_rx_read(uint8_t *buffer, uint8_t *length, int8_t *rssi_dbm, int8_t *snr_db, int8_t *signal_rssi_dbm)
{
    e22_lora_rx_packet_t packet;

    if ((buffer == 0) || (length == 0))
    {
        return false;
    }

    if (e22_lora_rx_read_packet(&packet) == false)
    {
        return false;
    }

    memcpy(buffer, packet.payload, packet.length);
    *length = packet.length;

    if (rssi_dbm != 0)
    {
        *rssi_dbm = packet.rssi_dbm;
    }
    if (snr_db != 0)
    {
        *snr_db = packet.snr_db;
    }
    if (signal_rssi_dbm != 0)
    {
        *signal_rssi_dbm = packet.signal_rssi_dbm;
    }

    return true;
}

static void e22_lora_resume_rx_if_needed(void)
{
    if (e22_context.resume_rx_after_tx == true)
    {
        e22_context.resume_rx_after_tx = false;
        e22_context.is_rx_enabled = true;
        (void)e22_lora_config_rx_continuous();
    }
    else if (e22_context.is_rx_enabled == false)
    {
        e22_lora_tx_rf_switch_close();
    }
}

static void e22_lora_handle_rx_done(sx126x_irq_mask_t irq_mask)
{
    sx126x_rx_buffer_status_t buffer_status;
    sx126x_pkt_status_lora_t pkt_status;

    e22_diag.rx_done_count++;

    if ((irq_mask & (SX126X_IRQ_CRC_ERROR | SX126X_IRQ_HEADER_ERROR)) != 0U)
    {
        if (e22_context.is_rx_enabled == true)
        {
            (void)e22_lora_config_rx_continuous();
        }
        return;
    }

    sx126x_get_rx_buffer_status(&e22_context, &buffer_status);
    if (buffer_status.pld_len_in_bytes != 0U)
    {
        sx126x_get_lora_pkt_status(&e22_context, &pkt_status);

        e22_lora_port_enter_critical();
        sx126x_read_buffer(&e22_context,
                           buffer_status.buffer_start_pointer,
                           e22_context.rx_buffer,
                           buffer_status.pld_len_in_bytes);
        e22_context.rx_length = buffer_status.pld_len_in_bytes;
        e22_context.rx_rssi_dbm = pkt_status.rssi_pkt_in_dbm;
        e22_context.rx_snr_db = pkt_status.snr_pkt_in_db;
        e22_context.rx_signal_rssi_dbm = pkt_status.signal_rssi_pkt_in_dbm;
        e22_context.rx_ready = true;
        e22_lora_port_exit_critical();

        e22_diag.rx_ok_count++;
        e22_diag.last_payload_length = buffer_status.pld_len_in_bytes;
        e22_diag.last_rssi_dbm = pkt_status.rssi_pkt_in_dbm;
        e22_diag.last_snr_db = pkt_status.snr_pkt_in_db;
        e22_diag.last_signal_rssi_dbm = pkt_status.signal_rssi_pkt_in_dbm;
    }

    if (e22_context.is_rx_enabled == true)
    {
        (void)e22_lora_config_rx_continuous();
    }
}

void e22_lora_irq_handler(void)
{
    sx126x_irq_mask_t irq_mask = SX126X_IRQ_NONE;

    if (e22_context.is_initialized == false)
    {
        return;
    }

    sx126x_get_irq_status(&e22_context, &irq_mask);
    if (irq_mask == SX126X_IRQ_NONE)
    {
        return;
    }

    e22_diag.last_irq = (uint16_t)irq_mask;
    e22_diag.irq_count++;
    sx126x_clear_irq_status(&e22_context, irq_mask);

    if ((irq_mask & SX126X_IRQ_PREAMBLE_DETECTED) != 0U)
    {
        e22_diag.preamble_count++;
    }
    if ((irq_mask & SX126X_IRQ_CRC_ERROR) != 0U)
    {
        e22_diag.crc_error_count++;
    }
    if ((irq_mask & SX126X_IRQ_HEADER_ERROR) != 0U)
    {
        e22_diag.header_error_count++;
    }
    if ((irq_mask & SX126X_IRQ_TIMEOUT) != 0U)
    {
        e22_diag.timeout_count++;
    }

    if ((irq_mask & SX126X_IRQ_RX_DONE) != 0U)
    {
        e22_lora_handle_rx_done(irq_mask);
    }

    if ((irq_mask & SX126X_IRQ_TX_DONE) != 0U)
    {
        e22_context.is_tx = false;
        e22_diag.tx_done_count++;
        e22_lora_resume_rx_if_needed();
    }
}

void e22_lora_tx_dio1_irq_handler(void)
{
    e22_lora_irq_handler();
}

void e22_lora_tx_get_diag(e22_lora_tx_diag_t *diag)
{
    if (diag != 0)
    {
        e22_lora_irq_handler();
        *diag = e22_diag;
        diag->is_busy = e22_context.is_tx;
        diag->is_tx_busy = e22_context.is_tx;
        diag->is_rx_enabled = e22_context.is_rx_enabled;
        diag->rx_ready = e22_context.rx_ready;
    }
}
