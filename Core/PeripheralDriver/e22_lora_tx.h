#ifndef E22_LORA_TX_H
#define E22_LORA_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "sx126x.h"

typedef struct
{
    uint32_t rf_freq_hz;
    uint16_t cal_img_low_mhz;
    uint16_t cal_img_high_mhz;
    uint8_t sync_word;
    int8_t tx_power_dbm;
    sx126x_ramp_time_t ramp_time;
    sx126x_pa_cfg_params_t pa_cfg;
    sx126x_lora_sf_t sf;
    sx126x_lora_bw_t bw;
    sx126x_lora_cr_t cr;
    bool ldro;
    uint16_t preamble_len;
    bool crc_on;
    bool invert_iq;
    sx126x_tcxo_ctrl_voltages_t tcxo_voltage;
    uint32_t tcxo_startup_time;
    uint32_t tx_timeout_ms;
} e22_lora_tx_config_t;

typedef struct
{
    uint32_t tx_start_count;
    uint32_t tx_done_count;
    uint32_t tx_timeout_count;
    uint16_t last_irq;
    uint8_t last_payload_length;
    bool is_busy;
} e22_lora_tx_diag_t;

void e22_lora_tx_get_default_config(e22_lora_tx_config_t *config);
bool e22_lora_tx_init(const e22_lora_tx_config_t *config);
bool e22_lora_tx_send(const uint8_t *payload, uint8_t length);
bool e22_lora_tx_is_busy(void);
void e22_lora_tx_dio1_irq_handler(void);
void e22_lora_tx_get_diag(e22_lora_tx_diag_t *diag);
void e22_lora_tx_rf_switch_close(void);

#ifdef __cplusplus
}
#endif

#endif /* E22_LORA_TX_H */
