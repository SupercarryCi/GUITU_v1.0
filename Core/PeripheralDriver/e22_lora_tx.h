#ifndef E22_LORA_TX_H
#define E22_LORA_TX_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "sx126x.h"

#define E22_LORA_MAX_PAYLOAD_LEN 255U

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
    uint8_t length;
    int8_t rssi_dbm;
    int8_t snr_db;
    int8_t signal_rssi_dbm;
    uint8_t payload[E22_LORA_MAX_PAYLOAD_LEN];
} e22_lora_rx_packet_t;

typedef struct
{
    uint32_t irq_count;
    uint32_t preamble_count;
    uint32_t tx_start_count;
    uint32_t tx_done_count;
    uint32_t tx_timeout_count;
    uint32_t rx_done_count;
    uint32_t rx_ok_count;
    uint32_t crc_error_count;
    uint32_t header_error_count;
    uint32_t timeout_count;
    uint16_t last_irq;
    uint8_t last_payload_length;
    int8_t last_rssi_dbm;
    int8_t last_snr_db;
    int8_t last_signal_rssi_dbm;
    bool is_busy;
    bool is_tx_busy;
    bool is_rx_enabled;
    bool rx_ready;
} e22_lora_tx_diag_t;

void e22_lora_tx_get_default_config(e22_lora_tx_config_t *config);
bool e22_lora_tx_init(const e22_lora_tx_config_t *config);
bool e22_lora_tx_send(const uint8_t *payload, uint8_t length);
bool e22_lora_tx_is_busy(void);
bool e22_lora_rx_start(void);
void e22_lora_rx_stop(void);
bool e22_lora_rx_available(void);
bool e22_lora_rx_is_active(void);
bool e22_lora_rx_read_packet(e22_lora_rx_packet_t *packet);
bool e22_lora_rx_read(uint8_t *buffer, uint8_t *length, int8_t *rssi_dbm, int8_t *snr_db, int8_t *signal_rssi_dbm);
void e22_lora_irq_handler(void);
void e22_lora_tx_dio1_irq_handler(void);
void e22_lora_tx_get_diag(e22_lora_tx_diag_t *diag);
void e22_lora_tx_rf_switch_close(void);

#ifdef __cplusplus
}
#endif

#endif /* E22_LORA_TX_H */
