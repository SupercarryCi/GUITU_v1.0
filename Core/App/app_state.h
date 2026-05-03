#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

#include "app_msg.h"
#include "ins_nav.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t init_done_mask;
    int32_t init_result;
    uint32_t fault_count;
    uint32_t last_fault_code;
    uint32_t last_tick_ms;
} AppSystemState_t;

typedef struct
{
    uint8_t frame_valid;
    uint16_t raw_len;
    uint32_t rx_count;
    uint32_t drop_count;
    uint32_t parse_error_count;
    uint32_t last_tick_ms;
    INS_Status last_ins_status;
    INS_SensorFrame frame;
    uint8_t raw[APP_GYRO_RX_MAX_LEN];
} GyroState_t;

typedef struct
{
    uint8_t valid;
    uint32_t update_count;
    uint32_t last_tick_ms;
    INS_State state;
} NavState_t;

typedef struct
{
    uint8_t valid;
    uint16_t raw[APP_ADC_CHANNEL_COUNT];
    uint16_t voltage_mv[APP_ADC_CHANNEL_COUNT];
    uint32_t update_count;
    uint32_t error_count;
    uint32_t last_tick_ms;
} AdcState_t;

typedef struct
{
    uint8_t valid;
    uint8_t spo2_percent;
    uint16_t heart_rate_bpm;
    uint16_t perfusion_permille;
    uint32_t update_count;
    uint32_t error_count;
    uint32_t last_tick_ms;
} Spo2State_t;

typedef struct
{
    AppCommandMsg_t last_command;
    uint32_t command_count;
    uint32_t render_count;
    uint32_t touch_count;
    uint32_t last_tick_ms;
} UiState_t;

typedef struct
{
    uint8_t enabled;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint32_t last_tick_ms;
    LoraPacketMsg_t last_rx;
} LoraState_t;

typedef enum
{
    RETURN_MODE_IDLE = 0,
    RETURN_MODE_RUNNING,
    RETURN_MODE_PAUSED,
    RETURN_MODE_DONE,
    RETURN_MODE_FAULT
} ReturnMode_t;

typedef struct
{
    ReturnMode_t mode;
    uint32_t path_points;
    uint32_t target_index;
    uint32_t step_count;
    uint32_t error_count;
    uint32_t last_tick_ms;
} ReturnState_t;

typedef struct
{
    AppSystemState_t system;
    GyroState_t gyro;
    NavState_t nav;
    AdcState_t adc;
    Spo2State_t spo2;
    UiState_t ui;
    LoraState_t lora;
    ReturnState_t return_home;
} AppSnapshot_t;

int32_t App_StateInit(void);

void App_StateSetSystem(const AppSystemState_t *state);
void App_StateGetSystem(AppSystemState_t *state);
void App_StateSetInitResult(uint32_t done_mask, int32_t result);
void App_StateAddFault(uint32_t fault_code);

void App_StateSetGyro(const GyroState_t *state);
void App_StateGetGyro(GyroState_t *state);

void App_StateSetNav(const NavState_t *state);
void App_StateGetNav(NavState_t *state);

void App_StateSetAdc(const AdcState_t *state);
void App_StateGetAdc(AdcState_t *state);

void App_StateSetSpo2(const Spo2State_t *state);
void App_StateGetSpo2(Spo2State_t *state);

void App_StateSetUi(const UiState_t *state);
void App_StateGetUi(UiState_t *state);

void App_StateSetLora(const LoraState_t *state);
void App_StateGetLora(LoraState_t *state);

void App_StateSetReturn(const ReturnState_t *state);
void App_StateGetReturn(ReturnState_t *state);

void App_StateGetSnapshot(AppSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

#endif
