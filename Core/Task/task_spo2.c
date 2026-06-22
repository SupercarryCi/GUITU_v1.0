#include "task_spo2.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "max30102.h"

#include <string.h>

#define TASK_SPO2_RECALC_SAMPLES     100U
#define TASK_SPO2_DISPLAY_MIN        60U
#define TASK_SPO2_DISPLAY_MAX        160U
#define TASK_SPO2_DISPLAY_DELTA_MAX  10U
#define TASK_SPO2_RELOCK_COUNT       3U

typedef struct
{
    uint16_t value;
    uint8_t count;
} TaskSpo2Candidate_t;

static uint32_t s_spo2_red_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_ir_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_calc_red_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_calc_ir_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint16_t s_spo2_write_index = 0U;
static uint16_t s_spo2_sample_count = 0U;
static uint16_t s_spo2_samples_since_calc = 0U;
static uint8_t s_spo2_has_first_result = 0U;
static Spo2State_t s_spo2_display_state;
static uint8_t s_spo2_has_display_spo2 = 0U;
static uint8_t s_spo2_has_display_hr = 0U;
static TaskSpo2Candidate_t s_spo2_candidate;
static TaskSpo2Candidate_t s_hr_candidate;

static void Task_Spo2ResetBuffer(void)
{
    memset(s_spo2_red_buffer, 0, sizeof(s_spo2_red_buffer));
    memset(s_spo2_ir_buffer, 0, sizeof(s_spo2_ir_buffer));
    memset(s_spo2_calc_red_buffer, 0, sizeof(s_spo2_calc_red_buffer));
    memset(s_spo2_calc_ir_buffer, 0, sizeof(s_spo2_calc_ir_buffer));
    s_spo2_write_index = 0U;
    s_spo2_sample_count = 0U;
    s_spo2_samples_since_calc = 0U;
    s_spo2_has_first_result = 0U;
    memset(&s_spo2_display_state, 0, sizeof(s_spo2_display_state));
    s_spo2_has_display_spo2 = 0U;
    s_spo2_has_display_hr = 0U;
    memset(&s_spo2_candidate, 0, sizeof(s_spo2_candidate));
    memset(&s_hr_candidate, 0, sizeof(s_hr_candidate));
}

static void Task_Spo2StoreSample(uint32_t red, uint32_t ir)
{
    s_spo2_red_buffer[s_spo2_write_index] = red;
    s_spo2_ir_buffer[s_spo2_write_index] = ir;

    s_spo2_write_index++;
    if (s_spo2_write_index >= MAX30102_ALGO_BUFFER_SIZE)
    {
        s_spo2_write_index = 0U;
    }

    if (s_spo2_sample_count < MAX30102_ALGO_BUFFER_SIZE)
    {
        s_spo2_sample_count++;
        if (s_spo2_sample_count == MAX30102_ALGO_BUFFER_SIZE)
        {
            /* 初始 5 秒窗口填满后先算一次，后续每 100 个新样本再刷新。 */
            s_spo2_samples_since_calc = TASK_SPO2_RECALC_SAMPLES;
        }
    }
    else if (s_spo2_samples_since_calc < 1000U)
    {
        s_spo2_samples_since_calc++;
    }
}

static void Task_Spo2CopyOrderedBuffer(void)
{
    uint16_t i;
    uint16_t index = s_spo2_write_index;

    for (i = 0U; i < MAX30102_ALGO_BUFFER_SIZE; i++)
    {
        s_spo2_calc_red_buffer[i] = s_spo2_red_buffer[index];
        s_spo2_calc_ir_buffer[i] = s_spo2_ir_buffer[index];

        index++;
        if (index >= MAX30102_ALGO_BUFFER_SIZE)
        {
            index = 0U;
        }
    }
}

static uint16_t Task_Spo2EstimatePerfusion(void)
{
    uint32_t i;
    uint32_t min_ir = 0xFFFFFFFFU;
    uint32_t max_ir = 0U;
    uint32_t sum_ir = 0U;
    uint32_t avg_ir;
    uint32_t perfusion;

    for (i = 0U; i < MAX30102_ALGO_BUFFER_SIZE; i++)
    {
        uint32_t value = s_spo2_calc_ir_buffer[i];
        if (value < min_ir)
        {
            min_ir = value;
        }
        if (value > max_ir)
        {
            max_ir = value;
        }
        sum_ir += value;
    }

    avg_ir = sum_ir / MAX30102_ALGO_BUFFER_SIZE;
    if (avg_ir == 0U)
    {
        return 0U;
    }

    perfusion = ((max_ir - min_ir) * 1000U) / avg_ir;
    if (perfusion > 65535U)
    {
        perfusion = 65535U;
    }

    return (uint16_t)perfusion;
}

static uint8_t Task_Spo2IsDisplayValueAllowed(uint16_t value,
                                               uint16_t last_value,
                                               uint8_t has_last,
                                               TaskSpo2Candidate_t *candidate)
{
    uint16_t diff;

    if ((value < TASK_SPO2_DISPLAY_MIN) || (value > TASK_SPO2_DISPLAY_MAX))
    {
        candidate->count = 0U;
        return 0U;
    }

    if (has_last == 0U)
    {
        candidate->count = 0U;
        return 1U;
    }

    diff = (value > last_value) ? (uint16_t)(value - last_value) : (uint16_t)(last_value - value);
    if (diff <= TASK_SPO2_DISPLAY_DELTA_MAX)
    {
        candidate->count = 0U;
        return 1U;
    }

    if (candidate->count == 0U)
    {
        candidate->value = value;
        candidate->count = 1U;
        return 0U;
    }

    diff = (value > candidate->value) ? (uint16_t)(value - candidate->value)
                                      : (uint16_t)(candidate->value - value);
    if (diff > TASK_SPO2_DISPLAY_DELTA_MAX)
    {
        candidate->value = value;
        candidate->count = 1U;
        return 0U;
    }

    candidate->count++;
    if (candidate->count < TASK_SPO2_RELOCK_COUNT)
    {
        return 0U;
    }

    /* 连续稳定的新数据确认后重新锁定，避免显示值永久停留在旧区间。 */
    candidate->count = 0U;
    return 1U;
}

static void Task_Spo2FillOutput(Spo2State_t *sample,
                                int32_t spo2,
                                int8_t spo2_valid,
                                int32_t heart_rate,
                                int8_t hr_valid)
{
    if ((spo2_valid == 1) && (spo2 > 0) && (spo2 <= 100))
    {
        uint16_t display_spo2 = (uint16_t)spo2;

        if (Task_Spo2IsDisplayValueAllowed(display_spo2,
                                           s_spo2_display_state.spo2_percent,
                                           s_spo2_has_display_spo2,
                                           &s_spo2_candidate) != 0U)
        {
            s_spo2_display_state.spo2_percent = (uint8_t)display_spo2;
            s_spo2_has_display_spo2 = 1U;
        }
    }
    else
    {
        s_spo2_candidate.count = 0U;
    }

    if ((hr_valid == 1) && (heart_rate > 0) && (heart_rate <= 65535))
    {
        uint16_t display_hr = (uint16_t)heart_rate;

        if (Task_Spo2IsDisplayValueAllowed(display_hr,
                                           s_spo2_display_state.heart_rate_bpm,
                                           s_spo2_has_display_hr,
                                           &s_hr_candidate) != 0U)
        {
            s_spo2_display_state.heart_rate_bpm = display_hr;
            s_spo2_has_display_hr = 1U;
        }
    }
    else
    {
        s_hr_candidate.count = 0U;
    }

    s_spo2_display_state.perfusion_permille = Task_Spo2EstimatePerfusion();
    *sample = s_spo2_display_state;
}

/*
 * 血氧链路：
 * Spo2Task 周期占用 I2C 总线 -> 读取 MAX30102 FIFO -> 更新 Spo2State_t。
 */
int32_t App_Spo2HardwareInit(void)
{
    Task_Spo2ResetBuffer();
    return MAX30102_Init(&APP_SPO2_I2C_HANDLE);
}

int32_t App_Spo2ReadSample(Spo2State_t *sample)
{
    uint8_t fifo_count;
    uint8_t i;
    int32_t spo2;
    int8_t spo2_valid;
    int32_t heart_rate;
    int8_t hr_valid;

    if (sample == NULL)
    {
        return -1;
    }

    if (MAX30102_ReadFifoSampleCount(&APP_SPO2_I2C_HANDLE, &fifo_count) != 0)
    {
        return -2;
    }

    if (fifo_count > MAX30102_FIFO_DEPTH)
    {
        fifo_count = MAX30102_FIFO_DEPTH;
    }

    for (i = 0U; i < fifo_count; i++)
    {
        uint32_t red;
        uint32_t ir;

        if (MAX30102_ReadFifo(&APP_SPO2_I2C_HANDLE, &red, &ir) != 0)
        {
            return -3;
        }
        Task_Spo2StoreSample(red, ir);
    }

    if (fifo_count == 0U)
    {
        return 0;
    }

    if (s_spo2_sample_count < MAX30102_ALGO_BUFFER_SIZE)
    {
        return 0;
    }

    if ((s_spo2_has_first_result != 0U) && (s_spo2_samples_since_calc < TASK_SPO2_RECALC_SAMPLES))
    {
        return 0;
    }

    Task_Spo2CopyOrderedBuffer();
    maxim_heart_rate_and_oxygen_saturation(s_spo2_calc_ir_buffer,
                                           (int32_t)MAX30102_ALGO_BUFFER_SIZE,
                                           s_spo2_calc_red_buffer,
                                           &spo2,
                                           &spo2_valid,
                                           &heart_rate,
                                           &hr_valid);

    s_spo2_has_first_result = 1U;
    s_spo2_samples_since_calc = 0U;
    Task_Spo2FillOutput(sample, spo2, spo2_valid, heart_rate, hr_valid);
    return 1;
}

int32_t Task_Spo2InitHardware(void)
{
    /* 初始化阶段也走 I2C 互斥锁，保持和运行态访问规则一致。 */
    if (osMutexAcquire(g_i2cBusMutex, osWaitForever) != osOK)
    {
        return -1;
    }

    if (App_Spo2HardwareInit() != 0)
    {
        osMutexRelease(g_i2cBusMutex);
        return -2;
    }

    osMutexRelease(g_i2cBusMutex);
    return 0;
}

void Task_Spo2Entry(void *argument)
{
    Spo2State_t spo2;
    uint32_t next_tick;

    (void)argument;
    memset(&spo2, 0, sizeof(spo2));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        Spo2State_t sample;
        int32_t result = 0;

        memset(&sample, 0, sizeof(sample));
        if (osMutexAcquire(g_i2cBusMutex, osWaitForever) == osOK)
        {
            result = App_Spo2ReadSample(&sample);
            osMutexRelease(g_i2cBusMutex);
        }
        else
        {
            result = -1;
        }

        if (result > 0)
        {
            sample.update_count = spo2.update_count + 1U;
            sample.error_count = spo2.error_count;
            spo2 = sample;
            App_StateSetSpo2(&spo2);
            osEventFlagsSet(g_sysEventFlags, SYS_EVT_SPO2_UPDATED);
        }
        else if (result < 0)
        {
            spo2.error_count++;
            App_StateSetSpo2(&spo2);
        }

        next_tick += APP_SPO2_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}
