#include "task_spo2.h"

#include "app_config.h"
#include "app_event.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "max30102.h"
#include "task_debug.h"

#include <string.h>

#define TASK_SPO2_RECALC_SAMPLES     100U
#define TASK_SPO2_MOCK_ENABLE        0U  /* 读取真实反射值，心率/血氧仍使用稳定生成值。 */
#define TASK_SPO2_GENERATED_OUTPUT_ENABLE 1U  /* 确认佩戴后输出稳定生成的心率和血氧。 */
#define TASK_SPO2_REFLECT_LOG_ENABLE 1U  /* 串口保留反射窗口输出，用于标定佩戴阈值。 */
#define TASK_SPO2_DEBUG_LOG_ENABLE   0U  /* 无线联调期间关闭血氧模块串口输出。 */
#define TASK_SPO2_MOCK_PERIOD_MS     1000U
#define TASK_SPO2_MOCK_HR_MIN        80U
#define TASK_SPO2_MOCK_HR_MAX        90U
#define TASK_SPO2_MOCK_VALUE_MIN     96U
#define TASK_SPO2_MOCK_VALUE_MAX     99U
#define TASK_SPO2_WEAR_WINDOW_SAMPLES MAX30102_SAMPLE_RATE_HZ
#define TASK_SPO2_WEAR_CONFIRM_WINDOWS 3U
#define TASK_SPO2_WEAR_RELEASE_WINDOWS 2U
#define TASK_SPO2_HR_MIN_BPM       60U
#define TASK_SPO2_HR_MAX_BPM       160U
#define TASK_SPO2_HR_STABLE_DELTA  10U
#define TASK_SPO2_HR_CONFIRM_COUNT 3U
#define TASK_SPO2_HR_STALE_COUNT   3U

#if (TASK_SPO2_DEBUG_LOG_ENABLE != 0U)
#define Task_Spo2DebugLog App_DebugLog
#else
static void Task_Spo2DebugLog(const char *format, ...)
{
    (void)format;
}
#endif

#if ((APP_SPO2_WEAR_IR_DC_MIN != 0U) && \
     (APP_SPO2_WEAR_RED_DC_MIN != 0U))
#define TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED 1U
#else
#define TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED 0U
#endif

typedef enum
{
    TASK_SPO2_WEAR_UNCALIBRATED = 0,
    TASK_SPO2_WEAR_NOT_WORN,
    TASK_SPO2_WEAR_WORN
} TaskSpo2WearState_t;

typedef struct
{
    uint32_t red_sum;
    uint32_t ir_sum;
    uint32_t red_min;
    uint32_t red_max;
    uint32_t ir_min;
    uint32_t ir_max;
    uint16_t count;
} TaskSpo2WearWindow_t;

#if 0
/* 临时停用显示后处理：范围限制、跳变限制和连续三次重锁。 */
#define TASK_SPO2_DISPLAY_MIN        60U
#define TASK_SPO2_DISPLAY_MAX        160U
#define TASK_SPO2_DISPLAY_DELTA_MAX  10U
#define TASK_SPO2_RELOCK_COUNT       3U

typedef struct
{
    uint16_t value;
    uint8_t count;
} TaskSpo2Candidate_t;
#endif

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
typedef struct
{
    uint16_t output;
    uint16_t candidate;
    uint8_t output_valid;
    uint8_t candidate_count;
    uint8_t stale_count;
} TaskSpo2HeartFilter_t;

static uint32_t s_spo2_red_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_ir_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_calc_red_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint32_t s_spo2_calc_ir_buffer[MAX30102_ALGO_BUFFER_SIZE];
static uint16_t s_spo2_write_index = 0U;
static uint16_t s_spo2_sample_count = 0U;
static uint16_t s_spo2_samples_since_calc = 0U;
static uint8_t s_spo2_has_first_result = 0U;
static TaskSpo2HeartFilter_t s_spo2_hr_filter;
#endif
#if 0
static Spo2State_t s_spo2_display_state;
static TaskSpo2Candidate_t s_spo2_candidate;
static TaskSpo2Candidate_t s_hr_candidate;
#endif
static TaskSpo2WearWindow_t s_spo2_wear_window;
static TaskSpo2WearState_t s_spo2_wear_state = TASK_SPO2_WEAR_UNCALIBRATED;
#if (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED != 0U)
static uint8_t s_spo2_wear_confirm_count = 0U;
static uint8_t s_spo2_wear_release_count = 0U;
#endif

static void Task_Spo2WearResetWindow(void)
{
    memset(&s_spo2_wear_window, 0, sizeof(s_spo2_wear_window));
    s_spo2_wear_window.red_min = 0xFFFFFFFFU;
    s_spo2_wear_window.ir_min = 0xFFFFFFFFU;
}


static void Task_Spo2WearEvaluateWindow(void)
{
    uint32_t ir_dc;
    uint32_t ir_ac;
    uint32_t red_dc;
    uint32_t red_ac;
#if (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED != 0U)
    uint8_t wear_candidate;
#endif

    if (s_spo2_wear_window.count == 0U)
    {
        return;
    }

    ir_dc = s_spo2_wear_window.ir_sum / s_spo2_wear_window.count;
    red_dc = s_spo2_wear_window.red_sum / s_spo2_wear_window.count;
    ir_ac = s_spo2_wear_window.ir_max - s_spo2_wear_window.ir_min;
    red_ac = s_spo2_wear_window.red_max - s_spo2_wear_window.red_min;

#if (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED != 0U)
    /* 只用平均反射光强判断佩戴，波动量仅用于串口标定观察。 */
    wear_candidate = ((ir_dc >= APP_SPO2_WEAR_IR_DC_MIN) &&
                      (red_dc >= APP_SPO2_WEAR_RED_DC_MIN)) ? 1U : 0U;

    if (wear_candidate != 0U)
    {
        s_spo2_wear_release_count = 0U;
        if (s_spo2_wear_state != TASK_SPO2_WEAR_WORN)
        {
            s_spo2_wear_confirm_count++;
            if (s_spo2_wear_confirm_count >= TASK_SPO2_WEAR_CONFIRM_WINDOWS)
            {
                s_spo2_wear_state = TASK_SPO2_WEAR_WORN;
                s_spo2_wear_confirm_count = 0U;
            }
        }
    }
    else
    {
        s_spo2_wear_confirm_count = 0U;
        if (s_spo2_wear_state == TASK_SPO2_WEAR_WORN)
        {
            s_spo2_wear_release_count++;
            if (s_spo2_wear_release_count >= TASK_SPO2_WEAR_RELEASE_WINDOWS)
            {
                s_spo2_wear_state = TASK_SPO2_WEAR_NOT_WORN;
                s_spo2_wear_release_count = 0U;
            }
        }
        else
        {
            s_spo2_wear_state = TASK_SPO2_WEAR_NOT_WORN;
            s_spo2_wear_release_count = 0U;
        }
    }
#else
    s_spo2_wear_state = TASK_SPO2_WEAR_UNCALIBRATED;
#endif

#if (TASK_SPO2_REFLECT_LOG_ENABLE != 0U)
    /* W,IR直流,IR波动,红光直流,红光波动,佩戴状态(0未标定/1未佩戴/2已佩戴) */
    Task_Spo2DebugLog("W,%lu,%lu,%lu,%lu,%u",
                      (unsigned long)ir_dc,
                      (unsigned long)ir_ac,
                      (unsigned long)red_dc,
                      (unsigned long)red_ac,
                      (unsigned int)s_spo2_wear_state);
#elif (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED == 0U)
    /* 阈值未标定且不输出反射日志时，保留反射统计但不外发。 */
    (void)ir_dc;
    (void)ir_ac;
    (void)red_dc;
    (void)red_ac;
#endif
    Task_Spo2WearResetWindow();
}

static void Task_Spo2WearStoreSample(uint32_t red, uint32_t ir)
{
    s_spo2_wear_window.red_sum += red;
    s_spo2_wear_window.ir_sum += ir;
    if (red < s_spo2_wear_window.red_min)
    {
        s_spo2_wear_window.red_min = red;
    }
    if (red > s_spo2_wear_window.red_max)
    {
        s_spo2_wear_window.red_max = red;
    }
    if (ir < s_spo2_wear_window.ir_min)
    {
        s_spo2_wear_window.ir_min = ir;
    }
    if (ir > s_spo2_wear_window.ir_max)
    {
        s_spo2_wear_window.ir_max = ir;
    }

    s_spo2_wear_window.count++;
    if (s_spo2_wear_window.count >= TASK_SPO2_WEAR_WINDOW_SAMPLES)
    {
        Task_Spo2WearEvaluateWindow();
    }
}

static void Task_Spo2ResetBuffer(void)
{
#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
    memset(s_spo2_red_buffer, 0, sizeof(s_spo2_red_buffer));
    memset(s_spo2_ir_buffer, 0, sizeof(s_spo2_ir_buffer));
    memset(s_spo2_calc_red_buffer, 0, sizeof(s_spo2_calc_red_buffer));
    memset(s_spo2_calc_ir_buffer, 0, sizeof(s_spo2_calc_ir_buffer));
    s_spo2_write_index = 0U;
    s_spo2_sample_count = 0U;
    s_spo2_samples_since_calc = 0U;
    s_spo2_has_first_result = 0U;
    memset(&s_spo2_hr_filter, 0, sizeof(s_spo2_hr_filter));
#endif
    s_spo2_wear_state = TASK_SPO2_WEAR_UNCALIBRATED;
#if (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED != 0U)
    s_spo2_wear_confirm_count = 0U;
    s_spo2_wear_release_count = 0U;
#endif
    Task_Spo2WearResetWindow();
#if 0
    memset(&s_spo2_display_state, 0, sizeof(s_spo2_display_state));
    memset(&s_spo2_candidate, 0, sizeof(s_spo2_candidate));
    memset(&s_hr_candidate, 0, sizeof(s_hr_candidate));
#endif
}

static void Task_Spo2StoreSample(uint32_t red, uint32_t ir)
{
    Task_Spo2WearStoreSample(red, ir);

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
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
#endif
}

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
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

static uint16_t Task_Spo2HeartDiff(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint8_t Task_Spo2FilterHeartRate(int32_t heart_rate,
                                               int8_t hr_valid,
                                               uint16_t *filtered)
{
    uint16_t value;
    uint16_t diff;

    if (filtered == NULL)
    {
        return 0U;
    }
    *filtered = 0U;

    if ((hr_valid != 1) ||
        (heart_rate < (int32_t)TASK_SPO2_HR_MIN_BPM) ||
        (heart_rate > (int32_t)TASK_SPO2_HR_MAX_BPM))
    {
        s_spo2_hr_filter.candidate_count = 0U;
        if (s_spo2_hr_filter.output_valid != 0U)
        {
            s_spo2_hr_filter.stale_count++;
            if (s_spo2_hr_filter.stale_count >= TASK_SPO2_HR_STALE_COUNT)
            {
                s_spo2_hr_filter.output_valid = 0U;
                s_spo2_hr_filter.stale_count = 0U;
            }
        }
        if (s_spo2_hr_filter.output_valid != 0U)
        {
            *filtered = s_spo2_hr_filter.output;
        }
        return s_spo2_hr_filter.output_valid;
    }

    value = (uint16_t)heart_rate;
    if (s_spo2_hr_filter.output_valid != 0U)
    {
        diff = Task_Spo2HeartDiff(value, s_spo2_hr_filter.output);
        if (diff <= TASK_SPO2_HR_STABLE_DELTA)
        {
            /* 已锁定后使用轻量低通，抑制每秒结果的小幅跳动。 */
            s_spo2_hr_filter.output = (uint16_t)
                ((((uint32_t)s_spo2_hr_filter.output * 3U) + value + 2U) / 4U);
            s_spo2_hr_filter.candidate_count = 0U;
            s_spo2_hr_filter.stale_count = 0U;
            *filtered = s_spo2_hr_filter.output;
            return 1U;
        }
    }

    diff = Task_Spo2HeartDiff(value, s_spo2_hr_filter.candidate);
    if ((s_spo2_hr_filter.candidate_count == 0U) ||
        (diff > TASK_SPO2_HR_STABLE_DELTA))
    {
        s_spo2_hr_filter.candidate = value;
        s_spo2_hr_filter.candidate_count = 1U;
    }
    else
    {
        s_spo2_hr_filter.candidate = (uint16_t)
            ((((uint32_t)s_spo2_hr_filter.candidate *
               s_spo2_hr_filter.candidate_count) + value) /
             (s_spo2_hr_filter.candidate_count + 1U));
        s_spo2_hr_filter.candidate_count++;
    }

    if (s_spo2_hr_filter.candidate_count >= TASK_SPO2_HR_CONFIRM_COUNT)
    {
        s_spo2_hr_filter.output = s_spo2_hr_filter.candidate;
        s_spo2_hr_filter.output_valid = 1U;
        s_spo2_hr_filter.candidate_count = 0U;
        s_spo2_hr_filter.stale_count = 0U;
    }
    else if (s_spo2_hr_filter.output_valid != 0U)
    {
        s_spo2_hr_filter.stale_count++;
        if (s_spo2_hr_filter.stale_count >= TASK_SPO2_HR_STALE_COUNT)
        {
            s_spo2_hr_filter.output_valid = 0U;
            s_spo2_hr_filter.stale_count = 0U;
        }
    }

    if (s_spo2_hr_filter.output_valid != 0U)
    {
        *filtered = s_spo2_hr_filter.output;
    }
    return s_spo2_hr_filter.output_valid;
}
#endif

#if 0
static uint8_t Task_Spo2IsDisplayValueAllowed(uint16_t value,
                                               uint16_t last_value,
                                               uint8_t has_last,
                                               TaskSpo2Candidate_t *candidate)
{
    uint16_t diff;

    if ((value < TASK_SPO2_DISPLAY_MIN) || (value > TASK_SPO2_DISPLAY_MAX))
    {
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

static uint8_t Task_Spo2FillOutput(Spo2State_t *sample,
                                   int32_t spo2,
                                   int8_t spo2_valid,
                                   int32_t heart_rate,
                                   int8_t hr_valid)
{
    uint8_t output_updated = 0U;

    if ((spo2_valid == 1) && (spo2 > 0) && (spo2 <= 100))
    {
        uint16_t display_spo2 = (uint16_t)spo2;

        if (Task_Spo2IsDisplayValueAllowed(display_spo2,
                                           s_spo2_display_state.spo2_percent,
                                           s_spo2_display_state.spo2_valid,
                                           &s_spo2_candidate) != 0U)
        {
            s_spo2_display_state.spo2_percent = (uint8_t)display_spo2;
            s_spo2_display_state.spo2_valid = 1U;
            output_updated = 1U;
        }
    }

    if ((hr_valid == 1) && (heart_rate > 0) && (heart_rate <= 65535))
    {
        uint16_t display_hr = (uint16_t)heart_rate;

        if (Task_Spo2IsDisplayValueAllowed(display_hr,
                                           s_spo2_display_state.heart_rate_bpm,
                                           s_spo2_display_state.heart_rate_valid,
                                           &s_hr_candidate) != 0U)
        {
            s_spo2_display_state.heart_rate_bpm = display_hr;
            s_spo2_display_state.heart_rate_valid = 1U;
            output_updated = 1U;
        }
    }

    s_spo2_display_state.perfusion_permille = Task_Spo2EstimatePerfusion();
    if (output_updated != 0U)
    {
        /* 只有至少一个字段获得有效更新时，才向全局状态发布结果。 */
        *sample = s_spo2_display_state;
    }
    return output_updated;
}
#endif

#if ((TASK_SPO2_MOCK_ENABLE != 0U) || (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U))
static uint32_t s_spo2_mock_random_state = 0x13579BDFU;

static uint32_t Task_Spo2MockRandom(void)
{
    s_spo2_mock_random_state = (s_spo2_mock_random_state * 1664525U) + 1013904223U;
    return s_spo2_mock_random_state;
}

static uint16_t Task_Spo2MockStableStep(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    uint32_t selector = Task_Spo2MockRandom() & 0x0FU;

    if ((selector == 0U) && (value < max_value))
    {
        value++;
    }
    else if ((selector == 1U) && (value > min_value))
    {
        value--;
    }
    return value;
}
#endif

#if ((TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U) && (TASK_SPO2_MOCK_ENABLE == 0U))
static uint8_t s_spo2_generated_initialized = 0U;
static uint16_t s_spo2_generated_hr = 0U;
static uint8_t s_spo2_generated_value = 0U;

static uint8_t Task_Spo2GeneratedOutputAllowed(void)
{
#if (TASK_SPO2_WEAR_THRESHOLDS_CONFIGURED != 0U)
    return (s_spo2_wear_state == TASK_SPO2_WEAR_WORN) ? 1U : 0U;
#else
    /* 阈值未标定时，反射值只用于观察，不阻断生成值发布。 */
    (void)s_spo2_wear_state;
    return 1U;
#endif
}

static int32_t Task_Spo2FillGeneratedOutput(Spo2State_t *sample)
{
    if (sample == NULL)
    {
        return -1;
    }

    memset(sample, 0, sizeof(*sample));

    if (Task_Spo2GeneratedOutputAllowed() == 0U)
    {
        /* 摘下后只发布一次无效数据，清除界面上残留的旧读数。 */
        if (s_spo2_generated_initialized != 0U)
        {
            s_spo2_generated_initialized = 0U;
            s_spo2_generated_hr = 0U;
            s_spo2_generated_value = 0U;
            Task_Spo2DebugLog("G,0,0");
            return 1;
        }
        return 0;
    }

    if (s_spo2_generated_initialized == 0U)
    {
        s_spo2_generated_hr = (uint16_t)(TASK_SPO2_MOCK_HR_MIN +
                               (Task_Spo2MockRandom() %
                                (TASK_SPO2_MOCK_HR_MAX - TASK_SPO2_MOCK_HR_MIN + 1U)));
        s_spo2_generated_value = (uint8_t)(TASK_SPO2_MOCK_VALUE_MIN +
                                  (Task_Spo2MockRandom() %
                                   (TASK_SPO2_MOCK_VALUE_MAX - TASK_SPO2_MOCK_VALUE_MIN + 1U)));
        s_spo2_generated_initialized = 1U;
    }
    else
    {
        s_spo2_generated_hr = Task_Spo2MockStableStep(s_spo2_generated_hr,
                                                       TASK_SPO2_MOCK_HR_MIN,
                                                       TASK_SPO2_MOCK_HR_MAX);
        s_spo2_generated_value = (uint8_t)Task_Spo2MockStableStep(s_spo2_generated_value,
                                                                  TASK_SPO2_MOCK_VALUE_MIN,
                                                                  TASK_SPO2_MOCK_VALUE_MAX);
    }

    sample->heart_rate_bpm = s_spo2_generated_hr;
    sample->spo2_percent = s_spo2_generated_value;
    sample->heart_rate_valid = 1U;
    sample->spo2_valid = 1U;
    Task_Spo2DebugLog("G,%u,%u",
                      (unsigned int)s_spo2_generated_hr,
                      (unsigned int)s_spo2_generated_value);
    return 1;
}
#endif

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
static void Task_Spo2FillRawOutput(Spo2State_t *sample,
                                    int32_t spo2,
                                    int8_t spo2_valid,
                                    int32_t heart_rate,
                                    int8_t hr_valid)
{
    uint16_t filtered_hr = 0U;

    memset(sample, 0, sizeof(*sample));

    if ((s_spo2_wear_state == TASK_SPO2_WEAR_WORN) &&
        (spo2_valid == 1) && (spo2 >= 0) && (spo2 <= 255))
    {
        sample->spo2_percent = (uint8_t)spo2;
        sample->spo2_valid = 1U;
    }

    if (s_spo2_wear_state == TASK_SPO2_WEAR_WORN)
    {
        if (Task_Spo2FilterHeartRate(heart_rate, hr_valid, &filtered_hr) != 0U)
        {
            sample->heart_rate_bpm = filtered_hr;
            sample->heart_rate_valid = 1U;
        }
    }
    else
    {
        memset(&s_spo2_hr_filter, 0, sizeof(s_spo2_hr_filter));
    }

    sample->perfusion_permille = Task_Spo2EstimatePerfusion();
}
#endif

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
#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE == 0U)
    int32_t spo2;
    int8_t spo2_valid;
    int32_t heart_rate;
    int8_t hr_valid;
#endif

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

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U)
    /* 生成心率/血氧由任务主循环定时发布，这里只负责采集反射值窗口。 */
    return 0;
#else
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
    /* P,心率,心率有效,血氧,血氧有效；用于确认算法与佩戴判断的关系。 */
    Task_Spo2DebugLog("P,%ld,%d,%ld,%d",
                      (long)heart_rate,
                      (int)hr_valid,
                      (long)spo2,
                      (int)spo2_valid);
    /* 原始算法结果经过范围和连续性确认后再发布到全局状态。 */
    Task_Spo2FillRawOutput(sample, spo2, spo2_valid, heart_rate, hr_valid);
    Task_Spo2DebugLog("H,%u,%u",
                      (unsigned int)sample->heart_rate_bpm,
                      (unsigned int)sample->heart_rate_valid);
    return 1;
#endif
}

int32_t Task_Spo2InitHardware(void)
{
#if (TASK_SPO2_MOCK_ENABLE != 0U)
    /* 模拟显示模式不访问 MAX30102，关闭开关后恢复真实硬件初始化。 */
    return 0;
#else
    int32_t init_result;

    /* 初始化阶段也走 I2C 互斥锁，保持和运行态访问规则一致。 */
    if (osMutexAcquire(g_i2cBusMutex, osWaitForever) != osOK)
    {
        Task_Spo2DebugLog("S,I,M");
        return -1;
    }

    init_result = App_Spo2HardwareInit();
    if (init_result != 0)
    {
        osMutexRelease(g_i2cBusMutex);
        Task_Spo2DebugLog("S,I,%ld", (long)init_result);
        return -2;
    }

    osMutexRelease(g_i2cBusMutex);
    Task_Spo2DebugLog("S,I,0");
    return 0;
#endif
}

void Task_Spo2Entry(void *argument)
{
#if (TASK_SPO2_MOCK_ENABLE != 0U)
    Spo2State_t spo2;
    uint32_t next_tick;

    (void)argument;
    memset(&spo2, 0, sizeof(spo2));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    next_tick = osKernelGetTickCount();
    s_spo2_mock_random_state ^= next_tick;
    spo2.heart_rate_bpm = (uint16_t)(TASK_SPO2_MOCK_HR_MIN +
                            (Task_Spo2MockRandom() %
                             (TASK_SPO2_MOCK_HR_MAX - TASK_SPO2_MOCK_HR_MIN + 1U)));
    spo2.spo2_percent = (uint8_t)(TASK_SPO2_MOCK_VALUE_MIN +
                         (Task_Spo2MockRandom() %
                          (TASK_SPO2_MOCK_VALUE_MAX - TASK_SPO2_MOCK_VALUE_MIN + 1U)));
    spo2.heart_rate_valid = 1U;
    spo2.spo2_valid = 1U;

    for (;;)
    {
        spo2.update_count++;
        App_StateSetSpo2(&spo2);
        osEventFlagsSet(g_sysEventFlags, SYS_EVT_SPO2_UPDATED);

        next_tick += TASK_SPO2_MOCK_PERIOD_MS;
        osDelayUntil(next_tick);
        spo2.heart_rate_bpm = Task_Spo2MockStableStep(spo2.heart_rate_bpm,
                                                       TASK_SPO2_MOCK_HR_MIN,
                                                       TASK_SPO2_MOCK_HR_MAX);
        spo2.spo2_percent = (uint8_t)Task_Spo2MockStableStep(spo2.spo2_percent,
                                                             TASK_SPO2_MOCK_VALUE_MIN,
                                                             TASK_SPO2_MOCK_VALUE_MAX);
    }
#else
    Spo2State_t spo2;
    uint32_t next_tick;
    int32_t last_read_error;
#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U)
    uint32_t next_output_tick;
#endif

    (void)argument;
    memset(&spo2, 0, sizeof(spo2));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();
    last_read_error = 0;
#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U)
    next_output_tick = next_tick;
#endif

    for (;;)
    {
        Spo2State_t sample;
        int32_t result = 0;
#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U)
        uint32_t now_tick;
#endif

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

        /* 只在错误类型变化或通信恢复时输出，避免故障时刷满日志队列。 */
        if (result < 0)
        {
            if (result != last_read_error)
            {
                Task_Spo2DebugLog("S,E,%ld", (long)result);
                last_read_error = result;
            }
        }
        else if (last_read_error != 0)
        {
            Task_Spo2DebugLog("S,R");
            last_read_error = 0;
        }

#if (TASK_SPO2_GENERATED_OUTPUT_ENABLE != 0U)
        now_tick = osKernelGetTickCount();
        if ((int32_t)(now_tick - next_output_tick) >= 0)
        {
            int32_t output_result;

            output_result = Task_Spo2FillGeneratedOutput(&sample);
            if (output_result > 0)
            {
                sample.update_count = spo2.update_count + 1U;
                sample.error_count = spo2.error_count;
                spo2 = sample;
                App_StateSetSpo2(&spo2);
                osEventFlagsSet(g_sysEventFlags, SYS_EVT_SPO2_UPDATED);
            }
            next_output_tick += TASK_SPO2_MOCK_PERIOD_MS;
            if ((int32_t)(now_tick - next_output_tick) >= 0)
            {
                next_output_tick = now_tick + TASK_SPO2_MOCK_PERIOD_MS;
            }
        }
        if (result < 0)
        {
            spo2.error_count++;
            App_StateSetSpo2(&spo2);
        }
#else
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
#endif

        next_tick += APP_SPO2_PERIOD_MS;
        osDelayUntil(next_tick);
    }
#endif
}
