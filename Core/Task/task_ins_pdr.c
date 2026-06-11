#include "task_ins_pdr.h"
#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_state.h"
#include "app_rtos.h"
#include "cmsis_os.h"
#include "pdr_ci.h"
#include "pedestrian_frame_transform.h"
#include "pedestrian_inertial_nav.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#define INS_PDR_DT_S 0.005f                     /* 陀螺仪频率200Hz */
#define INS_PDR_ENABLE_LOGIC 1U                 /* 0: 暂时保留接口，禁用未完成的 INS/PDR 逻辑 */
#define INS_PDR_ENABLE_INS 0U                   /* 0: 先禁用INS，只输出PDR判步位移 */

#define INS_PDR_ENABLE_HEADING_DIAG 1U          /* 航向诊断日志*/
#define INS_PDR_HEADING_DIAG_PERIOD_MS 200U     
#define INS_PDR_DIAG_LINE_LEN 64U

#define INS_PDR_HEADING_UPDATE_PERIOD_MS 50U    /* 未判步时也定期刷新行人航向，避免LoRa角度停住。 */
#define INS_PDR_HEADING_OFFSET_DEG (-18.50f)    /* 手臂佩戴偏差：例如139.00度修正到120.50度 */

#define INS_PDR_ENABLE_HEADING_SNAP 0U          /* 临时测试功能：1=四向吸附，0=关闭 */
#define INS_PDR_HEADING_SNAP_DEG 45.0f          /* 小于该偏差时吸附到0/90/-90/-180度 */
#define INS_PDR_HEADING_SNAP_CONFIRM_COUNT 2U   /* 新吸附方向连续出现2次才切换 */

#if (INS_PDR_ENABLE_LOGIC != 0U)
static PdrStepDetector s_pdr_det;
#if (INS_PDR_ENABLE_INS != 0U)
static pin_context_t s_pin_ctx;
static pin_vec3f_t s_last_ins_position_m;
static uint8_t s_has_last_ins_position = 0U;
#endif

static float task_ins_pdr_normalize_deg(float value)
{
    while (value > 180.0f)
    {
        value -= 360.0f;
    }
    while (value <= -180.0f)
    {
        value += 360.0f;
    }

    return value;
}

static float task_ins_pdr_snap_heading_deg(float heading_deg)
{
#if (INS_PDR_ENABLE_HEADING_SNAP != 0U)
    static float stable_heading_deg;
    static float pending_heading_deg;
    static uint8_t has_stable_heading = 0U;
    static uint8_t pending_count = 0U;
    const float heading = task_ins_pdr_normalize_deg(heading_deg);
    float candidate_heading_deg;

    if (fabsf(heading) < INS_PDR_HEADING_SNAP_DEG)
    {
        candidate_heading_deg = 0.0f;
    }
    else if (fabsf(heading - 90.0f) < INS_PDR_HEADING_SNAP_DEG)
    {
        candidate_heading_deg = 90.0f;
    }
    else if (fabsf(heading + 90.0f) < INS_PDR_HEADING_SNAP_DEG)
    {
        candidate_heading_deg = -90.0f;
    }
    else if ((180.0f - fabsf(heading)) < INS_PDR_HEADING_SNAP_DEG)
    {
        candidate_heading_deg = -180.0f;
    }
    else
    {
        candidate_heading_deg = heading;
    }

    if (has_stable_heading == 0U)
    {
        stable_heading_deg = candidate_heading_deg;
        has_stable_heading = 1U;
        pending_count = 0U;
        return stable_heading_deg;
    }

    if (candidate_heading_deg == stable_heading_deg)
    {
        pending_count = 0U;
        return stable_heading_deg;
    }

    if ((pending_count == 0U) || (candidate_heading_deg != pending_heading_deg))
    {
        pending_heading_deg = candidate_heading_deg;
        pending_count = 1U;
        return stable_heading_deg;
    }

    pending_count++;
    if (pending_count >= INS_PDR_HEADING_SNAP_CONFIRM_COUNT)
    {
        stable_heading_deg = candidate_heading_deg;
        pending_count = 0U;
    }

    return stable_heading_deg;
#else
    return heading_deg;
#endif
}

static int32_t task_ins_pdr_float_to_centi(float value)
{
    if (value >= 0.0f)
    {
        return (int32_t)((value * 100.0f) + 0.5f);
    }

    return (int32_t)((value * 100.0f) - 0.5f);
}

#if (INS_PDR_ENABLE_HEADING_DIAG != 0U)
static void task_ins_pdr_diag_uart_log(const char *fmt, ...)
{
    char line[INS_PDR_DIAG_LINE_LEN];
    va_list args;
    int len;

    if (fmt == NULL)
    {
        return;
    }

    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }
    if (len > ((int)sizeof(line) - 3))
    {
        len = (int)sizeof(line) - 3;
    }

    line[len++] = '\r';
    line[len++] = '\n';
    line[len] = '\0';

    /*
     * 本诊断临时直写调试串口，绕过 DebugTask 队列，便于排查日志链路是否被阻塞。
     */
    if (g_debugUartMutex != NULL)
    {
        if (osMutexAcquire(g_debugUartMutex, 10U) != osOK)
        {
            return;
        }
        (void)HAL_UART_Transmit(&APP_DEBUG_UART_HANDLE,
                                (uint8_t *)line,
                                (uint16_t)len,
                                20U);
        osMutexRelease(g_debugUartMutex);
    }
    else
    {
        (void)HAL_UART_Transmit(&APP_DEBUG_UART_HANDLE,
                                (uint8_t *)line,
                                (uint16_t)len,
                                20U);
    }
}

static void task_ins_pdr_log_heading_diag(const GyroState_t *gyro,
                                          const pft_quatf_t *quat,
                                          float dcm[3][3],
                                          float current_heading_deg)
{
    (void)gyro;
    (void)quat;
    (void)dcm;

    /* PH单位为0.01度，来源是当前实际用于PDR/LoRa的行人航向角。 */
    task_ins_pdr_diag_uart_log("PH:%ld",
                               (long)task_ins_pdr_float_to_centi(current_heading_deg));
}
#endif
#endif


void Task_Ins_Pdr_Entry(void *argument)
{
    GyroState_t gyro;
#if (INS_PDR_ENABLE_LOGIC != 0U)
    PdrStepOutput pdr_out;
    pft_input_t pft_in;
    pft_output_t pft_out;
    float ped_heading_deg;
#if (INS_PDR_ENABLE_INS != 0U)
    pin_config_t pin_cfg;

    pin_input_t pin_in;
    pin_state_t pin_state;
    pft_vec3f_t ins_delta_m;
    uint8_t use_pdr;
#endif
    pft_vec3f_t selected_delta_m;
    NavDeltaMsg_t nav_delta;
    uint32_t now_tick;
    uint32_t next_heading_update_tick;
#if (INS_PDR_ENABLE_HEADING_DIAG != 0U)
    uint32_t next_heading_diag_tick;
#endif
    uint8_t heading_update_due;
    uint8_t send_nav_delta;
#endif

    (void)argument;

#if (INS_PDR_ENABLE_LOGIC != 0U)
    pdr_step_init(&s_pdr_det);

#if (INS_PDR_ENABLE_INS != 0U)
    PIN_DefaultConfig(&pin_cfg);
    PIN_Init(&s_pin_ctx, &pin_cfg);//静止检测,zupt等参数在这
#endif
    next_heading_update_tick = 0U;
#if (INS_PDR_ENABLE_HEADING_DIAG != 0U)
    next_heading_diag_tick = 0U;
#endif
#endif

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        osEventFlagsWait(g_sysEventFlags, SYS_EVT_GYRO_UPDATED,
                         osFlagsWaitAny,osWaitForever);
        App_StateGetGyro(&gyro);

#if (INS_PDR_ENABLE_LOGIC != 0U)
        if (gyro.last_parse_result == 0)
        {
            /*坐标变换：PDR步进方向和INS都使用行人坐标系结果。 */
            pft_in.accel_body_mps2.x = gyro.frame.accel_mps2[0];
            pft_in.accel_body_mps2.y = gyro.frame.accel_mps2[1];
            pft_in.accel_body_mps2.z = gyro.frame.accel_mps2[2];

            pft_in.gyro_body_rps.x = gyro.frame.gyro_rad_s[0];
            pft_in.gyro_body_rps.y = gyro.frame.gyro_rad_s[1];
            pft_in.gyro_body_rps.z = gyro.frame.gyro_rad_s[2];

            pft_in.quat.w = gyro.frame.quat[0];
            pft_in.quat.x = gyro.frame.quat[1];
            pft_in.quat.y = gyro.frame.quat[2];
            pft_in.quat.z = gyro.frame.quat[3];

            pft_in.use_quaternion = 1U;
            pft_in.remove_gravity = 1U;
            pft_in.gravity_mps2 = 9.80665f;

            if (PFT_BodyAccelToPedestrian(&pft_in, &pft_out) != PFT_OK)
            {
                continue;
            }

            /* 当前调试阶段使用WIT原始yaw，并叠加安装偏移作为PDR航向角。 */
            ped_heading_deg = task_ins_pdr_snap_heading_deg(
                task_ins_pdr_normalize_deg(gyro.frame.angle_deg[2] +
                                           INS_PDR_HEADING_OFFSET_DEG));
            now_tick = osKernelGetTickCount();
            heading_update_due = ((int32_t)(now_tick - next_heading_update_tick) >= 0) ? 1U : 0U;
#if (INS_PDR_ENABLE_HEADING_DIAG != 0U)
            if ((int32_t)(now_tick - next_heading_diag_tick) >= 0)
            {
                task_ins_pdr_log_heading_diag(&gyro,
                                              &pft_in.quat,
                                              pft_out.dcm_body_to_ped,
                                              ped_heading_deg);
                next_heading_diag_tick = now_tick + INS_PDR_HEADING_DIAG_PERIOD_MS;
            }
#endif

            /*PDR更新,不要问为什么在这里*/
            pdr_out = pdr_step_update(&s_pdr_det,
                gyro.frame.acc_raw.x,
                gyro.frame.acc_raw.y,
                gyro.frame.acc_raw.z,
                gyro.frame.gyro_raw.x,
                gyro.frame.gyro_raw.y,
                gyro.frame.gyro_raw.z);

            selected_delta_m.x = 0.0f;
            selected_delta_m.y = 0.0f;
            selected_delta_m.z = 0.0f;
            send_nav_delta = 0U;

#if (INS_PDR_ENABLE_INS != 0U)
            /*惯性导航更新*/
            pin_in.dt_s = INS_PDR_DT_S;
            pin_in.linear_accel_ped_mps2.x = pft_out.linear_accel_ped_mps2.x;
            pin_in.linear_accel_ped_mps2.y = pft_out.linear_accel_ped_mps2.y;
            pin_in.linear_accel_ped_mps2.z = pft_out.linear_accel_ped_mps2.z;

            pin_in.gyro_body_rps.x = gyro.frame.gyro_rad_s[0];
            pin_in.gyro_body_rps.y = gyro.frame.gyro_rad_s[1];
            pin_in.gyro_body_rps.z = gyro.frame.gyro_rad_s[2];

            pin_in.zero_velocity = 0U;

            if (PIN_Update(&s_pin_ctx, &pin_in, &pin_state) != PIN_OK)
            {
                continue;
            }

            /*计算ins增量*/
            if (s_has_last_ins_position == 0U)
            {
                s_last_ins_position_m = pin_state.position_ped_m;
                s_has_last_ins_position = 1U;
                ins_delta_m.x = 0.0f;
                ins_delta_m.y = 0.0f;
                ins_delta_m.z = 0.0f;
            }
            else
            {
                ins_delta_m.x = pin_state.position_ped_m.x - s_last_ins_position_m.x;
                ins_delta_m.y = pin_state.position_ped_m.y - s_last_ins_position_m.y;
                ins_delta_m.z = pin_state.position_ped_m.z - s_last_ins_position_m.z;

                s_last_ins_position_m = pin_state.position_ped_m;
            }

            /*两种导航算法切换逻辑*/
            use_pdr = 1U;

            if ((fabsf(gyro.frame.angle_deg[0]) < 30.0f) &&
                (fabsf(gyro.frame.angle_deg[1]) < 30.0f))//手臂横过来我就ins，嘎嘎
            {
                use_pdr = 0U;
            }

            if (use_pdr != 0U)
            {
                /* PDR只在判到一步时输出位移，未判步时保持0增量，避免和INS增量重复累计。 */
                if (pdr_out.step_detected != 0U)
                {
                    (void)PFT_StepDistanceToPedDelta(pdr_out.delta_distance_m,
                                                     ped_heading_deg,
                                                     &selected_delta_m);
                    send_nav_delta = 1U;
                }
                else if (heading_update_due != 0U)
                {
                    send_nav_delta = 1U;
                }
            }
            else
            {
                selected_delta_m = ins_delta_m;
                send_nav_delta = 1U;
            }
#else
            /* INS暂时禁用：PDR没有判到步时不向Control队列投递零位移包。按理上正常投递不会有问题，但他就是出现了 */
            if (pdr_out.step_detected != 0U)
            {
                (void)PFT_StepDistanceToPedDelta(pdr_out.delta_distance_m,
                                                 ped_heading_deg,
                                                 &selected_delta_m);
                send_nav_delta = 1U;
            }
            else if (heading_update_due != 0U)
            {
                send_nav_delta = 1U;
            }
#endif

            if (send_nav_delta == 0U)
            {
                continue;
            }

            nav_delta.delta_m[0] = selected_delta_m.x;
            nav_delta.delta_m[1] = selected_delta_m.y;
            nav_delta.delta_m[2] = selected_delta_m.z;

#if (INS_PDR_ENABLE_INS != 0U)
            if (use_pdr != 0U)
            {
                /* PDR当前只输出步进位移，速度没有可靠来源，避免混入INS速度。 */
                nav_delta.velocity_mps[0] = 0.0f;
                nav_delta.velocity_mps[1] = 0.0f;
                nav_delta.velocity_mps[2] = 0.0f;
            }
            else
            {
                nav_delta.velocity_mps[0] = pin_state.velocity_ped_mps.x;
                nav_delta.velocity_mps[1] = pin_state.velocity_ped_mps.y;
                nav_delta.velocity_mps[2] = pin_state.velocity_ped_mps.z;
            }
#else
            /* PDR当前只输出步进位移，速度没有可靠来源。 */
            nav_delta.velocity_mps[0] = 0.0f;
            nav_delta.velocity_mps[1] = 0.0f;
            nav_delta.velocity_mps[2] = 0.0f;
#endif

            nav_delta.attitude_rad[0] = gyro.frame.angle_deg[0] * 0.0174532925f;
            nav_delta.attitude_rad[1] = gyro.frame.angle_deg[1] * 0.0174532925f;
            nav_delta.attitude_rad[2] = ped_heading_deg * 0.0174532925f;
            nav_delta.yaw_deg = ped_heading_deg;

            if (osMessageQueuePut(g_navDeltaQueue, &nav_delta, 0U, 0U) == osOK)
            {
                next_heading_update_tick = now_tick + INS_PDR_HEADING_UPDATE_PERIOD_MS;
                (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_NAV_DELTA_READY);
            }
        }
#else
        (void)gyro; /* 当前先不启用 INS/PDR 业务计算，避免影响其它模块测试。 */
#endif
    }
}
