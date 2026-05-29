#include "task_ins_pdr.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_state.h"
#include "app_rtos.h"
#include "cmsis_os.h"
#include "pdr_ci.h"
#include "pedestrian_frame_transform.h"
#include "pedestrian_inertial_nav.h"

#include <math.h>

#define INS_PDR_DT_S 0.005f   /* 陀螺仪频率200Hz */
#define INS_PDR_ENABLE_LOGIC 1U /* 0: 暂时保留接口，禁用未完成的 INS/PDR 逻辑 */

#if (INS_PDR_ENABLE_LOGIC != 0U)
static PdrStepDetector s_pdr_det;
static pin_context_t s_pin_ctx;
static pin_vec3f_t s_last_ins_position_m;
static uint8_t s_has_last_ins_position = 0U;
#endif


void Task_Ins_Pdr_Entry(void *argument)
{
    GyroState_t gyro;
#if (INS_PDR_ENABLE_LOGIC != 0U)
    PdrStepOutput pdr_out;
    pin_config_t pin_cfg;

    pft_input_t pft_in;
    pft_output_t pft_out;

    pin_input_t pin_in;
    pin_state_t pin_state;
    pft_vec3f_t ins_delta_m;
    pft_vec3f_t selected_delta_m;
    NavDeltaMsg_t nav_delta;
    uint8_t use_pdr;
#endif

    (void)argument;

#if (INS_PDR_ENABLE_LOGIC != 0U)
    pdr_step_init(&s_pdr_det);

    PIN_DefaultConfig(&pin_cfg);
    PIN_Init(&s_pin_ctx, &pin_cfg);//静止检测,zupt等参数在这
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
            /*坐标变换*/
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

            /*PDR更新,不要问为什么在这里*/
            pdr_out = pdr_step_update(&s_pdr_det,
                gyro.frame.acc_raw.x,
                gyro.frame.acc_raw.y,
                gyro.frame.acc_raw.z,
                gyro.frame.gyro_raw.x,
                gyro.frame.gyro_raw.y,
                gyro.frame.gyro_raw.z);

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
            selected_delta_m.x = 0.0f;
            selected_delta_m.y = 0.0f;
            selected_delta_m.z = 0.0f;
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
                                                     gyro.frame.angle_deg[2],
                                                     &selected_delta_m);
                }
            }
            else
            {
                selected_delta_m = ins_delta_m;
            }

            nav_delta.delta_m[0] = selected_delta_m.x;
            nav_delta.delta_m[1] = selected_delta_m.y;
            nav_delta.delta_m[2] = selected_delta_m.z;

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

            nav_delta.attitude_rad[0] = gyro.frame.angle_deg[0] * 0.0174532925f;
            nav_delta.attitude_rad[1] = gyro.frame.angle_deg[1] * 0.0174532925f;
            nav_delta.attitude_rad[2] = gyro.frame.angle_deg[2] * 0.0174532925f;
            nav_delta.yaw_deg = gyro.frame.angle_deg[2];

            if (osMessageQueuePut(g_navDeltaQueue, &nav_delta, 0U, 0U) == osOK)
            {
                (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_NAV_DELTA_READY);
            }
        }
#else
        (void)gyro; /* 当前先不启用 INS/PDR 业务计算，避免影响其它模块测试。 */
#endif
    }
}
