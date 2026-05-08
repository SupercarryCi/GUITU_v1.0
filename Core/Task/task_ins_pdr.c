#include "task_ins_pdr.h"
#include "app_event.h"
#include "app_state.h"
#include "app_rtos.h"
#include "cmsis_os.h"
#include "pdr_ci.h"
#include "pedestrian_frame_transform.h"
#include "pedestrian_inertial_nav.h"

#include <math.h>

#define INS_PDR_DT_S 0.005f   /* 陀螺仪频率200Hz */

static PdrStepDetector s_pdr_det;
static pin_context_t s_pin_ctx;
static pin_vec3f_t s_last_ins_position_m;
static uint8_t s_has_last_ins_position = 0U;


void Task_Ins_Pdr_Entry(void *argument)
{
    GyroState_t gyro;
    PdrStepOutput pdr_out;
    (void)argument;
    pin_config_t pin_cfg;

    pft_input_t pft_in;
    pft_output_t pft_out;

    pin_input_t pin_in;
    pin_state_t pin_state;

    NavState_t nav;

    pdr_step_init(&s_pdr_det);

    PIN_DefaultConfig(&pin_cfg);
    PIN_Init(&s_pin_ctx, &pin_cfg);//静止检测,zupt等参数在这

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);

    for (;;)
    {
        osEventFlagsWait(g_sysEventFlags, SYS_EVT_GYRO_UPDATED,
                         osFlagsWaitAny,osWaitForever);
        App_StateGetGyro(&gyro);

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

            pft_vec3f_t ins_delta_m;

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

            /*两种导航算法切换逻辑:纯角度...*/
            pft_vec3f_t selected_delta_m = {0};
            uint8_t use_pdr = 0U;

            if ((fabsf(gyro.frame.angle_deg[0]) < 30.0f) &&
                (fabsf(gyro.frame.angle_deg[1]) < 30.0f))
            {
                if (PFT_StepDistanceToPedDelta(pdr_out.delta_distance_m,
                                   gyro.frame.angle_deg[2],
                                   &selected_delta_m) == PFT_OK)
                {
                    use_pdr = 1U;
                }
            }

            if (use_pdr == 0U)
            {
                selected_delta_m = ins_delta_m;
            }

            App_StateGetNav(&nav);

            nav.update_count++;

            nav.data.position_m[0] += selected_delta_m.x;
            nav.data.position_m[1] += selected_delta_m.y;
            nav.data.position_m[2] += selected_delta_m.z;

            nav.data.velocity_mps[0] = pin_state.velocity_ped_mps.x;
            nav.data.velocity_mps[1] = pin_state.velocity_ped_mps.y;
            nav.data.velocity_mps[2] = pin_state.velocity_ped_mps.z;

            nav.data.attitude_rad[0] = gyro.frame.angle_deg[0] * 0.0174532925f;
            nav.data.attitude_rad[1] = gyro.frame.angle_deg[1] * 0.0174532925f;
            nav.data.attitude_rad[2] = gyro.frame.angle_deg[2] * 0.0174532925f;

            nav.data.YAW_deg = gyro.frame.angle_deg[2];

            App_StateSetNav(&nav);
            osEventFlagsSet(g_sysEventFlags, SYS_EVT_NAV_UPDATED);
        }
    }
}
