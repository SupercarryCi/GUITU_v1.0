#include "task_ui.h"

#include "app_config.h"
#include "app_buzzer.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "tft_port_stm32_hal.h"
#include "task_debug.h"
#include "../../compass_icon_c/compass_icons.h"
#include "../../ui_dashboard_q565.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define icon_data g_iconNgasData
#define Icon IconNgas_t
#define my_icon g_iconNgas
#include "../../天然气.h"
#undef icon_data
#undef Icon
#undef my_icon

#define icon_data g_iconLpgData
#define Icon IconLpg_t
#define my_icon g_iconLpg
#include "../../液化气.h"
#undef icon_data
#undef Icon
#undef my_icon

#define icon_data g_iconHrData
#define Icon IconHr_t
#define my_icon g_iconHr
#include "../../心率.h"
#undef icon_data
#undef Icon
#undef my_icon

#define icon_data g_iconSpo2Data
#define Icon IconSpo2_t
#define my_icon g_iconSpo2
#include "../../血氧.h"
#undef icon_data
#undef Icon
#undef my_icon

#define UI_RGB565(r, g, b)    (uint16_t)((((uint16_t)(r) & 0xF8U) << 8) | (((uint16_t)(g) & 0xFCU) << 3) | ((uint16_t)(b) >> 3))

#define UI_COLOR_BG           UI_RGB565(7U, 12U, 18U)      /*背景色，深蓝黑*/
#define UI_COLOR_PANEL        UI_RGB565(20U, 29U, 39U)     /*卡片底色，深蓝灰*/
#define UI_COLOR_PANEL_2      UI_RGB565(28U, 39U, 52U)     /*进度条轨道色，略浅深蓝灰*/
#define UI_COLOR_TEXT         ILI9488_COLOR_WHITE
#define UI_COLOR_MUTED        UI_RGB565(132U, 148U, 164U)  /*辅助文字色，灰蓝*/
#define UI_COLOR_NGAS         UI_RGB565(250U, 204U, 21U)   /*天然气卡片色，黄*/
#define UI_COLOR_LPG          UI_RGB565(56U, 189U, 248U)   /*液化气卡片色，天蓝*/
#define UI_COLOR_HR           UI_RGB565(248U, 113U, 113U)  /*心率卡片色，红*/
#define UI_COLOR_SPO2         UI_RGB565(45U, 212U, 191U)   /*血氧卡片色，绿*/
#define UI_COLOR_ACTION       UI_RGB565(59U, 130U, 246U)   /*操作按钮色，蓝*/

#define UI_LEFT_X             8U     //左侧面板起始x坐标
#define UI_LEFT_W             270U   //左侧面板宽度
#define UI_CARD_H             72U    //状态卡片高度
#define UI_CARD_GAP           6U     //状态卡片间隔
#define UI_RIGHT_X            288U   //右侧面板起始x坐标
#define UI_RIGHT_W            184U   //右侧面板宽度
#define UI_QUICK_CX           380U   //快速操作圆中心x坐标
#define UI_QUICK_CY           86U    //快速操作圆中心y坐标
#define UI_QUICK_R            66U    //快速操作圆半径
#define UI_HOME_BTN_Y         234U   //主页按钮y坐标
#define UI_HOME_BTN_H         76U    //主页按钮高度
#define UI_GAS_BAR_MAX_PPM    50000U //气体浓度条最大值
#define UI_PI                 3.14159265358979323846f
#define UI_RAD_TO_DEG          (180.0f / UI_PI)
#define UI_HAND_RAISED_ROLL_TARGET_DEG 90.0f
#define UI_HAND_RAISED_ROLL_TOL_DEG    30.0f
#define UI_HAND_RAISED_PITCH_MAX_DEG   30.0f
#define UI_RETURN_ENABLE_HAND_RAISED_FLIP 1U    /* 仅抬手时反向显示返航箭头。 */
#define UI_STATUS_ICON_SIZE   48U
#define UI_STATUS_VALUE_X_OFF 70U    //状态栏数值x偏移
#define UI_STATUS_VALUE_Y_OFF 36U    //状态栏数值y偏移
#define UI_STATUS_VALUE_W     128U   //状态栏数值刷新宽度
#define UI_STATUS_VALUE_H     18U    //状态栏数值刷新高度
#define UI_BASE_TEXT          "BASE" //BASE圆形按钮标题
#define UI_BASE_TEXT_X_OFF    (-20)
#define UI_BASE_TEXT_Y_OFF    (-30)
#define UI_BASE_DIST_X_OFF    (-10)
#define UI_BASE_DIST_Y_OFF    (-4)
#define UI_BASE_SIG_X_OFF     (-20)
#define UI_BASE_SIG_Y_OFF     24
#define UI_HOME_TEXT_X_OFF    22U    //HOME文字x偏移
#define UI_HOME_TEXT_Y_OFF    28U    //HOME文字y偏移
#define UI_HOME_ICON_X_OFF    142U   //HOME图标x偏移
#define UI_HOME_ICON_Y_OFF    50U    //HOME图标y偏移
#define UI_RETURN_ICON_X      160U
#define UI_RETURN_ICON_Y      42U
#define UI_RETURN_ICON_W      COMPASS_ICON_WIDTH
#define UI_RETURN_ICON_H      COMPASS_ICON_HEIGHT
#define UI_RETURN_TEXT_X      220U
#define UI_RETURN_TEXT_Y      230U
#define UI_RETURN_TEXT_W      210U
#define UI_RETURN_TEXT_H      48U
#define UI_RETURN_FRAME_NONE  0xFFU
#define UI_RETURN_TURN_UNSET  2
#define UI_RETURN_LOCAL_DEMO_DISTANCE_M 10.0f
#define UI_RETURN_LOCAL_DEMO_ARRIVE_M   1.0f

/* 临时 UI 卡顿定位开关，定位完成后可改为 0U。 */
#ifndef UI_DIAG_ENABLE
#define UI_DIAG_ENABLE        0U
#endif
#define UI_LCD_GAP_DELAY_MS   1U
#define UI_DASHBOARD_CHUNK_ROWS 8U
#define UI_DASHBOARD_CHUNK_PIXELS (UI_DASHBOARD_WIDTH * UI_DASHBOARD_CHUNK_ROWS)

#define UI_MAIN_TOP_VALUE_BG       0x0000U
#define UI_MAIN_TOP_TEXT_X_SCALE   2U
#define UI_MAIN_TOP_TEXT_Y_SCALE   2U
#define UI_MAIN_SIGNAL_VALUE_X     90U
#define UI_MAIN_SIGNAL_VALUE_Y     11U
#define UI_MAIN_SIGNAL_VALUE_W     90U
#define UI_MAIN_SIGNAL_VALUE_H     14U
#define UI_MAIN_CPU_VALUE_X        244U
#define UI_MAIN_CPU_VALUE_Y        11U
#define UI_MAIN_CPU_VALUE_W        40U
#define UI_MAIN_CPU_VALUE_H        14U
#define UI_MAIN_CPU_VALUE_BG       0x0000U
#define UI_MAIN_RAM_VALUE_X        416U
#define UI_MAIN_RAM_VALUE_Y        11U
#define UI_MAIN_RAM_VALUE_W        40U
#define UI_MAIN_RAM_VALUE_H        14U
#define UI_MAIN_RAM_VALUE_BG       0x0000U

#define UI_MAIN_CARD_ICON_X        18U
#define UI_MAIN_CARD_ICON_Y        12U
#define UI_MAIN_CARD_LABEL_X       78U
#define UI_MAIN_CARD_LABEL_Y       12U
#define UI_MAIN_CARD_UNIT_X        160U
#define UI_MAIN_CARD_UNIT_Y        40U
#define UI_MAIN_CARD_UNIT_X_SCALE  2U
#define UI_MAIN_CARD_UNIT_Y_SCALE  2U
#define UI_MAIN_CARD_VALUE_X_SCALE 3U
#define UI_MAIN_CARD_VALUE_Y_SCALE 3U
#define UI_MAIN_CARD_VALUE_X       78U
#define UI_MAIN_CARD_VALUE_Y       30U
#define UI_MAIN_CARD_VALUE_W       110U
#define UI_MAIN_CARD_VALUE_H       30U
#define UI_MAIN_DIGIT_WIDTH        15U
#define UI_MAIN_DIGIT_HEIGHT       21U
#define UI_MAIN_DIGIT_ADVANCE      18U
#define UI_MAIN_DIGIT_DASH_INDEX   10U
#define UI_MAIN_DIGIT_M_INDEX      11U
#define UI_MAIN_DIGIT_GLYPH_COUNT  12U
#define UI_MAIN_CARD0_VALUE_BG     0x0860U
#define UI_MAIN_CARD1_VALUE_BG     0x0041U
#define UI_MAIN_CARD2_VALUE_BG     0x0021U
#define UI_MAIN_CARD3_VALUE_BG     0x0042U

#define UI_MAIN_BASE_INFO_BG       0x010CU
#define UI_MAIN_BASE_DIST_X_SCALE  3U
#define UI_MAIN_BASE_DIST_Y_SCALE  3U
#define UI_MAIN_BASE_DIST_X        258U
#define UI_MAIN_BASE_DIST_Y        128U
#define UI_MAIN_BASE_DIST_W        128U
#define UI_MAIN_BASE_DIST_H        24U

#define UI_MAIN_BASE_X             228U
#define UI_MAIN_BASE_Y             39U
#define UI_MAIN_BASE_W             240U
#define UI_MAIN_BASE_H             138U

#define UI_MAIN_RETURN_X           228U
#define UI_MAIN_RETURN_Y           183U
#define UI_MAIN_RETURN_W           240U
#define UI_MAIN_RETURN_H           124U
#define UI_MAIN_RETURN_SPLIT_X     (UI_MAIN_RETURN_X + (UI_MAIN_RETURN_W / 2U))

#define UI_QUICK_BTN_X             28U
#define UI_QUICK_BTN_W             232U
#define UI_QUICK_BTN_H             54U
#define UI_QUICK_BTN1_Y            70U
#define UI_QUICK_BTN2_Y            136U
#define UI_QUICK_BTN3_Y            202U
#define UI_QUICK_TITLE_X           30U
#define UI_QUICK_TITLE_Y           20U
#define UI_QUICK_HZ_SCALE          2U
#define UI_QUICK_HZ_SPACING        4U

#define UI_LORA_POPUP_X            54U
#define UI_LORA_POPUP_Y            76U
#define UI_LORA_POPUP_W            372U
#define UI_LORA_POPUP_H            168U
#define UI_LORA_POPUP_CLOSE_W      28U
#define UI_LORA_POPUP_CLOSE_H      24U
#define UI_LORA_POPUP_CLOSE_X      (UI_LORA_POPUP_X + UI_LORA_POPUP_W - UI_LORA_POPUP_CLOSE_W - 10U)
#define UI_LORA_POPUP_CLOSE_Y      (UI_LORA_POPUP_Y + 10U)
#define UI_LORA_POPUP_TITLE_X      (UI_LORA_POPUP_X + 18U)
#define UI_LORA_POPUP_TITLE_Y      (UI_LORA_POPUP_Y + 16U)
#define UI_LORA_POPUP_TEXT_X       (UI_LORA_POPUP_X + 32U)
#define UI_LORA_POPUP_TEXT_Y       (UI_LORA_POPUP_Y + 78U)
#define UI_LORA_POPUP_HZ_TEXT_Y    (UI_LORA_POPUP_TEXT_Y + 2U)
#define UI_LORA_POPUP_TEXT_MAX     49U
#define UI_LORA_POPUP_LINE_CHARS   24U
#define UI_LORA_POPUP_ASCII_LINES  2U
#define UI_LORA_POPUP_HZ_MAX       6U
#define UI_LORA_POPUP_HZ_SCALE     2U
#define UI_LORA_POPUP_BG           UI_RGB565(13U, 20U, 28U)
#define UI_LORA_POPUP_BORDER       UI_RGB565(96U, 165U, 250U)

#define UI_MAIN_CPU_VALUE          23U
#define UI_MAIN_RAM_VALUE          58U

typedef enum
{
    UI_VIEW_MAIN = 0,  /*设计三个状态：主界面、快捷通信、返航引导*/
    UI_VIEW_QUICK,
    UI_VIEW_RETURN
} UiView_t;

typedef enum
{
    UI_TOUCH_NONE = 0,    /*触摸操作分类*/
    UI_TOUCH_QUICK,
    UI_TOUCH_RETURN_START,
    UI_TOUCH_RETURN_STOP,
    UI_TOUCH_QUICK_SEND,
    UI_TOUCH_QUICK_BACK
} UiTouchAction_t;

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint16_t color;
    uint16_t value_bg;
    const char *label;
    const char *unit;
    uint8_t icon;
} UiMainStatusLayout_t;

typedef enum
{
    UI_LORA_POPUP_NONE = 0,
    UI_LORA_POPUP_ASCII,
    UI_LORA_POPUP_HZ
} UiLoraPopupType_t;

typedef struct
{
    UiLoraPopupType_t type;
    uint8_t hz_count;
    uint8_t hz[UI_LORA_POPUP_HZ_MAX];
    char text[UI_LORA_POPUP_TEXT_MAX];
} UiLoraPopupContent_t;
static uint8_t s_touchPressed = 0U;//触摸状态量
static UiView_t s_view = UI_VIEW_MAIN;//视图状态量
static UiView_t s_lastRenderedView = (UiView_t)0xFFU;//上次渲染的视图状态量，初始值设置为无效值以确保首次渲染
static uint8_t s_mainValueValid = 0U;
static uint32_t s_lastStatusValue[4];
static uint8_t s_lastStatusDataValid[4];
static uint8_t s_lastReturnFrame = UI_RETURN_FRAME_NONE;
static uint32_t s_lastReturnDistanceM = 0xFFFFFFFFU;
static int8_t s_lastReturnTurn = UI_RETURN_TURN_UNSET;
static uint8_t s_returnLocalDemoActive = 0U;
static uint8_t s_returnLocalDemoArrived = 0U;
static float s_returnLocalDemoTargetX = 0.0f;
static float s_returnLocalDemoTargetY = 0.0f;
static float s_returnLocalDemoLastHeadingRad = 0.0f;
static uint16_t s_returnIconBuffer[COMPASS_ICON_BBOX_MAX_W * COMPASS_ICON_BBOX_MAX_H];
static uint16_t s_dashboardDecodeBuffer[UI_DASHBOARD_CHUNK_PIXELS];
static uint8_t s_mainBackgroundValid = 0U;
static uint8_t s_lastSignalValid = 0xFFU;
static int16_t s_lastSignalDbm = 0;
static uint32_t s_lastMainBaseDistanceM = 0xFFFFFFFFU;
static UiLoraPopupContent_t s_loraPopupContent;
static UiLoraPopupContent_t s_loraPopupPendingContent;
static uint8_t s_loraPopupActive = 0U;
static uint8_t s_loraPopupPendingValid = 0U;
static uint8_t s_loraPopupNeedRedraw = 0U;

static const UiMainStatusLayout_t s_mainStatusLayout[4] =
{
    {11U, 39U, 210U, 66U, UI_COLOR_NGAS,   UI_MAIN_CARD0_VALUE_BG, "NGAS", "PPM", 0U},
    {11U, 108U, 210U, 67U, UI_COLOR_SPO2,  UI_MAIN_CARD1_VALUE_BG, "LPG",  "PPM", 1U},
    {11U, 178U, 210U, 63U, UI_COLOR_HR,    UI_MAIN_CARD2_VALUE_BG, "HR",   "BPM", 2U},
    {11U, 244U, 210U, 63U, UI_COLOR_ACTION, UI_MAIN_CARD3_VALUE_BG, "SPO2", "%", 3U}
};

static void Ui_DiagLog(const char *fmt, ...)
{
#if (UI_DIAG_ENABLE != 0U)
    char line[96];
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
    if (len >= (int)sizeof(line))
    {
        line[sizeof(line) - 1U] = '\0';
    }

    /* UI诊断统一走 DebugTask，避免任务里直接抢调试串口。 */
    App_DebugLog("%s", line);
#else
    (void)fmt;
#endif
}
static void Ui_LcdGapDelay(void)
{
#if (UI_LCD_GAP_DELAY_MS != 0U)
    /* 给连续 LCD SPI 事务留出实际时间间隔，避免关闭日志后显示链路过紧。 */
    osDelay(UI_LCD_GAP_DELAY_MS);
#endif
}
static int32_t Ui_DashboardFlushBuffer(uint32_t pixel_count, uint32_t *next_y)
{
    uint32_t rows;

    if ((pixel_count == 0U) || (next_y == NULL))
    {
        return 0;
    }
    if ((pixel_count % UI_DASHBOARD_WIDTH) != 0U)
    {
        return -1;
    }

    rows = pixel_count / UI_DASHBOARD_WIDTH;
    if (ILI9488_DrawRGB565Image(&g_lcd,
                                0U,
                                (uint16_t)(*next_y),
                                (uint16_t)UI_DASHBOARD_WIDTH,
                                (uint16_t)rows,
                                s_dashboardDecodeBuffer) != 0)
    {
        return -1;
    }

    *next_y += rows;
    Ui_LcdGapDelay();
    return 0;
}

static int32_t Ui_DashboardPushPixel(uint16_t px, uint32_t *buffer_count, uint32_t *next_y)
{
    if ((buffer_count == NULL) || (next_y == NULL))
    {
        return -1;
    }

    s_dashboardDecodeBuffer[*buffer_count] = px;
    (*buffer_count)++;
    if (*buffer_count >= UI_DASHBOARD_CHUNK_PIXELS)
    {
        if (Ui_DashboardFlushBuffer(*buffer_count, next_y) != 0)
        {
            return -1;
        }
        *buffer_count = 0U;
    }

    return 0;
}

static int32_t Ui_DrawDashboardBackground(void)
{
    uint16_t cache[64] = {0};
    uint16_t prev = 0U;
    uint32_t src_index = 0U;
    uint32_t out_pixels = 0U;
    uint32_t buffer_count = 0U;
    uint32_t next_y = 0U;

    while (out_pixels < UI_DASHBOARD_PIXELS)
    {
        uint8_t op;

        if (src_index >= ui_dashboard_q565_size)
        {
            return -1;
        }
        op = ui_dashboard_q565[src_index++];

        if (op < 0x40U)
        {
            prev = cache[op & 0x3FU];
            if (Ui_DashboardPushPixel(prev, &buffer_count, &next_y) != 0)
            {
                return -1;
            }
            out_pixels++;
        }
        else if (op < 0x80U)
        {
            uint32_t run = (uint32_t)(op & 0x3FU) + 1U;
            if ((out_pixels + run) > UI_DASHBOARD_PIXELS)
            {
                return -1;
            }
            while (run > 0U)
            {
                if (Ui_DashboardPushPixel(prev, &buffer_count, &next_y) != 0)
                {
                    return -1;
                }
                out_pixels++;
                run--;
            }
        }
        else if (op < 0xC0U)
        {
            int r = (int)((prev >> 11) & 0x1FU);
            int g = (int)((prev >> 5) & 0x3FU);
            int b = (int)(prev & 0x1FU);

            r += (int)((op >> 4) & 0x03U) - 2;
            g += (int)((op >> 2) & 0x03U) - 2;
            b += (int)(op & 0x03U) - 2;
            if ((r < 0) || (r > 31) || (g < 0) || (g > 63) || (b < 0) || (b > 31))
            {
                return -1;
            }

            prev = (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
            cache[q565_hash_u16(prev)] = prev;
            if (Ui_DashboardPushPixel(prev, &buffer_count, &next_y) != 0)
            {
                return -1;
            }
            out_pixels++;
        }
        else if (op == 0xFEU)
        {
            if ((src_index + 1U) >= ui_dashboard_q565_size)
            {
                return -1;
            }
            prev = (uint16_t)((uint16_t)ui_dashboard_q565[src_index] |
                              ((uint16_t)ui_dashboard_q565[src_index + 1U] << 8));
            src_index += 2U;
            cache[q565_hash_u16(prev)] = prev;
            if (Ui_DashboardPushPixel(prev, &buffer_count, &next_y) != 0)
            {
                return -1;
            }
            out_pixels++;
        }
        else
        {
            return -1;
        }
    }

    if (buffer_count != 0U)
    {
        if (Ui_DashboardFlushBuffer(buffer_count, &next_y) != 0)
        {
            return -1;
        }
    }

    return (next_y == UI_DASHBOARD_HEIGHT) ? 0 : -1;
}


/*将浮点数安全转换为无符号 32 位整数，用于显示*/
static uint32_t Ui_FloatToU32(float value) 
{
    if (value <= 0.0f)
    {
        return 0U;
    }
    if (value >= 99999.0f)
    {
        return 99999U;
    }
    return (uint32_t)(value + 0.5f);
}

/*计算进度条的像素宽度，线性比例映射*/
/*计算基础距离，返回以米为单位的整数*/
static uint32_t Ui_BaseDistanceMeter(const AppSnapshot_t *snapshot)
{
    float x;
    float y;
    float z;

    x = snapshot->nav.data.position_m[0];
    y = snapshot->nav.data.position_m[1];
    z = snapshot->nav.data.position_m[2];

    return Ui_FloatToU32(sqrtf((x * x) + (y * y) + (z * z)));
}

static float Ui_CdegToRad(int16_t cdeg)
{
    return ((float)cdeg * UI_PI) / 18000.0f;
}
#if (UI_RETURN_ENABLE_HAND_RAISED_FLIP != 0U)
static uint8_t Ui_IsHandRaisedForIns(const AppSnapshot_t *snapshot)
{
    float roll_abs_deg;
    float pitch_abs_deg;

    if ((snapshot == NULL) || (snapshot->nav.update_count == 0U))
    {
        return 0U;
    }

    roll_abs_deg = fabsf(snapshot->nav.data.attitude_rad[0] * UI_RAD_TO_DEG);
    pitch_abs_deg = fabsf(snapshot->nav.data.attitude_rad[1] * UI_RAD_TO_DEG);

    /* 实测抬手稳定段 Roll 接近 +/-90 度，Pitch 仍接近 0 度。 */
    return ((fabsf(roll_abs_deg - UI_HAND_RAISED_ROLL_TARGET_DEG) <= UI_HAND_RAISED_ROLL_TOL_DEG) &&
            (pitch_abs_deg <= UI_HAND_RAISED_PITCH_MAX_DEG)) ? 1U : 0U;
}
#endif

static uint32_t Ui_DistanceMmToMeter(int32_t distance_mm)
{
    if (distance_mm <= 0)
    {
        return 0U;
    }

    return (uint32_t)((distance_mm + 500) / 1000);
}

static uint8_t Ui_ReturnHeadingToFrame(float heading_rad)
{
    float deg;
    float step_deg;
    uint8_t frame_index;

    while (heading_rad < 0.0f)
    {
        heading_rad += (2.0f * UI_PI);
    }
    while (heading_rad >= (2.0f * UI_PI))
    {
        heading_rad -= (2.0f * UI_PI);
    }

    step_deg = 360.0f / (float)COMPASS_ICON_COUNT;
    deg = (heading_rad * 180.0f) / UI_PI;
    frame_index = (uint8_t)((deg + (step_deg * 0.5f)) / step_deg);
    if (frame_index >= COMPASS_ICON_COUNT)
    {
        frame_index = 0U;
    }

    return (uint8_t)(frame_index + 1U);
}

static void Ui_DrawCompassIconFrame(uint8_t frame)
{
    const Icon_Info *icon;
    uint32_t total;
    uint32_t out_idx;
    uint16_t i;

    icon = CompassIcon_GetFrame(frame);
    if (icon == NULL)
    {
        return;
    }
    if ((icon->bbox_w > COMPASS_ICON_BBOX_MAX_W) ||
        (icon->bbox_h > COMPASS_ICON_BBOX_MAX_H))
    {
        return;
    }

    total = (uint32_t)icon->bbox_w * (uint32_t)icon->bbox_h;
    out_idx = 0U;
    for (i = 0U; i < icon->rle_count; i++)
    {
        uint16_t j;

        for (j = 0U; (j < icon->rle[i].count) && (out_idx < total); j++)
        {
            s_returnIconBuffer[out_idx++] = icon->rle[i].color;
        }
    }
    while (out_idx < total)
    {
        s_returnIconBuffer[out_idx++] = ILI9488_COLOR_BLACK;
    }

    (void)ILI9488_DrawRGB565Image(&g_lcd,
                                  (uint16_t)(UI_RETURN_ICON_X + icon->bbox_x),
                                  (uint16_t)(UI_RETURN_ICON_Y + icon->bbox_y),
                                  icon->bbox_w,
                                  icon->bbox_h,
                                  s_returnIconBuffer);
}
static void Ui_StartLocalReturnDemo(void)
{
    NavState_t nav;
    float yaw_rad;

    memset(&nav, 0, sizeof(nav));
    App_StateGetNav(&nav);

    yaw_rad = nav.data.YAW_deg * UI_PI / 180.0f;
    s_returnLocalDemoTargetX =
        nav.data.position_m[0] + (UI_RETURN_LOCAL_DEMO_DISTANCE_M * sinf(yaw_rad));
    s_returnLocalDemoTargetY =
        nav.data.position_m[1] + (UI_RETURN_LOCAL_DEMO_DISTANCE_M * cosf(yaw_rad));
    s_returnLocalDemoLastHeadingRad = 0.0f;
    s_returnLocalDemoArrived = 0U;
    s_returnLocalDemoActive = 1U;
}

static void Ui_StopLocalReturnDemo(void)
{
    s_returnLocalDemoActive = 0U;
    s_returnLocalDemoArrived = 0U;
    s_returnLocalDemoTargetX = 0.0f;
    s_returnLocalDemoTargetY = 0.0f;
    s_returnLocalDemoLastHeadingRad = 0.0f;
}

static int32_t Ui_GetLocalReturnGuidance(const AppSnapshot_t *snapshot,
                                        UiReturnGuidance_t *guidance)
{
    float delta_x;
    float delta_y;
    float distance_m;
    float target_bearing_rad;
    float yaw_rad;

    if ((snapshot == NULL) || (guidance == NULL))
    {
        return -1;
    }

    delta_x = s_returnLocalDemoTargetX - snapshot->nav.data.position_m[0];
    delta_y = s_returnLocalDemoTargetY - snapshot->nav.data.position_m[1];
    distance_m = sqrtf((delta_x * delta_x) + (delta_y * delta_y));

    if (s_returnLocalDemoArrived == 0U)
    {
        if (distance_m <= UI_RETURN_LOCAL_DEMO_ARRIVE_M)
        {
            s_returnLocalDemoArrived = 1U;
            distance_m = 0.0f;
        }
        else
        {
            target_bearing_rad = atan2f(delta_x, delta_y);
            yaw_rad = snapshot->nav.data.YAW_deg * UI_PI / 180.0f;
            s_returnLocalDemoLastHeadingRad = target_bearing_rad - yaw_rad;
        }
    }
    else
    {
        distance_m = 0.0f;
    }

    guidance->valid = 1U;
    guidance->heading_rad = s_returnLocalDemoLastHeadingRad;
#if (UI_RETURN_ENABLE_HAND_RAISED_FLIP != 0U)
    if (Ui_IsHandRaisedForIns(snapshot) != 0U)
    {
        guidance->heading_rad += UI_PI;
    }
#endif
    guidance->distance_m = Ui_FloatToU32(distance_m);
    guidance->turn_after_next = 0;
    return 0;
}

/*
 * 左半区使用本地10m目标；右半区读取 ControlTask 每500ms更新的真实引导。
 * 两种模式在这里统一转换成 UI 显示单位。
 */
__weak int32_t App_UiGetReturnGuidance(const AppSnapshot_t *snapshot, UiReturnGuidance_t *guidance)
{
    const ReturnGuideState_t *guide;

    if ((snapshot == NULL) || (guidance == NULL))
    {
        return -1;
    }

    if (s_returnLocalDemoActive != 0U)
    {
        return Ui_GetLocalReturnGuidance(snapshot, guidance);
    }

    guide = &snapshot->return_guide;
    if ((guide->valid != 0U) &&
        (guide->return_mode != 0U) &&
        (guide->route_valid != 0U))
    {
        guidance->valid = 1U;
        guidance->heading_rad = Ui_CdegToRad(guide->relative_bearing_cdeg);
#if (UI_RETURN_ENABLE_HAND_RAISED_FLIP != 0U)
        if (Ui_IsHandRaisedForIns(snapshot) != 0U)
        {
            /* 抬手时设备+x变为行人正方向，返航UI箭头需要反向显示。 */
            guidance->heading_rad += UI_PI;
        }
#endif
        guidance->distance_m = Ui_DistanceMmToMeter(guide->distance_to_next_mm);
        guidance->turn_after_next = guide->turn_after_next;
        return 0;
    }

    guidance->valid = 0U;
    guidance->heading_rad = 0.0f;
    guidance->distance_m = Ui_BaseDistanceMeter(snapshot);
    return 0;
}

/*依靠lora回传的rssi计算信号强度百分比，-120db——-40db*/

static const uint8_t *Ui_Font5x7(char c)//半拉字库
{
    static const uint8_t blank[7] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t minus[7] = {0x00U, 0x00U, 0x00U, 0x1FU, 0x00U, 0x00U, 0x00U};
    static const uint8_t colon[7] = {0x00U, 0x04U, 0x04U, 0x00U, 0x04U, 0x04U, 0x00U};
    static const uint8_t pct[7] = {0x19U, 0x19U, 0x02U, 0x04U, 0x08U, 0x13U, 0x13U};
    static const uint8_t dot[7] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x06U, 0x06U};
    static const uint8_t comma[7] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x06U, 0x04U};
    static const uint8_t n0[7] = {0x0EU, 0x11U, 0x13U, 0x15U, 0x19U, 0x11U, 0x0EU};
    static const uint8_t n1[7] = {0x04U, 0x0CU, 0x04U, 0x04U, 0x04U, 0x04U, 0x0EU};
    static const uint8_t n2[7] = {0x0EU, 0x11U, 0x01U, 0x02U, 0x04U, 0x08U, 0x1FU};
    static const uint8_t n3[7] = {0x1FU, 0x02U, 0x04U, 0x02U, 0x01U, 0x11U, 0x0EU};
    static const uint8_t n4[7] = {0x02U, 0x06U, 0x0AU, 0x12U, 0x1FU, 0x02U, 0x02U};
    static const uint8_t n5[7] = {0x1FU, 0x10U, 0x1EU, 0x01U, 0x01U, 0x11U, 0x0EU};
    static const uint8_t n6[7] = {0x06U, 0x08U, 0x10U, 0x1EU, 0x11U, 0x11U, 0x0EU};
    static const uint8_t n7[7] = {0x1FU, 0x01U, 0x02U, 0x04U, 0x08U, 0x08U, 0x08U};
    static const uint8_t n8[7] = {0x0EU, 0x11U, 0x11U, 0x0EU, 0x11U, 0x11U, 0x0EU};
    static const uint8_t n9[7] = {0x0EU, 0x11U, 0x11U, 0x0FU, 0x01U, 0x02U, 0x0CU};
    static const uint8_t A[7] = {0x0EU, 0x11U, 0x11U, 0x1FU, 0x11U, 0x11U, 0x11U};
    static const uint8_t B[7] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x11U, 0x11U, 0x1EU};
    static const uint8_t C[7] = {0x0EU, 0x11U, 0x10U, 0x10U, 0x10U, 0x11U, 0x0EU};
    static const uint8_t D[7] = {0x1EU, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x1EU};
    static const uint8_t E[7] = {0x1FU, 0x10U, 0x10U, 0x1EU, 0x10U, 0x10U, 0x1FU};
    static const uint8_t F[7] = {0x1FU, 0x10U, 0x10U, 0x1EU, 0x10U, 0x10U, 0x10U};
    static const uint8_t G[7] = {0x0EU, 0x11U, 0x10U, 0x17U, 0x11U, 0x11U, 0x0FU};
    static const uint8_t H[7] = {0x11U, 0x11U, 0x11U, 0x1FU, 0x11U, 0x11U, 0x11U};
    static const uint8_t I[7] = {0x0EU, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x0EU};
    static const uint8_t J[7] = {0x07U, 0x02U, 0x02U, 0x02U, 0x12U, 0x12U, 0x0CU};
    static const uint8_t K[7] = {0x11U, 0x12U, 0x14U, 0x18U, 0x14U, 0x12U, 0x11U};
    static const uint8_t L[7] = {0x10U, 0x10U, 0x10U, 0x10U, 0x10U, 0x10U, 0x1FU};
    static const uint8_t M[7] = {0x11U, 0x1BU, 0x15U, 0x15U, 0x11U, 0x11U, 0x11U};
    static const uint8_t N[7] = {0x11U, 0x19U, 0x15U, 0x13U, 0x11U, 0x11U, 0x11U};
    static const uint8_t O[7] = {0x0EU, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0EU};
    static const uint8_t P[7] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x10U, 0x10U, 0x10U};
    static const uint8_t Q[7] = {0x0EU, 0x11U, 0x11U, 0x11U, 0x15U, 0x12U, 0x0DU};
    static const uint8_t R[7] = {0x1EU, 0x11U, 0x11U, 0x1EU, 0x14U, 0x12U, 0x11U};
    static const uint8_t S[7] = {0x0FU, 0x10U, 0x10U, 0x0EU, 0x01U, 0x01U, 0x1EU};
    static const uint8_t T[7] = {0x1FU, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U};
    static const uint8_t U[7] = {0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0EU};
    static const uint8_t V[7] = {0x11U, 0x11U, 0x11U, 0x11U, 0x11U, 0x0AU, 0x04U};
    static const uint8_t W[7] = {0x11U, 0x11U, 0x11U, 0x15U, 0x15U, 0x1BU, 0x11U};
    static const uint8_t X[7] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x0AU, 0x11U, 0x11U};
    static const uint8_t Y[7] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x04U, 0x04U, 0x04U};
    static const uint8_t Z[7] = {0x1FU, 0x01U, 0x02U, 0x04U, 0x08U, 0x10U, 0x1FU};

    switch (c)
    {
        case '-': return minus;
        case ':': return colon;
        case '%': return pct;
        case '.': return dot;
        case ',': return comma;
        case '0': return n0;
        case '1': return n1;
        case '2': return n2;
        case '3': return n3;
        case '4': return n4;
        case '5': return n5;
        case '6': return n6;
        case '7': return n7;
        case '8': return n8;
        case '9': return n9;
        case 'A': return A;
        case 'B': return B;
        case 'C': return C;
        case 'D': return D;
        case 'E': return E;
        case 'F': return F;
        case 'G': return G;
        case 'H': return H;
        case 'I': return I;
        case 'J': return J;
        case 'K': return K;
        case 'L': return L;
        case 'M': return M;
        case 'N': return N;
        case 'O': return O;
        case 'P': return P;
        case 'Q': return Q;
        case 'R': return R;
        case 'S': return S;
        case 'T': return T;
        case 'U': return U;
        case 'V': return V;
        case 'W': return W;
        case 'X': return X;
        case 'Y': return Y;
        case 'Z': return Z;
        default: return blank;
    }
}

/*绘制字符*/
static void Ui_DrawCharScaled(uint16_t x,
                              uint16_t y,
                              char c,
                              uint8_t x_scale,
                              uint8_t y_scale,
                              uint16_t color)
{
    const uint8_t *glyph;
    uint8_t row;
    uint8_t col;

    if ((x_scale == 0U) || (y_scale == 0U))
    {
        return;
    }

    glyph = Ui_Font5x7(c);
    for (row = 0U; row < 7U; row++)
    {
        for (col = 0U; col < 5U; col++)
        {
            if ((glyph[row] & (uint8_t)(1U << (4U - col))) != 0U)
            {
                (void)ILI9488_FillRect(&g_lcd,
                                       (uint16_t)(x + (uint16_t)col * x_scale),
                                       (uint16_t)(y + (uint16_t)row * y_scale),
                                       x_scale,
                                       y_scale,
                                       color);
            }
        }
    }
}

/*绘制字符*/
static void Ui_DrawChar(uint16_t x, uint16_t y, char c, uint8_t scale, uint16_t color)
{
    Ui_DrawCharScaled(x, y, c, scale, scale, color);
}

/*从左到右依次绘制字符串中的每个字符*/
static void Ui_DrawText(uint16_t x, uint16_t y, const char *text, uint8_t scale, uint16_t color)
{
    uint16_t cursor_x;

    cursor_x = x;
    while (*text != '\0')
    {
        Ui_DrawChar(cursor_x, y, *text, scale, color);
        cursor_x = (uint16_t)(cursor_x + (uint16_t)(6U * scale));
        text++;
    }
}

static void Ui_DrawTextScaled(uint16_t x,
                              uint16_t y,
                              const char *text,
                              uint8_t x_scale,
                              uint8_t y_scale,
                              uint16_t color)
{
    uint16_t cursor_x;

    if (text == NULL)
    {
        return;
    }

    cursor_x = x;
    while (*text != '\0')
    {
        Ui_DrawCharScaled(cursor_x, y, *text, x_scale, y_scale, color);
        cursor_x = (uint16_t)(cursor_x + (uint16_t)(6U * x_scale));
        text++;
    }
}

/* 主数值专用 15x21 字模，常量保存在 Flash。 */
static const uint16_t s_uiMainDigit15x21[UI_MAIN_DIGIT_GLYPH_COUNT][UI_MAIN_DIGIT_HEIGHT] =
{
    {0x0F80U, 0x0F80U, 0x1FE0U, 0x3FF0U, 0x3870U, 0x3870U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7870U, 0x3870U, 0x3FF0U, 0x1FE0U, 0x07C0U},
    {0x0380U, 0x0380U, 0x0380U, 0x0780U, 0x0F80U, 0x1F80U, 0x3F80U, 0x3B80U, 0x3B80U, 0x2380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U, 0x0380U},
    {0x0FC0U, 0x0FC0U, 0x1FF0U, 0x3FF0U, 0x7878U, 0x7038U, 0x7038U, 0x0038U, 0x0038U, 0x0078U, 0x00F0U, 0x01E0U, 0x03C0U, 0x0780U, 0x0F00U, 0x0F00U, 0x1E00U, 0x3C00U, 0x3FF8U, 0x7FF8U, 0x7FF8U},
    {0x0F80U, 0x0F80U, 0x3FE0U, 0x3FE0U, 0x78F0U, 0x7070U, 0x0070U, 0x00F0U, 0x00F0U, 0x07E0U, 0x07C0U, 0x07F0U, 0x0078U, 0x0038U, 0x7038U, 0x7038U, 0x7038U, 0x3878U, 0x3FF0U, 0x1FE0U, 0x0FC0U},
    {0x00E0U, 0x00E0U, 0x01E0U, 0x03E0U, 0x03E0U, 0x07E0U, 0x0EE0U, 0x1CE0U, 0x1CE0U, 0x1CE0U, 0x38E0U, 0x70E0U, 0x60E0U, 0x7FF8U, 0x7FF8U, 0x7FF8U, 0x7FF8U, 0x00E0U, 0x00E0U, 0x00E0U, 0x00E0U},
    {0x1FF0U, 0x1FF0U, 0x1FF0U, 0x3FF0U, 0x3800U, 0x3800U, 0x3FC0U, 0x3FE0U, 0x3FE0U, 0x7FF0U, 0x7870U, 0x0038U, 0x0038U, 0x0038U, 0x7038U, 0x7038U, 0x7038U, 0x3870U, 0x3FF0U, 0x1FE0U, 0x0F80U},
    {0x07C0U, 0x07C0U, 0x0FF0U, 0x1FF0U, 0x3C78U, 0x3838U, 0x7000U, 0x7000U, 0x7000U, 0x77C0U, 0x7FE0U, 0x7FF0U, 0x7878U, 0x7038U, 0x7038U, 0x7038U, 0x3038U, 0x3878U, 0x3FF0U, 0x1FE0U, 0x07C0U},
    {0x7FF8U, 0x7FF8U, 0x7FF8U, 0x7FF8U, 0x0078U, 0x00F0U, 0x01E0U, 0x01C0U, 0x01C0U, 0x03C0U, 0x0380U, 0x0380U, 0x0700U, 0x0700U, 0x0700U, 0x0700U, 0x0700U, 0x0E00U, 0x0E00U, 0x0E00U, 0x0E00U},
    {0x0FC0U, 0x0FC0U, 0x3FF0U, 0x3FF0U, 0x7878U, 0x7038U, 0x7038U, 0x7878U, 0x7878U, 0x3FF0U, 0x1FE0U, 0x3FF0U, 0x7878U, 0x7038U, 0x7038U, 0x7038U, 0x7038U, 0x7878U, 0x3FF0U, 0x1FE0U, 0x0FC0U},
    {0x0F80U, 0x0F80U, 0x1FE0U, 0x3FF0U, 0x7870U, 0x7030U, 0x7038U, 0x7038U, 0x7038U, 0x7878U, 0x3FF8U, 0x1FF8U, 0x0FB8U, 0x0038U, 0x0038U, 0x0038U, 0x7070U, 0x78F0U, 0x3FE0U, 0x3FC0U, 0x0F80U},
    {0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x3FF8U, 0x3FF8U, 0x3FF8U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U, 0x0000U},
    {0x6038U, 0x6038U, 0x7078U, 0x7078U, 0x78F8U, 0x78F8U, 0x7DF8U, 0x7DF8U, 0x6FB8U, 0x6FB8U, 0x6778U, 0x6778U, 0x6338U, 0x6338U, 0x6038U, 0x6038U, 0x6038U, 0x6038U, 0x6038U, 0x6038U, 0x6038U}
};

static int32_t Ui_DrawMainValueDetailedArea(uint16_t x,
                                            uint16_t y,
                                            uint16_t width,
                                            uint16_t height,
                                            const char *text,
                                            uint16_t color,
                                            uint16_t background)
{
    size_t char_index;
    size_t text_length;
    uint32_t pixel_index;
    uint16_t row;
    uint16_t col;

    if (text == NULL)
    {
        return -1;
    }

    text_length = strlen(text);
    if ((text_length == 0U) ||
        ((((text_length - 1U) * UI_MAIN_DIGIT_ADVANCE) + UI_MAIN_DIGIT_WIDTH) > width) ||
        (((uint32_t)width * height) > UI_DASHBOARD_CHUNK_PIXELS) ||
        (height < UI_MAIN_DIGIT_HEIGHT))
    {
        return -1;
    }

    for (char_index = 0U; char_index < text_length; char_index++)
    {
        if (((text[char_index] < '0') || (text[char_index] > '9')) &&
            (text[char_index] != '-') &&
            (text[char_index] != 'M'))
        {
            return -1;
        }
    }

    for (pixel_index = 0U; pixel_index < ((uint32_t)width * height); pixel_index++)
    {
        s_dashboardDecodeBuffer[pixel_index] = background;
    }

    for (char_index = 0U; char_index < text_length; char_index++)
    {
        uint8_t glyph_index;
        uint16_t x_offset;

        if (text[char_index] == '-')
        {
            glyph_index = UI_MAIN_DIGIT_DASH_INDEX;
        }
        else if (text[char_index] == 'M')
        {
            glyph_index = UI_MAIN_DIGIT_M_INDEX;
        }
        else
        {
            glyph_index = (uint8_t)(text[char_index] - '0');
        }
        x_offset = (uint16_t)(char_index * UI_MAIN_DIGIT_ADVANCE);

        for (row = 0U; row < UI_MAIN_DIGIT_HEIGHT; row++)
        {
            uint16_t bits = s_uiMainDigit15x21[glyph_index][row];
            for (col = 0U; col < UI_MAIN_DIGIT_WIDTH; col++)
            {
                if ((bits & (uint16_t)(1U << (14U - col))) != 0U)
                {
                    s_dashboardDecodeBuffer[((uint32_t)row * width) + x_offset + col] = color;
                }
            }
        }
    }

    return ILI9488_DrawRGB565Image(&g_lcd,
                                    x,
                                    y,
                                    width,
                                    height,
                                    s_dashboardDecodeBuffer);
}

static int32_t Ui_DrawMainValueDetailed(uint16_t x,
                                        uint16_t y,
                                        const char *text,
                                        uint16_t color,
                                        uint16_t background)
{
    return Ui_DrawMainValueDetailedArea(x,
                                        y,
                                        UI_MAIN_CARD_VALUE_W,
                                        UI_MAIN_CARD_VALUE_H,
                                        text,
                                        color,
                                        background);
}

typedef enum
{
    UI_HZ_KUAI = 0,
    UI_HZ_JIE,
    UI_HZ_ZHI,
    UI_HZ_LING,
    UI_HZ_QING,
    UI_HZ_QIU,
    UI_HZ_ZHIYUAN_ZHI,
    UI_HZ_YUAN,
    UI_HZ_WEI,
    UI_HZ_XIAN,
    UI_HZ_QU,
    UI_HZ_YU,
    UI_HZ_AN,
    UI_HZ_QUAN,
    UI_HZ_FAN,
    UI_HZ_HANG,
    UI_HZ_CUN,
    UI_HZ_ZAI,
    UI_HZ_BAO_ZHA,
    UI_HZ_ZHA,
    UI_HZ_FENG,
    UI_HZ_JIE_GOU,
    UI_HZ_GOU,
    UI_HZ_QING_KUANG,
    UI_HZ_KUANG,
    UI_HZ_SHANG,
    UI_HZ_BAO_REPORT,
    UI_HZ_QIAN,
    UI_HZ_JIN,
    UI_HZ_HOU,
    UI_HZ_ZUO,
    UI_HZ_YOU,
    UI_HZ_ZHUAN,
    UI_HZ_COUNT
} UiHzGlyphId_t;

static const uint16_t s_uiHzGlyph[UI_HZ_COUNT][16] =
{
    {0x0000U, 0x1080U, 0x1080U, 0x13F8U, 0x1888U, 0x5488U, 0x5088U, 0x5088U, 0x17FCU, 0x10A0U, 0x10A0U, 0x1110U, 0x1110U, 0x1208U, 0x1404U, 0x0000U},
    {0x0000U, 0x1040U, 0x1040U, 0x13FCU, 0x7C40U, 0x13F8U, 0x1048U, 0x17FCU, 0x1848U, 0x73F8U, 0x1040U, 0x1278U, 0x1240U, 0x1540U, 0x78FCU, 0x0000U},
    {0x0000U, 0x1100U, 0x1118U, 0x11E0U, 0x7D04U, 0x1104U, 0x10FCU, 0x1400U, 0x19F8U, 0x7108U, 0x1108U, 0x11F8U, 0x1108U, 0x1108U, 0x71F8U, 0x0000U},
    {0x0000U, 0x0100U, 0x0280U, 0x0440U, 0x0820U, 0x1210U, 0x610CU, 0x0100U, 0x1FF0U, 0x0010U, 0x0020U, 0x0040U, 0x0E80U, 0x0180U, 0x0060U, 0x0000U},
    {0x0000U, 0x2040U, 0x13F8U, 0x1040U, 0x03F8U, 0x0040U, 0x77FCU, 0x1000U, 0x13F8U, 0x1208U, 0x13F8U, 0x1608U, 0x1BF8U, 0x1208U, 0x0238U, 0x0000U},
    {0x0000U, 0x0100U, 0x0120U, 0x0110U, 0x7FFCU, 0x0100U, 0x2108U, 0x1110U, 0x13A0U, 0x0540U, 0x0920U, 0x1110U, 0x610CU, 0x0100U, 0x0700U, 0x0000U},
    {0x0000U, 0x0100U, 0x0100U, 0x7FFCU, 0x0100U, 0x0100U, 0x3FF0U, 0x0810U, 0x0820U, 0x0440U, 0x0280U, 0x0100U, 0x0280U, 0x0C60U, 0x701CU, 0x0000U},
    {0x0000U, 0x101CU, 0x17E0U, 0x1288U, 0x7950U, 0x13F8U, 0x1080U, 0x17FCU, 0x1880U, 0x71F8U, 0x1148U, 0x1250U, 0x1220U, 0x1450U, 0x718CU, 0x0000U},
    {0x0000U, 0x0800U, 0x0FF0U, 0x1020U, 0x2040U, 0x5FFCU, 0x1000U, 0x13F0U, 0x1210U, 0x1210U, 0x1210U, 0x1260U, 0x2204U, 0x2204U, 0x41FCU, 0x0000U},
    {0x0000U, 0x0040U, 0x7840U, 0x48A0U, 0x4910U, 0x5208U, 0x55F4U, 0x4800U, 0x4888U, 0x4848U, 0x4A48U, 0x7110U, 0x4110U, 0x4020U, 0x47FCU, 0x0000U},
    {0x0000U, 0x3FFCU, 0x2000U, 0x2010U, 0x2810U, 0x2420U, 0x2220U, 0x2140U, 0x2080U, 0x2140U, 0x2220U, 0x2410U, 0x2810U, 0x2000U, 0x3FFCU, 0x0000U},
    {0x0000U, 0x1020U, 0x1028U, 0x1024U, 0x13FCU, 0x7820U, 0x13A4U, 0x12A4U, 0x12A4U, 0x13A8U, 0x1028U, 0x1894U, 0x6334U, 0x004CU, 0x0084U, 0x0000U},
    {0x0000U, 0x0200U, 0x0100U, 0x7FFCU, 0x4004U, 0x4204U, 0x0200U, 0x7FFCU, 0x0420U, 0x0820U, 0x1E40U, 0x0180U, 0x0260U, 0x0C10U, 0x7008U, 0x0000U},
    {0x0000U, 0x0100U, 0x0100U, 0x0280U, 0x0440U, 0x0820U, 0x1010U, 0x6FECU, 0x0100U, 0x0100U, 0x0FE0U, 0x0100U, 0x0100U, 0x0100U, 0x7FFCU, 0x0000U},
    {0x0000U, 0x0038U, 0x23C0U, 0x1200U, 0x1200U, 0x03F8U, 0x0208U, 0x7290U, 0x1250U, 0x1220U, 0x1250U, 0x1488U, 0x1108U, 0x2800U, 0x47FCU, 0x0000U},
    {0x0000U, 0x0840U, 0x1020U, 0x3C00U, 0x25FCU, 0x3400U, 0x2C00U, 0x24F0U, 0x7C90U, 0x2490U, 0x3490U, 0x2C90U, 0x2514U, 0x2514U, 0x4E0CU, 0x0000U},
    /* 存 */
    {0x0000U, 0x0200U, 0x0200U, 0x7FFCU, 0x0400U, 0x0800U, 0x09F8U, 0x1810U, 0x2820U, 0x4820U, 0x0BFCU, 0x0820U, 0x0820U, 0x0820U, 0x08E0U, 0x0000U},
    /* 在 */
    {0x0000U, 0x0200U, 0x0200U, 0x7FFCU, 0x0400U, 0x0840U, 0x0840U, 0x1840U, 0x2BF8U, 0x4840U, 0x0840U, 0x0840U, 0x0840U, 0x0FFCU, 0x0800U, 0x0000U},
    /* 爆 */
    {0x0000U, 0x13F8U, 0x1208U, 0x17F8U, 0x5A08U, 0x53F8U, 0x5110U, 0x53F8U, 0x1110U, 0x17FCU, 0x1248U, 0x2D54U, 0x28E0U, 0x2150U, 0x42C8U, 0x0000U},
    /* 炸 */
    {0x0000U, 0x1100U, 0x1100U, 0x15FCU, 0x5A80U, 0x5280U, 0x5480U, 0x50F8U, 0x1080U, 0x1080U, 0x1080U, 0x28FCU, 0x2480U, 0x2080U, 0x4080U, 0x0000U},
    /* 风 */
    {0x0000U, 0x3FF0U, 0x2010U, 0x2050U, 0x2850U, 0x2490U, 0x2290U, 0x2110U, 0x2110U, 0x2290U, 0x2290U, 0x2454U, 0x2854U, 0x200CU, 0x4004U, 0x0000U},
    /* 结 */
    {0x0000U, 0x1040U, 0x1040U, 0x27FCU, 0x4840U, 0x7840U, 0x13F8U, 0x2000U, 0x4000U, 0x7BF8U, 0x0208U, 0x0208U, 0x1A08U, 0x63F8U, 0x0208U, 0x0000U},
    /* 构 */
    {0x0000U, 0x1080U, 0x1080U, 0x10FCU, 0x7D04U, 0x1204U, 0x1044U, 0x3844U, 0x3484U, 0x50A4U, 0x11F4U, 0x1014U, 0x1004U, 0x1004U, 0x1018U, 0x0000U},
    /* 情 */
    {0x0000U, 0x1040U, 0x13F8U, 0x1040U, 0x1BF8U, 0x5440U, 0x57FCU, 0x5000U, 0x13F8U, 0x1208U, 0x13F8U, 0x1208U, 0x13F8U, 0x1208U, 0x1238U, 0x0000U},
    /* 况 */
    {0x0000U, 0x0000U, 0x27F8U, 0x1408U, 0x1408U, 0x0408U, 0x0408U, 0x07F8U, 0x1120U, 0x1120U, 0x2120U, 0x2220U, 0x4224U, 0x4424U, 0x081CU, 0x0000U},
    /* 上 */
    {0x0000U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x01F8U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x0100U, 0x7FFCU, 0x0000U},
    /* 报 */
    {0x0000U, 0x1000U, 0x11F8U, 0x1108U, 0x7D08U, 0x1138U, 0x1100U, 0x15F8U, 0x1948U, 0x7148U, 0x1150U, 0x1120U, 0x1150U, 0x1188U, 0x7104U, 0x0000U},
    /* 前 */
    {0x0000U, 0x1010U, 0x0820U, 0x7FFCU, 0x0000U, 0x3F08U, 0x2148U, 0x2148U, 0x3F48U, 0x2148U, 0x2148U, 0x3F48U, 0x2108U, 0x2108U, 0x2738U, 0x0000U},
    /* 进 */
    {0x0000U, 0x0110U, 0x2110U, 0x1110U, 0x17FCU, 0x0110U, 0x0110U, 0x7110U, 0x17FCU, 0x1110U, 0x1110U, 0x1110U, 0x1210U, 0x2800U, 0x47FCU, 0x0000U},
    /* 后 */
    {0x0000U, 0x0038U, 0x1FC0U, 0x1000U, 0x1000U, 0x1FFCU, 0x1000U, 0x1000U, 0x17F8U, 0x1408U, 0x1408U, 0x2408U, 0x2408U, 0x47F8U, 0x0408U, 0x0000U},
    /* 左 */
    {0x0000U, 0x0200U, 0x0200U, 0x0200U, 0x7FFCU, 0x0400U, 0x0400U, 0x0800U, 0x0FF8U, 0x1080U, 0x1080U, 0x2080U, 0x4080U, 0x0080U, 0x1FFCU, 0x0000U},
    /* 右 */
    {0x0000U, 0x0200U, 0x0200U, 0x0200U, 0x7FFCU, 0x0400U, 0x0400U, 0x0800U, 0x1FF0U, 0x2810U, 0x4810U, 0x0810U, 0x0810U, 0x0FF0U, 0x0810U, 0x0000U},
    /* 转 */
    {0x0000U, 0x1020U, 0x1020U, 0x7DFCU, 0x2020U, 0x2040U, 0x51FCU, 0x7C40U, 0x1080U, 0x11FCU, 0x1C04U, 0x7008U, 0x1050U, 0x1020U, 0x1010U, 0x0000U},
};

static void Ui_DrawHzGlyph(uint16_t x, uint16_t y, uint8_t glyph_id, uint8_t scale, uint16_t color)
{
    uint8_t row;
    uint8_t col;
    uint8_t run_start;
    uint16_t bits;

    if ((glyph_id >= (uint8_t)UI_HZ_COUNT) || (scale == 0U))
    {
        return;
    }

    /* Draw each continuous row segment once to reduce SPI transactions. */
    for (row = 0U; row < 16U; row++)
    {
        bits = s_uiHzGlyph[glyph_id][row];
        col = 0U;
        while (col < 16U)
        {
            while ((col < 16U) && ((bits & (uint16_t)(1U << (15U - col))) == 0U))
            {
                col++;
            }
            run_start = col;
            while ((col < 16U) && ((bits & (uint16_t)(1U << (15U - col))) != 0U))
            {
                col++;
            }
            if (run_start < col)
            {
                (void)ILI9488_FillRect(&g_lcd,
                                       (uint16_t)(x + ((uint16_t)run_start * scale)),
                                       (uint16_t)(y + ((uint16_t)row * scale)),
                                       (uint16_t)((uint16_t)(col - run_start) * scale),
                                       scale,
                                       color);
            }
        }
    }
}

static uint16_t Ui_HzTextWidth(uint8_t count, uint8_t scale)
{
    if (count == 0U)
    {
        return 0U;
    }
    return (uint16_t)(((uint16_t)count * 16U * scale) + (((uint16_t)count - 1U) * UI_QUICK_HZ_SPACING));
}

static void Ui_DrawHzText(uint16_t x, uint16_t y, const uint8_t *text, uint8_t count, uint8_t scale, uint16_t color)
{
    uint8_t i;
    uint16_t cursor_x;

    if (text == NULL)
    {
        return;
    }

    cursor_x = x;
    for (i = 0U; i < count; i++)
    {
        Ui_DrawHzGlyph(cursor_x, y, text[i], scale, color);
        cursor_x = (uint16_t)(cursor_x + (uint16_t)(16U * scale) + UI_QUICK_HZ_SPACING);
    }
}

static void Ui_DrawHzTextCentered(uint16_t x, uint16_t y, uint16_t w, const uint8_t *text, uint8_t count, uint8_t scale, uint16_t color)
{
    uint16_t text_w;
    uint16_t draw_x;

    text_w = Ui_HzTextWidth(count, scale);
    draw_x = (text_w >= w) ? x : (uint16_t)(x + ((w - text_w) / 2U));
    Ui_DrawHzText(draw_x, y, text, count, scale, color);
}
/* 弹窗中文整块写入，避免逐笔SPI事务导致字形顶部丢失。 */
static void Ui_DrawHzTextCenteredBuffered(uint16_t x,
                                         uint16_t y,
                                         uint16_t w,
                                         const uint8_t *text,
                                         uint8_t count,
                                         uint8_t scale,
                                         uint16_t color,
                                         uint16_t background)
{
    uint16_t text_w;
    uint16_t text_h;
    uint16_t draw_x;
    uint16_t glyph_x;
    uint16_t bits;
    uint32_t pixel_count;
    uint32_t buffer_index;
    uint8_t glyph;
    uint8_t row;
    uint8_t col;
    uint8_t scale_x;
    uint8_t scale_y;

    if ((text == NULL) || (count == 0U) || (scale == 0U))
    {
        return;
    }

    text_w = Ui_HzTextWidth(count, scale);
    text_h = (uint16_t)(16U * scale);
    pixel_count = (uint32_t)text_w * (uint32_t)text_h;
    if (pixel_count > (sizeof(s_returnIconBuffer) / sizeof(s_returnIconBuffer[0])))
    {
        return;
    }

    for (buffer_index = 0U; buffer_index < pixel_count; buffer_index++)
    {
        s_returnIconBuffer[buffer_index] = background;
    }

    glyph_x = 0U;
    for (glyph = 0U; glyph < count; glyph++)
    {
        if (text[glyph] >= (uint8_t)UI_HZ_COUNT)
        {
            glyph_x = (uint16_t)(glyph_x + (uint16_t)(16U * scale) + UI_QUICK_HZ_SPACING);
            continue;
        }

        for (row = 0U; row < 16U; row++)
        {
            bits = s_uiHzGlyph[text[glyph]][row];
            for (col = 0U; col < 16U; col++)
            {
                if ((bits & (uint16_t)(1U << (15U - col))) == 0U)
                {
                    continue;
                }

                for (scale_y = 0U; scale_y < scale; scale_y++)
                {
                    for (scale_x = 0U; scale_x < scale; scale_x++)
                    {
                        buffer_index =
                            ((uint32_t)((uint16_t)(row * scale) + scale_y) * text_w) +
                            glyph_x +
                            (uint16_t)(col * scale) +
                            scale_x;
                        s_returnIconBuffer[buffer_index] = color;
                    }
                }
            }
        }

        glyph_x = (uint16_t)(glyph_x + (uint16_t)(16U * scale) + UI_QUICK_HZ_SPACING);
    }

    draw_x = (text_w >= w) ? x : (uint16_t)(x + ((w - text_w) / 2U));
    (void)ILI9488_DrawRGB565Image(&g_lcd,
                                  draw_x,
                                  y,
                                  text_w,
                                  text_h,
                                  s_returnIconBuffer);
}

static const uint16_t *Ui_GetStatusIconData(uint8_t icon)
{
    switch (icon)
    {
        case 0U: return g_iconNgasData;
        case 1U: return g_iconLpgData;
        case 2U: return g_iconHrData;
        case 3U: return g_iconSpo2Data;
        default: return NULL;
    }
}

static void Ui_DrawStatusBitmap(uint16_t x, uint16_t y, const uint16_t *data)
{
    uint16_t row;

    if (data == NULL)
    {
        return;
    }

    for (row = 0U; row < UI_STATUS_ICON_SIZE; row++)
    {
        uint16_t col = 0U;

        while (col < UI_STATUS_ICON_SIZE)
        {
            uint16_t start;

            while ((col < UI_STATUS_ICON_SIZE) &&
                   (data[(row * UI_STATUS_ICON_SIZE) + col] == 0x0000U))
            {
                col++;
            }

            start = col;
            while ((col < UI_STATUS_ICON_SIZE) &&
                   (data[(row * UI_STATUS_ICON_SIZE) + col] != 0x0000U))
            {
                col++;
            }

            if (col > start)
            {
                (void)ILI9488_DrawRGB565Image(&g_lcd,
                                               (uint16_t)(x + start),
                                               (uint16_t)(y + row),
                                               (uint16_t)(col - start),
                                               1U,
                                               &data[(row * UI_STATUS_ICON_SIZE) + start]);
            }
        }
    }
}

static void Ui_DrawStatusIcon(uint16_t x, uint16_t y, uint8_t icon, uint16_t color)
{
    (void)color;
    Ui_DrawStatusBitmap(x, y, Ui_GetStatusIconData(icon));
}

/*将数值和单位拼接成字符串后以 2 倍大小绘制*/
/*绘制状态卡片*/
/*
┌─────────┬──────────────────────────────┐
│  color  │  [icon]  LABEL               │ y+10
│ 5px色条 │         XXXXunit             │ y+36
│         │                              │
│         │  ████████??????????????????  │ y+62 (5px高进度条)
└─────────┴──────────────────────────────┘
 ←8px→   ←12px→←──── 246px ────→


*/
static void Ui_DrawMainFallbackPanels(void)
{
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        (void)ILI9488_FillRect(&g_lcd,
                               s_mainStatusLayout[i].x,
                               s_mainStatusLayout[i].y,
                               s_mainStatusLayout[i].w,
                               s_mainStatusLayout[i].h,
                               UI_COLOR_PANEL);
    }
    (void)ILI9488_FillRect(&g_lcd, UI_MAIN_BASE_X, UI_MAIN_BASE_Y, UI_MAIN_BASE_W, UI_MAIN_BASE_H, UI_COLOR_ACTION);
    (void)ILI9488_FillRect(&g_lcd, UI_MAIN_RETURN_X, UI_MAIN_RETURN_Y, UI_MAIN_RETURN_W, UI_MAIN_RETURN_H, UI_RGB565(230U, 126U, 34U));
}

static void Ui_UpdateTopSignal(const AppSnapshot_t *snapshot, uint8_t force)
{
    char text[16];
    uint8_t valid;
    int16_t rssi;

    valid = snapshot->lora.last_rx.rssi_valid;
    rssi = snapshot->lora.last_rx.rssi_dbm;
    if ((force == 0U) && (s_lastSignalValid == valid) &&
        ((valid == 0U) || (s_lastSignalDbm == rssi)))
    {
        return;
    }

    (void)ILI9488_FillRect(&g_lcd,
                           UI_MAIN_SIGNAL_VALUE_X,
                           UI_MAIN_SIGNAL_VALUE_Y,
                           UI_MAIN_SIGNAL_VALUE_W,
                           UI_MAIN_SIGNAL_VALUE_H,
                           UI_MAIN_TOP_VALUE_BG);
    if (valid != 0U)
    {
        (void)snprintf(text, sizeof(text), "%dDBM", (int)rssi);
    }
    else
    {
        (void)snprintf(text, sizeof(text), "--DBM");
    }
    Ui_DrawTextScaled(UI_MAIN_SIGNAL_VALUE_X, UI_MAIN_SIGNAL_VALUE_Y, text, UI_MAIN_TOP_TEXT_X_SCALE, UI_MAIN_TOP_TEXT_Y_SCALE, UI_COLOR_TEXT);
    s_lastSignalValid = valid;
    s_lastSignalDbm = rssi;
}

static void Ui_DrawMainTopFixedValues(void)
{
    char text[8];

    (void)ILI9488_FillRect(&g_lcd,
                           UI_MAIN_CPU_VALUE_X,
                           UI_MAIN_CPU_VALUE_Y,
                           UI_MAIN_CPU_VALUE_W,
                           UI_MAIN_CPU_VALUE_H,
                           UI_MAIN_CPU_VALUE_BG);
    (void)snprintf(text, sizeof(text), "%u%%", (unsigned int)UI_MAIN_CPU_VALUE);
    Ui_DrawTextScaled(UI_MAIN_CPU_VALUE_X, UI_MAIN_CPU_VALUE_Y, text, UI_MAIN_TOP_TEXT_X_SCALE, UI_MAIN_TOP_TEXT_Y_SCALE, UI_COLOR_SPO2);

    (void)ILI9488_FillRect(&g_lcd,
                           UI_MAIN_RAM_VALUE_X,
                           UI_MAIN_RAM_VALUE_Y,
                           UI_MAIN_RAM_VALUE_W,
                           UI_MAIN_RAM_VALUE_H,
                           UI_MAIN_RAM_VALUE_BG);
    (void)snprintf(text, sizeof(text), "%u%%", (unsigned int)UI_MAIN_RAM_VALUE);
    Ui_DrawTextScaled(UI_MAIN_RAM_VALUE_X, UI_MAIN_RAM_VALUE_Y, text, UI_MAIN_TOP_TEXT_X_SCALE, UI_MAIN_TOP_TEXT_Y_SCALE, UI_COLOR_ACTION);
}

static void Ui_UpdateMainBaseInfo(const AppSnapshot_t *snapshot, uint8_t force)
{
    char text[16];
    uint32_t distance_m;

    distance_m = Ui_BaseDistanceMeter(snapshot);
    if ((force != 0U) || (s_lastMainBaseDistanceM != distance_m))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               UI_MAIN_BASE_DIST_X,
                               UI_MAIN_BASE_DIST_Y,
                               UI_MAIN_BASE_DIST_W,
                               UI_MAIN_BASE_DIST_H,
                               UI_MAIN_BASE_INFO_BG);
        (void)snprintf(text, sizeof(text), "%luM", (unsigned long)distance_m);
        if (Ui_DrawMainValueDetailedArea(UI_MAIN_BASE_DIST_X,
                                         UI_MAIN_BASE_DIST_Y,
                                         UI_MAIN_BASE_DIST_W,
                                         UI_MAIN_BASE_DIST_H,
                                         text,
                                         UI_COLOR_TEXT,
                                         UI_MAIN_BASE_INFO_BG) != 0)
        {
            Ui_DrawTextScaled(UI_MAIN_BASE_DIST_X, UI_MAIN_BASE_DIST_Y, text, UI_MAIN_BASE_DIST_X_SCALE, UI_MAIN_BASE_DIST_Y_SCALE, UI_COLOR_TEXT);
        }
        s_lastMainBaseDistanceM = distance_m;
    }
}
static void Ui_DrawMainStaticOverlay(const AppSnapshot_t *snapshot)
{
    uint8_t i;

    Ui_UpdateTopSignal(snapshot, 1U);
    Ui_DrawMainTopFixedValues();
    Ui_UpdateMainBaseInfo(snapshot, 1U);

    for (i = 0U; i < 4U; i++)
    {
        const UiMainStatusLayout_t *card = &s_mainStatusLayout[i];
        Ui_DrawStatusIcon((uint16_t)(card->x + UI_MAIN_CARD_ICON_X - 2U),
                          (uint16_t)(card->y + UI_MAIN_CARD_ICON_Y - 3U),
                          card->icon,
                          card->color);
        Ui_DrawText((uint16_t)(card->x + UI_MAIN_CARD_LABEL_X),
                    (uint16_t)(card->y + UI_MAIN_CARD_LABEL_Y),
                    card->label,
                    2U,
                    UI_COLOR_TEXT);
        Ui_DrawTextScaled((uint16_t)(card->x + UI_MAIN_CARD_UNIT_X),
                          (uint16_t)(card->y + UI_MAIN_CARD_UNIT_Y),
                          card->unit,
                          1U,
                          2U,
                          card->color);
    }
}
static void Ui_UpdateMainStatusCard(uint8_t index, uint32_t value, uint8_t data_valid, uint8_t force)
{
    char text[16];
    const UiMainStatusLayout_t *card;

    if (index >= 4U)
    {
        return;
    }
    if ((force == 0U) &&
        (s_mainValueValid != 0U) &&
        (s_lastStatusDataValid[index] == data_valid) &&
        ((data_valid == 0U) || (s_lastStatusValue[index] == value)))
    {
        return;
    }

    card = &s_mainStatusLayout[index];
    if (data_valid != 0U)
    {
        (void)snprintf(text, sizeof(text), "%lu", (unsigned long)value);
    }
    else
    {
        (void)snprintf(text, sizeof(text), "---");
    }
    if (Ui_DrawMainValueDetailed((uint16_t)(card->x + UI_MAIN_CARD_VALUE_X),
                                 (uint16_t)(card->y + UI_MAIN_CARD_VALUE_Y),
                                 text,
                                 card->color,
                                 card->value_bg) != 0)
    {
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(card->x + UI_MAIN_CARD_VALUE_X),
                               (uint16_t)(card->y + UI_MAIN_CARD_VALUE_Y),
                               UI_MAIN_CARD_VALUE_W,
                               UI_MAIN_CARD_VALUE_H,
                               card->value_bg);
        Ui_DrawTextScaled((uint16_t)(card->x + UI_MAIN_CARD_VALUE_X),
                          (uint16_t)(card->y + UI_MAIN_CARD_VALUE_Y),
                          text,
                          UI_MAIN_CARD_VALUE_X_SCALE,
                          UI_MAIN_CARD_VALUE_Y_SCALE,
                          card->color);
    }
    /* Redraw unit because the value refresh area overlaps it. */
    Ui_DrawTextScaled((uint16_t)(card->x + UI_MAIN_CARD_UNIT_X),
                      (uint16_t)(card->y + UI_MAIN_CARD_UNIT_Y),
                      card->unit,
                      UI_MAIN_CARD_UNIT_X_SCALE,
                      UI_MAIN_CARD_UNIT_Y_SCALE,
                      card->color);
    s_lastStatusValue[index] = value;
    s_lastStatusDataValid[index] = data_valid;
}
static void Ui_DrawMainView(const AppSnapshot_t *snapshot, uint8_t full_redraw)
{
    uint8_t force_update;
    uint8_t adc_data_valid;
    uint32_t value[4];

    value[0] = Ui_FloatToU32(snapshot->adc.gas_concentration[0]);
    value[1] = Ui_FloatToU32(snapshot->adc.gas_concentration[1]);
    value[2] = snapshot->spo2.heart_rate_bpm;
    value[3] = snapshot->spo2.spo2_percent;
    adc_data_valid = (snapshot->adc.update_count != 0U) ? 1U : 0U;
    force_update = ((full_redraw != 0U) || (s_mainValueValid == 0U)) ? 1U : 0U;

    if (full_redraw != 0U)
    {
        if (s_mainBackgroundValid == 0U)
        {
            s_mainBackgroundValid = (Ui_DrawDashboardBackground() == 0) ? 1U : 0U;
        }
        if (s_mainBackgroundValid == 0U)
        {
            Ui_DrawMainFallbackPanels();
        }
        Ui_DrawMainStaticOverlay(snapshot);
    }

    Ui_UpdateTopSignal(snapshot, force_update);
    Ui_UpdateMainBaseInfo(snapshot, force_update);
    Ui_UpdateMainStatusCard(0U, value[0], adc_data_valid, force_update);
    Ui_UpdateMainStatusCard(1U, value[1], adc_data_valid, force_update);
    Ui_UpdateMainStatusCard(2U, value[2], snapshot->spo2.heart_rate_valid, force_update);
    Ui_UpdateMainStatusCard(3U, value[3], snapshot->spo2.spo2_valid, force_update);
    s_mainValueValid = 1U;
}
static void Ui_DrawQuickButton(uint16_t y, const uint8_t *text, uint8_t count)
{
    (void)ILI9488_FillRect(&g_lcd, UI_QUICK_BTN_X, y, UI_QUICK_BTN_W, UI_QUICK_BTN_H, UI_COLOR_PANEL);
    Ui_DrawHzTextCentered(UI_QUICK_BTN_X,
                          (uint16_t)(y + 11U),
                          UI_QUICK_BTN_W,
                          text,
                          count,
                          UI_QUICK_HZ_SCALE,
                          UI_COLOR_TEXT);
}

static void Ui_DrawQuickView(void)
{
    static const uint8_t title[] = {UI_HZ_KUAI, UI_HZ_JIE, UI_HZ_ZHI, UI_HZ_LING};
    static const uint8_t support[] = {UI_HZ_QING, UI_HZ_QIU, UI_HZ_ZHIYUAN_ZHI, UI_HZ_YUAN};
    static const uint8_t danger[] = {UI_HZ_WEI, UI_HZ_XIAN, UI_HZ_QU, UI_HZ_YU};
    static const uint8_t safe[] = {UI_HZ_AN, UI_HZ_QUAN, UI_HZ_QU, UI_HZ_YU};

    Ui_DrawHzText(UI_QUICK_TITLE_X, UI_QUICK_TITLE_Y, title, (uint8_t)sizeof(title), UI_QUICK_HZ_SCALE, UI_COLOR_TEXT);//快捷指令
    Ui_DrawQuickButton(UI_QUICK_BTN1_Y, support, (uint8_t)sizeof(support));  //请求支援
    Ui_DrawQuickButton(UI_QUICK_BTN2_Y, danger, (uint8_t)sizeof(danger));    //危险区域
    Ui_DrawQuickButton(UI_QUICK_BTN3_Y, safe, (uint8_t)sizeof(safe));        //安全区域
}

static void Ui_LoraPopupSetHz(UiLoraPopupContent_t *content, const uint8_t *hz, uint8_t count)
{
    if ((content == NULL) || (hz == NULL) || (count > UI_LORA_POPUP_HZ_MAX))
    {
        return;
    }

    memset(content, 0, sizeof(*content));
    content->type = UI_LORA_POPUP_HZ;
    content->hz_count = count;
    memcpy(content->hz, hz, count);
}

static char Ui_LoraPopupNormalizeChar(uint8_t data)
{
    if ((data >= (uint8_t)'a') && (data <= (uint8_t)'z'))
    {
        data = (uint8_t)(data - (uint8_t)('a' - 'A'));
    }

    if (((data >= (uint8_t)'A') && (data <= (uint8_t)'Z')) ||
        ((data >= (uint8_t)'0') && (data <= (uint8_t)'9')) ||
        (data == (uint8_t)' ') || (data == (uint8_t)'-') ||
        (data == (uint8_t)':') || (data == (uint8_t)'%') ||
        (data == (uint8_t)',') || (data == (uint8_t)'.'))
    {
        return (char)data;
    }

    return '.';
}

static void Ui_LoraPopupSetAscii(UiLoraPopupContent_t *content, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint16_t out_len;

    if (content == NULL)
    {
        return;
    }

    memset(content, 0, sizeof(*content));
    content->type = UI_LORA_POPUP_ASCII;

    if ((data == NULL) || (len == 0U))
    {
        (void)snprintf(content->text, sizeof(content->text), "EMPTY");
        return;
    }

    out_len = (len < (UI_LORA_POPUP_TEXT_MAX - 1U)) ? len : (UI_LORA_POPUP_TEXT_MAX - 1U);
    for (i = 0U; i < out_len; i++)
    {
        content->text[i] = Ui_LoraPopupNormalizeChar(data[i]);
    }
    content->text[out_len] = '\0';
}

static void Ui_LoraPopupParsePacket(const LoraPacketMsg_t *packet, UiLoraPopupContent_t *content)
{
    static const uint8_t explosion_risk[] =
    {
        UI_HZ_CUN, UI_HZ_ZAI, UI_HZ_BAO_ZHA, UI_HZ_ZHA, UI_HZ_FENG, UI_HZ_XIAN
    };
    static const uint8_t structure_risk[] =
    {
        UI_HZ_CUN, UI_HZ_ZAI, UI_HZ_JIE_GOU, UI_HZ_GOU, UI_HZ_FENG, UI_HZ_XIAN
    };
    static const uint8_t status_report[] =
    {
        UI_HZ_QING_KUANG, UI_HZ_KUANG, UI_HZ_SHANG, UI_HZ_BAO_REPORT
    };
    static const uint8_t return_req[] = {UI_HZ_FAN, UI_HZ_HANG, UI_HZ_QING, UI_HZ_QIU};

    if ((packet == NULL) || (content == NULL))
    {
        return;
    }

    if (packet->len == 1U)
    {
        switch (packet->payload[0])
        {
            case (uint8_t)'1':
                Ui_LoraPopupSetHz(content, explosion_risk, (uint8_t)sizeof(explosion_risk));
                return;
            case (uint8_t)'2':
                Ui_LoraPopupSetHz(content, structure_risk, (uint8_t)sizeof(structure_risk));
                return;
            case (uint8_t)'3':
                Ui_LoraPopupSetHz(content, status_report, (uint8_t)sizeof(status_report));
                return;
            case (uint8_t)'R':
                Ui_LoraPopupSetHz(content, return_req, (uint8_t)sizeof(return_req));
                return;
            default:
                break;
        }
    }

    Ui_LoraPopupSetAscii(content, packet->payload, packet->len);
}

static void Ui_LoraPopupPush(const UiLoraPopupContent_t *content)
{
    if ((content == NULL) || (content->type == UI_LORA_POPUP_NONE))
    {
        return;
    }

    App_BuzzerRequestLoraPopup();

    if (s_loraPopupActive != 0U)
    {
        s_loraPopupPendingContent = *content;
        s_loraPopupPendingValid = 1U;
        return;
    }

    s_loraPopupContent = *content;
    s_loraPopupActive = 1U;
    s_loraPopupNeedRedraw = 1U;
}

static void Ui_ProcessLoraRxQueue(void)
{
    LoraPacketMsg_t packet;
    UiLoraPopupContent_t content;

    if (g_loraRxQueue == NULL)
    {
        return;
    }

    if (s_view != UI_VIEW_MAIN)
    {
        while (osMessageQueueGet(g_loraRxQueue, &packet, 0, 0U) == osOK)
        {
        }
        s_loraPopupActive = 0U;
        s_loraPopupPendingValid = 0U;
        s_loraPopupNeedRedraw = 0U;
        return;
    }

    while (osMessageQueueGet(g_loraRxQueue, &packet, 0, 0U) == osOK)
    {
        memset(&content, 0, sizeof(content));
        Ui_LoraPopupParsePacket(&packet, &content);
        Ui_LoraPopupPush(&content);
    }
}

static void Ui_DrawRectBorder(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t thickness, uint16_t color)
{
    uint16_t i;

    if ((w == 0U) || (h == 0U) || (thickness == 0U))
    {
        return;
    }

    for (i = 0U; i < thickness; i++)
    {
        (void)ILI9488_FillRect(&g_lcd, x, (uint16_t)(y + i), w, 1U, color);
        (void)ILI9488_FillRect(&g_lcd, x, (uint16_t)(y + h - 1U - i), w, 1U, color);
        (void)ILI9488_FillRect(&g_lcd, (uint16_t)(x + i), y, 1U, h, color);
        (void)ILI9488_FillRect(&g_lcd, (uint16_t)(x + w - 1U - i), y, 1U, h, color);
    }
}

static void Ui_DrawLoraPopupAscii(const char *text)
{
    char line[UI_LORA_POPUP_LINE_CHARS + 1U];
    uint8_t row;
    uint8_t col;
    uint8_t src;

    if (text == NULL)
    {
        return;
    }

    src = 0U;
    for (row = 0U; row < UI_LORA_POPUP_ASCII_LINES; row++)
    {
        col = 0U;
        while ((col < UI_LORA_POPUP_LINE_CHARS) && (text[src] != '\0'))
        {
            line[col] = text[src];
            col++;
            src++;
        }
        line[col] = '\0';

        if (col != 0U)
        {
            Ui_DrawText(UI_LORA_POPUP_TEXT_X,
                        (uint16_t)(UI_LORA_POPUP_TEXT_Y + ((uint16_t)row * 26U)),
                        line,
                        2U,
                        UI_COLOR_TEXT);
        }

        if (text[src] == '\0')
        {
            break;
        }
    }
}

static void Ui_DrawLoraPopup(void)
{
    (void)ILI9488_FillRect(&g_lcd, UI_LORA_POPUP_X, UI_LORA_POPUP_Y, UI_LORA_POPUP_W, UI_LORA_POPUP_H, UI_LORA_POPUP_BG);
    Ui_DrawRectBorder(UI_LORA_POPUP_X, UI_LORA_POPUP_Y, UI_LORA_POPUP_W, UI_LORA_POPUP_H, 2U, UI_LORA_POPUP_BORDER);
    Ui_DrawText(UI_LORA_POPUP_TITLE_X, UI_LORA_POPUP_TITLE_Y, "LORA RX", 2U, UI_LORA_POPUP_BORDER);

    Ui_DrawRectBorder(UI_LORA_POPUP_CLOSE_X, UI_LORA_POPUP_CLOSE_Y, UI_LORA_POPUP_CLOSE_W, UI_LORA_POPUP_CLOSE_H, 2U, UI_COLOR_HR);
    Ui_DrawText((uint16_t)(UI_LORA_POPUP_CLOSE_X + 9U),
                (uint16_t)(UI_LORA_POPUP_CLOSE_Y + 5U),
                "X",
                2U,
                UI_COLOR_HR);

    if (s_loraPopupContent.type == UI_LORA_POPUP_HZ)
    {
        Ui_DrawHzTextCenteredBuffered(UI_LORA_POPUP_X,
                                      UI_LORA_POPUP_HZ_TEXT_Y,
                                      UI_LORA_POPUP_W,
                                      s_loraPopupContent.hz,
                                      s_loraPopupContent.hz_count,
                                      UI_LORA_POPUP_HZ_SCALE,
                                      UI_COLOR_TEXT,
                                      UI_LORA_POPUP_BG);
    }
    else
    {
        Ui_DrawLoraPopupAscii(s_loraPopupContent.text);
    }

    s_loraPopupNeedRedraw = 0U;
}

static void Ui_LoraPopupClose(void)
{
    if (s_loraPopupPendingValid != 0U)
    {
        s_loraPopupContent = s_loraPopupPendingContent;
        memset(&s_loraPopupPendingContent, 0, sizeof(s_loraPopupPendingContent));
        s_loraPopupPendingValid = 0U;
        s_loraPopupActive = 1U;
        s_loraPopupNeedRedraw = 1U;
        return;
    }

    memset(&s_loraPopupContent, 0, sizeof(s_loraPopupContent));
    s_loraPopupActive = 0U;
    s_loraPopupNeedRedraw = 0U;
    s_mainBackgroundValid = 0U;
    s_mainValueValid = 0U;
    s_lastRenderedView = (UiView_t)0xFFU;
}

static void Ui_DrawReturnInstruction(uint32_t distance_m, int8_t turn_after_next)
{
    static const uint8_t forward_text[] = {UI_HZ_QIAN, UI_HZ_JIN};
    static const uint8_t left_text[] = {UI_HZ_HOU, UI_HZ_ZUO, UI_HZ_ZHUAN};
    static const uint8_t right_text[] = {UI_HZ_HOU, UI_HZ_YOU, UI_HZ_ZHUAN};
    const uint8_t *turn_text = NULL;
    char distance_text[16];
    size_t distance_length;
    uint16_t forward_width;
    uint16_t turn_width;
    uint16_t distance_width;
    uint16_t total_width;
    uint16_t cursor_x;
    uint8_t ascii_scale = 2U;

    (void)snprintf(distance_text, sizeof(distance_text), "%luM", (unsigned long)distance_m);
    distance_length = strlen(distance_text);
    forward_width = Ui_HzTextWidth((uint8_t)sizeof(forward_text), 1U);
    turn_width = 0U;
    /* 实测返航提示左右相反，这里只取反显示，不改算法输出。 */
    if (turn_after_next < 0)
    {
        turn_text = right_text;
    }
    else if (turn_after_next > 0)
    {
        turn_text = left_text;
    }
    if (turn_text != NULL)
    {
        turn_width = Ui_HzTextWidth((uint8_t)sizeof(left_text), 1U);
    }

    distance_width = (uint16_t)(distance_length * 6U * ascii_scale);
    total_width = (uint16_t)(forward_width + UI_QUICK_HZ_SPACING + distance_width);
    if (turn_width != 0U)
    {
        total_width = (uint16_t)(total_width + UI_QUICK_HZ_SPACING + turn_width);
    }
    if (total_width > UI_RETURN_TEXT_W)
    {
        ascii_scale = 1U;
        distance_width = (uint16_t)(distance_length * 6U);
        total_width = (uint16_t)(forward_width + UI_QUICK_HZ_SPACING + distance_width);
        if (turn_width != 0U)
        {
            total_width = (uint16_t)(total_width + UI_QUICK_HZ_SPACING + turn_width);
        }
    }

    cursor_x = (total_width >= UI_RETURN_TEXT_W) ? UI_RETURN_TEXT_X
                                                  : (uint16_t)(UI_RETURN_TEXT_X + ((UI_RETURN_TEXT_W - total_width) / 2U));
    Ui_DrawHzText(cursor_x, UI_RETURN_TEXT_Y, forward_text, (uint8_t)sizeof(forward_text), 1U, ILI9488_COLOR_WHITE);
    cursor_x = (uint16_t)(cursor_x + forward_width + UI_QUICK_HZ_SPACING);
    Ui_DrawText(cursor_x,
                (uint16_t)(UI_RETURN_TEXT_Y + 1U),
                distance_text,
                ascii_scale,
                ILI9488_COLOR_WHITE);

    if (turn_text != NULL)
    {
        cursor_x = (uint16_t)(cursor_x + distance_width + UI_QUICK_HZ_SPACING);
        Ui_DrawHzText(cursor_x, UI_RETURN_TEXT_Y, turn_text, (uint8_t)sizeof(left_text), 1U, ILI9488_COLOR_WHITE);
    }
}
static void Ui_DrawReturnView(const AppSnapshot_t *snapshot, uint8_t full_redraw)
{

    UiReturnGuidance_t guidance;
    uint8_t frame;

    memset(&guidance, 0, sizeof(guidance));
    guidance.distance_m = Ui_BaseDistanceMeter(snapshot);
    if (App_UiGetReturnGuidance(snapshot, &guidance) != 0)
    {
        guidance.valid = 0U;
        guidance.heading_rad = 0.0f;
        guidance.distance_m = Ui_BaseDistanceMeter(snapshot);
    }
    if (guidance.valid == 0U)
    {
        guidance.heading_rad = 0.0f;
    }

    frame = Ui_ReturnHeadingToFrame(guidance.heading_rad);
    if ((full_redraw != 0U) || (frame != s_lastReturnFrame))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               UI_RETURN_ICON_X,
                               UI_RETURN_ICON_Y,
                               UI_RETURN_ICON_W,
                               UI_RETURN_ICON_H,
                               ILI9488_COLOR_BLACK);
        Ui_LcdGapDelay();
        Ui_DiagLog("U I+ f%u", (unsigned int)frame);
        Ui_DrawCompassIconFrame(frame);
        Ui_LcdGapDelay();
        Ui_DiagLog("U I- f%u", (unsigned int)frame);
        s_lastReturnFrame = frame;
    }

    if ((full_redraw != 0U) ||
        (guidance.distance_m != s_lastReturnDistanceM) ||
        (guidance.turn_after_next != s_lastReturnTurn))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               UI_RETURN_TEXT_X,
                               UI_RETURN_TEXT_Y,
                               UI_RETURN_TEXT_W,
                               UI_RETURN_TEXT_H,
                               ILI9488_COLOR_BLACK);
        Ui_LcdGapDelay();
        Ui_DrawReturnInstruction(guidance.distance_m, guidance.turn_after_next);
        s_lastReturnDistanceM = guidance.distance_m;
        s_lastReturnTurn = guidance.turn_after_next;
    }

    if (full_redraw != 0U)
    {
        Ui_DrawText(170U, 288U, "TAP TO EXIT", 2U, UI_COLOR_MUTED);
    }
}

static uint8_t Ui_PointInRect(uint16_t x, uint16_t y, uint16_t rx, uint16_t ry, uint16_t rw, uint16_t rh)
{
    return ((x >= rx) && (x < (uint16_t)(rx + rw)) && (y >= ry) && (y < (uint16_t)(ry + rh))) ? 1U : 0U;
}

static UiTouchAction_t Ui_HandleTouch(uint16_t x, uint16_t y, AppCommandMsg_t *command)
{
    memset(command, 0, sizeof(*command));

    if (s_view == UI_VIEW_MAIN)
    {
        if (s_loraPopupActive != 0U)
        {
            if (Ui_PointInRect(x, y, UI_LORA_POPUP_CLOSE_X, UI_LORA_POPUP_CLOSE_Y, UI_LORA_POPUP_CLOSE_W, UI_LORA_POPUP_CLOSE_H) != 0U)
            {
                Ui_LoraPopupClose();
            }
            return UI_TOUCH_NONE;
        }

        if (Ui_PointInRect(x, y, UI_MAIN_BASE_X, UI_MAIN_BASE_Y, UI_MAIN_BASE_W, UI_MAIN_BASE_H) != 0U)
        {
            s_view = UI_VIEW_QUICK;
            return UI_TOUCH_QUICK;
        }
        if (Ui_PointInRect(x, y, UI_MAIN_RETURN_X, UI_MAIN_RETURN_Y, UI_MAIN_RETURN_W, UI_MAIN_RETURN_H) != 0U)
        {
            if (x < UI_MAIN_RETURN_SPLIT_X)
            {
                /* 左半区建立入口航向前方10m的本地演示目标。 */
                Ui_StartLocalReturnDemo();
            }
            else
            {
                Ui_StopLocalReturnDemo();
            }

            s_view = UI_VIEW_RETURN;
            command->id = APP_CMD_RETURN_HOME_START;
            return UI_TOUCH_RETURN_START;
        }
    }
    else if (s_view == UI_VIEW_QUICK)
    {
        if (Ui_PointInRect(x, y, UI_QUICK_BTN_X, UI_QUICK_BTN1_Y, UI_QUICK_BTN_W, UI_QUICK_BTN_H) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 1U;
            return UI_TOUCH_QUICK_SEND;
        }
        if (Ui_PointInRect(x, y, UI_QUICK_BTN_X, UI_QUICK_BTN2_Y, UI_QUICK_BTN_W, UI_QUICK_BTN_H) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 2U;
            return UI_TOUCH_QUICK_SEND;
        }
        if (Ui_PointInRect(x, y, UI_QUICK_BTN_X, UI_QUICK_BTN3_Y, UI_QUICK_BTN_W, UI_QUICK_BTN_H) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 3U;
            return UI_TOUCH_QUICK_SEND;
        }

        s_view = UI_VIEW_MAIN;
        return UI_TOUCH_QUICK_BACK;
    }
    else
    {
        s_view = UI_VIEW_MAIN;
        Ui_StopLocalReturnDemo();
        command->id = APP_CMD_RETURN_HOME_STOP;
        return UI_TOUCH_RETURN_STOP;
    }

    return UI_TOUCH_NONE;
}

int32_t App_UiHardwareInit(void)
{
    if (TFT_Port_Init() != 0)
    {
        return -1;
    }

    if (Ui_DrawDashboardBackground() == 0)
    {
        s_mainBackgroundValid = 1U;
    }
    else
    {
        s_mainBackgroundValid = 0U;
        (void)ILI9488_Fill(&g_lcd, UI_COLOR_BG);
    }
    return 0;
}

void App_UiRender(const AppSnapshot_t *snapshot)
{
    uint8_t full_redraw;

    if (snapshot == NULL)
    {
        return;
    }

    Ui_ProcessLoraRxQueue();

    full_redraw = 0U;
    if (s_lastRenderedView != s_view)
    {
        s_lastRenderedView = s_view;
        full_redraw = 1U;
        if (s_view == UI_VIEW_MAIN)
        {
            s_mainValueValid = 0U;
        }
        else if (s_view == UI_VIEW_RETURN)
        {
            s_lastReturnFrame = UI_RETURN_FRAME_NONE;
            s_lastReturnDistanceM = 0xFFFFFFFFU;
            s_lastReturnTurn = UI_RETURN_TURN_UNSET;
        }
        Ui_DiagLog("U F+ v%u", (unsigned int)s_view);
        if (s_view != UI_VIEW_MAIN)
        {
            s_mainBackgroundValid = 0U;
            (void)ILI9488_Fill(&g_lcd, (s_view == UI_VIEW_RETURN) ? ILI9488_COLOR_BLACK : UI_COLOR_BG);
            Ui_LcdGapDelay();
        }
        Ui_DiagLog("U F- v%u", (unsigned int)s_view);
    }

    if (s_view == UI_VIEW_MAIN)
    {
        if (s_loraPopupActive != 0U)
        {
            if (full_redraw != 0U)
            {
                Ui_DrawMainView(snapshot, full_redraw);
            }
            if ((s_loraPopupNeedRedraw != 0U) || (full_redraw != 0U))
            {
                Ui_DrawLoraPopup();
            }
            return;
        }

        Ui_DrawMainView(snapshot, full_redraw);
    }
    else if (s_view == UI_VIEW_QUICK)
    {
        if (full_redraw != 0U)
        {
            Ui_DrawQuickView();
        }
    }
    else
    {
        Ui_DiagLog("U RV+");
        Ui_DrawReturnView(snapshot, full_redraw);
        Ui_DiagLog("U RV-");
    }
}

int32_t App_UiPollTouch(AppCommandMsg_t *command)
{
    XPT2046_Point point;
    UiTouchAction_t action;

    if (command == NULL)
    {
        return 0;
    }

    if (XPT2046_ReadScreen(&g_touch, &point) == 0)
    {
        if (s_touchPressed != 0U)
        {
            return 0;
        }

        s_touchPressed = 1U;
        action = Ui_HandleTouch(point.x, point.y, command);
        if ((action == UI_TOUCH_RETURN_START) ||
            (action == UI_TOUCH_RETURN_STOP) ||
            (action == UI_TOUCH_QUICK_SEND))
        {
            return 1;
        }
        return 0;
    }

    s_touchPressed = 0U;
    return 0;
}

int32_t Task_UiInitHardware(void)
{
    return App_UiHardwareInit();
}

void Task_UiEntry(void *argument)
{
    UiState_t ui;
    AppSnapshot_t snapshot;
    uint32_t next_touch_tick;
    uint32_t next_display_tick;
    uint32_t loop_count;
    uint32_t loop_start_tick;
    uint32_t phase_tick;
    uint32_t snapshot_ms;
    uint32_t display_wait_ms;
    uint32_t display_ms;
    uint32_t touch_wait_ms;
    uint32_t touch_ms;
    uint32_t now;

    (void)argument;
    memset(&ui, 0, sizeof(ui));
    loop_count = 0U;

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    now = osKernelGetTickCount();
    next_touch_tick = now;
    next_display_tick = now;

    for (;;)
    {
        AppCommandMsg_t command;

        loop_count++;
        loop_start_tick = osKernelGetTickCount();
        snapshot_ms = 0xFFFFFFFFU;
        display_wait_ms = 0xFFFFFFFFU;
        display_ms = 0xFFFFFFFFU;
        touch_wait_ms = 0xFFFFFFFFU;
        touch_ms = 0xFFFFFFFFU;

        /* 触摸先于显示处理，避免输入额外等待一个显示周期。 */
        memset(&command, 0, sizeof(command));
        Ui_DiagLog("U%lu T+", (unsigned long)loop_count);
        phase_tick = osKernelGetTickCount();
        if (osMutexAcquire(g_spiTouchMutex, osWaitForever) == osOK)
        {
            touch_wait_ms = osKernelGetTickCount() - phase_tick;
            phase_tick = osKernelGetTickCount();
            if (App_UiPollTouch(&command) > 0)
            {
                if (osMessageQueuePut(g_uiCmdQueue, &command, 0U, 0U) == osOK)
                {
                    ui.last_command = command;
                    ui.command_count++;
                    ui.touch_count++;
                    (void)osEventFlagsSet(g_sysEventFlags, SYS_EVT_UI_COMMAND);
                }
            }
            touch_ms = osKernelGetTickCount() - phase_tick;
            osMutexRelease(g_spiTouchMutex);
        }
        Ui_DiagLog("U%lu T-%lu", (unsigned long)loop_count, (unsigned long)touch_ms);

        now = osKernelGetTickCount();
        if ((int32_t)(now - next_display_tick) >= 0)
        {
            memset(&snapshot, 0, sizeof(snapshot));
            Ui_DiagLog("U%lu S+", (unsigned long)loop_count);
            phase_tick = osKernelGetTickCount();
            App_StateGetSnapshot(&snapshot);
            snapshot_ms = osKernelGetTickCount() - phase_tick;
            Ui_DiagLog("U%lu S-%lu", (unsigned long)loop_count, (unsigned long)snapshot_ms);

            Ui_DiagLog("U%lu D+", (unsigned long)loop_count);
            phase_tick = osKernelGetTickCount();
            if (osMutexAcquire(g_spiDisplayMutex, osWaitForever) == osOK)
            {
                display_wait_ms = osKernelGetTickCount() - phase_tick;
                Ui_DiagLog("U%lu R+%lu", (unsigned long)loop_count, (unsigned long)display_wait_ms);
                phase_tick = osKernelGetTickCount();
                App_UiRender(&snapshot);
                display_ms = osKernelGetTickCount() - phase_tick;
                Ui_DiagLog("U%lu R-%lu", (unsigned long)loop_count, (unsigned long)display_ms);
                osMutexRelease(g_spiDisplayMutex);
                ui.render_count++;
            }

            next_display_tick += APP_UI_PERIOD_MS;
            now = osKernelGetTickCount();
            if ((int32_t)(now - next_display_tick) >= 0)
            {
                /* 显示超时时跳过已错过的周期，避免连续追帧占用任务。 */
                next_display_tick = now + APP_UI_PERIOD_MS;
            }
        }

        App_StateSetUi(&ui);
        Ui_DiagLog("U%lu E v%u s%lu dw%lu d%lu tw%lu t%lu a%lu",
                   (unsigned long)loop_count,
                   (unsigned int)s_view,
                   (unsigned long)snapshot_ms,
                   (unsigned long)display_wait_ms,
                   (unsigned long)display_ms,
                   (unsigned long)touch_wait_ms,
                   (unsigned long)touch_ms,
                   (unsigned long)(osKernelGetTickCount() - loop_start_tick));

        next_touch_tick += APP_UI_TOUCH_PERIOD_MS;
        now = osKernelGetTickCount();
        if ((int32_t)(now - next_touch_tick) >= 0)
        {
            /* 显示耗时超过触摸周期时，下一个系统节拍立即恢复轮询。 */
            next_touch_tick = now + 1U;
        }
        osDelayUntil(next_touch_tick);
    }
}
