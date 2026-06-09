#include "task_ui.h"

#include "app_config.h"
#include "app_event.h"
#include "app_msg.h"
#include "app_rtos.h"
#include "app_state.h"
#include "cmsis_os.h"
#include "tft_port_stm32_hal.h"

#include <math.h>
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

static uint8_t s_touchPressed = 0U;//触摸状态量
static UiView_t s_view = UI_VIEW_MAIN;//视图状态量
static UiView_t s_lastRenderedView = (UiView_t)0xFFU;//上次渲染的视图状态量，初始值设置为无效值以确保首次渲染
static uint8_t s_mainValueValid = 0U;
static uint32_t s_lastStatusValue[4];
static uint16_t s_lastStatusBarW[4];
static uint32_t s_lastBaseDistanceM = 0U;
static uint8_t s_lastBaseSignal = 0U;


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
static uint16_t Ui_BarWidthU32(uint32_t value, uint32_t max_value, uint16_t max_width)
{
    if ((max_value == 0U) || (value >= max_value))
    {
        return max_width;
    }

    return (uint16_t)((value * max_width) / max_value);
}

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

static uint32_t Ui_DistanceMmToMeter(int32_t distance_mm)
{
    if (distance_mm <= 0)
    {
        return 0U;
    }

    return (uint32_t)((distance_mm + 500) / 1000);
}

/*
 * 返航引导数据接口。
 * ControlTask 每 500ms 更新 return_guide，这里只负责转换成 UI 显示单位。
 */
__weak int32_t App_UiGetReturnGuidance(const AppSnapshot_t *snapshot, UiReturnGuidance_t *guidance)
{
    const ReturnGuideState_t *guide;

    if ((snapshot == NULL) || (guidance == NULL))
    {
        return -1;
    }

    guide = &snapshot->return_guide;
    if ((guide->valid != 0U) &&
        (guide->return_mode != 0U) &&
        (guide->route_valid != 0U))
    {
        guidance->valid = 1U;
        guidance->heading_rad = Ui_CdegToRad(guide->relative_bearing_cdeg);
        guidance->distance_m = Ui_DistanceMmToMeter(guide->distance_to_next_mm);
        return 0;
    }

    guidance->valid = 0U;
    guidance->heading_rad = 0.0f;
    guidance->distance_m = Ui_BaseDistanceMeter(snapshot);
    return 0;
}

/*依靠lora回传的rssi计算信号强度百分比，-120db——-40db*/
static uint8_t Ui_SignalPercent(const AppSnapshot_t *snapshot)
{
    int16_t rssi;

    if (snapshot->lora.last_rx.rssi_valid == 0U)
    {
        return 0U;
    }

    rssi = snapshot->lora.last_rx.rssi_dbm;
    if (rssi <= -120)
    {
        return 0U;
    }
    if (rssi >= -40)
    {
        return 100U;
    }

    return (uint8_t)(((int32_t)rssi + 120) * 100 / 80);
}


static const uint8_t *Ui_Font5x7(char c)//半拉字库
{
    static const uint8_t blank[7] = {0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    static const uint8_t minus[7] = {0x00U, 0x00U, 0x00U, 0x1FU, 0x00U, 0x00U, 0x00U};
    static const uint8_t colon[7] = {0x00U, 0x04U, 0x04U, 0x00U, 0x04U, 0x04U, 0x00U};
    static const uint8_t pct[7] = {0x19U, 0x19U, 0x02U, 0x04U, 0x08U, 0x13U, 0x13U};
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
    static const uint8_t G[7] = {0x0EU, 0x11U, 0x10U, 0x17U, 0x11U, 0x11U, 0x0FU};
    static const uint8_t H[7] = {0x11U, 0x11U, 0x11U, 0x1FU, 0x11U, 0x11U, 0x11U};
    static const uint8_t I[7] = {0x0EU, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x0EU};
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
    static const uint8_t X[7] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x0AU, 0x11U, 0x11U};
    static const uint8_t Y[7] = {0x11U, 0x11U, 0x0AU, 0x04U, 0x04U, 0x04U, 0x04U};

    switch (c)
    {
        case '-': return minus;
        case ':': return colon;
        case '%': return pct;
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
        case 'G': return G;
        case 'H': return H;
        case 'I': return I;
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
        case 'X': return X;
        case 'Y': return Y;
        default: return blank;
    }
}

/*绘制字符*/
static void Ui_DrawChar(uint16_t x, uint16_t y, char c, uint8_t scale, uint16_t color)
{
    const uint8_t *glyph;
    uint8_t row;
    uint8_t col;

    glyph = Ui_Font5x7(c);
    for (row = 0U; row < 7U; row++)
    {
        for (col = 0U; col < 5U; col++)
        {
            if ((glyph[row] & (uint8_t)(1U << (4U - col))) != 0U)
            {
                (void)ILI9488_FillRect(&g_lcd,
                                       (uint16_t)(x + (uint16_t)col * scale),
                                       (uint16_t)(y + (uint16_t)row * scale),
                                       scale,
                                       scale,
                                       color);
            }
        }
    }
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

//花园
static void Ui_FillCircle(uint16_t cx, uint16_t cy, uint16_t radius, uint16_t color)
{
    int16_t dy;
    int16_t dx;
    int32_t r2;

    r2 = (int32_t)radius * (int32_t)radius;
    for (dy = -(int16_t)radius; dy <= (int16_t)radius; dy++)
    {
        dx = (int16_t)radius;
        while ((((int32_t)dx * dx) + ((int32_t)dy * dy)) > r2)
        {
            dx--;
        }
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)((int16_t)cx - dx),
                               (uint16_t)((int16_t)cy + dy),
                               (uint16_t)((dx * 2) + 1),
                               1U,
                               color);
    }
}

//三像素宽的圆圈
static void Ui_DrawCircleBorder(uint16_t cx, uint16_t cy, uint16_t radius, uint16_t color)
{
    Ui_FillCircle(cx, cy, radius, color);
    Ui_FillCircle(cx, cy, (uint16_t)(radius - 3U), UI_COLOR_BG);
}

static void Ui_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    int16_t dx;
    int16_t dy;
    int16_t sx;
    int16_t sy;
    int16_t err;
    int16_t e2;
    int16_t x;
    int16_t y;

    x = (int16_t)x0;
    y = (int16_t)y0;
    dx = (x0 > x1) ? (int16_t)(x0 - x1) : (int16_t)(x1 - x0);
    dy = (y0 > y1) ? -(int16_t)(y0 - y1) : -(int16_t)(y1 - y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = (int16_t)(dx + dy);

    for (;;)
    {
        (void)ILI9488_DrawPixel(&g_lcd, (uint16_t)x, (uint16_t)y, color);
        if ((x == (int16_t)x1) && (y == (int16_t)y1))
        {
            break;
        }
        e2 = (int16_t)(2 * err);
        if (e2 >= dy)
        {
            err = (int16_t)(err + dy);
            x = (int16_t)(x + sx);
        }
        if (e2 <= dx)
        {
            err = (int16_t)(err + dx);
            y = (int16_t)(y + sy);
        }
    }
}

static float Ui_NormalizeHeading(float heading_rad)
{
    while (heading_rad > UI_PI)
    {
        heading_rad -= (2.0f * UI_PI);
    }
    while (heading_rad < -UI_PI)
    {
        heading_rad += (2.0f * UI_PI);
    }

    return heading_rad;
}

static void Ui_RotatePoint(int16_t local_x,
                           int16_t local_y,
                           float heading_rad,
                           uint16_t cx,
                           uint16_t cy,
                           uint16_t *screen_x,
                           uint16_t *screen_y)
{
    float cos_h;
    float sin_h;

    cos_h = cosf(heading_rad);
    sin_h = sinf(heading_rad);
    *screen_x = (uint16_t)((int16_t)cx + (int16_t)(((float)local_x * cos_h) - ((float)local_y * sin_h)));
    *screen_y = (uint16_t)((int16_t)cy + (int16_t)(((float)local_x * sin_h) + ((float)local_y * cos_h)));
}

static void Ui_DrawThickLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
    Ui_DrawLine(x0, y0, x1, y1, color);
    Ui_DrawLine((uint16_t)(x0 + 1U), y0, (uint16_t)(x1 + 1U), y1, color);
    Ui_DrawLine((uint16_t)(x0 - 1U), y0, (uint16_t)(x1 - 1U), y1, color);
    Ui_DrawLine(x0, (uint16_t)(y0 + 1U), x1, (uint16_t)(y1 + 1U), color);
    Ui_DrawLine(x0, (uint16_t)(y0 - 1U), x1, (uint16_t)(y1 - 1U), color);
}

static void Ui_SwapI32(int32_t *a, int32_t *b)
{
    int32_t tmp;

    tmp = *a;
    *a = *b;
    *b = tmp;
}

static void Ui_FillTriangle(uint16_t x0,
                            uint16_t y0,
                            uint16_t x1,
                            uint16_t y1,
                            uint16_t x2,
                            uint16_t y2,
                            uint16_t color)
{
    int32_t ax = x0;
    int32_t ay = y0;
    int32_t bx = x1;
    int32_t by = y1;
    int32_t cx = x2;
    int32_t cy = y2;
    int32_t y;

    if (ay > by)
    {
        Ui_SwapI32(&ay, &by);
        Ui_SwapI32(&ax, &bx);
    }
    if (by > cy)
    {
        Ui_SwapI32(&by, &cy);
        Ui_SwapI32(&bx, &cx);
    }
    if (ay > by)
    {
        Ui_SwapI32(&ay, &by);
        Ui_SwapI32(&ax, &bx);
    }

    for (y = ay; y <= cy; y++)
    {
        int32_t left;
        int32_t right;

        if ((cy - ay) == 0)
        {
            left = ax;
        }
        else
        {
            left = ax + ((cx - ax) * (y - ay)) / (cy - ay);
        }

        if (y < by)
        {
            right = ((by - ay) == 0) ? bx : ax + ((bx - ax) * (y - ay)) / (by - ay);
        }
        else
        {
            right = ((cy - by) == 0) ? cx : bx + ((cx - bx) * (y - by)) / (cy - by);
        }

        if (left > right)
        {
            Ui_SwapI32(&left, &right);
        }

        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)left,
                               (uint16_t)y,
                               (uint16_t)(right - left + 1),
                               1U,
                               color);
    }
}

/* 纸飞机方向：0 表示屏幕正上方，正值顺时针旋转。 */
static void Ui_DrawPaperPlane(uint16_t cx, uint16_t cy, uint16_t size, float heading_rad, uint16_t color)
{
    uint16_t tip_x;
    uint16_t tip_y;
    uint16_t left_x;
    uint16_t left_y;
    uint16_t tail_x;
    uint16_t tail_y;
    uint16_t right_x;
    uint16_t right_y;
    uint16_t fold_x;
    uint16_t fold_y;
    int16_t half_w;
    int16_t lower_y;
    int16_t tail_y_local;

    heading_rad = Ui_NormalizeHeading(heading_rad);
    half_w = (int16_t)((size * 58U) / 100U);
    lower_y = (int16_t)((size * 42U) / 100U);
    tail_y_local = (int16_t)((size * 18U) / 100U);

    Ui_RotatePoint(0, (int16_t)(-(int16_t)size), heading_rad, cx, cy, &tip_x, &tip_y);
    Ui_RotatePoint((int16_t)(-half_w), lower_y, heading_rad, cx, cy, &left_x, &left_y);
    Ui_RotatePoint(0, tail_y_local, heading_rad, cx, cy, &tail_x, &tail_y);
    Ui_RotatePoint(half_w, lower_y, heading_rad, cx, cy, &right_x, &right_y);
    Ui_RotatePoint((int16_t)(-(int16_t)(size / 6U)), (int16_t)(size / 10U), heading_rad, cx, cy, &fold_x, &fold_y);

    /* 先填充两侧机翼，再补轮廓，避免只显示线框。 */
    Ui_FillTriangle(tip_x, tip_y, left_x, left_y, tail_x, tail_y, color);
    Ui_FillTriangle(tip_x, tip_y, tail_x, tail_y, right_x, right_y, color);

    Ui_DrawThickLine(tip_x, tip_y, left_x, left_y, color);
    Ui_DrawThickLine(left_x, left_y, tail_x, tail_y, color);
    Ui_DrawThickLine(tail_x, tail_y, right_x, right_y, color);
    Ui_DrawThickLine(right_x, right_y, tip_x, tip_y, color);
    Ui_DrawThickLine(tip_x, tip_y, tail_x, tail_y, color);
    Ui_DrawThickLine(tip_x, tip_y, fold_x, fold_y, color);
}

static void Ui_DrawButtonArrow(uint16_t cx, uint16_t cy, uint16_t color)
{
    Ui_DrawPaperPlane(cx, cy, 30U, 0.0f, color);
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
static void Ui_DrawValueLine(uint16_t x, uint16_t y, uint32_t value, const char *unit, uint16_t color)
{
    char text[24];

    (void)snprintf(text, sizeof(text), "%lu%s", (unsigned long)value, unit);
    Ui_DrawText(x, y, text, 2U, color);
}

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
static void Ui_DrawStatusCardStatic(uint16_t y, const char *label, uint8_t icon, uint16_t color)
{
    (void)ILI9488_FillRect(&g_lcd, UI_LEFT_X, y, UI_LEFT_W, UI_CARD_H, UI_COLOR_PANEL);
    (void)ILI9488_FillRect(&g_lcd, UI_LEFT_X, y, 5U, UI_CARD_H, color);
    Ui_DrawStatusIcon((uint16_t)(UI_LEFT_X + 12U), (uint16_t)(y + 10U), icon, color);
    Ui_DrawText((uint16_t)(UI_LEFT_X + 70U), (uint16_t)(y + 10U), label, 2U, UI_COLOR_TEXT);
    (void)ILI9488_FillRect(&g_lcd, (uint16_t)(UI_LEFT_X + 12U), (uint16_t)(y + 62U), (uint16_t)(UI_LEFT_W - 24U), 5U, UI_COLOR_PANEL_2);
}

static void Ui_UpdateStatusCardValue(uint8_t index,
                                     uint16_t y,
                                     uint32_t value,
                                     const char *unit,
                                     uint32_t max_value,
                                     uint16_t color,
                                     uint8_t force)
{
    uint16_t bar_w;

    bar_w = Ui_BarWidthU32(value, max_value, (uint16_t)(UI_LEFT_W - 24U));

    if ((force != 0U) ||
        (s_mainValueValid == 0U) ||
        (s_lastStatusValue[index] != value))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(UI_LEFT_X + UI_STATUS_VALUE_X_OFF),
                               (uint16_t)(y + UI_STATUS_VALUE_Y_OFF),
                               UI_STATUS_VALUE_W,
                               UI_STATUS_VALUE_H,
                               UI_COLOR_PANEL);
        Ui_DrawValueLine((uint16_t)(UI_LEFT_X + UI_STATUS_VALUE_X_OFF),
                         (uint16_t)(y + UI_STATUS_VALUE_Y_OFF),
                         value,
                         unit,
                         color);
        s_lastStatusValue[index] = value;
    }

    if ((force != 0U) ||
        (s_mainValueValid == 0U) ||
        (s_lastStatusBarW[index] != bar_w))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(UI_LEFT_X + 12U),
                               (uint16_t)(y + 62U),
                               (uint16_t)(UI_LEFT_W - 24U),
                               5U,
                               UI_COLOR_PANEL_2);
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(UI_LEFT_X + 12U),
                               (uint16_t)(y + 62U),
                               bar_w,
                               5U,
                               color);
        s_lastStatusBarW[index] = bar_w;
    }
}

static void Ui_DrawQuickCircleStatic(void)
{
    Ui_FillCircle(UI_QUICK_CX, UI_QUICK_CY, UI_QUICK_R, UI_COLOR_PANEL);
    Ui_DrawCircleBorder(UI_QUICK_CX, UI_QUICK_CY, UI_QUICK_R, UI_COLOR_ACTION);
    Ui_DrawText((uint16_t)((int16_t)UI_QUICK_CX + UI_BASE_TEXT_X_OFF),
                (uint16_t)((int16_t)UI_QUICK_CY + UI_BASE_TEXT_Y_OFF),
                UI_BASE_TEXT,
                2U,
                UI_COLOR_TEXT);
}

static void Ui_UpdateQuickCircleValue(const AppSnapshot_t *snapshot, uint8_t force)
{
    char text[24];
    uint32_t distance_m;
    uint8_t signal;

    distance_m = Ui_BaseDistanceMeter(snapshot);
    signal = Ui_SignalPercent(snapshot);

    if ((force != 0U) ||
        (s_mainValueValid == 0U) ||
        (s_lastBaseDistanceM != distance_m))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(UI_QUICK_CX - 46U),
                               (uint16_t)(UI_QUICK_CY - 8U),
                               96U,
                               22U,
                               UI_COLOR_PANEL);
        (void)snprintf(text, sizeof(text), "%luM", (unsigned long)distance_m);
        Ui_DrawText((uint16_t)((int16_t)UI_QUICK_CX + UI_BASE_DIST_X_OFF),
                    (uint16_t)((int16_t)UI_QUICK_CY + UI_BASE_DIST_Y_OFF),
                    text,
                    2U,
                    UI_COLOR_ACTION);
        s_lastBaseDistanceM = distance_m;
    }

    if ((force != 0U) ||
        (s_mainValueValid == 0U) ||
        (s_lastBaseSignal != signal))
    {
        (void)ILI9488_FillRect(&g_lcd,
                               (uint16_t)(UI_QUICK_CX - 44U),
                               (uint16_t)(UI_QUICK_CY + 22U),
                               72U,
                               10U,
                               UI_COLOR_PANEL);
        (void)snprintf(text, sizeof(text), "SIG:%u%%", signal);
        Ui_DrawText((uint16_t)((int16_t)UI_QUICK_CX + UI_BASE_SIG_X_OFF),
                    (uint16_t)((int16_t)UI_QUICK_CY + UI_BASE_SIG_Y_OFF),
                    text,
                    1U,
                    UI_COLOR_MUTED);
        s_lastBaseSignal = signal;
    }
}

static void Ui_DrawBackCircle(void)
{
    Ui_FillCircle(UI_QUICK_CX, UI_QUICK_CY, UI_QUICK_R, UI_COLOR_PANEL);
    Ui_DrawCircleBorder(UI_QUICK_CX, UI_QUICK_CY, UI_QUICK_R, UI_COLOR_ACTION);
    Ui_DrawText((uint16_t)(UI_QUICK_CX - 30U), (uint16_t)(UI_QUICK_CY - 16U), "BACK", 2U, UI_COLOR_TEXT);
    Ui_DrawText((uint16_t)(UI_QUICK_CX - 20U), (uint16_t)(UI_QUICK_CY + 18U), "TAP", 1U, UI_COLOR_MUTED);
}

static void Ui_DrawReturnButton(void)
{
    (void)ILI9488_FillRect(&g_lcd, UI_RIGHT_X, UI_HOME_BTN_Y, UI_RIGHT_W, UI_HOME_BTN_H, UI_COLOR_ACTION);
    Ui_DrawText((uint16_t)(UI_RIGHT_X + UI_HOME_TEXT_X_OFF),
                (uint16_t)(UI_HOME_BTN_Y + UI_HOME_TEXT_Y_OFF),
                "HOME",
                3U,
                UI_COLOR_TEXT);
    Ui_DrawButtonArrow((uint16_t)(UI_RIGHT_X + UI_HOME_ICON_X_OFF),
                       (uint16_t)(UI_HOME_BTN_Y + UI_HOME_ICON_Y_OFF),
                       UI_COLOR_TEXT);
}

static void Ui_DrawMainView(const AppSnapshot_t *snapshot, uint8_t full_redraw)
{
    uint16_t y;
    uint8_t force_update;
    uint32_t value[4];

    value[0] = Ui_FloatToU32(snapshot->adc.gas_concentration[0]);
    value[1] = Ui_FloatToU32(snapshot->adc.gas_concentration[1]);
    value[2] = snapshot->spo2.heart_rate_bpm;
    value[3] = snapshot->spo2.spo2_percent;
    force_update = ((full_redraw != 0U) || (s_mainValueValid == 0U)) ? 1U : 0U;

    if (full_redraw != 0U)
    {
        y = 10U;
        Ui_DrawStatusCardStatic(y, "NGAS", 0U, UI_COLOR_NGAS);
        y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
        Ui_DrawStatusCardStatic(y, "LPG", 1U, UI_COLOR_LPG);
        y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
        Ui_DrawStatusCardStatic(y, "HR", 2U, UI_COLOR_HR);
        y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
        Ui_DrawStatusCardStatic(y, "SPO2", 3U, UI_COLOR_SPO2);

        (void)ILI9488_FillRect(&g_lcd, UI_RIGHT_X, 8U, UI_RIGHT_W, 312U, UI_COLOR_BG);
        Ui_DrawQuickCircleStatic();
        Ui_DrawReturnButton();
    }

    y = 10U;
    Ui_UpdateStatusCardValue(0U, y, value[0], "PPM", UI_GAS_BAR_MAX_PPM, UI_COLOR_NGAS, force_update);
    y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
    Ui_UpdateStatusCardValue(1U, y, value[1], "PPM", UI_GAS_BAR_MAX_PPM, UI_COLOR_LPG, force_update);
    y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
    Ui_UpdateStatusCardValue(2U, y, value[2], "BPM", 200U, UI_COLOR_HR, force_update);
    y = (uint16_t)(y + UI_CARD_H + UI_CARD_GAP);
    Ui_UpdateStatusCardValue(3U, y, value[3], "%", 100U, UI_COLOR_SPO2, force_update);
    Ui_UpdateQuickCircleValue(snapshot, force_update);
    s_mainValueValid = 1U;
}

static void Ui_DrawQuickButton(uint16_t y, const char *label)
{
    (void)ILI9488_FillRect(&g_lcd, 28U, y, 232U, 54U, UI_COLOR_PANEL);
    Ui_DrawText(58U, (uint16_t)(y + 18U), label, 2U, UI_COLOR_TEXT);
}

static void Ui_DrawQuickView(void)
{
    Ui_DrawText(30U, 24U, "QUICK CMD", 2U, UI_COLOR_TEXT);
    Ui_DrawQuickButton(70U, "SEND 1");
    Ui_DrawQuickButton(136U, "SEND 2");
    Ui_DrawQuickButton(202U, "SEND 3");
    Ui_DrawBackCircle();
}

static void Ui_DrawReturnView(const AppSnapshot_t *snapshot)
{
    char text[24];
    UiReturnGuidance_t guidance;

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

    (void)ILI9488_FillRect(&g_lcd, 120U, 24U, 240U, 180U, ILI9488_COLOR_BLACK);
    Ui_DrawPaperPlane(240U, 122U, 78U, guidance.heading_rad, ILI9488_COLOR_WHITE);
    (void)ILI9488_FillRect(&g_lcd, 150U, 222U, 210U, 48U, ILI9488_COLOR_BLACK);
    (void)snprintf(text, sizeof(text), "%luM", (unsigned long)guidance.distance_m);
    Ui_DrawText(190U, 230U, text, 3U, ILI9488_COLOR_WHITE);
    Ui_DrawText(170U, 288U, "TAP TO EXIT", 2U, UI_COLOR_MUTED);
}

static uint8_t Ui_PointInRect(uint16_t x, uint16_t y, uint16_t rx, uint16_t ry, uint16_t rw, uint16_t rh)
{
    return ((x >= rx) && (x < (uint16_t)(rx + rw)) && (y >= ry) && (y < (uint16_t)(ry + rh))) ? 1U : 0U;
}

static uint8_t Ui_PointInQuickCircle(uint16_t x, uint16_t y)
{
    int32_t dx;
    int32_t dy;

    dx = (int32_t)x - (int32_t)UI_QUICK_CX;
    dy = (int32_t)y - (int32_t)UI_QUICK_CY;
    return (((dx * dx) + (dy * dy)) <= ((int32_t)UI_QUICK_R * (int32_t)UI_QUICK_R)) ? 1U : 0U;
}

static UiTouchAction_t Ui_HandleTouch(uint16_t x, uint16_t y, AppCommandMsg_t *command)
{
    memset(command, 0, sizeof(*command));

    if (s_view == UI_VIEW_MAIN)
    {
        if (Ui_PointInQuickCircle(x, y) != 0U)
        {
            s_view = UI_VIEW_QUICK;
            return UI_TOUCH_QUICK;
        }
        if (Ui_PointInRect(x, y, UI_RIGHT_X, UI_HOME_BTN_Y, UI_RIGHT_W, UI_HOME_BTN_H) != 0U)
        {
            s_view = UI_VIEW_RETURN;
            command->id = APP_CMD_RETURN_HOME_START;
            return UI_TOUCH_RETURN_START;
        }
    }
    else if (s_view == UI_VIEW_QUICK)
    {
        if (Ui_PointInQuickCircle(x, y) != 0U)
        {
            s_view = UI_VIEW_MAIN;
            return UI_TOUCH_QUICK_BACK;
        }
        if (Ui_PointInRect(x, y, 28U, 70U, 232U, 54U) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 1U;
            return UI_TOUCH_QUICK_SEND;
        }
        if (Ui_PointInRect(x, y, 28U, 136U, 232U, 54U) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 2U;
            return UI_TOUCH_QUICK_SEND;
        }
        if (Ui_PointInRect(x, y, 28U, 202U, 232U, 54U) != 0U)
        {
            command->id = APP_CMD_LORA_SEND;
            command->param0 = 3U;
            return UI_TOUCH_QUICK_SEND;
        }
    }
    else
    {
        s_view = UI_VIEW_MAIN;
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

    (void)ILI9488_Fill(&g_lcd, UI_COLOR_BG);
    return 0;
}

void App_UiRender(const AppSnapshot_t *snapshot)
{
    uint8_t full_redraw;

    if (snapshot == NULL)
    {
        return;
    }

    full_redraw = 0U;
    if (s_lastRenderedView != s_view)
    {
        s_lastRenderedView = s_view;
        full_redraw = 1U;
        if (s_view == UI_VIEW_MAIN)
        {
            s_mainValueValid = 0U;
        }
        (void)ILI9488_Fill(&g_lcd, (s_view == UI_VIEW_RETURN) ? ILI9488_COLOR_BLACK : UI_COLOR_BG);
    }

    if (s_view == UI_VIEW_MAIN)
    {
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
        Ui_DrawReturnView(snapshot);
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
    uint32_t next_tick;

    (void)argument;
    memset(&ui, 0, sizeof(ui));

    osEventFlagsWait(g_sysEventFlags,
                     SYS_EVT_INIT_DONE,
                     osFlagsWaitAny | osFlagsNoClear,
                     osWaitForever);
    next_tick = osKernelGetTickCount();

    for (;;)
    {
        AppCommandMsg_t command;

        memset(&snapshot, 0, sizeof(snapshot));
        App_StateGetSnapshot(&snapshot);

        if (osMutexAcquire(g_spiDisplayMutex, osWaitForever) == osOK)
        {
            App_UiRender(&snapshot);
            osMutexRelease(g_spiDisplayMutex);
            ui.render_count++;
        }

        memset(&command, 0, sizeof(command));
        if (osMutexAcquire(g_spiTouchMutex, osWaitForever) == osOK)
        {
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
            osMutexRelease(g_spiTouchMutex);
        }

        App_StateSetUi(&ui);
        next_tick += APP_UI_PERIOD_MS;
        osDelayUntil(next_tick);
    }
}

