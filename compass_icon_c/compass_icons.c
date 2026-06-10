#include "compass_icons.h"

#include <stddef.h>
#include <string.h>

/*
 * 方向文件按文件名排序接入：
 * 1.c 为屏幕正上方，后续文件按顺时针方向递增。
 * 每个文件内部仍保留原始 icon_x_rle 符号名，所以这里用 include 聚合到同一编译单元。
 */
#include "1.c"
#include "2.c"
#include "3.c"
#include "4.c"
#include "5.c"
#include "6.c"
#include "7.c"
#include "8.c"
#include "9.c"
#include "10.c"
#include "11.c"
#include "12.c"
#include "13.c"
#include "14.c"
#include "15.c"
#include "16.c"
#include "17.c"
#include "18.c"
#include "19.c"
#include "20.c"
#include "21.c"
#include "22.c"
#include "23.c"
#include "24.c"
#include "25.c"
#include "26.c"
#include "27.c"
#include "28.c"
#include "29.c"

void icon_decompress(const Icon_Info *info, uint16_t *output)
{
    uint16_t idx;
    uint16_t i;
    uint16_t j;

    if ((info == NULL) || (output == NULL))
    {
        return;
    }

    idx = 0U;
    for (i = 0U; i < info->rle_count; i++)
    {
        for (j = 0U; j < info->rle[i].count; j++)
        {
            output[idx++] = info->rle[i].color;
        }
    }
}

void icon_render(const Icon_Info *info, uint16_t *canvas)
{
    uint32_t idx;
    uint16_t i;
    uint16_t j;

    if ((info == NULL) || (canvas == NULL))
    {
        return;
    }

    memset(canvas, 0, COMPASS_ICON_WIDTH * COMPASS_ICON_HEIGHT * sizeof(uint16_t));
    idx = 0U;
    for (i = 0U; i < info->rle_count; i++)
    {
        uint16_t color;

        color = info->rle[i].color;
        for (j = 0U; j < info->rle[i].count; j++, idx++)
        {
            uint16_t lx;
            uint16_t ly;
            uint16_t cx;
            uint16_t cy;

            lx = (uint16_t)(idx % info->bbox_w);
            ly = (uint16_t)(idx / info->bbox_w);
            cx = (uint16_t)(info->bbox_x + lx);
            cy = (uint16_t)(info->bbox_y + ly);
            if ((cx < COMPASS_ICON_WIDTH) && (cy < COMPASS_ICON_HEIGHT))
            {
                canvas[(uint32_t)cy * COMPASS_ICON_WIDTH + cx] = color;
            }
        }
    }
}

const Icon_Info compass_icons[COMPASS_ICON_COUNT] = {
    { 10U, 20U, 140U, 120U, 950U, icon_17_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_18_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_19_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_20_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_21_rle},
    { 10U, 20U, 140U, 120U, 940U, icon_22_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_23_rle},
    { 10U, 20U, 140U, 120U, 920U, icon_24_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_25_rle},
    { 10U, 20U, 140U, 120U, 930U, icon_26_rle},
    { 10U, 20U, 140U, 120U, 940U, icon_27_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_28_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_29_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_1_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_2_rle},
    { 10U, 20U, 140U, 120U, 970U, icon_3_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_4_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_5_rle},
    { 10U, 20U, 140U, 120U, 940U, icon_6_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_7_rle},
    { 10U, 20U, 140U, 120U, 930U, icon_8_rle},
    { 10U, 20U, 140U, 120U, 920U, icon_9_rle},
    { 10U, 20U, 140U, 120U, 930U, icon_10_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_11_rle},
    { 10U, 20U, 140U, 120U, 940U, icon_12_rle},
    { 10U, 20U, 140U, 120U, 970U, icon_13_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_14_rle},
    { 10U, 20U, 140U, 120U, 960U, icon_15_rle},
    { 10U, 20U, 140U, 120U, 950U, icon_16_rle}
};

const Icon_Info *CompassIcon_GetFrame(uint8_t frame)
{
    if ((frame == 0U) || (frame > COMPASS_ICON_COUNT))
    {
        return NULL;
    }

    return &compass_icons[frame - 1U];
}
