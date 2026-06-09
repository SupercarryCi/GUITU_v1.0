#include "compass_icons.h"
#include <string.h>

void icon_decompress(const Icon_Info *info, uint16_t *output) {
    uint16_t idx = 0;
    for (uint16_t i = 0; i < info->rle_count; i++) {
        for (uint16_t j = 0; j < info->rle[i].count; j++) {
            output[idx++] = info->rle[i].color;
        }
    }
}

void icon_render(const Icon_Info *info, uint16_t *canvas) {
    memset(canvas, 0, 160 * 160 * 2);
    uint16_t idx = 0;
    for (uint16_t i = 0; i < info->rle_count; i++) {
        uint16_t color = info->rle[i].color;
        for (uint16_t j = 0; j < info->rle[i].count; j++, idx++) {
            uint8_t lx = idx % info->bbox_w;
            uint8_t ly = idx / info->bbox_w;
            uint16_t cx = info->bbox_x + lx;
            uint16_t cy = info->bbox_y + ly;
            if (cx < 160 && cy < 160) {
                canvas[cy * 160 + cx] = color;
            }
        }
    }
}

extern const RLE_Entry icon_1_rle[950];
extern const RLE_Entry icon_2_rle[960];
extern const RLE_Entry icon_3_rle[970];
extern const RLE_Entry icon_4_rle[950];
extern const RLE_Entry icon_5_rle[950];
extern const RLE_Entry icon_6_rle[940];
extern const RLE_Entry icon_7_rle[950];
extern const RLE_Entry icon_8_rle[930];
extern const RLE_Entry icon_9_rle[920];
extern const RLE_Entry icon_10_rle[930];
extern const RLE_Entry icon_11_rle[960];
extern const RLE_Entry icon_12_rle[940];
extern const RLE_Entry icon_13_rle[970];
extern const RLE_Entry icon_14_rle[960];
extern const RLE_Entry icon_15_rle[960];
extern const RLE_Entry icon_16_rle[950];
extern const RLE_Entry icon_17_rle[950];
extern const RLE_Entry icon_18_rle[950];
extern const RLE_Entry icon_19_rle[960];
extern const RLE_Entry icon_20_rle[950];
extern const RLE_Entry icon_21_rle[950];
extern const RLE_Entry icon_22_rle[940];
extern const RLE_Entry icon_23_rle[950];
extern const RLE_Entry icon_24_rle[920];
extern const RLE_Entry icon_25_rle[950];
extern const RLE_Entry icon_26_rle[930];
extern const RLE_Entry icon_27_rle[940];
extern const RLE_Entry icon_28_rle[960];
extern const RLE_Entry icon_29_rle[960];
extern const RLE_Entry icon_30_rle[950];

const Icon_Info compass_icons[30] = {
    { 10, 20,140,120, 950,icon_1_rle},
    { 10, 20,140,120, 960,icon_2_rle},
    { 10, 20,140,120, 970,icon_3_rle},
    { 10, 20,140,120, 950,icon_4_rle},
    { 10, 20,140,120, 950,icon_5_rle},
    { 10, 20,140,120, 940,icon_6_rle},
    { 10, 20,140,120, 950,icon_7_rle},
    { 10, 20,140,120, 930,icon_8_rle},
    { 10, 20,140,120, 920,icon_9_rle},
    { 10, 20,140,120, 930,icon_10_rle},
    { 10, 20,140,120, 960,icon_11_rle},
    { 10, 20,140,120, 940,icon_12_rle},
    { 10, 20,140,120, 970,icon_13_rle},
    { 10, 20,140,120, 960,icon_14_rle},
    { 10, 20,140,120, 960,icon_15_rle},
    { 10, 20,140,120, 950,icon_16_rle},
    { 10, 20,140,120, 950,icon_17_rle},
    { 10, 20,140,120, 950,icon_18_rle},
    { 10, 20,140,120, 960,icon_19_rle},
    { 10, 20,140,120, 950,icon_20_rle},
    { 10, 20,140,120, 950,icon_21_rle},
    { 10, 20,140,120, 940,icon_22_rle},
    { 10, 20,140,120, 950,icon_23_rle},
    { 10, 20,140,120, 920,icon_24_rle},
    { 10, 20,140,120, 950,icon_25_rle},
    { 10, 20,140,120, 930,icon_26_rle},
    { 10, 20,140,120, 940,icon_27_rle},
    { 10, 20,140,120, 960,icon_28_rle},
    { 10, 20,140,120, 960,icon_29_rle},
    { 10, 20,140,120, 950,icon_30_rle}
};
