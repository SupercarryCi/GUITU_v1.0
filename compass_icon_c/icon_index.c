/* Icon binary data index */
#include <stdint.h>

typedef struct {
    uint32_t offset;    // Offset in flash (bytes)
    uint16_t rle_count; // Number of RLE entries
    uint8_t  bbox_x;    // BBox X position
    uint8_t  bbox_y;    // BBox Y position
    uint8_t  bbox_w;    // BBox width
    uint8_t  bbox_h;    // BBox height
} Icon_Index;

const Icon_Index icon_index[30] = {
    {0x00000, 1487,  6, 12, 84, 72},  // icon_1
    {0x0173C, 1491,  6, 12, 84, 72},  // icon_2
    {0x02E88, 1497,  6, 12, 84, 72},  // icon_3
    {0x045EC, 1438,  6, 12, 84, 72},  // icon_4
    {0x05C64, 1458,  6, 12, 84, 72},  // icon_5
    {0x0732C, 1438,  6, 12, 84, 72},  // icon_6
    {0x089A4, 1437,  6, 12, 84, 72},  // icon_7
    {0x0A018, 1382,  6, 12, 84, 72},  // icon_8
    {0x0B5B0, 1371,  6, 12, 84, 72},  // icon_9
    {0x0CB1C, 1382,  6, 12, 84, 72},  // icon_10
    {0x0E0B4, 1450,  6, 12, 84, 72},  // icon_11
    {0x0F75C, 1419,  6, 12, 84, 72},  // icon_12
    {0x10D88, 1465,  6, 12, 84, 72},  // icon_13
    {0x1246C, 1454,  6, 12, 84, 72},  // icon_14
    {0x13B24, 1496,  6, 12, 84, 72},  // icon_15
    {0x15284, 1483,  6, 12, 84, 72},  // icon_16
    {0x169B0, 1482,  6, 12, 84, 72},  // icon_17
    {0x180D8, 1481,  6, 12, 84, 72},  // icon_18
    {0x197FC, 1494,  6, 12, 84, 72},  // icon_19
    {0x1AF54, 1447,  6, 12, 84, 72},  // icon_20
    {0x1C5F0, 1420,  6, 12, 84, 72},  // icon_21
    {0x1DC20, 1413,  6, 12, 84, 72},  // icon_22
    {0x1F234, 1434,  6, 12, 84, 72},  // icon_23
    {0x2089C, 1374,  6, 12, 84, 72},  // icon_24
    {0x21E14, 1435,  6, 12, 84, 72},  // icon_25
    {0x23480, 1415,  6, 12, 84, 72},  // icon_26
    {0x24A9C, 1427,  6, 12, 84, 72},  // icon_27
    {0x260E8, 1475,  6, 12, 84, 72},  // icon_28
    {0x277F4, 1490,  6, 12, 84, 72},  // icon_29
    {0x28F3C, 1487,  6, 12, 84, 72}  // icon_30
};

// Total binary size: 173688 bytes (169.6 KB)
