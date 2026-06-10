/* Compass Icons 160x160 - RLE + BBox Compressed */
/* 29 frames, 160x160 display, ~140x120 actual content */
/* Format: RGB565 with RLE compression */
/* Flash saving: ~93% compared to raw 160x160x2B */

#ifndef COMPASS_ICONS_H
#define COMPASS_ICONS_H

#include <stdint.h>

#define COMPASS_ICON_COUNT       29U
#define COMPASS_ICON_WIDTH       160U
#define COMPASS_ICON_HEIGHT      160U
#define COMPASS_ICON_BBOX_MAX_W  140U
#define COMPASS_ICON_BBOX_MAX_H  120U

typedef struct {
    uint16_t color;
    uint16_t count;
} RLE_Entry;

typedef struct {
    uint8_t  bbox_x;
    uint8_t  bbox_y;
    uint8_t  bbox_w;
    uint8_t  bbox_h;
    uint16_t rle_count;
    const RLE_Entry *rle;
} Icon_Info;

void icon_decompress(const Icon_Info *info, uint16_t *output);
void icon_render(const Icon_Info *info, uint16_t *canvas);
const Icon_Info *CompassIcon_GetFrame(uint8_t frame);

extern const Icon_Info compass_icons[COMPASS_ICON_COUNT];

#endif
