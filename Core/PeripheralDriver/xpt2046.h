#ifndef XPT2046_H
#define XPT2046_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

typedef struct
{
  uint16_t x;
  uint16_t y;
} XPT2046_Point;

typedef struct
{
  uint16_t raw_x_min;
  uint16_t raw_x_max;
  uint16_t raw_y_min;
  uint16_t raw_y_max;
  uint16_t screen_width;
  uint16_t screen_height;
  uint8_t swap_xy;
  uint8_t invert_x;
  uint8_t invert_y;
} XPT2046_Calibration;

typedef struct
{
  void *ctx;
  int (*transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
  void (*cs)(void *ctx, uint8_t level);
  int (*irq_read)(void *ctx);
  void (*delay_us)(void *ctx, uint32_t us);
} XPT2046_IO;

typedef struct
{
  XPT2046_IO io;
  XPT2046_Calibration cal;
  uint8_t cmd_x;
  uint8_t cmd_y;
} XPT2046_Handle;

int XPT2046_Init(XPT2046_Handle *touch, const XPT2046_IO *io);
void XPT2046_SetCalibration(XPT2046_Handle *touch, const XPT2046_Calibration *cal);
void XPT2046_SetReadCommands(XPT2046_Handle *touch, uint8_t cmd_x, uint8_t cmd_y);
int XPT2046_IsPressed(XPT2046_Handle *touch);
int XPT2046_ReadRaw(XPT2046_Handle *touch, XPT2046_Point *point);
int XPT2046_ReadScreen(XPT2046_Handle *touch, XPT2046_Point *point);

#ifdef __cplusplus
}
#endif

#endif
