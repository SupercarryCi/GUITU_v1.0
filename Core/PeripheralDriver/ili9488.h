#ifndef ILI9488_H
#define ILI9488_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#define ILI9488_NATIVE_WIDTH   320U
#define ILI9488_NATIVE_HEIGHT  480U

#define ILI9488_COLOR_BLACK    0x0000U
#define ILI9488_COLOR_BLUE     0x001FU
#define ILI9488_COLOR_GREEN    0x07E0U
#define ILI9488_COLOR_CYAN     0x07FFU
#define ILI9488_COLOR_RED      0xF800U
#define ILI9488_COLOR_MAGENTA  0xF81FU
#define ILI9488_COLOR_YELLOW   0xFFE0U
#define ILI9488_COLOR_WHITE    0xFFFFU

typedef enum
{
  ILI9488_ROTATION_0 = 0,
  ILI9488_ROTATION_90,
  ILI9488_ROTATION_180,
  ILI9488_ROTATION_270
} ILI9488_Rotation;

typedef struct
{
  void *ctx;
  int (*write)(void *ctx, const uint8_t *data, size_t len);
  void (*cs)(void *ctx, uint8_t level);
  void (*dc)(void *ctx, uint8_t level);
  void (*reset)(void *ctx, uint8_t level);
  void (*backlight)(void *ctx, uint8_t level);
  void (*delay_ms)(void *ctx, uint32_t ms);
} ILI9488_IO;

typedef struct
{
  ILI9488_IO io;
  uint16_t width;
  uint16_t height;
  ILI9488_Rotation rotation;
} ILI9488_Handle;

int ILI9488_Init(ILI9488_Handle *lcd, const ILI9488_IO *io);
int ILI9488_SetRotation(ILI9488_Handle *lcd, ILI9488_Rotation rotation);
int ILI9488_SetWindow(ILI9488_Handle *lcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
int ILI9488_Fill(ILI9488_Handle *lcd, uint16_t color);
int ILI9488_FillRect(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
int ILI9488_DrawPixel(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t color);
int ILI9488_WritePixels(ILI9488_Handle *lcd, const uint16_t *colors, size_t count);
int ILI9488_DrawRGB565Image(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *colors);
int ILI9488_DisplayOn(ILI9488_Handle *lcd);
int ILI9488_DisplayOff(ILI9488_Handle *lcd);
void ILI9488_SetBacklight(ILI9488_Handle *lcd, uint8_t on);

#ifdef __cplusplus
}
#endif

#endif
