#include "ili9488.h"

#define ILI9488_CMD_SWRESET  0x01U
#define ILI9488_CMD_SLPOUT   0x11U
#define ILI9488_CMD_DISPOFF  0x28U
#define ILI9488_CMD_DISPON   0x29U
#define ILI9488_CMD_CASET    0x2AU
#define ILI9488_CMD_PASET    0x2BU
#define ILI9488_CMD_RAMWR    0x2CU
#define ILI9488_CMD_MADCTL   0x36U
#define ILI9488_CMD_COLMOD   0x3AU

#define ILI9488_MADCTL_MY    0x80U
#define ILI9488_MADCTL_MX    0x40U
#define ILI9488_MADCTL_MV    0x20U
#define ILI9488_MADCTL_BGR   0x08U

#define ILI9488_SPI_CHUNK    192U

typedef struct
{
  uint8_t cmd;
  uint8_t len;
  uint8_t data[16];
} ILI9488_InitCmd;

static const ILI9488_InitCmd ili9488_init_table[] =
{
  {0xF7U, 4U, {0xA9U, 0x51U, 0x2CU, 0x82U}},
  {0xC0U, 2U, {0x11U, 0x09U}},
  {0xC1U, 1U, {0x41U}},
  {0xC5U, 3U, {0x00U, 0x0AU, 0x80U}},
  {0xB1U, 2U, {0xB0U, 0x11U}},
  {0xB4U, 1U, {0x02U}},
  {0xB6U, 2U, {0x02U, 0x42U}},
  {0xB7U, 1U, {0xC6U}},
  {0xBEU, 2U, {0x00U, 0x04U}},
  {0xE9U, 1U, {0x00U}},
  {ILI9488_CMD_MADCTL, 1U, {ILI9488_MADCTL_BGR | ILI9488_MADCTL_MX | ILI9488_MADCTL_MV}},
  {ILI9488_CMD_COLMOD, 1U, {0x66U}},
  {0xE0U, 15U, {0x00U, 0x07U, 0x10U, 0x09U, 0x17U, 0x0BU, 0x41U, 0x89U, 0x4BU, 0x0AU, 0x0CU, 0x0EU, 0x18U, 0x1BU, 0x0FU}},
  {0xE1U, 15U, {0x00U, 0x17U, 0x1AU, 0x04U, 0x0EU, 0x06U, 0x2FU, 0x45U, 0x43U, 0x02U, 0x0AU, 0x09U, 0x32U, 0x36U, 0x0FU}}
};

static int ili9488_has_io(const ILI9488_Handle *lcd)
{
  return (lcd != NULL) &&
         (lcd->io.write != NULL) &&
         (lcd->io.cs != NULL) &&
         (lcd->io.dc != NULL) &&
         (lcd->io.delay_ms != NULL);
}

static int ili9488_write_bytes(ILI9488_Handle *lcd, const uint8_t *data, size_t len)
{
  if ((len == 0U) || (data == NULL))
  {
    return 0;
  }

  return lcd->io.write(lcd->io.ctx, data, len);
}

static int ili9488_write_command(ILI9488_Handle *lcd, uint8_t cmd)
{
  int ret;

  lcd->io.cs(lcd->io.ctx, 0U);
  lcd->io.dc(lcd->io.ctx, 0U);
  ret = ili9488_write_bytes(lcd, &cmd, 1U);
  lcd->io.cs(lcd->io.ctx, 1U);

  return ret;
}

static int ili9488_write_data(ILI9488_Handle *lcd, const uint8_t *data, size_t len)
{
  int ret;

  lcd->io.cs(lcd->io.ctx, 0U);
  lcd->io.dc(lcd->io.ctx, 1U);
  ret = ili9488_write_bytes(lcd, data, len);
  lcd->io.cs(lcd->io.ctx, 1U);

  return ret;
}

static int ili9488_write_command_data(ILI9488_Handle *lcd, uint8_t cmd, const uint8_t *data, size_t len)
{
  int ret;

  ret = ili9488_write_command(lcd, cmd);
  if (ret != 0)
  {
    return ret;
  }

  return ili9488_write_data(lcd, data, len);
}

static void ili9488_rgb565_to_rgb666(uint16_t color, uint8_t out[3])
{
  out[0] = (uint8_t)((color >> 8) & 0xF8U);
  out[1] = (uint8_t)((color >> 3) & 0xFCU);
  out[2] = (uint8_t)(color << 3);
}

int ILI9488_Init(ILI9488_Handle *lcd, const ILI9488_IO *io)
{
  size_t i;
  int ret;

  if ((lcd == NULL) || (io == NULL))
  {
    return -1;
  }

  lcd->io = *io;
  lcd->width = ILI9488_NATIVE_WIDTH;
  lcd->height = ILI9488_NATIVE_HEIGHT;
  lcd->rotation = ILI9488_ROTATION_0;

  if (!ili9488_has_io(lcd))
  {
    return -1;
  }

  lcd->io.cs(lcd->io.ctx, 1U);
  lcd->io.dc(lcd->io.ctx, 1U);

  if (lcd->io.reset != NULL)
  {
    lcd->io.reset(lcd->io.ctx, 0U);
    lcd->io.delay_ms(lcd->io.ctx, 100U);
    lcd->io.reset(lcd->io.ctx, 1U);
    lcd->io.delay_ms(lcd->io.ctx, 120U);
  }

  ret = ili9488_write_command(lcd, ILI9488_CMD_SWRESET);
  if (ret != 0)
  {
    return ret;
  }
  lcd->io.delay_ms(lcd->io.ctx, 120U);

  for (i = 0U; i < (sizeof(ili9488_init_table) / sizeof(ili9488_init_table[0])); i++)
  {
    ret = ili9488_write_command_data(lcd,
                                     ili9488_init_table[i].cmd,
                                     ili9488_init_table[i].data,
                                     ili9488_init_table[i].len);
    if (ret != 0)
    {
      return ret;
    }
  }

  ret = ili9488_write_command(lcd, ILI9488_CMD_SLPOUT);
  if (ret != 0)
  {
    return ret;
  }
  lcd->io.delay_ms(lcd->io.ctx, 120U);

  ret = ili9488_write_command(lcd, ILI9488_CMD_DISPON);
  if (ret != 0)
  {
    return ret;
  }

  ret = ILI9488_SetRotation(lcd, ILI9488_ROTATION_0);
  if (ret != 0)
  {
    return ret;
  }

  ILI9488_SetBacklight(lcd, 1U);
  return 0;
}

int ILI9488_SetRotation(ILI9488_Handle *lcd, ILI9488_Rotation rotation)
{
  uint8_t madctl;

  if (!ili9488_has_io(lcd))
  {
    return -1;
  }

  lcd->rotation = rotation;
  switch (rotation)
  {
    case ILI9488_ROTATION_0:
      lcd->width = ILI9488_NATIVE_WIDTH;
      lcd->height = ILI9488_NATIVE_HEIGHT;
      madctl = ILI9488_MADCTL_BGR;
      break;

    case ILI9488_ROTATION_90:
      lcd->width = ILI9488_NATIVE_HEIGHT;
      lcd->height = ILI9488_NATIVE_WIDTH;
      madctl = ILI9488_MADCTL_BGR | ILI9488_MADCTL_MX | ILI9488_MADCTL_MV;
      break;

    case ILI9488_ROTATION_180:
      lcd->width = ILI9488_NATIVE_WIDTH;
      lcd->height = ILI9488_NATIVE_HEIGHT;
      madctl = ILI9488_MADCTL_BGR | ILI9488_MADCTL_MX | ILI9488_MADCTL_MY;
      break;

    case ILI9488_ROTATION_270:
      lcd->width = ILI9488_NATIVE_HEIGHT;
      lcd->height = ILI9488_NATIVE_WIDTH;
      madctl = ILI9488_MADCTL_BGR | ILI9488_MADCTL_MY | ILI9488_MADCTL_MV;
      break;

    default:
      return -1;
  }

  return ili9488_write_command_data(lcd, ILI9488_CMD_MADCTL, &madctl, 1U);
}

int ILI9488_SetWindow(ILI9488_Handle *lcd, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t data[4];
  int ret;

  if (!ili9488_has_io(lcd))
  {
    return -1;
  }
  if ((x0 > x1) || (y0 > y1) || (x1 >= lcd->width) || (y1 >= lcd->height))
  {
    return -1;
  }

  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)x0;
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)x1;
  ret = ili9488_write_command_data(lcd, ILI9488_CMD_CASET, data, sizeof(data));
  if (ret != 0)
  {
    return ret;
  }

  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)y0;
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)y1;
  ret = ili9488_write_command_data(lcd, ILI9488_CMD_PASET, data, sizeof(data));
  if (ret != 0)
  {
    return ret;
  }

  return ili9488_write_command(lcd, ILI9488_CMD_RAMWR);
}

int ILI9488_WritePixels(ILI9488_Handle *lcd, const uint16_t *colors, size_t count)
{
  uint8_t buffer[ILI9488_SPI_CHUNK];
  size_t i;
  size_t packed;
  size_t todo;

  if (!ili9488_has_io(lcd) || ((colors == NULL) && (count != 0U)))
  {
    return -1;
  }

  lcd->io.cs(lcd->io.ctx, 0U);
  lcd->io.dc(lcd->io.ctx, 1U);

  while (count > 0U)
  {
    todo = count;
    if (todo > (sizeof(buffer) / 3U))
    {
      todo = sizeof(buffer) / 3U;
    }

    packed = 0U;
    for (i = 0U; i < todo; i++)
    {
      ili9488_rgb565_to_rgb666(colors[i], &buffer[packed]);
      packed += 3U;
    }

    if (ili9488_write_bytes(lcd, buffer, packed) != 0)
    {
      lcd->io.cs(lcd->io.ctx, 1U);
      return -1;
    }

    colors += todo;
    count -= todo;
  }

  lcd->io.cs(lcd->io.ctx, 1U);
  return 0;
}

int ILI9488_FillRect(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  uint8_t pixel[3];
  uint8_t buffer[ILI9488_SPI_CHUNK];
  uint32_t pixels;
  size_t i;
  size_t chunk_pixels;
  size_t chunk_len;
  int ret;

  if (!ili9488_has_io(lcd))
  {
    return -1;
  }
  if ((w == 0U) || (h == 0U))
  {
    return 0;
  }
  if (((uint32_t)x + (uint32_t)w > lcd->width) || ((uint32_t)y + (uint32_t)h > lcd->height))
  {
    return -1;
  }

  ret = ILI9488_SetWindow(lcd, x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
  if (ret != 0)
  {
    return ret;
  }

  ili9488_rgb565_to_rgb666(color, pixel);
  for (i = 0U; i < sizeof(buffer); i += 3U)
  {
    buffer[i] = pixel[0];
    buffer[i + 1U] = pixel[1];
    buffer[i + 2U] = pixel[2];
  }

  pixels = (uint32_t)w * (uint32_t)h;
  lcd->io.cs(lcd->io.ctx, 0U);
  lcd->io.dc(lcd->io.ctx, 1U);

  while (pixels > 0U)
  {
    chunk_pixels = sizeof(buffer) / 3U;
    if (pixels < chunk_pixels)
    {
      chunk_pixels = pixels;
    }
    chunk_len = chunk_pixels * 3U;

    if (ili9488_write_bytes(lcd, buffer, chunk_len) != 0)
    {
      lcd->io.cs(lcd->io.ctx, 1U);
      return -1;
    }

    pixels -= (uint32_t)chunk_pixels;
  }

  lcd->io.cs(lcd->io.ctx, 1U);
  return 0;
}

int ILI9488_Fill(ILI9488_Handle *lcd, uint16_t color)
{
  if (!ili9488_has_io(lcd))
  {
    return -1;
  }

  return ILI9488_FillRect(lcd, 0U, 0U, lcd->width, lcd->height, color);
}

int ILI9488_DrawPixel(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t color)
{
  int ret;

  ret = ILI9488_SetWindow(lcd, x, y, x, y);
  if (ret != 0)
  {
    return ret;
  }

  return ILI9488_WritePixels(lcd, &color, 1U);
}

int ILI9488_DrawRGB565Image(ILI9488_Handle *lcd, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *colors)
{
  int ret;
  size_t count;

  if ((colors == NULL) || (w == 0U) || (h == 0U))
  {
    return -1;
  }
  if (!ili9488_has_io(lcd))
  {
    return -1;
  }
  if (((uint32_t)x + (uint32_t)w > lcd->width) || ((uint32_t)y + (uint32_t)h > lcd->height))
  {
    return -1;
  }

  ret = ILI9488_SetWindow(lcd, x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
  if (ret != 0)
  {
    return ret;
  }

  count = (size_t)w * (size_t)h;
  return ILI9488_WritePixels(lcd, colors, count);
}

int ILI9488_DisplayOn(ILI9488_Handle *lcd)
{
  if (!ili9488_has_io(lcd))
  {
    return -1;
  }

  return ili9488_write_command(lcd, ILI9488_CMD_DISPON);
}

int ILI9488_DisplayOff(ILI9488_Handle *lcd)
{
  if (!ili9488_has_io(lcd))
  {
    return -1;
  }

  return ili9488_write_command(lcd, ILI9488_CMD_DISPOFF);
}

void ILI9488_SetBacklight(ILI9488_Handle *lcd, uint8_t on)
{
  if ((lcd != NULL) && (lcd->io.backlight != NULL))
  {
    lcd->io.backlight(lcd->io.ctx, on != 0U);
  }
}
