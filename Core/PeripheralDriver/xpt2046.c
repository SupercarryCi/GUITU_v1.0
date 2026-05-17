#include "xpt2046.h"

#define XPT2046_CMD_X_DEFAULT  0xD0U
#define XPT2046_CMD_Y_DEFAULT  0x90U
#define XPT2046_READ_TIMES     5U
#define XPT2046_LOST_VAL       1U

static int xpt2046_has_io(const XPT2046_Handle *touch)
{
  return (touch != NULL) &&
         (touch->io.transfer != NULL) &&
         (touch->io.cs != NULL);
}

static uint16_t xpt2046_clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static uint16_t xpt2046_map_u16(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_max)
{
  uint32_t num;
  uint32_t den;

  if ((out_max == 0U) || (in_min == in_max))
  {
    return 0U;
  }

  value = xpt2046_clamp_u16(value, in_min, in_max);
  num = (uint32_t)(value - in_min) * (uint32_t)(out_max - 1U);
  den = (uint32_t)(in_max - in_min);

  return (uint16_t)(num / den);
}

static int xpt2046_read_adc_once(XPT2046_Handle *touch, uint8_t cmd, uint16_t *value)
{
  uint8_t tx[3];
  uint8_t rx[3];
  int ret;

  tx[0] = cmd;
  tx[1] = 0x00U;
  tx[2] = 0x00U;
  rx[0] = 0U;
  rx[1] = 0U;
  rx[2] = 0U;

  touch->io.cs(touch->io.ctx, 0U);
  ret = touch->io.transfer(touch->io.ctx, tx, rx, sizeof(tx));
  touch->io.cs(touch->io.ctx, 1U);
  if (ret != 0)
  {
    return ret;
  }

  /* XPT2046/ADS7846 hardware SPI read returns the 12-bit ADC value left-aligned. */
  *value = (uint16_t)((((uint16_t)rx[1] << 8) | rx[2]) >> 3);
  return 0;
}

static int xpt2046_read_filtered(XPT2046_Handle *touch, uint8_t cmd, uint16_t *value)
{
  uint16_t buf[XPT2046_READ_TIMES];
  uint16_t temp;
  uint32_t sum;
  uint8_t i;
  uint8_t j;

  for (i = 0U; i < XPT2046_READ_TIMES; i++)
  {
    if (xpt2046_read_adc_once(touch, cmd, &buf[i]) != 0)
    {
      return -1;
    }
  }

  for (i = 0U; i < (XPT2046_READ_TIMES - 1U); i++)
  {
    for (j = (uint8_t)(i + 1U); j < XPT2046_READ_TIMES; j++)
    {
      if (buf[i] > buf[j])
      {
        temp = buf[i];
        buf[i] = buf[j];
        buf[j] = temp;
      }
    }
  }

  sum = 0U;
  for (i = XPT2046_LOST_VAL; i < (XPT2046_READ_TIMES - XPT2046_LOST_VAL); i++)
  {
    sum += buf[i];
  }

  *value = (uint16_t)(sum / (XPT2046_READ_TIMES - (2U * XPT2046_LOST_VAL)));
  return 0;
}

int XPT2046_Init(XPT2046_Handle *touch, const XPT2046_IO *io)
{
  if ((touch == NULL) || (io == NULL))
  {
    return -1;
  }

  touch->io = *io;
  touch->cmd_x = XPT2046_CMD_X_DEFAULT;
  touch->cmd_y = XPT2046_CMD_Y_DEFAULT;
  touch->cal.raw_x_min = 200U;
  touch->cal.raw_x_max = 3900U;
  touch->cal.raw_y_min = 200U;
  touch->cal.raw_y_max = 3900U;
  touch->cal.screen_width = 320U;
  touch->cal.screen_height = 480U;
  touch->cal.swap_xy = 0U;
  touch->cal.invert_x = 0U;
  touch->cal.invert_y = 0U;

  if (!xpt2046_has_io(touch))
  {
    return -1;
  }

  touch->io.cs(touch->io.ctx, 1U);
  return 0;
}

void XPT2046_SetCalibration(XPT2046_Handle *touch, const XPT2046_Calibration *cal)
{
  if ((touch != NULL) && (cal != NULL))
  {
    touch->cal = *cal;
  }
}

void XPT2046_SetReadCommands(XPT2046_Handle *touch, uint8_t cmd_x, uint8_t cmd_y)
{
  if (touch != NULL)
  {
    touch->cmd_x = cmd_x;
    touch->cmd_y = cmd_y;
  }
}

int XPT2046_IsPressed(XPT2046_Handle *touch)
{
  if (!xpt2046_has_io(touch))
  {
    return 0;
  }
  if (touch->io.irq_read == NULL)
  {
    return 1;
  }

  return touch->io.irq_read(touch->io.ctx) == 0;
}

int XPT2046_ReadRaw(XPT2046_Handle *touch, XPT2046_Point *point)
{
  uint16_t x;
  uint16_t y;

  if (!xpt2046_has_io(touch) || (point == NULL))
  {
    return -1;
  }
  if (!XPT2046_IsPressed(touch))
  {
    return -1;
  }

  if (touch->io.delay_us != NULL)
  {
    touch->io.delay_us(touch->io.ctx, 10U);
  }

  if (xpt2046_read_filtered(touch, touch->cmd_x, &x) != 0)
  {
    return -1;
  }
  if (xpt2046_read_filtered(touch, touch->cmd_y, &y) != 0)
  {
    return -1;
  }

  point->x = x;
  point->y = y;
  return 0;
}

int XPT2046_ReadScreen(XPT2046_Handle *touch, XPT2046_Point *point)
{
  XPT2046_Point raw;
  uint16_t x;
  uint16_t y;
  uint16_t out_w;
  uint16_t out_h;

  if ((touch == NULL) || (point == NULL))
  {
    return -1;
  }
  if (XPT2046_ReadRaw(touch, &raw) != 0)
  {
    return -1;
  }

  out_w = touch->cal.screen_width;
  out_h = touch->cal.screen_height;
  x = xpt2046_map_u16(raw.x, touch->cal.raw_x_min, touch->cal.raw_x_max, out_w);
  y = xpt2046_map_u16(raw.y, touch->cal.raw_y_min, touch->cal.raw_y_max, out_h);

  if (touch->cal.invert_x != 0U)
  {
    x = (uint16_t)((out_w - 1U) - x);
  }
  if (touch->cal.invert_y != 0U)
  {
    y = (uint16_t)((out_h - 1U) - y);
  }

  if (touch->cal.swap_xy != 0U)
  {
    point->x = y;
    point->y = x;
  }
  else
  {
    point->x = x;
    point->y = y;
  }

  return 0;
}
