#include "tft_port_stm32_hal.h"

#include "main.h"
#include "spi.h"

/* Edit these defaults when CubeMX generated names differ in the target project. */
#ifndef TFT_LCD_SPI_HANDLE
#define TFT_LCD_SPI_HANDLE        hspi1
#endif

#ifndef TFT_TOUCH_SPI_HANDLE
#define TFT_TOUCH_SPI_HANDLE      hspi2
#endif

#ifndef TFT_LCD_CS_GPIO_Port
#define TFT_LCD_CS_GPIO_Port      SPI1_CS_GPIO_Port
#define TFT_LCD_CS_Pin            SPI1_CS_Pin
#endif

#ifndef TFT_LCD_DC_GPIO_Port
#define TFT_LCD_DC_GPIO_Port      D_C_GPIO_Port
#define TFT_LCD_DC_Pin            D_C_Pin
#endif

#ifndef TFT_LCD_RESET_GPIO_Port
#define TFT_LCD_RESET_GPIO_Port   RESET_t_GPIO_Port
#define TFT_LCD_RESET_Pin         RESET_t_Pin
#endif

#ifndef TFT_LCD_BL_GPIO_Port
#define TFT_LCD_BL_GPIO_Port      LED_GPIO_Port
#define TFT_LCD_BL_Pin            LED_Pin
#endif

#ifndef TFT_TOUCH_CS_GPIO_Port
#define TFT_TOUCH_CS_GPIO_Port    SPI2_CS_GPIO_Port
#define TFT_TOUCH_CS_Pin          SPI2_CS_Pin
#endif

#ifndef TFT_TOUCH_IRQ_GPIO_Port
#define TFT_TOUCH_IRQ_GPIO_Port   IRQ_GPIO_Port
#define TFT_TOUCH_IRQ_Pin         IRQ_Pin
#endif

ILI9488_Handle g_lcd;
XPT2046_Handle g_touch;

static int tft_lcd_write(void *ctx, const uint8_t *data, size_t len)
{
  SPI_HandleTypeDef *spi;
  size_t offset;
  size_t remain;
  uint16_t chunk;

  spi = (SPI_HandleTypeDef *)ctx;
  offset = 0U;
  while (offset < len)
  {
    remain = len - offset;
    chunk = (remain > 0xFFFFU) ? 0xFFFFU : (uint16_t)remain;

    if (HAL_SPI_Transmit(spi, (uint8_t *)&data[offset], chunk, HAL_MAX_DELAY) != HAL_OK)
    {
      return -1;
    }
    offset += chunk;
  }

  return 0;
}

static void tft_lcd_cs(void *ctx, uint8_t level)
{
  (void)ctx;
  HAL_GPIO_WritePin(TFT_LCD_CS_GPIO_Port, TFT_LCD_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tft_lcd_dc(void *ctx, uint8_t level)
{
  (void)ctx;
  HAL_GPIO_WritePin(TFT_LCD_DC_GPIO_Port, TFT_LCD_DC_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tft_lcd_reset(void *ctx, uint8_t level)
{
  (void)ctx;
  HAL_GPIO_WritePin(TFT_LCD_RESET_GPIO_Port, TFT_LCD_RESET_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tft_lcd_backlight(void *ctx, uint8_t level)
{
  (void)ctx;
  HAL_GPIO_WritePin(TFT_LCD_BL_GPIO_Port, TFT_LCD_BL_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void tft_delay_ms(void *ctx, uint32_t ms)
{
  (void)ctx;
  HAL_Delay(ms);
}

static int tft_touch_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
  SPI_HandleTypeDef *spi;
  size_t offset;
  size_t remain;
  uint16_t chunk;

  spi = (SPI_HandleTypeDef *)ctx;
  offset = 0U;
  while (offset < len)
  {
    remain = len - offset;
    chunk = (remain > 0xFFFFU) ? 0xFFFFU : (uint16_t)remain;

    if (HAL_SPI_TransmitReceive(spi, (uint8_t *)&tx[offset], &rx[offset], chunk, HAL_MAX_DELAY) != HAL_OK)
    {
      return -1;
    }
    offset += chunk;
  }

  return 0;
}

static void tft_touch_cs(void *ctx, uint8_t level)
{
  (void)ctx;
  HAL_GPIO_WritePin(TFT_TOUCH_CS_GPIO_Port, TFT_TOUCH_CS_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static int tft_touch_irq_read(void *ctx)
{
  (void)ctx;
  return HAL_GPIO_ReadPin(TFT_TOUCH_IRQ_GPIO_Port, TFT_TOUCH_IRQ_Pin) == GPIO_PIN_SET ? 1 : 0;
}

static void tft_touch_delay_us(void *ctx, uint32_t us)
{
  uint32_t start;
  uint32_t ticks;

  (void)ctx;
  ticks = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
  start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < ticks)
  {
  }
}

static void tft_delay_us_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void TFT_Port_Backlight(uint8_t on)
{
  tft_lcd_backlight(NULL, on);
}

int TFT_Port_Init(void)
{
  ILI9488_IO lcd_io;
  XPT2046_IO touch_io;

  tft_delay_us_init();

  /* Keep both chip selects idle high before any bus transaction. */
  tft_lcd_cs(NULL, 1U);
  tft_touch_cs(NULL, 1U);
  tft_lcd_dc(NULL, 1U);
  tft_lcd_backlight(NULL, 0U);

  lcd_io.ctx = &TFT_LCD_SPI_HANDLE;
  lcd_io.write = tft_lcd_write;
  lcd_io.cs = tft_lcd_cs;
  lcd_io.dc = tft_lcd_dc;
  lcd_io.reset = tft_lcd_reset;
  lcd_io.backlight = tft_lcd_backlight;
  lcd_io.delay_ms = tft_delay_ms;

  touch_io.ctx = &TFT_TOUCH_SPI_HANDLE;
  touch_io.transfer = tft_touch_transfer;
  touch_io.cs = tft_touch_cs;
  touch_io.irq_read = tft_touch_irq_read;
  touch_io.delay_us = tft_touch_delay_us;

  if (ILI9488_Init(&g_lcd, &lcd_io) != 0)
  {
    return -1;
  }

  /* Landscape UI: logical LCD size becomes 480 x 320. */
  if (ILI9488_SetRotation(&g_lcd, ILI9488_ROTATION_90) != 0)
  {
    return -1;
  }

  if (XPT2046_Init(&g_touch, &touch_io) != 0)
  {
    return -1;
  }

  /*
   * XPT2046 swaps coordinates after range mapping. For landscape, set the
   * pre-swap range as 320 x 480 so the final output is 480 x 320.
   */
  g_touch.cal.screen_width = g_lcd.height;
  g_touch.cal.screen_height = g_lcd.width;
  g_touch.cal.swap_xy = 1U;
  g_touch.cal.invert_x = 0U;
  g_touch.cal.invert_y = 0U;

  return 0;
}
