/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *
  * USB-C Power Meter Firmware
  *
  * BTN_MODE:
  *   Short press = next display page
  *   Long hold   = rotate OLED 90 degrees portrait / landscape
  *
  * D2_LED:
  *   Firmware heartbeat blink
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef BTN_MODE_Pin
#define BTN_MODE_Pin GPIO_PIN_0
#define BTN_MODE_GPIO_Port GPIOA
#endif

#ifndef D2_LED_Pin
#define D2_LED_Pin GPIO_PIN_2
#define D2_LED_GPIO_Port GPIOA
#endif

#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_BUF_SIZE           (OLED_WIDTH * OLED_HEIGHT / 8)

#define OLED_COL_OFFSET         2   /* SH1106 offset fix */
#define OLED_HARD_CLEAR_COLS    132

#define OLED_ADDR_3C            (0x3C << 1)
#define OLED_ADDR_3D            (0x3D << 1)

#define INA226_ADDR             (0x40 << 1)

#define INA226_REG_CONFIG       0x00
#define INA226_REG_BUS_V        0x02
#define INA226_REG_POWER        0x03
#define INA226_REG_CURRENT      0x04
#define INA226_REG_CALIB        0x05

#define INA226_CAL_VALUE        5120u  /* 10mOhm shunt */

#define BUTTON_DEBOUNCE_MS      35u
#define BUTTON_LONG_MS          1200u

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

static uint8_t oled_ready = 0;
static uint8_t ina_ready = 0;

static uint8_t oled_addr = OLED_ADDR_3C;

static uint8_t oled_portrait = 0;  /* 1 = portrait */

static uint8_t display_page = 0;
static uint8_t render_needed = 1;

static uint8_t oled_buf[OLED_BUF_SIZE];

static int32_t bus_mV = 0;
static int32_t current_mA = 0;
static int32_t power_mW = 0;

static uint64_t charge_uAh = 0;
static uint64_t energy_uWh = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static uint8_t INA226_Init(void);
static uint8_t INA226_ReadAll(void);

static uint8_t OLED_Init(void);
static void OLED_Clear(void);
static void OLED_HardClear(void);
static void OLED_Update(void);
static void OLED_SetPortrait(uint8_t portrait);

static uint8_t OLED_LogicalWidth(void);
static uint8_t OLED_LogicalHeight(void);
static uint16_t OLED_TextWidth(const char *s, uint8_t scale);

static void OLED_DrawText(uint8_t x, uint8_t y, const char *s, uint8_t scale);
static void OLED_DrawCenteredText(uint8_t y, const char *s, uint8_t scale);
static void OLED_DrawCenteredTextFit(uint8_t y, const char *s, uint8_t max_scale);

static void Button_Task(void);
static void Render_Display(void);
static void Render_Landscape(void);
static void Render_Portrait(void);
static void Error_Handler_Blink(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint8_t INA226_WriteReg(uint8_t reg, uint16_t value)
{
  uint8_t data[2];

  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)(value & 0xFF);

  return HAL_I2C_Mem_Write(&hi2c1,
                           INA226_ADDR,
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           data,
                           2,
                           100) == HAL_OK;
}

static uint8_t INA226_ReadReg(uint8_t reg, uint16_t *value)
{
  uint8_t data[2];

  if (HAL_I2C_Mem_Read(&hi2c1,
                       INA226_ADDR,
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       data,
                       2,
                       100) != HAL_OK)
  {
    return 0;
  }

  *value = ((uint16_t)data[0] << 8) | data[1];
  return 1;
}

static uint8_t INA226_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, INA226_ADDR, 2, 100) != HAL_OK)
  {
    return 0;
  }

  if (!INA226_WriteReg(INA226_REG_CALIB, INA226_CAL_VALUE))
  {
    return 0;
  }

  if (!INA226_WriteReg(INA226_REG_CONFIG, 0x4127))
  {
    return 0;
  }

  return 1;
}

static uint8_t INA226_ReadAll(void)
{
  uint16_t raw_bus = 0;
  uint16_t raw_current = 0;
  uint16_t raw_power = 0;

  if (!INA226_ReadReg(INA226_REG_BUS_V, &raw_bus))
  {
    return 0;
  }

  if (!INA226_ReadReg(INA226_REG_CURRENT, &raw_current))
  {
    return 0;
  }

  if (!INA226_ReadReg(INA226_REG_POWER, &raw_power))
  {
    return 0;
  }

  /* raw to mV/mA/mW */
  bus_mV = ((int32_t)raw_bus * 125) / 100;
  current_mA = ((int32_t)((int16_t)raw_current)) / 10;
  power_mW = ((int32_t)raw_power * 25) / 10;

  return 1;
}

static uint8_t OLED_Cmd(uint8_t cmd)
{
  uint8_t packet[2];

  packet[0] = 0x00;
  packet[1] = cmd;

  return HAL_I2C_Master_Transmit(&hi2c1, oled_addr, packet, 2, 100) == HAL_OK;
}

static uint8_t OLED_Data(uint8_t *data, uint16_t len)
{
  uint8_t packet[17];

  packet[0] = 0x40;

  while (len > 0)
  {
    uint8_t chunk = (len > 16) ? 16 : len;

    memcpy(&packet[1], data, chunk);

    if (HAL_I2C_Master_Transmit(&hi2c1,
                                oled_addr,
                                packet,
                                chunk + 1,
                                100) != HAL_OK)
    {
      return 0;
    }

    data += chunk;
    len -= chunk;
  }

  return 1;
}

static uint8_t OLED_Init(void)
{
  if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_3C, 2, 100) == HAL_OK)
  {
    oled_addr = OLED_ADDR_3C;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_ADDR_3D, 2, 100) == HAL_OK)
  {
    oled_addr = OLED_ADDR_3D;
  }
  else
  {
    return 0;
  }

  HAL_Delay(100);

  OLED_Cmd(0xAE);              /* Display off */
  OLED_Cmd(0xD5); OLED_Cmd(0x80);
  OLED_Cmd(0xA8); OLED_Cmd(0x3F);
  OLED_Cmd(0xD3); OLED_Cmd(0x00);
  OLED_Cmd(0x40);
  OLED_Cmd(0x8D); OLED_Cmd(0x14);
  OLED_Cmd(0x20); OLED_Cmd(0x02);  /* Page addressing mode */

  OLED_Cmd(0xA1);
  OLED_Cmd(0xC8);

  OLED_Cmd(0xDA); OLED_Cmd(0x12);
  OLED_Cmd(0x81); OLED_Cmd(0x8F);
  OLED_Cmd(0xD9); OLED_Cmd(0xF1);
  OLED_Cmd(0xDB); OLED_Cmd(0x40);
  OLED_Cmd(0xA4);
  OLED_Cmd(0xA6);

  OLED_HardClear();
  OLED_HardClear();

  OLED_Cmd(0xAF);              /* Display on */

  OLED_Clear();
  OLED_DrawCenteredText(10, "USB POWER", 3);
  OLED_DrawCenteredText(38, "METER", 3);
  OLED_Update();

  HAL_Delay(700);

  return 1;
}

static void OLED_SetPortrait(uint8_t portrait)
{
  oled_portrait = portrait ? 1 : 0;

  OLED_Clear();
  OLED_HardClear();

  render_needed = 1;
}

static void OLED_Clear(void)
{
  memset(oled_buf, 0x00, sizeof(oled_buf));
}

static void OLED_HardClear(void)
{
  uint8_t zeros[16] = {0};

  for (uint8_t page = 0; page < 8; page++)
  {
    OLED_Cmd(0xB0 + page);
    OLED_Cmd(0x00);
    OLED_Cmd(0x10);

    uint16_t remaining = OLED_HARD_CLEAR_COLS;

    while (remaining > 0)
    {
      uint8_t chunk = (remaining > 16) ? 16 : remaining;
      OLED_Data(zeros, chunk);
      remaining -= chunk;
    }
  }
}

static void OLED_Update(void)
{
  for (uint8_t page = 0; page < 8; page++)
  {
    OLED_Cmd(0xB0 + page);
    OLED_Cmd(0x00 | (OLED_COL_OFFSET & 0x0F));
    OLED_Cmd(0x10 | ((OLED_COL_OFFSET >> 4) & 0x0F));
    OLED_Data(&oled_buf[OLED_WIDTH * page], OLED_WIDTH);
  }
}

static uint8_t OLED_LogicalWidth(void)
{
  if (oled_portrait)
  {
    return OLED_HEIGHT;
  }

  return OLED_WIDTH;
}

static uint8_t OLED_LogicalHeight(void)
{
  if (oled_portrait)
  {
    return OLED_WIDTH;
  }

  return OLED_HEIGHT;
}

static void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t color)
{
  uint8_t physical_x;
  uint8_t physical_y;

  if (x >= OLED_LogicalWidth() || y >= OLED_LogicalHeight())
  {
    return;
  }

  if (oled_portrait)
  {
    /* rotate 90 */
    physical_x = y;
    physical_y = OLED_HEIGHT - 1 - x;
  }
  else
  {
    physical_x = x;
    physical_y = y;
  }

  if (color)
  {
    oled_buf[physical_x + (physical_y / 8) * OLED_WIDTH] |= (1 << (physical_y & 7));
  }
  else
  {
    oled_buf[physical_x + (physical_y / 8) * OLED_WIDTH] &= ~(1 << (physical_y & 7));
  }
}

static void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  for (uint8_t yy = 0; yy < h; yy++)
  {
    for (uint8_t xx = 0; xx < w; xx++)
    {
      OLED_DrawPixel(x + xx, y + yy, 1);
    }
  }
}

static void Glyph3x5(char c, uint8_t r[5])
{
  if (c >= 'a' && c <= 'z')
  {
    c = c - 32;
  }

#define GLYPH(a,b,c,d,e) do { r[0]=(a); r[1]=(b); r[2]=(c); r[3]=(d); r[4]=(e); return; } while (0)

  switch (c)
  {
    case '0': GLYPH(7,5,5,5,7);
    case '1': GLYPH(2,6,2,2,7);
    case '2': GLYPH(7,1,7,4,7);
    case '3': GLYPH(7,1,7,1,7);
    case '4': GLYPH(5,5,7,1,1);
    case '5': GLYPH(7,4,7,1,7);
    case '6': GLYPH(7,4,7,5,7);
    case '7': GLYPH(7,1,2,2,2);
    case '8': GLYPH(7,5,7,5,7);
    case '9': GLYPH(7,5,7,1,7);

    case 'A': GLYPH(7,5,7,5,5);
    case 'B': GLYPH(6,5,6,5,6);
    case 'C': GLYPH(7,4,4,4,7);
    case 'D': GLYPH(6,5,5,5,6);
    case 'E': GLYPH(7,4,6,4,7);
    case 'F': GLYPH(7,4,6,4,4);
    case 'G': GLYPH(7,4,5,5,7);
    case 'H': GLYPH(5,5,7,5,5);
    case 'I': GLYPH(7,2,2,2,7);
    case 'J': GLYPH(1,1,1,5,7);
    case 'K': GLYPH(5,5,6,5,5);
    case 'L': GLYPH(4,4,4,4,7);
    case 'M': GLYPH(5,7,7,5,5);
    case 'N': GLYPH(5,7,7,7,5);
    case 'O': GLYPH(7,5,5,5,7);
    case 'P': GLYPH(7,5,7,4,4);
    case 'Q': GLYPH(7,5,5,7,1);
    case 'R': GLYPH(7,5,7,6,5);
    case 'S': GLYPH(7,4,7,1,7);
    case 'T': GLYPH(7,2,2,2,2);
    case 'U': GLYPH(5,5,5,5,7);
    case 'V': GLYPH(5,5,5,5,2);
    case 'W': GLYPH(5,5,7,7,5);
    case 'X': GLYPH(5,5,2,5,5);
    case 'Y': GLYPH(5,5,2,2,2);
    case 'Z': GLYPH(7,1,2,4,7);

    case '.': GLYPH(0,0,0,0,2);
    case '-': GLYPH(0,0,7,0,0);
    case ':': GLYPH(0,2,0,2,0);
    case '/': GLYPH(1,1,2,4,4);
    case ' ': GLYPH(0,0,0,0,0);
    default:  GLYPH(0,0,0,0,0);
  }

#undef GLYPH
}

static void OLED_DrawChar(uint8_t x, uint8_t y, char c, uint8_t scale)
{
  uint8_t rows[5];

  Glyph3x5(c, rows);

  for (uint8_t row = 0; row < 5; row++)
  {
    for (uint8_t col = 0; col < 3; col++)
    {
      if (rows[row] & (1 << (2 - col)))
      {
        OLED_FillRect(x + col * scale,
                      y + row * scale,
                      scale,
                      scale);
      }
    }
  }
}

static void OLED_DrawText(uint8_t x, uint8_t y, const char *s, uint8_t scale)
{
  while (*s)
  {
    OLED_DrawChar(x, y, *s, scale);
    x += 4 * scale;
    s++;

    if (x > (OLED_LogicalWidth() - 4 * scale))
    {
      break;
    }
  }
}

static uint16_t OLED_TextWidth(const char *s, uint8_t scale)
{
  uint16_t count = 0;

  while (*s)
  {
    count++;
    s++;
  }

  return count * 4 * scale;
}

static void OLED_DrawCenteredText(uint8_t y, const char *s, uint8_t scale)
{
  uint16_t width = OLED_TextWidth(s, scale);
  uint8_t x = 0;

  if (width < OLED_LogicalWidth())
  {
    x = (OLED_LogicalWidth() - width) / 2;
  }

  OLED_DrawText(x, y, s, scale);
}

static void OLED_DrawCenteredTextFit(uint8_t y, const char *s, uint8_t max_scale)
{
  uint8_t scale = max_scale;

  while (scale > 1 && OLED_TextWidth(s, scale) > OLED_LogicalWidth())
  {
    scale--;
  }

  OLED_DrawCenteredText(y, s, scale);
}

static void Button_Task(void)
{
  uint32_t now = HAL_GetTick();

  static GPIO_PinState last_raw = GPIO_PIN_SET;
  static GPIO_PinState stable = GPIO_PIN_SET;
  static uint32_t last_change_ms = 0;
  static uint32_t press_start_ms = 0;
  static uint8_t long_done = 0;

  GPIO_PinState raw = HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin);

  if (raw != last_raw)
  {
    last_raw = raw;
    last_change_ms = now;
  }

  if ((now - last_change_ms) >= BUTTON_DEBOUNCE_MS)
  {
    if (raw != stable)
    {
      stable = raw;

      if (stable == GPIO_PIN_RESET)
      {
        press_start_ms = now;
        long_done = 0;
      }
      else
      {
        if (!long_done)
        {
          display_page++;
          if (display_page > 4)
          {
            display_page = 0;
          }

          render_needed = 1;
        }
      }
    }

    if (stable == GPIO_PIN_RESET && !long_done)
    {
      if ((now - press_start_ms) >= BUTTON_LONG_MS)
      {
        long_done = 1;
        OLED_SetPortrait(!oled_portrait);
      }
    }
  }
}

static void Render_Display(void)
{
  OLED_Clear();

  if (!ina_ready)
  {
    OLED_DrawCenteredText(2, "USB POWER", 2);
    OLED_DrawCenteredText(22, "INA226 ERR", 2);
    OLED_DrawCenteredText(42, "CHECK I2C", 2);
    OLED_Update();
    return;
  }

  if (oled_portrait)
  {
    Render_Portrait();
  }
  else
  {
    Render_Landscape();
  }

  OLED_Update();
}

static void Render_Landscape(void)
{
  char line[32];

  if (display_page == 0)
  {
    OLED_DrawCenteredText(0, "USB POWER METER", 2);

    snprintf(line, sizeof(line), "%ld.%02ldV",
             bus_mV / 1000,
             (bus_mV % 1000) / 10);
    OLED_DrawText(2, 18, "VOLT", 2);
    OLED_DrawText(44, 18, line, 2);

    int32_t abs_i = current_mA;
    if (abs_i < 0)
    {
      abs_i = -abs_i;
    }

    if (current_mA < 0)
    {
      snprintf(line, sizeof(line), "-%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }
    else
    {
      snprintf(line, sizeof(line), "%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }

    OLED_DrawText(2, 33, "CURR", 2);
    OLED_DrawText(44, 33, line, 2);

    snprintf(line, sizeof(line), "%ld.%02ldW",
             power_mW / 1000,
             (power_mW % 1000) / 10);

    OLED_DrawText(2, 48, "PWR", 2);
    OLED_DrawText(44, 48, line, 2);
  }
  else if (display_page == 1)
  {
    OLED_DrawCenteredText(3, "VOLTAGE", 2);

    snprintf(line, sizeof(line), "%ld.%02ldV",
             bus_mV / 1000,
             (bus_mV % 1000) / 10);

    OLED_DrawCenteredTextFit(26, line, 3);
  }
  else if (display_page == 2)
  {
    OLED_DrawCenteredText(3, "CURRENT", 2);

    int32_t abs_i = current_mA;
    if (abs_i < 0)
    {
      abs_i = -abs_i;
    }

    if (current_mA < 0)
    {
      snprintf(line, sizeof(line), "-%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }
    else
    {
      snprintf(line, sizeof(line), "%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }

    OLED_DrawCenteredTextFit(26, line, 3);
  }
  else if (display_page == 3)
  {
    OLED_DrawCenteredText(3, "POWER", 2);

    snprintf(line, sizeof(line), "%ld.%02ldW",
             power_mW / 1000,
             (power_mW % 1000) / 10);

    OLED_DrawCenteredTextFit(26, line, 3);
  }
  else
  {
    OLED_DrawCenteredText(0, "ENERGY", 2);

    snprintf(line, sizeof(line), "mAh %lu.%03lu",
             (uint32_t)(charge_uAh / 1000),
             (uint32_t)(charge_uAh % 1000));
    OLED_DrawCenteredText(22, line, 2);

    snprintf(line, sizeof(line), "mWh %lu.%03lu",
             (uint32_t)(energy_uWh / 1000),
             (uint32_t)(energy_uWh % 1000));
    OLED_DrawCenteredText(42, line, 2);
  }
}

static void Render_Portrait(void)
{
  char line[32];

  if (display_page == 0)
  {
    OLED_DrawCenteredText(0, "USB PWR", 2);

    snprintf(line, sizeof(line), "V %ld.%02ld",
             bus_mV / 1000,
             (bus_mV % 1000) / 10);
    OLED_DrawCenteredText(22, line, 2);

    int32_t abs_i = current_mA;
    if (abs_i < 0)
    {
      abs_i = -abs_i;
    }

    if (current_mA < 0)
    {
      snprintf(line, sizeof(line), "I -%ld.%02ld",
               abs_i / 1000,
               (abs_i % 1000) / 10);
    }
    else
    {
      snprintf(line, sizeof(line), "I %ld.%02ld",
               abs_i / 1000,
               (abs_i % 1000) / 10);
    }

    OLED_DrawCenteredText(47, line, 2);

    snprintf(line, sizeof(line), "P %ld.%02ld",
             power_mW / 1000,
             (power_mW % 1000) / 10);
    OLED_DrawCenteredText(72, line, 2);

    OLED_DrawCenteredText(102, "HOLD ROT", 1);
  }
  else if (display_page == 1)
  {
    OLED_DrawCenteredText(6, "VOLT", 2);

    snprintf(line, sizeof(line), "%ld.%02ldV",
             bus_mV / 1000,
             (bus_mV % 1000) / 10);

    OLED_DrawCenteredTextFit(52, line, 3);
  }
  else if (display_page == 2)
  {
    OLED_DrawCenteredText(6, "CURR", 2);

    int32_t abs_i = current_mA;
    if (abs_i < 0)
    {
      abs_i = -abs_i;
    }

    if (current_mA < 0)
    {
      snprintf(line, sizeof(line), "-%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }
    else
    {
      snprintf(line, sizeof(line), "%ld.%03ldA",
               abs_i / 1000,
               abs_i % 1000);
    }

    OLED_DrawCenteredTextFit(52, line, 3);
  }
  else if (display_page == 3)
  {
    OLED_DrawCenteredText(6, "POWER", 2);

    snprintf(line, sizeof(line), "%ld.%02ldW",
             power_mW / 1000,
             (power_mW % 1000) / 10);

    OLED_DrawCenteredTextFit(52, line, 3);
  }
  else
  {
    OLED_DrawCenteredText(0, "ENERGY", 2);

    OLED_DrawCenteredText(25, "mAh", 2);
    snprintf(line, sizeof(line), "%lu.%03lu",
             (uint32_t)(charge_uAh / 1000),
             (uint32_t)(charge_uAh % 1000));
    OLED_DrawCenteredTextFit(43, line, 2);

    OLED_DrawCenteredText(72, "mWh", 2);
    snprintf(line, sizeof(line), "%lu.%03lu",
             (uint32_t)(energy_uWh / 1000),
             (uint32_t)(energy_uWh % 1000));
    OLED_DrawCenteredTextFit(90, line, 2);
  }
}

static void Error_Handler_Blink(void)
{
  while (1)
  {
    HAL_GPIO_TogglePin(D2_LED_GPIO_Port, D2_LED_Pin);
    HAL_Delay(100);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  HAL_Delay(100);

  oled_ready = OLED_Init();
  ina_ready = INA226_Init();

  uint32_t last_led_ms = HAL_GetTick();
  uint32_t last_measure_ms = HAL_GetTick();
  uint32_t last_integrate_ms = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    Button_Task();

    if ((now - last_led_ms) >= 250)
    {
      last_led_ms = now;
      HAL_GPIO_TogglePin(D2_LED_GPIO_Port, D2_LED_Pin);
    }

    if ((now - last_measure_ms) >= 250)
    {
      uint32_t dt_ms = now - last_integrate_ms;

      last_measure_ms = now;
      last_integrate_ms = now;

      if (ina_ready)
      {
        if (INA226_ReadAll())
        {
          if (current_mA > 0)
          {
            charge_uAh += ((uint64_t)current_mA * dt_ms) / 3600u;
          }

          if (power_mW > 0)
          {
            energy_uWh += ((uint64_t)power_mW * dt_ms) / 3600u;
          }

          render_needed = 1;
        }
        else
        {
          ina_ready = 0;
          render_needed = 1;
        }
      }
      else
      {
        ina_ready = INA226_Init();
        render_needed = 1;
      }
    }

    if (render_needed && oled_ready)
    {
      render_needed = 0;
      Render_Display();
    }

    HAL_Delay(5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(D2_LED_GPIO_Port, D2_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : D2_LED_Pin */
  GPIO_InitStruct.Pin = D2_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(D2_LED_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* active low */
  GPIO_InitStruct.Pin = BTN_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_MODE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */

  __disable_irq();
  Error_Handler_Blink();

  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */

  (void)file;
  (void)line;

  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
