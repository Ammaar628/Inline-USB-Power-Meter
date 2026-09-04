#include "power_meter_app.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define INA226_ADDR             (0x40U << 1)
#define OLED_ADDR_3C            (0x3CU << 1)
#define OLED_ADDR_3D            (0x3DU << 1)

#define INA226_REG_CONFIG       0x00U
#define INA226_REG_BUS_VOLTAGE  0x02U
#define INA226_REG_POWER        0x03U
#define INA226_REG_CURRENT      0x04U
#define INA226_REG_CALIBRATION  0x05U

#define OLED_WIDTH              128U
#define OLED_HEIGHT             64U

/*
 * INA226 setup for this PCB:
 *   Shunt resistance: 10 milliohms
 *   Current LSB:      0.1 mA/bit
 *   Calibration:     0.00512 / (0.0001 A * 0.01 ohm) = 5120
 *   Power LSB:        25 * Current LSB = 2.5 mW/bit
 *
 * Configuration 0x4527:
 *   16-sample averaging
 *   1.1 ms bus conversion
 *   1.1 ms shunt conversion
 *   continuous shunt and bus measurement
 */
#define INA226_CONFIG_VALUE     0x4527U
#define INA226_CAL_VALUE        5120U

static I2C_HandleTypeDef *app_i2c;
static uint16_t oled_address;
static uint8_t oled_present;
static uint8_t ina226_present;
static uint8_t oled_buffer[OLED_WIDTH * OLED_HEIGHT / 8U];
static uint32_t last_update_ms;
static uint32_t last_led_ms;

static HAL_StatusTypeDef ina226_write_u16(uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;

    return HAL_I2C_Mem_Write(app_i2c, INA226_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U, 100U);
}

static HAL_StatusTypeDef ina226_read_u16(uint8_t reg, uint16_t *value)
{
    uint8_t data[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(app_i2c, INA226_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, data, 2U, 100U);
    if (status == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }

    return status;
}

static HAL_StatusTypeDef oled_write_command(uint8_t command)
{
    uint8_t data[2] = {0x00U, command};
    return HAL_I2C_Master_Transmit(app_i2c, oled_address, data, 2U, 100U);
}

static void oled_clear(void)
{
    memset(oled_buffer, 0, sizeof(oled_buffer));
}

static void oled_set_pixel(uint8_t x, uint8_t y)
{
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT))
    {
        return;
    }

    oled_buffer[x + ((uint16_t)(y / 8U) * OLED_WIDTH)] |=
        (uint8_t)(1U << (y & 7U));
}

static const uint8_t *glyph_for(char c)
{
    static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t minus[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};

    static const uint8_t digits[10][5] =
    {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E}
    };

    static const uint8_t letters[26][5] =
    {
        {0x7E, 0x11, 0x11, 0x11, 0x7E}, /* A */
        {0x7F, 0x49, 0x49, 0x49, 0x36}, /* B */
        {0x3E, 0x41, 0x41, 0x41, 0x22}, /* C */
        {0x7F, 0x41, 0x41, 0x22, 0x1C}, /* D */
        {0x7F, 0x49, 0x49, 0x49, 0x41}, /* E */
        {0x7F, 0x09, 0x09, 0x09, 0x01}, /* F */
        {0x3E, 0x41, 0x49, 0x49, 0x7A}, /* G */
        {0x7F, 0x08, 0x08, 0x08, 0x7F}, /* H */
        {0x00, 0x41, 0x7F, 0x41, 0x00}, /* I */
        {0x20, 0x40, 0x41, 0x3F, 0x01}, /* J */
        {0x7F, 0x08, 0x14, 0x22, 0x41}, /* K */
        {0x7F, 0x40, 0x40, 0x40, 0x40}, /* L */
        {0x7F, 0x02, 0x0C, 0x02, 0x7F}, /* M */
        {0x7F, 0x04, 0x08, 0x10, 0x7F}, /* N */
        {0x3E, 0x41, 0x41, 0x41, 0x3E}, /* O */
        {0x7F, 0x09, 0x09, 0x09, 0x06}, /* P */
        {0x3E, 0x41, 0x51, 0x21, 0x5E}, /* Q */
        {0x7F, 0x09, 0x19, 0x29, 0x46}, /* R */
        {0x46, 0x49, 0x49, 0x49, 0x31}, /* S */
        {0x01, 0x01, 0x7F, 0x01, 0x01}, /* T */
        {0x3F, 0x40, 0x40, 0x40, 0x3F}, /* U */
        {0x1F, 0x20, 0x40, 0x20, 0x1F}, /* V */
        {0x3F, 0x40, 0x38, 0x40, 0x3F}, /* W */
        {0x63, 0x14, 0x08, 0x14, 0x63}, /* X */
        {0x07, 0x08, 0x70, 0x08, 0x07}, /* Y */
        {0x61, 0x51, 0x49, 0x45, 0x43}  /* Z */
    };

    if ((c >= '0') && (c <= '9'))
    {
        return digits[(uint8_t)c - (uint8_t)'0'];
    }
    if ((c >= 'A') && (c <= 'Z'))
    {
        return letters[(uint8_t)c - (uint8_t)'A'];
    }
    if (c == '-')
    {
        return minus;
    }
    if (c == '.')
    {
        return dot;
    }
    if (c == ':')
    {
        return colon;
    }

    return blank;
}

static void oled_draw_char(uint8_t x, uint8_t y, char c, uint8_t scale)
{
    const uint8_t *glyph = glyph_for(c);
    uint8_t column;
    uint8_t row;
    uint8_t sx;
    uint8_t sy;

    for (column = 0U; column < 5U; column++)
    {
        for (row = 0U; row < 7U; row++)
        {
            if ((glyph[column] & (1U << row)) != 0U)
            {
                for (sx = 0U; sx < scale; sx++)
                {
                    for (sy = 0U; sy < scale; sy++)
                    {
                        oled_set_pixel((uint8_t)(x + column * scale + sx),
                                       (uint8_t)(y + row * scale + sy));
                    }
                }
            }
        }
    }
}

static void oled_draw_text(uint8_t x, uint8_t y, const char *text, uint8_t scale)
{
    while (*text != '\0')
    {
        oled_draw_char(x, y, *text, scale);
        x = (uint8_t)(x + 6U * scale);
        text++;
    }
}

static void oled_update(void)
{
    uint8_t page;
    uint8_t tx[OLED_WIDTH + 1U];

    tx[0] = 0x40U;

    for (page = 0U; page < 8U; page++)
    {
        /*
         * Page addressing works with SSD1306 and SH1106.
         * Column 2 accounts for the common SH1106 132-column RAM offset.
         * On SSD1306 modules this only shifts the image by two pixels.
         */
        (void)oled_write_command((uint8_t)(0xB0U + page));
        (void)oled_write_command(0x02U);
        (void)oled_write_command(0x10U);

        memcpy(&tx[1], &oled_buffer[(uint16_t)page * OLED_WIDTH], OLED_WIDTH);
        (void)HAL_I2C_Master_Transmit(app_i2c, oled_address, tx,
                                      sizeof(tx), 200U);
    }
}

static void oled_init(void)
{
    static const uint8_t commands[] =
    {
        0xAE,       /* display off */
        0xD5, 0x80, /* display clock */
        0xA8, 0x3F, /* multiplex: 64 rows */
        0xD3, 0x00, /* display offset */
        0x40,       /* start line */
        0x8D, 0x14, /* SSD1306 charge pump on */
        0xAD, 0x8B, /* SH1106 DC-DC on */
        0x20, 0x02, /* page addressing mode on SSD1306 */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction */
        0xDA, 0x12, /* COM pin configuration */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* pre-charge */
        0xDB, 0x40, /* VCOM detect */
        0xA4,       /* use display RAM */
        0xA6,       /* normal display */
        0xAF        /* display on */
    };
    uint32_t i;

    HAL_Delay(100U);
    for (i = 0U; i < sizeof(commands); i++)
    {
        (void)oled_write_command(commands[i]);
    }

    oled_clear();
    oled_update();
}

static void show_status(const char *line1, const char *line2)
{
    if (oled_present == 0U)
    {
        return;
    }

    oled_clear();
    oled_draw_text(10U, 14U, line1, 1U);
    oled_draw_text(10U, 34U, line2, 1U);
    oled_update();
}

static void format_voltage(char *text, size_t size, uint32_t millivolts)
{
    (void)snprintf(text, size, "%lu.%02lu V",
                   (unsigned long)(millivolts / 1000U),
                   (unsigned long)((millivolts % 1000U) / 10U));
}

static void format_current(char *text, size_t size, int32_t milliamps)
{
    uint32_t magnitude;

    if (milliamps < 0)
    {
        magnitude = (uint32_t)(-milliamps);
        (void)snprintf(text, size, "-%lu.%03lu A",
                       (unsigned long)(magnitude / 1000U),
                       (unsigned long)(magnitude % 1000U));
    }
    else
    {
        magnitude = (uint32_t)milliamps;
        (void)snprintf(text, size, "%lu.%03lu A",
                       (unsigned long)(magnitude / 1000U),
                       (unsigned long)(magnitude % 1000U));
    }
}

static void format_power(char *text, size_t size, uint32_t milliwatts)
{
    (void)snprintf(text, size, "%lu.%02lu W",
                   (unsigned long)(milliwatts / 1000U),
                   (unsigned long)((milliwatts % 1000U) / 10U));
}

void PowerMeter_Init(I2C_HandleTypeDef *hi2c)
{
    app_i2c = hi2c;
    oled_present = 0U;
    ina226_present = 0U;

    if (HAL_I2C_IsDeviceReady(app_i2c, OLED_ADDR_3C, 3U, 100U) == HAL_OK)
    {
        oled_address = OLED_ADDR_3C;
        oled_present = 1U;
    }
    else if (HAL_I2C_IsDeviceReady(app_i2c, OLED_ADDR_3D, 3U, 100U) == HAL_OK)
    {
        oled_address = OLED_ADDR_3D;
        oled_present = 1U;
    }

    if (oled_present != 0U)
    {
        oled_init();
        show_status("USB POWER METER", "STARTING");
    }

    if (HAL_I2C_IsDeviceReady(app_i2c, INA226_ADDR, 3U, 100U) == HAL_OK)
    {
        if ((ina226_write_u16(INA226_REG_CONFIG, INA226_CONFIG_VALUE) == HAL_OK) &&
            (ina226_write_u16(INA226_REG_CALIBRATION, INA226_CAL_VALUE) == HAL_OK))
        {
            ina226_present = 1U;
        }
    }

    if (ina226_present == 0U)
    {
        show_status("INA226 ERROR", "CHECK SOLDER");
    }

    last_update_ms = HAL_GetTick();
    last_led_ms = HAL_GetTick();
}

void PowerMeter_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint16_t raw_bus;
    uint16_t raw_current_u16;
    uint16_t raw_power;
    int16_t raw_current;
    uint32_t bus_millivolts;
    int32_t current_milliamps;
    uint32_t power_milliwatts;
    char voltage_text[16];
    char current_text[16];
    char power_text[16];

#if defined(D2_LED_GPIO_Port) && defined(D2_LED_Pin)
    if ((now - last_led_ms) >= 500U)
    {
        HAL_GPIO_TogglePin(D2_LED_GPIO_Port, D2_LED_Pin);
        last_led_ms = now;
    }
#endif

    if ((now - last_update_ms) < 200U)
    {
        return;
    }
    last_update_ms = now;

    if ((oled_present == 0U) || (ina226_present == 0U))
    {
        return;
    }

    if ((ina226_read_u16(INA226_REG_BUS_VOLTAGE, &raw_bus) != HAL_OK) ||
        (ina226_read_u16(INA226_REG_CURRENT, &raw_current_u16) != HAL_OK) ||
        (ina226_read_u16(INA226_REG_POWER, &raw_power) != HAL_OK))
    {
        ina226_present = 0U;
        show_status("I2C ERROR", "CHECK INA226");
        return;
    }

    raw_current = (int16_t)raw_current_u16;

    /* INA226 bus-voltage LSB is 1.25 mV. */
    bus_millivolts = (((uint32_t)raw_bus * 5U) + 2U) / 4U;

    /* Selected current LSB is 0.1 mA; round to the nearest whole mA. */
    if (raw_current >= 0)
    {
        current_milliamps = ((int32_t)raw_current + 5) / 10;
    }
    else
    {
        current_milliamps = -(((int32_t)(-raw_current) + 5) / 10);
    }

    /* Selected power LSB is 2.5 mW. */
    power_milliwatts = (((uint32_t)raw_power * 5U) + 1U) / 2U;

    format_voltage(voltage_text, sizeof(voltage_text), bus_millivolts);
    format_current(current_text, sizeof(current_text), current_milliamps);
    format_power(power_text, sizeof(power_text), power_milliwatts);

    oled_clear();
    oled_draw_text(17U, 0U, "USB POWER METER", 1U);
    oled_draw_text(2U, 14U, voltage_text, 2U);
    oled_draw_text(2U, 31U, current_text, 2U);
    oled_draw_text(2U, 48U, power_text, 2U);
    oled_update();
}
