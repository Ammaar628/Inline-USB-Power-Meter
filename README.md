# Inline USB-C Power Meter

A compact inline USB-C power meter built around an **STM32G030K6T6** microcontroller and **INA226** current/power monitor. The meter sits between a USB-C power source and load and displays real-time electrical measurements on a 128×64 OLED.

The board is designed for USB-C power up to **20 V, 3 A, and 60 W**. It uses a **10 mΩ shunt resistor** for current measurement and passes the USB-C CC lines through so the connected source and device can negotiate power normally.

## Measurements

The OLED provides five display pages:

1. **Summary** - voltage, current, and power
2. **Voltage** - large voltage reading
3. **Current** - large current reading
4. **Power** - large power reading
5. **Energy** - accumulated mAh and mWh

## How to Use

1. Connect the USB-C charger or power source to the **source/input side** of the meter.
2. Connect the device being powered to the **load side**.
3. The meter powers on automatically and begins measuring voltage, current, and power.
4. Use the buttons to change the display or reset the meter.

### MODE Button

- **Short press:** cycle to the next display page.
- **Long hold (~1.2 s):** rotate the OLED 90° between landscape and portrait orientation.

The pages cycle in this order:

`Summary → Voltage → Current → Power → Energy → Summary`

### RESET Button

- **Press:** restart the microcontroller and meter firmware.
- Resetting also clears the accumulated **mAh** and **mWh** readings.

## Hardware

- STM32G030K6T6 microcontroller
- INA226 current/power monitor
- 10 mΩ current-sense shunt
- 128×64 I2C OLED
- USB-C pass-through connection
- 3.3 V buck-regulated control electronics

## Firmware

Firmware was developed in **STM32CubeIDE** using the STM32 HAL libraries. The STM32 reads the INA226 over I2C, updates the OLED, handles the MODE button, tracks accumulated charge/energy, and drives a heartbeat LED.
