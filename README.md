# SSD1322
Driver for SSD1322 OLED

## ESP32

This driver is written for use with the ESP32 platform, however it can be easily ported into different platforms. It is using 4-wire SPI protocol.


## LVGL

This driver does not have any draw methods as it was intended to be used with [lvgl graphics library](https://lvgl.io/).


## Modules Pin out

| ESP32    | HSPI | VSPI |
|----------|------|------|
| CS0      | 15   | 5    |
| SCLK     | 14   | 18   |
| MISO     | 12   | 19   |
| MOSI     | 13   | 23   |
| QUADWP   | 2    | 22   |
| QUADHD   | 4    | 21   |

| SSD1322 Pin | Description | Description |
|-------------|-------------|-------------|
| 01          | GND         | GND         |
| 02          | VCC         | 3V3         |
| 03          | NC          |             |
| 04          | D0/CLK      | SPI Clock   |
| 05          | D1/DIN      | SPI Data Out|
| 06          | D2          |             |
| 07          | D3          |             |
| 08          | D4          |             |
| 09          | D5          |             |
| 10          | D6          |             |
| 11          | D7          |             |
| 12          | E/RD#       | Enable      |
| 13          | R/W#        | Read/Write  |
| 14          | D/C#        | Data/Command|
| 15          | RES#        | Reset       |
| 16          | CS#         | Chip Select |

### Example pinout

| OLED Module | ESP32 MCU |
| ----------- | --------- |
| Pin  1 GND  | GND       |
| Pin  2 VCC  | 3.3V      |
| Pin  4 SCLK | GPIO  18  |
| Pin  5 SDIN | GPIO  23  |
| Pin 14 DC   | GPIO  16  |
| Pin 15 RES  | GPIO  17  |
| Pin 16 CS   | GPIO  5   |
