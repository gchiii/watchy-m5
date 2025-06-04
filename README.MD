# M5STICKC PLUS2


| Resources             | Parameters                                                                                                             |
|-----------------------|------------------------------------------------------------------------------------------------------------------------|
| ESP32                 | ESP32-PICO-V3-02240MHz dual core,support wifi,2 MB SPI PSRAM,8 MB SPI flash                                            |
| PSRAM                 | 2 MB PSRAM                                                                                                             |
| Flash                 | 8 MB flash                                                                                                             |
| Power Input           | 5V @ 500mA                                                                                                             |
| Port                  | TypeC x 1, GROVE(I2C+I/0+UART) x 1                                                                                     |
| LCD screen            | 1.14 inch, 135*240 Colorful TFT LCD, ST7789v2                                                                          |
| Button                | Custom button x 3                                                                                                      |
| LED                   | Green LED x 1(non-programmable) (sleep status indicator)Red LED X 1(same control pin G19 as infrared transmitter tube) |
| IMU                   | MPU6886                                                                                                                |
| Buzzer                | built-in buzzer                                                                                                        |
| MIC                   | SPM1423                                                                                                                |
| RTC                   | BM8563                                                                                                                 |
| Battery               | 200mAh @ 3.7V                                                                                                          |
| Antenna               | 2.4G 3D Antenna                                                                                                        |
| PIN port              | G0, G25/G36, G26, G32, G33                                                                                             |

# IO / Peripheral mapping

## Single Pin elements
- IR emitter and Red LED - GPIO19
- Button A - GPIO37
- Button B - GPIO39
- Button C - GPIO35
- Buzzer   - GPIO2

## Color TFT Screen
- ST7789V2
- 135 x 240

| ESP32-PICO-V3-02 |G15|G13|G14|G12|G5|G27 |
| -- |--|--|--|--|--|--|
| TFT Screen|TFT_MOSI|TFT_CLK|TFT_DC|TFT_RST|TFT_CS|TFT_BL|

## Microphone

- SPM1423
- CLK: G0
- DATA G34

## IMU (MPU6886)
- SCL: G22
- SDA: G21

## RTC (BM8563)
- SCL: G22
- SDA: G21

