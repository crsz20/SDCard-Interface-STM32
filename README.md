# SDCard-Interface-STM32
An interface between the STM32 NUCLEO F446RE and a micro-SD card for storing data from sensors of the race car's dashboard (accelerometer, gyroscope, and so on).
***
## Objective:
- [x] Implement SDIO communications
- [x] Interface the micro-SD card with the MCU for storage capabilities.
- [ ] Provide LED light indication for card detection and file completion
- [ ] Condense the program to a callable function
- [ ] Store the output as a CSV file
- [ ] Build a report generator with Excel and/or MatLab


## Technologies:
* General Purpose I/O (GPIO)
* 4-bit SDIO communication protocol
* HAL Drivers
* [File Handling Library by Controllers Tech](https://controllerstech.com/interface-sd-card-with-sdio-in-stm32/)

## Hardware and Equipment:

1. [STM32 Nucleo-FE446RE Micro-Controller Unit (MCU)](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) - Equipped with the ARM Cortex M4 processor
2. [Adafruit's Micro-SD Card Breakout for SD SPI or SDIO*](https://www.adafruit.com/product/4682)

![STM32 Nucleo-FE446RE MCU](https://www.st.com/bin/ecommerce/api/image.PF262063.en.feature-description-include-personalized-no-cpn-medium.jpg)

![Adafruit's Micro-SD Card Breakout](https://cdn-learn.adafruit.com/guides/images/000/003/056/medium640/4682-04.jpg)

## Pinout
* PC8 ---> DAT0
* PC9 ---> DAT1
* PC10 --> DAT2
* PC11 --> CD / DAT3
* PC12 --> CLK
* PD2 ---> CMD
