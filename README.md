# SDCard-Interface-STM32
An interface between the STM32 NUCLEO F446RE and a micro-SD card for storing data from sensors of the race car's dashboard. A Data Logger for UTD's Dallas Formula Racing.

<p align="center">
  <img width="640" height="320" src="https://github.com/crsz20/SDCard-Interface-STM32/blob/master/images/Title_SDIO%20Card%20Interface.png">
</p>

***
## Objective:
- [x] Implement SDIO communications
- [x] Interface the micro-SD card with the MCU for storage capabilities.
- [x] Provide LED light indication for card detection
- [x] Condense the program to a callable function
- [x] Store the output as a CSV file
- [ ] Improve modularity for a swappable interface
- [ ] Build a report generator with Excel and/or MatLab, or use DFR's DataAnalyzer


## Technologies:
* General Purpose I/O (GPIO)
* 4-bit SDIO communication protocol
* [File Handling Library by Controllers Tech](https://controllerstech.com/interface-sd-card-with-sdio-in-stm32/) (Modified)
* HAL drivers & [FAT FS](http://elm-chan.org/fsw/ff/00index_e.html)

## Hardware and Equipment:

1. [STM32 Nucleo-FE446RE Micro-Controller Unit (MCU)](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) - Equipped with the ARM Cortex M4 processor
2. [Adafruit's Micro-SD Card Breakout for SD SPI or SDIO*](https://www.adafruit.com/product/4682)

![STM32 Nucleo-FE446RE MCU](https://www.st.com/bin/ecommerce/api/image.PF262063.en.feature-description-include-personalized-no-cpn-medium.jpg)

<img src="https://cdn-learn.adafruit.com/guides/images/000/003/056/medium640/4682-04.jpg" alt="Adafruit's Micro-SD Card Breakout" width="400">

## Pinout
* PC8 ---> DAT0
* PC9 ---> DAT1
* PC10 --> DAT2
* PC11 --> DAT3
* PC12 --> CLK
* PD2 ---> CMD
* PB6 ---> CD (Card Detection)

***

## Expected Input
<details>
  <summary>CAN Bus</summary>
  <p>- RPM <br>
    - TPS (Throttle Position Sensor) <br>
    - Fuel Open Time <br>
    - Ignition Angle <br>
    - Barometer <br>
    - Map (Manifold Absolute Pressure) <br>
    - Radiator Air Temp <br>
    - Radiator Coolant Temp <br>
    - AFR (Air Fuel Ratio) <br>
    - Oil Pressure <br>
    - Mass Air Flow Sensor <br>
    - Wheel Speed <br>
    - Battery Voltage
   </p>
</details>
<details>
  <summary>GPS</summary>
  <p>- Second, minute, hour, day, month, year <br>
    - Speed <br>
    - Latitude <br>
    - Longitude <br>
    - Ellipsoidal Height <br>
    - Sea Level Height
   </p>
</details>
<details>
  <summary>Analog-to-Digital Converter (ADC)</summary>
  <p>- Damper Sensing<br>
    - Steer Sensing <br>
    - Brake Sensing
   </p>
</details>
<details>
  <summary>Accelerometer & Gyroscope</summary>
  <p> - X, Y, & Z <br>
    - Roll <br>
    - Pitch <br>
    - Yaw
  </p>
</details>

### Note
In order to enable the GPS-based file naming, set the macro `_USE_LFN` to 2. This enables the Long File Naming Convention, and must be done everytime you make changes to the .ioc file and allow code generation. `_USE_LFN` can be found in FATFS > Target > ffconf.h
