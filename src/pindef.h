/* 09:32 15/03/2023 - change triggering comment */
#ifndef PINDEF_H
#define PINDEF_H

//PCF pin definitions
#define powerLED 0
#define pgnLED 1
#define cleanLED 2
#define pgnBtn 3
#define cup1LED 4
#define cup2LED 5
#define steamLED 6

#define hotWaterBtn 8
#define steamBtn 9
#define cup2Btn 10
#define cup1Btn 11
#define sol2Pin 12
#define sol3Pin 13

// STM32F4 pins definitions
#define thermoDO      PB4
#define thermoDI      PA7
#define thermoCS      PA6
#define thermoCLK     PA5
#define thermoRDY     PB5

#define zcPin         PC15
#define pumpPin       PA1
#define heaterPin     PB9
#define shutdownPin   PA4

#define HX711_sck_1   PB0
#define HX711_dout_1  PB8

#define USART_LCD     Serial2 // PA2(TX) & PA3(RX)
#define USART_ESP     Serial1 // PA9(TX) & PA10(RX)
#define USART_DEBUG   Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#endif
