/* 09:32 15/03/2023 - change triggering comment */
#ifndef PINDEF_H
#define PINDEF_H

// STM32F4 pins definitions
#define thermoDO      PB4
#define thermoDI      PA7
#define thermoCS      PA6
#define thermoCLK     PA5
#define thermoRDY     PB5

#define cup1DtcPin    PA0
#define cup2DtcPin    PC13
#define steamPin      PC14
#define waterPin      PB1

#define zcPin         PC15
#define pumpPin       PA1
#define sol3Pin       PB3
#define sol2Pin       PA15
#define heaterPin     PB9
#define shutdownPin   PA4

#define HX711_sck_1   PB0
#define HX711_dout_1  PB8

#define USART_LCD     Serial2 // PA2(TX) & PA3(RX)
#define USART_ESP     Serial1 // PA9(TX) & PA10(RX)
#define USART_DEBUG   Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#endif
