/* 09:32 15/03/2023 - change triggering comment */
#ifndef PINDEF_H
#define PINDEF_H

// STM32F4 pins definitions
#define thermoDO      PB4
#define thermoDI      PA7
#define thermoCS      PA6
#define thermoCLK     PA5
#define thermoRDY     PB9

#define zcPin         PA0
#define cup1DtcPin    PA4
#define cup2DtcPin    PC14
#define sol2Pin       PB5
#define sol3Pin       PB3
#define relayPin      PA15
#define dimmerPin     PA1
#define steamPin      PC15
#define valvePin      PC13
#if defined(SINGLE_BOARD)
#define waterPin      PB15
#else
#define waterPin      PB1
#endif

#ifdef PCBV2
// PCB V2
#define steamValveRelayPin PB12
#define steamBoilerRelayPin PB13
#endif

#define HX711_sck_1   PB0
#define HX711_dout_1  PB8

#define USART_LCD     Serial2 // PA2(TX) & PA3(RX)
#define USART_ESP     Serial1 // PA9(TX) & PA10(RX)
#define USART_DEBUG   Serial  // USB-CDC (Takes PA8,PA9,PA10,PA11)

#endif
