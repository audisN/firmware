#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#define USB_VID 0x303A
#define USB_PID 0x1001

static const uint8_t TX = 43;
static const uint8_t RX = 44;

static const uint8_t SDA = 16;
static const uint8_t SCL = 15;

// Default SPI is mapped to the LR2021 radio (the display runs on SPI3 through LovyanGFX)
static const uint8_t SS = 3;
static const uint8_t MOSI = 21;
static const uint8_t MISO = 2;
static const uint8_t SCK = 14;

#endif /* Pins_Arduino_h */
