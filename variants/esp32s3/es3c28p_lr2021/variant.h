// LCDWIKI ES3C28P (ESP32-S3R8, 2.8" ILI9341 320x240 IPS, FT6336G touch) + NiceRF LoRa2021F33-2G4 (LR2021)
// Board docs: https://www.lcdwiki.com/2.8inch_ESP32-S3_Display
// Radio DIO map from NiceRF (meshtastic/firmware#11835): DIO9 = IRQ, DIO5 = 2.4G LNA,
// DIO6 = sub-GHz PA TXEN, DIO7 = 2.4G PA VCC enable, DIO8 = 2.4G TXEN. TCXO 3.3 V.
#ifndef _VARIANT_ES3C28P_LR2021_
#define _VARIANT_ES3C28P_LR2021_

#if HAS_TFT
#define HAS_SPI_TFT 1
#define HAS_TOUCHSCREEN 1
#endif

// BOOT key
#define BUTTON_PIN 0
#define BUTTON_ACTIVE_LOW true
#define BUTTON_ACTIVE_PULLUP true

// I2C (header P4): FT6336G touch 0x38 and ES8311 codec 0x18 are on this bus, 4.7k pull-ups on board.
// Free for external I2C devices.
#define I2C_SDA 16
#define I2C_SCL 15

// Battery: BAT+ -> 200k / 200k divider -> IO9 (ADC1_CH8)
#define BATTERY_PIN 9
#define ADC_CHANNEL ADC_CHANNEL_8
#define ADC_MULTIPLIER 2.0

// GPS on the UART header P2 (UART0 pins; console runs over native USB CDC)
#define GPS_RX_PIN 44 // header P2 "RX" <- GPS TX
#define GPS_TX_PIN 43 // header P2 "TX" -> GPS RX
// The u-blox module used here came configured for UBX at 230400 (drone GPS), a speed the Meshtastic probe
// does not try. Fixing the speed makes the probe use only 230400; Meshtastic then reconfigures the module.
#define GPS_BAUDRATE 230400

// microSD: SDIO slot used in 1-bit SDMMC mode (SD_SCLK_PIN / SD_MOSI_PIN / SD_MISO_PIN set in platformio.ini)

// ---- NiceRF LoRa2021F33-2G4 (LR2021) on Arduino SPI (SPI2/FSPI); the display uses SPI3 ----
#define USE_LR2021

#define LORA_SCK 14
#define LORA_MISO 2
#define LORA_MOSI 21
#define LORA_CS 3

#define LR2021_SPI_SCK_PIN LORA_SCK
#define LR2021_SPI_MISO_PIN LORA_MISO
#define LR2021_SPI_MOSI_PIN LORA_MOSI
#define LR2021_SPI_NSS_PIN LORA_CS
#define LR2021_IRQ_PIN 1           // IO1 (was speaker amp AUDIO_EN) <- module IRQ = LR2021 DIO9
#define LR2021_BUSY_PIN 42         // IO42 (was RGB LED data) <- module BUSY
#define LR2021_NRESET_PIN RADIOLIB_NC // module RESET held high by RC, not wired to the MCU

#define LR2021_IRQ_DIO_NUM 9
#define LR2021_DIO3_TCXO_VOLTAGE 3.3f
#define LR2021_DIO_AS_RF_SWITCH // table in rfswitch.h
#define LR2021_MAX_POWER 22     // chip setting; the module's external PA gives ~29.8 dBm at 868 MHz
#define LR2021_MAX_POWER_HF 2   // HF TX is blocked in rfswitch.h anyway; NiceRF limit is register 4 (+2 dBm)

#endif
