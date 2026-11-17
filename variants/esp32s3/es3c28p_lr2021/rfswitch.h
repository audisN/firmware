#pragma once
#include "RadioLib.h"

#ifndef LR20x0
#define LR20x0 LR2021
#endif

// NiceRF LoRa2021F33-2G4 (NiceRF, meshtastic/firmware#11835):
// DIO5 = 2.4G LNA, DIO6 = sub-GHz PA TXEN, DIO7 = 2.4G PA VCC enable, DIO8 = 2.4G TXEN.
// This device runs Meshtastic on sub-GHz only: MODE_TX_HF keeps every line low so the 2.4 GHz PA
// is never powered. MODE_RX_HF is only for the spectrum scan of 1500-2600 MHz.
static const uint32_t lr20x0_rfswitch_dio_pins[] = {RADIOLIB_LR2021_DIO5, RADIOLIB_LR2021_DIO6, RADIOLIB_LR2021_DIO7,
                                                    RADIOLIB_LR2021_DIO8, RADIOLIB_NC};

static const Module::RfSwitchMode_t lr20x0_rfswitch_table[] = {
    // clang-format off
    // mode              DIO5  DIO6  DIO7  DIO8
    {LR20x0::MODE_STBY,  {LOW,  LOW,  LOW,  LOW }},
    {LR20x0::MODE_RX,    {LOW,  LOW,  LOW,  LOW }}, // sub-GHz RX
    {LR20x0::MODE_TX,    {LOW,  HIGH, LOW,  LOW }}, // sub-GHz TX through the external PA
    {LR20x0::MODE_RX_HF, {HIGH, LOW,  HIGH, LOW }}, // HF RX (spectrum only): LNA + FEM supply, untested
    {LR20x0::MODE_TX_HF, {LOW,  LOW,  LOW,  LOW }}, // HF TX blocked
    END_OF_MODE_TABLE,
    // clang-format on
};
