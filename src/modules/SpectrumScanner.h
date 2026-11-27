#pragma once

#include "configuration.h"

#ifdef HAS_SPECTRUM

#include "concurrency/Lock.h"
#include "concurrency/OSThread.h"
#include "spectrum/SpectrumApi.h"

class RadioLibInterface;

/**
 * Spectrum scanner: runs on the main loop (like the radio), pauses the mesh while scanning.
 * The screen task requests scans and reads frames through spectrum/SpectrumApi.h.
 */
class SpectrumScanner : public concurrency::OSThread
{
  public:
    SpectrumScanner();

    bool requestSweep(uint8_t range, uint16_t stepKHz, uint32_t startKHz);
    bool requestFft(uint32_t centerKHz, uint32_t searchKHz, uint8_t span);
    void stop();
    bool getFrame(SpectrumFrame *frame);

  protected:
    int32_t runOnce() override;

  private:
    struct Settings {
        bool run = false;
        uint8_t mode = SPECTRUM_SWEEP;
        uint8_t range = SPECTRUM_RANGE_SUBGHZ;
        uint16_t stepKHz = 1000;
        uint32_t startKHz = 700000;  // sweep window start
        uint32_t centerKHz = 869525; // FFT center as requested
        uint32_t searchKHz = 0;      // FFT: look for the strongest signal around centerKHz first
        uint8_t span = SPECTRUM_FFT_WIDE;
        bool operator==(const Settings &o) const
        {
            return run == o.run && mode == o.mode && range == o.range && stepKHz == o.stepKHz && startKHz == o.startKHz &&
                   centerKHz == o.centerKHz && searchKHz == o.searchKHz && span == o.span;
        }
    };

    int32_t runSweep(RadioLibInterface *radio);
    int32_t runFft(RadioLibInterface *radio);
    void applySettings(const Settings &next); // under lock
    void clearData();                         // under lock

    concurrency::Lock lock; // guards want, lastPollMs and the published data below
    Settings want;          // requested by the screen
    Settings cur;           // being scanned
    bool active = false;
    uint32_t lastPollMs = 0;

    // published data (under lock)
    uint16_t bins = 0;
    uint32_t startHz = 0, binHz = 0;
    int16_t data[SPECTRUM_MAX_BINS];
    uint32_t sweeps = 0, sweepMs = 0;
    uint16_t progress = 0;
    int16_t peak = SPECTRUM_NO_DATA;
    uint32_t peakHz = 0;

    // sweep in progress (main loop only)
    uint32_t point = 0; // next 250 kHz point of the sweep
    int16_t colMax = SPECTRUM_NO_DATA;
    int16_t sweepPeak = SPECTRUM_NO_DATA;
    uint32_t sweepPeakHz = 0;
    uint32_t sweepStartMs = 0;
    // FFT (main loop only)
    bool fftCalibrated = false;
    uint32_t fftCenterHz = 0; // where the FFT really is (requested center + search result)
    float fftOffsetDb = -190; // relative FFT dB -> estimated dBm (measured on entry)
};

extern SpectrumScanner *spectrumScanner;

#endif
