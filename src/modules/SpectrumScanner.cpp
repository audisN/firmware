// Spectrum scanner (board option HAS_SPECTRUM): sweeps a frequency window with the LoRa radio while the mesh is
// paused. The screen (device-ui) drives it through spectrum/SpectrumApi.h.

#include "configuration.h"

#ifdef HAS_SPECTRUM

#include "SpectrumScanner.h"
#include "concurrency/LockGuard.h"
#include "mesh/RadioLibInterface.h"
#include "spectrum/SpectrumApi.h"
#include <string.h>

SpectrumScanner *spectrumScanner = nullptr;

namespace
{
constexpr uint32_t c_idleIntervalMs = 200;
constexpr uint32_t c_pointHz = 250000;       // measuring step and filter inside a column
constexpr uint16_t c_chunk = 32;             // points per radio command (~35 ms), then let the loop run
constexpr uint32_t c_watchdogMs = 15 * 1000; // stop if the screen stops asking for data
} // namespace

SpectrumScanner::SpectrumScanner() : concurrency::OSThread("Spectrum")
{
    spectrumScanner = this;
    clearData();
}

void SpectrumScanner::clearData()
{
    for (int i = 0; i < SPECTRUM_MAX_BINS; i++)
        data[i] = SPECTRUM_NO_DATA;
    sweeps = 0;
    sweepMs = 0;
    progress = 0;
    peak = SPECTRUM_NO_DATA;
    peakHz = 0;
}

bool SpectrumScanner::requestSweep(uint8_t range, uint16_t stepKHz, uint32_t startKHz)
{
    if (stepKHz != 125 && stepKHz != 250 && stepKHz != 500 && stepKHz != 1000)
        stepKHz = 1000;
    range = range == SPECTRUM_RANGE_HF ? SPECTRUM_RANGE_HF : SPECTRUM_RANGE_SUBGHZ;
    concurrency::LockGuard guard(&lock);
    want.run = true;
    want.range = range;
    want.stepKHz = stepKHz;
    want.startKHz = spectrumClampStartKHz(range, stepKHz, startKHz);
    lastPollMs = millis();
    return true;
}

void SpectrumScanner::stop()
{
    concurrency::LockGuard guard(&lock);
    want.run = false;
}

bool SpectrumScanner::getFrame(SpectrumFrame *frame)
{
    concurrency::LockGuard guard(&lock);
    lastPollMs = millis();
    frame->range = cur.range;
    frame->active = active;
    frame->bins = bins;
    frame->startHz = startHz;
    frame->binHz = binHz;
    frame->sweeps = sweeps;
    frame->sweepMs = sweepMs;
    frame->progress = progress;
    frame->peak = peak;
    frame->peakHz = peakHz;
    memcpy(frame->data, data, sizeof(data));
    return true;
}

// switch to new settings; a sweep window moved by whole columns keeps its data shifted along
void SpectrumScanner::applySettings(const Settings &next)
{
    const bool sameGrid = active && next.range == cur.range &&
                          next.stepKHz == cur.stepKHz;
    if (sameGrid) {
        const int32_t shift = ((int32_t)next.startKHz - (int32_t)cur.startKHz) / (int32_t)cur.stepKHz; // columns
        if (shift > 0 && shift < bins) {
            memmove(data, data + shift, (bins - shift) * sizeof(data[0]));
            for (int i = bins - shift; i < bins; i++)
                data[i] = SPECTRUM_NO_DATA;
        } else if (shift < 0 && -shift < bins) {
            memmove(data - shift, data, (bins + shift) * sizeof(data[0]));
            for (int i = 0; i < -shift; i++)
                data[i] = SPECTRUM_NO_DATA;
        } else if (shift != 0) {
            for (int i = 0; i < SPECTRUM_MAX_BINS; i++)
                data[i] = SPECTRUM_NO_DATA;
        }
        progress = 0;
    } else {
        clearData();
    }
    cur = next;
    bins = spectrumColumns(cur.range, cur.stepKHz);
    startHz = cur.startKHz * 1000;
    binHz = cur.stepKHz * 1000;
    point = 0;
    colMax = SPECTRUM_NO_DATA;
    sweepPeak = SPECTRUM_NO_DATA;
    sweepStartMs = millis();
    LOG_INFO("Spectrum: range %u step %u kHz start %u kHz", cur.range, cur.stepKHz, (unsigned)cur.startKHz);
}

int32_t SpectrumScanner::runOnce()
{
    Settings next;
    uint32_t polled;
    {
        concurrency::LockGuard guard(&lock);
        next = want;
        polled = lastPollMs;
    }
    RadioLibInterface *radio = RadioLibInterface::instance;

    if (next.run && millis() - polled > c_watchdogMs) {
        LOG_WARN("Spectrum: screen stopped polling, stop scan");
        concurrency::LockGuard guard(&lock);
        want.run = false;
        next.run = false;
    }

    if (!next.run || !radio) {
        if (active) {
            radio->spectrumEnd();
            concurrency::LockGuard guard(&lock);
            active = false;
        }
        return c_idleIntervalMs;
    }

    if (!active) {
        if (!radio->spectrumBegin()) {
            LOG_DEBUG("Spectrum: radio busy, retry");
            return 100; // radio busy (sending), try again shortly
        }
        concurrency::LockGuard guard(&lock);
        applySettings(next);
        active = true;
    } else if (!(next == cur)) {
        concurrency::LockGuard guard(&lock);
        applySettings(next);
    }

    return runSweep(radio);
}

int32_t SpectrumScanner::runSweep(RadioLibInterface *radio)
{
    const uint32_t pointHz = cur.stepKHz < 250 ? cur.stepKHz * 1000 : c_pointHz; // also the filter width
    const uint32_t perCol = cur.stepKHz * 1000 / pointHz;
    const uint32_t total = (uint32_t)bins * perCol;
    const uint32_t loHz = startHz, hiHz = startHz + bins * binHz;
    const uint16_t n = total - point > c_chunk ? c_chunk : (uint16_t)(total - point);
    const uint32_t f0 = loHz + point * pointHz + pointHz / 2;
    int16_t v[c_chunk];
    if (!radio->spectrumSweep(loHz, hiHz, f0, pointHz, n, v)) {
        LOG_WARN("Spectrum: sweep command failed at %u kHz", (unsigned)(f0 / 1000));
        return 50;
    }

    concurrency::LockGuard guard(&lock);
    for (uint16_t k = 0; k < n; k++, point++) {
        if (v[k] > colMax)
            colMax = v[k];
        if (v[k] > sweepPeak) {
            sweepPeak = v[k];
            sweepPeakHz = f0 + k * pointHz;
        }
        if (point % perCol == perCol - 1) {
            const uint16_t col = point / perCol;
            data[col] = colMax;
            progress = col + 1;
            colMax = SPECTRUM_NO_DATA;
        }
    }
    if (point >= total) {
        const uint32_t now = millis();
        sweeps++;
        sweepMs = now - sweepStartMs;
        sweepStartMs = now;
        peak = sweepPeak;
        peakHz = sweepPeakHz;
        sweepPeak = SPECTRUM_NO_DATA;
        point = 0;
    }
    return 1;
}

// --- C++ API used by device-ui (spectrum/SpectrumApi.h) ---

bool spectrumStartSweep(uint8_t range, uint16_t stepKHz, uint32_t startKHz)
{
    return spectrumScanner && spectrumScanner->requestSweep(range, stepKHz, startKHz);
}

void spectrumStop(void)
{
    if (spectrumScanner)
        spectrumScanner->stop();
}

bool spectrumGetFrame(SpectrumFrame *frame)
{
    return spectrumScanner && spectrumScanner->getFrame(frame);
}

#endif
