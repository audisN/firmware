// Spectrum scanner (board option HAS_SPECTRUM): sweeps a frequency window or shows an FFT around one frequency with
// the LoRa radio while the mesh is paused. The screen (device-ui) drives it through spectrum/SpectrumApi.h.

#include "configuration.h"

#ifdef HAS_SPECTRUM

#include "SpectrumScanner.h"
#include "concurrency/LockGuard.h"
#include "mesh/RadioLibInterface.h"
#include "spectrum/SpectrumApi.h"
#include <math.h>
#include <string.h>

SpectrumScanner *spectrumScanner = nullptr;

namespace
{
constexpr uint32_t c_idleIntervalMs = 200;
constexpr uint32_t c_pointHz = 250000;       // measuring step and filter inside a column
constexpr uint16_t c_chunk = 32;             // points per radio command (~35 ms), then let the loop run
constexpr uint32_t c_fftIntervalMs = 20;
constexpr uint8_t c_fftSegments = 4;         // FFT frames averaged on the chip per result
constexpr uint32_t c_watchdogMs = 15 * 1000; // stop if the screen stops asking for data
constexpr int c_fftRawBins = 256;           // the chip's FFT: 250 kHz, bin 0 = DC, 128..255 negative
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
    want.mode = SPECTRUM_SWEEP;
    want.range = range;
    want.stepKHz = stepKHz;
    want.startKHz = spectrumClampStartKHz(range, stepKHz, startKHz);
    lastPollMs = millis();
    return true;
}

bool SpectrumScanner::requestFft(uint32_t centerKHz)
{
    concurrency::LockGuard guard(&lock);
    want.run = true;
    want.mode = SPECTRUM_FFT;
    want.centerKHz = centerKHz;
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
    frame->mode = cur.mode;
    frame->range = cur.range;
    frame->active = active;
    frame->bins = bins;
    frame->startHz = startHz;
    frame->binHz = binHz;
    frame->centerHz = cur.centerKHz * 1000;
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
    const bool sameGrid = active && cur.mode == SPECTRUM_SWEEP && next.mode == SPECTRUM_SWEEP && next.range == cur.range &&
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
    const bool fftMoved = next.mode == SPECTRUM_FFT && cur.mode != SPECTRUM_FFT; // calibrate on entry, not on every pan
    cur = next;
    if (cur.mode == SPECTRUM_SWEEP) {
        bins = spectrumColumns(cur.range, cur.stepKHz);
        startHz = cur.startKHz * 1000;
        binHz = cur.stepKHz * 1000;
    } else {
        bins = SPECTRUM_FFT_BINS;
        startHz = cur.centerKHz * 1000 - SPECTRUM_FFT_SPAN_HZ / 2;
        binHz = SPECTRUM_FFT_SPAN_HZ / SPECTRUM_FFT_BINS;
        if (fftMoved)
            fftCalibrated = false;
    }
    point = 0;
    colMax = SPECTRUM_NO_DATA;
    sweepPeak = SPECTRUM_NO_DATA;
    sweepStartMs = millis();
    LOG_INFO("Spectrum: mode %u range %u step %u kHz start %u kHz center %u kHz", cur.mode, cur.range, cur.stepKHz,
             (unsigned)cur.startKHz, (unsigned)cur.centerKHz);
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

#ifdef SPECTRUM_DIAG
    // bench test without the screen: sweep 750-960 MHz, then an FFT on the mesh frequency, logging the results
    static uint8_t diagStep = 0;
    if (diagStep == 0 && millis() > 20000) {
        diagStep = 1;
        requestSweep(SPECTRUM_RANGE_SUBGHZ, 1000, 750000);
        return 10;
    }
    if (diagStep == 1 && sweeps >= 3) {
        diagStep = 2;
        LOG_INFO("SpecDiag: sweep %u ms, peak %d dBm at %u kHz", (unsigned)sweepMs, peak / 2, (unsigned)(peakHz / 1000));
        for (int i = 0; i < bins; i += 10)
            LOG_INFO("SpecDiag: %u MHz %d dBm", (unsigned)((startHz + i * binHz) / 1000000),
                     data[i] == SPECTRUM_NO_DATA ? -999 : data[i] / 2);
        requestFft(869525);
        return 10;
    }
    if (diagStep == 2 && cur.mode == SPECTRUM_FFT && sweeps >= 50) {
        diagStep = 3;
        LOG_INFO("SpecDiag: fft %u frames, offset %.1f dB, peak %d dBm at %u Hz", (unsigned)sweeps, fftOffsetDb, peak / 2,
                 (unsigned)peakHz);
        for (int i = 0; i < SPECTRUM_FFT_BINS; i += 8)
            LOG_INFO("SpecDiag: fft bin %d %d dBm", i, data[i] / 2);
        stop();
    }
    if (diagStep == 1 || diagStep == 2) {
        concurrency::LockGuard guard(&lock);
        lastPollMs = millis(); // nobody polls during the bench test
        next = want;
        polled = lastPollMs;
    }
#endif

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

    return cur.mode == SPECTRUM_FFT ? runFft(radio) : runSweep(radio);
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
        if (sweeps % 20 == 1)
            LOG_DEBUG("Spectrum: sweep %u, %u ms, %u columns, peak %d dBm at %u kHz", (unsigned)sweeps, (unsigned)sweepMs, bins,
                      peak / 2, (unsigned)(peakHz / 1000));
    }
    return 1;
}

int32_t SpectrumScanner::runFft(RadioLibInterface *radio)
{
    const uint32_t centerHz = cur.centerKHz * 1000;
    if (!fftCalibrated) {
        // the chip's FFT is in relative dB: scale it so the total power matches the RSSI over the same 250 kHz
        int16_t rssi = SPECTRUM_NO_DATA;
        const bool okRssi = radio->spectrumSweep(centerHz - 10000000, centerHz + 10000000, centerHz, c_pointHz, 1, &rssi);
        uint8_t raw[c_fftRawBins];
        if (!okRssi || !radio->spectrumFft(centerHz, c_fftSegments, raw)) {
            // keep the last offset (about -190 dB on this board) rather than holding the FFT back
            LOG_WARN("Spectrum: FFT calibration failed, using offset %.1f dB", fftOffsetDb);
            fftCalibrated = true;
            return 20;
        }
        double sum = 0;
        for (int i = 0; i < c_fftRawBins; i++)
            sum += pow(10.0, raw[i] / 20.0); // 0.5 dB steps -> power
        fftOffsetDb = rssi / 2.0f - (float)(10.0 * log10(sum));
        fftCalibrated = true;
        LOG_INFO("Spectrum: FFT at %u kHz, RSSI %d dBm, offset %.1f dB", (unsigned)cur.centerKHz, rssi / 2, fftOffsetDb);
    }

    uint8_t raw[c_fftRawBins];
    if (!radio->spectrumFft(centerHz, c_fftSegments, raw)) {
        LOG_WARN("Spectrum: FFT frame failed");
        return 100;
    }
    concurrency::LockGuard guard(&lock);
    int16_t best = SPECTRUM_NO_DATA;
    uint16_t bestBin = 0;
    for (int i = 0; i < SPECTRUM_FFT_BINS; i++) {
        // low to high frequency, the middle SPECTRUM_FFT_BINS of the chip's bins
        const uint8_t r = raw[(i - SPECTRUM_FFT_BINS / 2 + c_fftRawBins) % c_fftRawBins];
        const int16_t v = (int16_t)lroundf((r / 2.0f + fftOffsetDb) * 2.0f);
        data[i] = v;
        if (v > best) {
            best = v;
            bestBin = i;
        }
    }
    sweeps++;
    peak = best;
    peakHz = startHz + bestBin * binHz + binHz / 2;
    progress = SPECTRUM_FFT_BINS;
    return c_fftIntervalMs;
}

// --- C++ API used by device-ui (spectrum/SpectrumApi.h) ---

bool spectrumStartSweep(uint8_t range, uint16_t stepKHz, uint32_t startKHz)
{
    return spectrumScanner && spectrumScanner->requestSweep(range, stepKHz, startKHz);
}

bool spectrumStartFft(uint32_t centerKHz)
{
    return spectrumScanner && spectrumScanner->requestFft(centerKHz);
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
