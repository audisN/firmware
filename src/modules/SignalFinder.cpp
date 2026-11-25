// Signal finder (board option HAS_SPECTRUM), see spectrum/SignalFinderApi.h. Energy mode sweeps 433-950 MHz and
// 2100-2700 MHz; packets mode listens on the channels of known systems (Meshtastic, LoRaWAN, ExpressLRS).
// The mesh is paused while it runs.

#include "configuration.h"

#if defined(HAS_SPECTRUM) && defined(HAS_SIGNAL_FINDER)

#include "SignalFinder.h"
#include "concurrency/LockGuard.h"
#include <algorithm>
#include <string.h>

SignalFinder *signalFinder = nullptr;

namespace
{
struct Range {
    uint32_t loKHz, hiKHz;
};
constexpr Range c_ranges[2] = {{433000, 950000}, {2100000, 2700000}};
constexpr uint32_t c_pointKHz = 250;     // energy sweep step and filter
constexpr uint16_t c_segPoints = 160;    // 40 MHz calibrated as one window
constexpr uint16_t c_chunk = 32;         // points per radio command
constexpr int16_t c_marginHalfDb = 20;   // heard: 10 dB above the segment's noise
constexpr uint32_t c_watchdogMs = 15000; // stop if the screen stops asking
constexpr uint32_t c_budgetMs = 120;     // radio work per call in packets mode

// known systems: single LoRa channels (listened to for a header) and ELRS hop bands (short CADs)
const struct LoraChannel {
    uint8_t system;
    uint32_t khz;
    uint8_t sync;
} c_loraChans[] = {
    {FINDER_SYS_MESHTASTIC, 869525, 0x2B}, // EU_868 LongFast default
    {FINDER_SYS_MESHTASTIC, 906875, 0x2B}, // US
    {FINDER_SYS_MESHTASTIC, 433875, 0x2B}, // EU_433
    {FINDER_SYS_LORAWAN, 868100, 0x34},    {FINDER_SYS_LORAWAN, 868300, 0x34}, {FINDER_SYS_LORAWAN, 868500, 0x34},
    {FINDER_SYS_LORAWAN, 903900, 0x34},    {FINDER_SYS_LORAWAN, 904300, 0x34}, {FINDER_SYS_LORAWAN, 904700, 0x34},
    {FINDER_SYS_LORAWAN, 433175, 0x34},    {FINDER_SYS_LORAWAN, 433375, 0x34}, {FINDER_SYS_LORAWAN, 433575, 0x34},
};
constexpr uint8_t c_loraChanCount = sizeof(c_loraChans) / sizeof(c_loraChans[0]);
const struct ElrsBand {
    uint8_t system;
    uint32_t firstKHz, stepKHz;
    uint8_t count, sfLo, sfHi;
    float bwKHz;
    uint16_t windowMs;
} c_elrsBands[] = {
    {FINDER_SYS_ELRS868, 863275, 525, 13, 6, 9, 500, 30},
    {FINDER_SYS_ELRS915, 903500, 600, 40, 6, 9, 500, 20},
    {FINDER_SYS_ELRS433, 433100, 0, 3, 6, 9, 500, 40}, // irregular channels, see elrsChannelKHz()
    {FINDER_SYS_ELRS24, 2400400, 1000, 80, 5, 8, 812.5f, 15},
};
constexpr uint8_t c_elrsBandCount = sizeof(c_elrsBands) / sizeof(c_elrsBands[0]);

uint32_t elrsChannelKHz(const ElrsBand &b, uint8_t ch)
{
    static const uint32_t eu433[] = {433100, 433925, 434450};
    return b.stepKHz ? b.firstKHz + ch * b.stepKHz : eu433[ch % 3];
}

RadioLibInterface::SpectrumProbe loraCad(uint8_t sf, float bwKHz)
{
    RadioLibInterface::SpectrumProbe p{};
    p.modem = RADIOLIB_LR2021_PACKET_TYPE_LORA;
    p.sf = sf;
    p.bwKHz = bwKHz;
    return p;
}

// CAD window: a few symbols, so one CAD fits even for SF12
uint32_t cadWindowMs(uint8_t sf, float bwKHz)
{
    const uint32_t symbolUs = (uint32_t)((1UL << sf) * 1000.0f / bwKHz);
    return std::min<uint32_t>(10 + 6 * symbolUs / 1000, 300);
}
} // namespace

SignalFinder::SignalFinder() : concurrency::OSThread("SignalFinder")
{
    signalFinder = this;
    for (auto &p : elrsPeak)
        p = INT16_MIN;
}

bool SignalFinder::start(uint8_t newMode)
{
    concurrency::LockGuard guard(&lock);
    if (newMode != mode) { // the list belongs to the mode
        for (auto &e : entries)
            e = Entry();
        mode = newMode;
        target = chan = 0;
        memset(elrsHits, 0, sizeof(elrsHits));
        memset(elrsChans, 0, sizeof(elrsChans));
    }
    want = true;
    lastPollMs = millis();
    return true;
}

void SignalFinder::stop()
{
    concurrency::LockGuard guard(&lock);
    want = false;
}

void SignalFinder::clear()
{
    concurrency::LockGuard guard(&lock);
    for (auto &e : entries)
        e = Entry();
}

uint16_t SignalFinder::getList(FinderSignal *list, uint16_t max, FinderStatus *status)
{
    concurrency::LockGuard guard(&lock);
    lastPollMs = millis();
    const uint32_t now = millis();
    uint16_t n = 0;
    for (const auto &e : entries) {
        // a single energy reading or LoRa CAD hit without a header is often noise: show it once it repeats
        if (!e.used || n >= max || (e.s.hits < 2 && !e.s.confirmed && e.s.system <= FINDER_SYS_LORAWAN))
            continue;
        list[n] = e.s;
        list[n].ageS = (uint16_t)std::min<uint32_t>((now - e.lastMs) / 1000, 65535);
        n++;
    }
    std::sort(list, list + n, [](const FinderSignal &a, const FinderSignal &b) {
        return a.confirmed != b.confirmed ? a.confirmed : a.peak > b.peak;
    });
    if (status) {
        status->active = active;
        status->mode = mode;
        status->watching = watching;
        status->sweepKHz = sweepKHz;
        status->cycles = cycles;
        status->count = n;
    }
    return n;
}

int32_t SignalFinder::runOnce()
{
#ifdef SIGNAL_FINDER_BENCH
    static uint32_t benchLogMs = 0;
    if (millis() > 20000 && millis() < 400000) {
        if (!want)
            start(FINDER_MODE_PACKETS);
        concurrency::LockGuard guard(&lock);
        lastPollMs = millis();
    }
    if (active && millis() - benchLogMs > 20000) {
        benchLogMs = millis();
        FinderSignal l[FINDER_MAX_SIGNALS];
        FinderStatus st;
        const uint16_t n = getList(l, FINDER_MAX_SIGNALS, &st);
        LOG_INFO("FinderBench: %u found after %u passes", n, (unsigned)st.cycles);
        for (uint16_t i = 0; i < n && i < 10; i++)
            LOG_INFO("FinderBench: sys %u %u kHz SF%u/%u hits %u ch %u peak %d ok %u age %u", l[i].system, (unsigned)l[i].freqKHz,
                     l[i].sf, l[i].bwKHz, l[i].hits, l[i].channels, l[i].peak / 2, l[i].confirmed, l[i].ageS);
    }
#endif
    bool run;
    uint32_t polled;
    {
        concurrency::LockGuard guard(&lock);
        run = want;
        polled = lastPollMs;
    }
    if (run && millis() - polled > c_watchdogMs) {
        LOG_WARN("SignalFinder: screen stopped polling, stop");
        concurrency::LockGuard guard(&lock);
        want = run = false;
    }
    RadioLibInterface *radio = RadioLibInterface::instance;
    if (!run || !radio) {
        if (active) {
            radio->spectrumEnd();
            concurrency::LockGuard guard(&lock);
            active = false;
            watching = FINDER_SYS_NONE;
        }
        return 200;
    }
    if (!active) {
        if (radio->spectrumActive())
            return 200; // the spectrum scanner still holds the radio
        if (!radio->spectrumBegin())
            return 100; // sending, try again shortly
        concurrency::LockGuard guard(&lock);
        active = true;
        range = 0;
        segStartKHz = c_ranges[0].loKHz;
        segFill = 0;
        LOG_INFO("SignalFinder: started, mode %u", mode);
    }
    if (mode == FINDER_MODE_PACKETS)
        watchStep(radio);
    else
        sweepStep(radio);
    return 1;
}

// ---------------------------------------------------------------------------------------------------------------
// energy mode

void SignalFinder::sweepStep(RadioLibInterface *radio)
{
    const Range &r = c_ranges[range];
    const uint16_t segPoints = (uint16_t)std::min<uint32_t>(c_segPoints, (r.hiKHz - segStartKHz) / c_pointKHz);
    const uint16_t n = std::min<uint16_t>(c_chunk, segPoints - segFill);
    const uint32_t loHz = segStartKHz * 1000, hiHz = (segStartKHz + segPoints * c_pointKHz) * 1000;
    const uint32_t f0 = loHz + (segFill * c_pointKHz + c_pointKHz / 2) * 1000;
    if (!radio->spectrumSweep(loHz, hiHz, f0, c_pointKHz * 1000, n, seg + segFill)) {
        LOG_WARN("SignalFinder: sweep failed at %u kHz", (unsigned)(f0 / 1000));
        return;
    }
    segFill += n;
    {
        concurrency::LockGuard guard(&lock);
        sweepKHz = segStartKHz + segFill * c_pointKHz;
    }
    if (segFill < segPoints)
        return;

    finishSegment();
    segStartKHz += segPoints * c_pointKHz;
    segFill = 0;
    if (segStartKHz + c_pointKHz <= r.hiKHz)
        return;
    // range done
    range ^= 1;
    segStartKHz = c_ranges[range].loKHz;
    concurrency::LockGuard guard(&lock);
    if (range == 0)
        cycles++;
    expire();
}

// signals in the segment: runs of points 10 dB above its noise (20th percentile)
void SignalFinder::finishSegment()
{
    const uint16_t n = segFill;
    int16_t sorted[c_segPoints];
    memcpy(sorted, seg, n * sizeof(int16_t));
    std::nth_element(sorted, sorted + n / 5, sorted + n);
    const int16_t threshold = sorted[n / 5] + c_marginHalfDb;
    for (uint16_t i = 0; i < n;) {
        if (seg[i] <= threshold) {
            i++;
            continue;
        }
        uint16_t j = i, best = i;
        while (j < n && seg[j] > threshold) {
            if (seg[j] > seg[best])
                best = j;
            j++;
        }
        concurrency::LockGuard guard(&lock);
        addBurst(segStartKHz + i * c_pointKHz, segStartKHz + j * c_pointKHz, seg[best],
                 segStartKHz + best * c_pointKHz + c_pointKHz / 2);
        i = j;
    }
}

void SignalFinder::addBurst(uint32_t loKHz, uint32_t hiKHz, int16_t peak, uint32_t peakKHz)
{
    const uint32_t now = millis();
    const uint32_t centre = loKHz / 2 + hiKHz / 2;
    const uint16_t width = (uint16_t)std::min<uint32_t>(hiKHz - loKHz, 65535);
    for (auto &e : entries) {
        if (!e.used || e.s.system != FINDER_SYS_NONE)
            continue;
        const uint32_t w = std::max<uint32_t>(e.s.widthKHz, width);
        const uint32_t d = centre > e.s.freqKHz ? centre - e.s.freqKHz : e.s.freqKHz - centre;
        if (d > std::max<uint32_t>(c_pointKHz, w / 2))
            continue;
        e.s.last = peak;
        e.s.peak = std::max(e.s.peak, peak);
        e.s.hits++;
        e.s.loKHz = std::min(e.s.loKHz, loKHz);
        e.s.hiKHz = std::max(e.s.hiKHz, hiKHz);
        e.s.widthKHz = std::max(e.s.widthKHz, width);
        e.lastMs = now;
        return;
    }
    // new: a free slot, else the one not heard for longest
    Entry *slot = nullptr;
    for (auto &e : entries)
        if (!e.used) {
            slot = &e;
            break;
        }
    if (!slot)
        for (auto &e : entries)
            if (!slot || e.lastMs < slot->lastMs)
                slot = &e;
    *slot = Entry();
    slot->used = true;
    slot->s.freqKHz = peakKHz;
    slot->s.loKHz = loKHz;
    slot->s.hiKHz = hiKHz;
    slot->s.widthKHz = width;
    slot->s.peak = slot->s.last = peak;
    slot->s.hits = 1;
    slot->lastMs = now;
}

void SignalFinder::expire()
{
    const uint32_t now = millis();
    for (auto &e : entries) {
        if (!e.used)
            continue;
        const uint32_t limit = e.s.system != FINDER_SYS_NONE ? 600000 : (e.s.hits < 2 ? 20000 : 120000);
        if (now - e.lastMs > limit)
            e = Entry();
    }
}

// ---------------------------------------------------------------------------------------------------------------
// packets mode: Meshtastic and LoRaWAN channels are checked with a CAD on each spreading factor; when exactly one
// answers, the radio listens for a header with the system's sync word. ExpressLRS hops too fast to listen to: its
// channels are checked with short CADs, and a pass where one spreading factor answers on several channels counts.

SignalFinder::Entry *SignalFinder::findOrAdd(uint8_t system, uint32_t freqKHz, uint8_t sf)
{
    for (auto &e : entries)
        if (e.used && e.s.system == system && e.s.freqKHz == freqKHz && e.s.sf == sf)
            return &e;
    Entry *slot = nullptr;
    for (auto &e : entries)
        if (!e.used) {
            slot = &e;
            break;
        }
    if (!slot)
        for (auto &e : entries)
            if (!slot || e.lastMs < slot->lastMs)
                slot = &e;
    *slot = Entry();
    slot->used = true;
    slot->s.system = system;
    slot->s.freqKHz = slot->s.loKHz = slot->s.hiKHz = freqKHz;
    slot->s.sf = sf;
    slot->s.peak = slot->s.last = INT16_MIN;
    return slot;
}

void SignalFinder::watchLoRaChannel(RadioLibInterface *radio, uint8_t system, uint32_t freqKHz, uint8_t sync)
{
    // Meshtastic EU_868 presets: BW250 SF7-11 (Short/Medium/LongFast), BW125 SF10-12 (Long Moderate/Slow);
    // LoRaWAN EU868: BW125 SF7-12
    struct Combo {
        uint8_t sf;
        float bw;
    };
    static const Combo mesh[] = {{7, 250}, {8, 250}, {9, 250}, {10, 250}, {11, 250}, {10, 125}, {11, 125}, {12, 125}};
    static const Combo lorawan[] = {{7, 125}, {8, 125}, {9, 125}, {10, 125}, {11, 125}, {12, 125}};
    const Combo *combos = system == FINDER_SYS_MESHTASTIC ? mesh : lorawan;
    const uint8_t count = system == FINDER_SYS_MESHTASTIC ? sizeof(mesh) / sizeof(mesh[0]) : sizeof(lorawan) / sizeof(lorawan[0]);

    uint16_t mask = 0;
    int16_t rssi = INT16_MIN;
    for (uint8_t i = 0; i < count; i++) {
        bool found = false;
        int16_t r = INT16_MIN;
        if (radio->spectrumDetect(loraCad(combos[i].sf, combos[i].bw), freqKHz * 1000, cadWindowMs(combos[i].sf, combos[i].bw),
                                  &found, &r) &&
            found) {
            mask |= 1 << i;
            rssi = std::max(rssi, r);
        }
    }
    if (!mask)
        return;
    if (mask & (mask - 1)) {
        LOG_DEBUG("SignalFinder: %u kHz CAD on several settings (0x%02x): not counted", (unsigned)freqKHz, mask);
        return;
    }
    uint8_t i = 0;
    while (!(mask & (1 << i)))
        i++;
    // one setting answered: listen for a header with the system's sync word. The CAD has used up the preamble of the
    // packet it heard, so wait for the next one (a relay, an ACK, the next uplink)
    RadioLibInterface::SpectrumProbe p = loraCad(combos[i].sf, combos[i].bw);
    p.packet = true;
    p.loraSync = sync;
    bool header = false;
    int16_t hr = INT16_MIN;
    (void)radio->spectrumDetect(p, freqKHz * 1000, 2500, &header, &hr);
    if (header)
        rssi = std::max(rssi, hr);

    concurrency::LockGuard guard(&lock);
    Entry *e = findOrAdd(system, freqKHz, combos[i].sf);
    e->s.bwKHz = (uint16_t)combos[i].bw;
    e->s.hits++;
    e->s.last = rssi;
    e->s.peak = std::max(e->s.peak, rssi);
    e->s.confirmed |= header;
    e->lastMs = millis();
    LOG_INFO("SignalFinder: system %u at %u kHz SF%u/%u, %d dBm%s", system, (unsigned)freqKHz, combos[i].sf,
             (unsigned)combos[i].bw, rssi / 2, header ? ", header received" : "");
}

// one ELRS pass done: one spreading factor answering on several channels, clearly more than any other, is ELRS
// The band's channels sampled a few times with the energy sweep (max hold): a hopping transmitter shows up on several
// of them, 10 dB above the band's noise.
bool SignalFinder::elrsEnergyConfirmed(RadioLibInterface *radio, uint8_t band)
{
    const ElrsBand &b = c_elrsBands[band];
    const uint8_t n = std::min<uint8_t>(b.count, 64);
    int16_t maxHold[64];
    for (uint8_t k = 0; k < n; k++)
        maxHold[k] = INT16_MIN;
    const uint32_t loHz = elrsChannelKHz(b, 0) * 1000, hiHz = elrsChannelKHz(b, b.count - 1) * 1000 + 1;
    const uint8_t passes = n > 20 ? 8 : 20;
    for (uint8_t pass = 0; pass < passes; pass++) {
        for (uint8_t k = 0; k < n; k++) {
            int16_t v;
            const uint32_t f = elrsChannelKHz(b, k) * 1000;
            if (radio->spectrumSweep(loHz, hiHz, f, 250000, 1, &v))
                maxHold[k] = std::max(maxHold[k], v);
        }
    }
    int16_t sorted[64];
    memcpy(sorted, maxHold, n * sizeof(int16_t));
    std::nth_element(sorted, sorted + n / 3, sorted + n);
    const int16_t noise = sorted[n / 3];
    uint8_t loud = 0;
    for (uint8_t k = 0; k < n; k++)
        if (maxHold[k] > noise + 20)
            loud++;
    LOG_DEBUG("SignalFinder: ELRS band %u energy check: %u of %u channels 10 dB above %d dBm", band, loud, n, noise / 2);
    return loud >= 2 || (n <= 3 && loud >= 1);
}

void SignalFinder::finishElrs(RadioLibInterface *radio, uint8_t band, uint8_t system, uint32_t centreKHz, uint16_t bwKHz)
{
    int best = 0;
    for (int i = 1; i < 8; i++)
        if (elrsHits[i] > elrsHits[best])
            best = i;
    uint16_t other = 0;
    for (int i = 0; i < 8; i++)
        if (i != best)
            other = std::max(other, elrsHits[i]);
    uint8_t chans = 0;
    for (uint8_t b : elrsChans[best])
        chans += __builtin_popcount(b);
    bool echo = false;
    if (elrsHits[best] >= 2 && chans >= 2 && elrsHits[best] >= 2 * other && !elrsEnergyConfirmed(radio, band)) {
        // a strong transmitter nearby trips the CAD on the other ELRS bands too: their channels show no energy
        echo = true;
    }
    if (echo) {
        LOG_DEBUG("SignalFinder: ELRS (system %u) SF%u: no energy on its channels, not counted", system, best + 5);
    } else if (elrsHits[best] >= 2 && chans >= 2 && elrsHits[best] >= 2 * other) {
        concurrency::LockGuard guard(&lock);
        Entry *e = findOrAdd(system, centreKHz, best + 5);
        e->s.bwKHz = bwKHz;
        e->s.hits += elrsHits[best];
        e->s.last = elrsPeak[best];
        e->s.peak = std::max(e->s.peak, elrsPeak[best]);
        for (int k = 0; k < 10; k++)
            e->chanMask[k] |= elrsChans[best][k];
        uint8_t all = 0;
        for (uint8_t b : e->chanMask)
            all += __builtin_popcount(b);
        e->s.channels = all;
        e->lastMs = millis();
        LOG_INFO("SignalFinder: ELRS (system %u) SF%u, %u hits on %u channels, %d dBm", system, best + 5, elrsHits[best], chans,
                 elrsPeak[best] / 2);
    } else if (elrsHits[best]) {
        LOG_DEBUG("SignalFinder: ELRS pass (system %u): hits per SF %u %u %u %u %u %u %u %u, not counted", system, elrsHits[0],
                  elrsHits[1], elrsHits[2], elrsHits[3], elrsHits[4], elrsHits[5], elrsHits[6], elrsHits[7]);
    }
    memset(elrsHits, 0, sizeof(elrsHits));
    memset(elrsChans, 0, sizeof(elrsChans));
    for (auto &p : elrsPeak)
        p = INT16_MIN;
}

// Meshtastic EU_868 LongFast (the default) gets its own frequent listen: a packet lasts ~0.7 s, and a full pass over
// all the systems takes much longer than that
void SignalFinder::listenMeshtastic(RadioLibInterface *radio)
{
    RadioLibInterface::SpectrumProbe p = loraCad(11, 250);
    p.packet = true;
    p.loraSync = 0x2B;
    bool header = false;
    int16_t r = INT16_MIN;
    {
        concurrency::LockGuard guard(&lock);
        watching = FINDER_SYS_MESHTASTIC;
    }
    if (!radio->spectrumDetect(p, 869525000, 1000, &header, &r) || !header)
        return;
    concurrency::LockGuard guard(&lock);
    Entry *e = findOrAdd(FINDER_SYS_MESHTASTIC, 869525, 11);
    e->s.bwKHz = 250;
    e->s.hits++;
    e->s.last = r;
    e->s.peak = std::max(e->s.peak, r);
    e->s.confirmed = true;
    e->lastMs = millis();
    LOG_INFO("SignalFinder: Meshtastic LongFast header at 869.525 MHz, %d dBm", r / 2);
}

void SignalFinder::watchStep(RadioLibInterface *radio)
{
    static uint8_t calls = 0;
    if (++calls % 4 == 0) { // every 4th call (~1 s of other work in between)
        listenMeshtastic(radio);
        return;
    }
    if (target == 0) { // single LoRa channels, one per call
        const LoraChannel &c = c_loraChans[chan];
        {
            concurrency::LockGuard guard(&lock);
            watching = c.system;
        }
        watchLoRaChannel(radio, c.system, c.khz, c.sync);
        if (++chan >= c_loraChanCount) {
            target = 1;
            chan = 0;
            elrsBand = 0;
        }
        return;
    }

    // ELRS: short CADs over the band's channels, all its spreading factors on each
    const ElrsBand &b = c_elrsBands[elrsBand];
    {
        concurrency::LockGuard guard(&lock);
        watching = b.system;
    }
    const uint32_t start = millis();
    while (chan < b.count && millis() - start < c_budgetMs) {
        const uint32_t f = elrsChannelKHz(b, chan);
        for (uint8_t sf = b.sfLo; sf <= b.sfHi; sf++) {
            bool found = false;
            int16_t r = INT16_MIN;
            if (radio->spectrumDetect(loraCad(sf, b.bwKHz), f * 1000, b.windowMs, &found, &r) && found) {
                elrsHits[sf - 5]++;
                elrsChans[sf - 5][chan / 8] |= 1 << (chan % 8);
                elrsPeak[sf - 5] = std::max(elrsPeak[sf - 5], r);
            }
        }
        chan++;
    }
    if (chan < b.count)
        return;
    finishElrs(radio, elrsBand, b.system, (elrsChannelKHz(b, 0) + elrsChannelKHz(b, b.count - 1)) / 2, (uint16_t)b.bwKHz);
    chan = 0;
    if (++elrsBand < c_elrsBandCount)
        return;
    target = 0; // all done: start over
    concurrency::LockGuard guard(&lock);
    cycles++;
    expire();
}

// --- C++ API used by device-ui (spectrum/SignalFinderApi.h) ---

bool finderStart(uint8_t mode)
{
    return signalFinder && signalFinder->start(mode);
}

void finderStop(void)
{
    if (signalFinder)
        signalFinder->stop();
}

uint16_t finderGetList(FinderSignal *list, uint16_t max, FinderStatus *status)
{
    return signalFinder ? signalFinder->getList(list, max, status) : 0;
}

void finderClear(void)
{
    if (signalFinder)
        signalFinder->clear();
}

#endif
