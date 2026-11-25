#pragma once

#include "configuration.h"

#if defined(HAS_SPECTRUM) && defined(HAS_SIGNAL_FINDER)

#include "concurrency/Lock.h"
#include "concurrency/OSThread.h"
#include "mesh/RadioLibInterface.h"
#include "spectrum/SignalFinderApi.h"

/**
 * Signal finder (see spectrum/SignalFinderApi.h): an energy list of 433-950 / 2100-2700 MHz, or a watch of the
 * channels of known systems (Meshtastic, LoRaWAN, ExpressLRS). Runs on the main loop, pauses the mesh, and does
 * at most ~0.15 s of radio work per call.
 */
class SignalFinder : public concurrency::OSThread
{
  public:
    SignalFinder();

    bool start(uint8_t mode);
    void stop();
    void clear();
    uint16_t getList(FinderSignal *list, uint16_t max, FinderStatus *status);

  protected:
    int32_t runOnce() override;

  private:
    struct Entry {
        bool used = false;
        FinderSignal s{};
        uint32_t lastMs = 0;
        uint8_t chanMask[10] = {}; // ELRS channels heard (up to 80)
    };

    // energy mode
    void sweepStep(RadioLibInterface *radio);
    void finishSegment();
    void addBurst(uint32_t loKHz, uint32_t hiKHz, int16_t peak, uint32_t peakKHz);
    // packets mode
    void watchStep(RadioLibInterface *radio);
    void listenMeshtastic(RadioLibInterface *radio);
    void watchLoRaChannel(RadioLibInterface *radio, uint8_t system, uint32_t freqKHz, uint8_t sync);
    void finishElrs(RadioLibInterface *radio, uint8_t band, uint8_t system, uint32_t centreKHz, uint16_t bwKHz);
    bool elrsEnergyConfirmed(RadioLibInterface *radio, uint8_t band);
    Entry *findOrAdd(uint8_t system, uint32_t freqKHz, uint8_t sf);
    void expire();

    concurrency::Lock lock; // guards entries and the status below
    bool want = false;
    bool active = false;
    uint8_t mode = FINDER_MODE_ENERGY;
    uint32_t lastPollMs = 0;
    static constexpr int kEntries = 48;
    Entry entries[kEntries];
    uint32_t cycles = 0;
    uint32_t sweepKHz = 0;
    uint8_t watching = FINDER_SYS_NONE;

    // energy sweep in progress (main loop only)
    uint8_t range = 0; // 0: 433-950 MHz, 1: 2100-2700 MHz
    uint32_t segStartKHz = 0;
    uint16_t segFill = 0;
    int16_t seg[160];

    // packets watch in progress (main loop only)
    uint8_t target = 0;        // 0: single LoRa channels, 1: ELRS bands
    uint8_t chan = 0;          // which channel
    uint8_t elrsBand = 0;      // which ELRS band
    uint16_t elrsHits[8] = {}; // ELRS, this pass: CAD hits per spreading factor (index sf - 5)
    uint8_t elrsChans[8][10] = {};
    int16_t elrsPeak[8] = {};
};

extern SignalFinder *signalFinder;

#endif
