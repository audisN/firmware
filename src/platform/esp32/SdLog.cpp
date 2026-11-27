#include "configuration.h"
#if defined(ARCH_ESP32) && defined(SD_LOG)
// Field debug log: every log line also goes into a PSRAM ring, and a low-priority task appends it to
// /sdcard/meshlog.txt once a second (the card is mounted by device-ui). The task also writes a heartbeat every
// 10 s, so a hang shows whether only the main loop stopped or the whole chip.

#include "SdLog.h"
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>

namespace
{
constexpr size_t kRing = 64 * 1024;
constexpr long kMaxFile = 4L * 1024 * 1024;
const char *kFile = "/sdcard/meshlog.txt";
const char *kOld = "/sdcard/meshlog.old.txt";

char *ring = nullptr;
size_t head = 0, tail = 0; // head: next write, tail: next read
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t lastLogMs = 0;

void put(const char *s, size_t n)
{
    if (!ring)
        return;
    portENTER_CRITICAL(&mux);
    for (size_t i = 0; i < n; i++) {
        ring[head] = s[i];
        head = (head + 1) % kRing;
        if (head == tail) // full: drop the oldest byte
            tail = (tail + 1) % kRing;
    }
    portEXIT_CRITICAL(&mux);
}

void task(void *)
{
    static char chunk[4096];
    uint32_t lastBeat = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - lastBeat >= 10000) {
            lastBeat = now;
            char b[160];
            int n = snprintf(b, sizeof(b), "%lu HB heap=%u min=%u psram=%u lastlog=%lus ago\n", (unsigned long)now,
                             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                             (unsigned long)((now - lastLogMs) / 1000));
            put(b, n);
        }
        if (head == tail)
            continue;
        FILE *f = fopen(kFile, "a");
        if (!f)
            continue; // card not mounted (yet); the ring keeps the newest 64 KB
        for (;;) {
            size_t n = 0;
            portENTER_CRITICAL(&mux);
            while (tail != head && n < sizeof(chunk)) {
                chunk[n++] = ring[tail];
                tail = (tail + 1) % kRing;
            }
            portEXIT_CRITICAL(&mux);
            if (!n)
                break;
            fwrite(chunk, 1, n, f);
        }
        long size = ftell(f);
        fclose(f);
        static bool told = false;
        if (!told) {
            told = true;
            LOG_INFO("SD log: writing %s (%ld bytes)", kFile, size);
        }
        if (size > kMaxFile) {
            remove(kOld);
            rename(kFile, kOld);
        }
    }
}
} // namespace

void sdLogBegin(void)
{
    if (ring)
        return;
    ring = (char *)heap_caps_malloc(kRing, MALLOC_CAP_SPIRAM);
    if (!ring)
        return;
    char b[96];
    int n = snprintf(b, sizeof(b), "\n===== boot, reset reason %d =====\n", (int)esp_reset_reason());
    put(b, n);
    xTaskCreatePinnedToCore(task, "sdlog", 4096, nullptr, 1, nullptr, 0);
}

void sdLogLine(const char *level, const char *text)
{
    if (!ring)
        return;
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    lastLogMs = now;
    char b[24];
    int n = snprintf(b, sizeof(b), "%lu %s ", (unsigned long)now, level);
    put(b, n);
    put(text, strlen(text));
}
#endif
