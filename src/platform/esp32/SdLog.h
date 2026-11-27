#pragma once
#include "configuration.h"
#if defined(ARCH_ESP32) && defined(SD_LOG)
void sdLogBegin(void);
void sdLogLine(const char *level, const char *text); // text ends with '\n'
#endif
