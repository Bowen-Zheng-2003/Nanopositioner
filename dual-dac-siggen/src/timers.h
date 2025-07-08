#ifndef SIGGEN_TIMERS_H_
#define SIGGEN_TIMERS_H_

#include <Arduino.h>

#define FREQ_MAX 0b111111111111111

void setup_timers(void);
void write_freq(uint16_t per);

#endif 