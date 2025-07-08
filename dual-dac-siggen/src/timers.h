#ifndef SIGGEN_TIMERS_H_
#define SIGGEN_TIMERS_H_

#include <Arduino.h>

#define FREQ_MAX 0b111111111111111

void setup_timers(void);
void write_periods(uint16_t per_0, uint16_t per_1);

#endif 