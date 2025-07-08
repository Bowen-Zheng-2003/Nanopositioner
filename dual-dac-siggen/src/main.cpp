#include <Arduino.h>
#include "timers.h"

void setup() {
  // start 
  setup_timers();
  // le blink 
  pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t last_blink = 0;
float phase = 0.0F;

void loop() {
  if(last_blink + 50 < millis()){
    last_blink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // do sinusoid ? 
    phase += 0.5;
    if(phase > TWO_PI) phase = 0.0F;
    // to do 0-1.0F wave 
    float out_0 = (sin(phase) + 1.0F) / 2.0F;
    float out_1 = (cos(phase) + 1.0F) / 2.0F;
    // und align...
    // write_freqs(out_0 * FREQ_MAX, out_1 * FREQ_MAX);
    // write_freq((1.0F * FREQ_MAX)); // at slowest
    // write_freq(1);                 // at fastest 
    write_freqs(255, 254);
  }
}

