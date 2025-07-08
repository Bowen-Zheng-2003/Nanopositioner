#include <Arduino.h>
#include "timers.h"

#define DEADBAND 1.0F

void write_freqs(float hz_0, float hz_1){
  // TODO: deadband... around zero... 
  if(hz_0 < DEADBAND && hz_0 > -DEADBAND){
    hz_0 = hz_0 < 0.0F ? -DEADBAND : DEADBAND;
  }
  if(hz_1 < DEADBAND && hz_1 > -DEADBAND){
    hz_1 = hz_1 < 0.0F ? -DEADBAND : DEADBAND;
  }
  // 
  float per_0 = 120000000.0F / (hz_0 * 4096.0F);
  float per_1 = 120000000.0F / (hz_1 * 4096.0F);
  // 
  write_periods((int16_t)per_0, (int16_t)per_1);
}

void setup() {
  // start 
  setup_timers();
  // le blink 
  pinMode(LED_BUILTIN, OUTPUT);
}

uint32_t last_blink = 0;
float phase = 0.0F;

// speed scaling, then dir flip ? 

void loop() {
  if(last_blink + 50 < millis()){
    last_blink = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // do sinusoid ? 
    phase += 0.1;
    if(phase > TWO_PI) phase = 0.0F;
    // // to do 0-1.0F wave 
    // float out_0 = (sin(phase) + 1.0F) / 2.0F;
    // float out_1 = (cos(phase) + 1.0F) / 2.0F;
    // // say we go 500 - 1khz 
    // out_0 = out_0 * 500.0F + 500.0F;
    // out_1 = out_1 * 500.0F + 500.0F;
    // do -1k - 1khz 
    float out_0 = sin(phase) * 1000.0F;
    float out_1 = cos(phase) * 500.0F;
    // 
    write_freqs(out_0, out_1);
    // und align...
    // write_freqs(out_0 * FREQ_MAX, out_1 * FREQ_MAX);
    // write_freq((1.0F * FREQ_MAX)); // at slowest
    // write_freq(1);                 // at fastest 
    // write_periods(48, 48);
  }
}

