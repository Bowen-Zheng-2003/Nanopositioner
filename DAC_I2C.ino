#include <Wire.h>

#define MCP4728_ADDR 0x60  // Default I2C address for MCP4728

static uint16_t valueA = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();  // Initialize I2C communication
  Wire.setClock(400000);  // Set to 400kHz, which is the maximum for standard mode I2C
}

void loop() {
  // Basic DAC control
  static const uint16_t maxValue = 4000;
  static const uint16_t stepSizeA = 700;

  // Increasing sawtooth waveform
  // valueA = (valueA + stepSizeA) % (maxValue + 1);
  // Decreasing sawtooth waveform
  valueA = (valueA + maxValue - stepSizeA) % (maxValue + 1);
  
  // Inline the writeToChannel function
  Wire.beginTransmission(MCP4728_ADDR);
  Wire.write(0x40);  // Channel 0
  Wire.write((valueA >> 8) & 0x0F);
  Wire.write(valueA & 0xFF);
  Wire.endTransmission();
}
