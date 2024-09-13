#include <Wire.h>

#define MCP4728_ADDR 0x60  // Default I2C address for MCP4728
static int valueA = 0;

void setup() {
  Wire.begin();
  Wire.setClock(400000);  // Set I2C clock to 400kHz for faster communication
  Serial.begin(115200);   // Initialize serial communication for debugging
}

void loop() {
  const int maxValue = 4000;
  const int stepSizeA = 425;  // Smaller step size for channel A

  // Update channel A more frequently
  valueA = (valueA + stepSizeA) % (maxValue + 1);
  writeToChannel(0, valueA);
  Serial.print("Channel A: ");
  Serial.println(valueA);
  // delayMicroseconds(100);  // Small delay to control update rate
}

void writeToChannel(uint8_t channel, uint16_t value) {
  Wire.beginTransmission(MCP4728_ADDR);
  
  // Command byte: 0 1 0 0 DAC1 DAC0 0 UDAC
  // DAC1 DAC0: Channel select bits
  // UDAC: Update DAC register bit (0 for this example)
  Wire.write(0x40 | (channel << 1));
  
  // Data bytes
  Wire.write((value >> 8) & 0x0F);  // Upper 4 bits of the 12-bit value
  Wire.write(value & 0xFF);         // Lower 8 bits of the 12-bit value
  
  Wire.endTransmission();
}