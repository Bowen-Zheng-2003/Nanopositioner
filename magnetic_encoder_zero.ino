#include <Arduino.h>
#include "SPI.h"
const int X_CS_PIN = A2;
const int Y_CS_PIN = A3;

int position_x = 0;
int position_y = 0;
int old_x_pos = 0;
int old_y_pos = 0;
int new_x_pos = 0;
int new_y_pos = 0;
double counts_microns = 2000.0/4096;

void checkEvenParity(uint32_t data0, uint32_t data1, uint32_t data2) { // check even parity
  uint32_t first18bits = (data0 << 10) | ((uint32_t)data1 << 2) | ((data2 >> 6) & 0x03);
  Serial.print("Binary: ");
  Serial.println(first18bits, BIN);
  // Count set bits in first 18 bits
  uint8_t count = 0;
  while (first18bits) {
    count += first18bits & 1;
    first18bits >>= 1;
  }
  if (count % 2 == 0) {
    Serial.println("Even parity: PASSES");
  } else {
    Serial.println("Even parity: FAILS");
  }
}

// void checkSensorDistance(uint32_t data0, uint32_t data1, uint32_t data2){
//   // Assemble 24 bits from data0, data1, and data2
//   uint32_t combined = (data0 << 16) | (data1 << 8) | data2;

//   // Extract bits 15-17 (assuming LSB 0 indexing)
//   uint8_t bit15 = (combined >> 8) & 0x1;
//   uint8_t bit16 = (combined >> 7) & 0x1;
//   uint8_t bit17 = (combined >> 6) & 0x1;

//   if (bit15 == 0 && bit16 == 0 && bit17 == 0){
//     Serial.println("Sensor Distance: Best Accuracy");
//   }
//   else if (bit15 == 0 && bit16 == 0 && bit17 == 1){
//     Serial.println("Sensor Distance: Magnet is moving away from chip");
//   }
//   else if (bit15 == 0 && bit16 == 1 && bit17 == 0){
//     Serial.println("Sensor Distance: Magnet is moving towards chip");
//   }
//   else if (bit15 == 0 && bit16 == 1 && bit17 == 1){
//     Serial.println("Sensor Distance: Reduced accuracy (move magnet closer to sensor)");
//   }
//   else if (bit15 == 1 && bit16 == 1 && bit17 == 1){
//     Serial.println("Sensor Distance: Worst accuracy (move magnet closer to sensor)");
//   }
//   else{
//     Serial.println("Sensor Distance: Uninterpretable Error");
//   }
// }

uint16_t SPI_transact(int CS_PIN) {
  digitalWrite(CS_PIN, 0);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3)); // SPI_MODE3 samples on falling edge 
  // whereas SPI_MODE1 samples on rising edge, so use SPI_MODE1 if you want magnetic field strength
  uint8_t data0 = SPI.transfer(0); // does 8 pulses
  uint8_t data1 = SPI.transfer(0);
  uint8_t data2 = SPI.transfer(0); // do this 3 times to get 24 pulses
  // each SPI.transfer returns a byte
  SPI.endTransaction();
  digitalWrite(CS_PIN, 1);

  // Bit-wise Right Shift
  uint16_t output = ((uint16_t)data0 << 5) | (data1 >> 4);

  // Serial.print("Binary: ");
  // Serial.println(output, BIN);

  // checkEvenParity(data0, data1, data2);
  // checkSensorDistance(data0, data1, data2);

  // if (CS_PIN == X_CS_PIN){
  //   Serial.print("X Position is: ");
  // }
  // else if (CS_PIN == Y_CS_PIN){
  //   Serial.print("Y Position is: ");
  // }
  
  // Serial.println(output);
  return output;
}

int track_position(int current_pos, int new_pos, int old_pos){
  int diff = new_pos-old_pos;
  if(abs(diff) > 4096/4) { // wrap around occurred
    if(diff > 0) { // backward motion occurred
      current_pos = current_pos - (4096-new_pos) - old_pos; 
    }
    else { // forward motion occured
      current_pos = current_pos + (4096-old_pos) + new_pos; 
    }
  }
  else { // no wrap around
    current_pos = current_pos + diff;
  }

  return current_pos;
}

void setup(){
  pinMode(X_CS_PIN, OUTPUT); // Chip Select
  pinMode(Y_CS_PIN, OUTPUT); // Chip Select
  pinMode(SCK, OUTPUT); // CLK
  pinMode(MISO, INPUT); // MISO

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  while (! Serial) {
    // wait until Serial is connected
  }

  int trash_x = SPI_transact(X_CS_PIN); // upon start up, first value is trashed
  int trash_y = SPI_transact(Y_CS_PIN); // upon start up, first value is trashed
  delay(1000);
  old_x_pos = SPI_transact(X_CS_PIN);
  old_y_pos = SPI_transact(Y_CS_PIN);
}

void loop() {
  new_x_pos = SPI_transact(X_CS_PIN);
  position_x = track_position(position_x, new_x_pos, old_x_pos);
  Serial.print("X Pos: ");
  Serial.println(position_x*counts_microns);
  old_x_pos = new_x_pos;

  new_y_pos = SPI_transact(Y_CS_PIN);
  position_y = track_position(position_y, new_y_pos, old_y_pos);
  Serial.print("Y Pos: ");
  Serial.println(position_y*counts_microns);
  old_y_pos = new_y_pos;
  // delay(100);
  
}
