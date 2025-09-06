#include <Arduino.h>
#include "SPI.h"
const int X_CS_PIN = A2;
const int Y_CS_PIN = A3;
const int Y_PRG_PIN = A4;

uint64_t position_x = 0;
uint64_t position_y = 0;

const int ZERO_POS_WIDTH = 12;
const int ZERO_POS_HIGH = 46;
const int OTP_BITS_TOTAL = 56;
const int shift_amount = OTP_BITS_TOTAL - (ZERO_POS_HIGH + 1); // 56 - 47 = 9


// void checkEvenParity(uint32_t data0, uint32_t data1, uint32_t data2) { // check even parity
//   uint32_t first18bits = (data0 << 10) | ((uint32_t)data1 << 2) | ((data2 >> 6) & 0x03);
//   // Count set bits in first 18 bits
//   uint8_t count = 0;
//   while (first18bits) {
//     count += first18bits & 1;
//     first18bits >>= 1;
//   }
//   if (count % 2 == 0) {
//     Serial.println("Even parity: PASSES");
//   } else {
//     Serial.println("Even parity: FAILS");
//   }
// }

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

uint64_t read_otp(int CS_PIN, int PRG_PIN) {
  digitalWrite(PRG_PIN, HIGH);   
  digitalWrite(SCK, LOW);        // ensure CLK is low
  digitalWrite(CS_PIN, HIGH);
  delayMicroseconds(1);
  digitalWrite(CS_PIN, LOW);     // enter OTP mode

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  uint64_t otp_data = 0;
  for (int i = 0; i < 7; i++) {   // 7 x 8 = 56 bits (OTP = 52 bits, padded)
    otp_data <<= 8;
    otp_data |= SPI.transfer(0);
  }

  SPI.endTransaction();
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(PRG_PIN, LOW);    // exit OTP mode
  Serial.print("OTP binary: ");
  Serial.println(otp_data, BIN);

  return otp_data;
}

void writeZeroPositionOTP(int CS_PIN, int PRG_PIN, uint16_t newZeroPosition) {
  // Assume newZeroPosition is 12-bit value to write into bits 35-46 of 52-bit OTP register

  // 1. Read current OTP contents first (optional) or start with default all zero except zero position bits
  uint64_t otp = read_otp(CS_PIN, PRG_PIN); // start with 0 (or read current content if you want)
  Serial.print("Initial OTP data: ");
  Serial.println(otp, BIN);

  // 2. Read 38-49 position bits (0xFFF means 12 bits) to get zero positioning offset:
  uint16_t zeroPosBits = (otp >> 38) & 0xFFF;
  Serial.print("Zero position data (35-46): ");
  Serial.println(zeroPosBits, BIN);

  // 3. Find difference to recalibrate the zero:
  uint16_t difference = newZeroPosition - zeroPosBits;
  Serial.print("New offset value: ");
  Serial.println(difference, BIN);

  // 4. Create new 64 bit int
  uint64_t difference_64 = ((uint64_t)difference) << 38;
  Serial.print("Difference 64 value: ");
  Serial.println(difference_64, BIN);

  // 5. Replace the values in OTP with new offset value
  uint64_t mask = ((uint64_t)0xFFF) << 38;  // 12 bits mask shifted to position 38
  otp &= ~mask;  // Clear bits 38-49 in otp
  otp |= (difference_64 & mask);  // Only bits in mask will be set
  Serial.print("Final Output: ");
  Serial.println(otp, BIN);

  // Enter OTP write mode
  digitalWrite(PRG_PIN, HIGH);
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(SCK, LOW);  // ensure CLK low before CSn falls
  delayMicroseconds(1);

  // Start OTP access: CSn rising edge already HIGH
  digitalWrite(CS_PIN, LOW); 

  // Begin SPI transaction, mode 1 (CPOL=0, CPHA=1)
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // Shift out 52 bits (as 7 bytes to cover 56 bits)
  // We'll send msb first: shift bits from highest (bit 51) down to bit 0
  for (int i = 6; i >= 0; i--) { 
    byte b = (otp >> (i*8)) & 0xFF;
    SPI.transfer(b);
  }

  SPI.endTransaction();

  // Finish OTP operation
  digitalWrite(CS_PIN, HIGH);
  digitalWrite(PRG_PIN, LOW);
  delay(10000);

  // Now zero position temporarily updated in OTP register
  
  Serial.print("Temporarily wrote zero position bits: ");
  Serial.println(newZeroPosition);
}

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

  Serial.print("Binary: ");
  Serial.println(output, BIN);

  // checkEvenParity(data0, data1, data2);
  // checkSensorDistance(data0, data1, data2);

  if (CS_PIN == X_CS_PIN){
    Serial.print("X Position is: ");
  }
  else if (CS_PIN == Y_CS_PIN){
    Serial.print("Y Position is: ");
  }
  
  Serial.println(output);
  return output;
}

void setup(){
  pinMode(X_CS_PIN, OUTPUT); // Chip Select
  pinMode(Y_CS_PIN, OUTPUT); // Chip Select
  pinMode(SCK, OUTPUT); // CLK
  pinMode(MISO, INPUT); // MISO
  digitalWrite(SCK, 1);  //CLK
  digitalWrite(X_CS_PIN, 1);   //Csn
  digitalWrite(Y_CS_PIN, 1);   //Csn
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  while (! Serial) {
    // wait until Serial is connected
  }
  position_y = SPI_transact(Y_CS_PIN);
  writeZeroPositionOTP(Y_CS_PIN, Y_PRG_PIN, position_y);
  position_x = read_otp(Y_CS_PIN, Y_PRG_PIN);
  delay(5000);
}

void loop() {
  // position_x = SPI_transact(X_CS_PIN);
  // delay(500);  
  // position_y = SPI_transact(Y_CS_PIN);
  delay(5000); 
  
}
