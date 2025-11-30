const int delay_um = 1; // microseconds

// const byte CLK_HIGH = 0b00010000; //Clock -> pin 2
// const byte CLK_LOW  = 0b00000000; //Clock -> pin 2
// const byte DATA = 0b00001000; //Data -> pin 3

void setup() {
  DDRE = DDRE | 0b00010000; // 0 is input, 1 is output, so I'm setting pin 2 as output and pin 3 as input

  Serial.begin(115200);
  delay(200); //Minimum delay after powering encoder
  //Initialize CLK high
  PORTE = 0b00010000;
}

void loop() {
  // PORTE = 0b00000000;
  // delay(100);
  // PORTE = 0b00010000;
  // delay(100);
  // Serial.println((PINE & 0b00100000) == 0b00100000);
  // delay(10);

  unsigned long enc_data = takeReading();
  Serial.println(enc_data, BIN);
  delay(500);
  // Serial.println(encoder_counts*0.078125);
}

unsigned long takeReading() {
  unsigned long data = 0; 
  unsigned long trash = 0;
  unsigned long bit = 0;

  PORTE = 0b00000000;
  delayMicroseconds(delay_um);

  for(int i=0; i<31; i++){
    PORTE = 0b00010000;

    //Store encoder position bits
    bit = (PINE & 0b00100000) == 0b00100000;

    PORTE = 0b00000000;
    delayMicroseconds(delay_um);
    

    // if ((i < 17) & (i > 0)){
    data <<= 1;
    data |= bit;
    // }
    
    // data = (PIND & 0b00001000) ? 1 : 0;  // Mask isolates bit 3 (1<<3), non-zero means HIGH

    // Serial.print("Data: ");
    // Serial.print(bit);
  }

  // Serial.println(" ");
  // Serial.print("Data: ");
  // Serial.println(data);
  delayMicroseconds(30);
  return data;
}
