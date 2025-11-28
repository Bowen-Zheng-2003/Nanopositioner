const int delay_um = 2; // microseconds

const byte CLK_HIGH = B00000100; //Clock -> pin 2
const byte CLK_LOW  = B00000000; //Clock -> pin 2
const byte DATA = B00001000; //Data -> pin 3

void setup() {
  DDRD = DDRD | B00001100; // 0 is input, 1 is output, so I'm setting pin 2 as output and pin 3 as input

  Serial.begin(115200);
  delay(200); //Minimum delay after powering encoder
  //Initialize CLK high
  PORTD = CLK_HIGH;
}

void loop() {
  unsigned long enc_data = takeReading();
  // Serial.println(enc_data);
  // delay(500);
  // Serial.println(encoder_counts*0.078125);
}

unsigned long takeReading() {
  unsigned long data = 0; 
  unsigned long trash = 0;
  unsigned long bit = 0;

  PORTD = CLK_LOW;
  delayMicroseconds(delay_um);

  for(int i=0; i<31; i++){
    PORTD = CLK_HIGH;
    delayMicroseconds(delay_um);

    //Store encoder position bits
    bit = digitalRead(3);

    PORTD = CLK_LOW;
    delayMicroseconds(delay_um);
    

    // if ((i < 17) & (i > 0)){
    data <<= 1;
    data |= bit;
    // }
    
    // data = (PIND & 0b00001000) ? 1 : 0;  // Mask isolates bit 3 (1<<3), non-zero means HIGH

    // Serial.print("Data: ");
    Serial.print(bit);
  }

  Serial.println(" ");
  // Serial.print("Data: ");
  // Serial.println(data);
  delayMicroseconds(30);
  return data;
}
