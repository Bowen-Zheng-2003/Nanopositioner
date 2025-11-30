//// RELATIVE POSITIONING
unsigned long previousMillis = 0;    // Stores last time Serial was printed
const long interval = 100;           // Interval at which to print (milliseconds)

volatile long tracker = 0;
volatile long encoder_counts = 0;
float counts_to_micron = 5;

void setup() {
  DDRE = 0b00000000; // For pins 2 & 3
  // set up external interrupt registers 
  EICRA = 0;
  EICRB = 0b00000101 ; // 0101 indicates that any change (rising/falling) on interrupts 4 and 5 will signal change (now counting in quad)
  EIMSK = 0b00110000;
  EIFR = 0b00110000; // For pins 18 & 19

  // setting up reading the complementary values (NOT A & NOT B)
  DDRD = 0b00000000;


  Serial.begin(115200);
  
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;   // Save the last time you printed
    // Serial.print("Encoder counts: ");
    Serial.println(encoder_counts);   // Print the encoder counts
    // Serial.print("Reference counts: ");
    // Serial.println(tracker);   // Print the encoder counts
  }
}

ISR(INT4_vect){ // INT4 is pin 2, port E (Channel A)
  if ((PINE & 0b00010000) == 0b00010000){ // checks if CHANNEL A is rising edge
    if((PIND & 0b00001000) == 0b00001000){ // checking pin 18, which is NOT A
      // encoder_counts = encoder_counts + 0; // if they're both high, there's an error
    }
    else if((PIND & 0b00001000) == 0b00000000){
      // encoder_counts = encoder_counts + 1; // if they're opposite, there's no error

      if((PINE & 0b00100000) == 0b00100000){
        encoder_counts = encoder_counts - 1;
      }
      else if((PINE & 0b00100000) == 0b00000000){
        encoder_counts = encoder_counts + 1;
      }
    }
  }
  else if ((PINE & 0b00010000) == 0b00000000){ // checks if CHANNEL A is falling edge
    if((PIND & 0b00001000) == 0b00001000){
      // encoder_counts = encoder_counts + 1;

      if((PINE & 0b00100000) == 0b00100000){
        encoder_counts = encoder_counts + 1;
      }
      else if((PINE & 0b00100000) == 0b00000000){
        encoder_counts = encoder_counts - 1;
      }
    }
    else if((PIND & 0b00001000) == 0b00000000){
      // encoder_counts = encoder_counts + 0;
    }
  }
  // tracker ++;
}

ISR(INT5_vect){ // INT5 is pin 3, port E (Channel B)
  if ((PINE & 0b00100000) == 0b00100000){ // checks if CHANNEL B is rising edge
    if((PIND & 0b00000100) == 0b00000100){ // checking pin 19, which is NOT B
      // encoder_counts = encoder_counts + 0; // if they're both high, there's an error
    }
    else if((PIND & 0b00000100) == 0b00000000){
      // encoder_counts = encoder_counts + 1; // if they're opposite, there's no error

      if((PINE & 0b00010000) == 0b00010000){
        encoder_counts = encoder_counts + 1;
      }
      else if((PINE & 0b00010000) == 0b00000000){
        encoder_counts = encoder_counts - 1;
      }
    }
  }
  else if ((PINE & 0b00100000) == 0b00000000){ // checks if CHANNEL B is falling edge
    if((PIND & 0b00000100) == 0b00000100){
      // encoder_counts = encoder_counts + 1;
    }
    else if((PIND & 0b00000100) == 0b00000000){
      // encoder_counts = encoder_counts + 0;
      if((PINE & 0b00010000) == 0b00010000){
        encoder_counts = encoder_counts - 1;
      }
      else if((PINE & 0b00010000) == 0b00000000){
        encoder_counts = encoder_counts + 1;
      }
    }
  }
  // tracker ++;
}
