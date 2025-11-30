// #define CHANNEL_A 2
// #define CHANNEL_B 3
// #define CHANNEL_NOT_A 18
// #define CHANNEL_NOT_B 19
#define CHANNEL_A A2
#define CHANNEL_B A3
#define CHANNEL_NOT_A A4
#define CHANNEL_NOT_B A5

volatile long encoder_counts = 0;
volatile unsigned long isr_start_time = 0;
volatile unsigned long isr_end_time = 0;
volatile unsigned long last_isr_duration = 0;
volatile int xEncoderStatus  = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)

void setup() {
  Serial.begin(115200);

  pinMode(CHANNEL_A, INPUT);
  pinMode(CHANNEL_B, INPUT);
  pinMode(CHANNEL_NOT_A, INPUT);
  pinMode(CHANNEL_NOT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(CHANNEL_A), updateXPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CHANNEL_B), updateXPosition, CHANGE);
}

void loop() {
  Serial.print("Encoder counts: ");
  Serial.print(encoder_counts*0.01953);
  Serial.print("  Last ISR duration (us): ");
  Serial.println(last_isr_duration);
  delay(100);
}

void updateXPosition() {
  isr_start_time = micros();
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  if (digitalRead(CHANNEL_A) != digitalRead(CHANNEL_NOT_A)){
    xEncoderStatus |= digitalRead(CHANNEL_A);   
  }
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  if (digitalRead(CHANNEL_B) != digitalRead(CHANNEL_NOT_B)){
    xEncoderStatus |= digitalRead(CHANNEL_B);   
  }
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  xEncoderStatus &= 15;
  if (xEncoderStatus==2 || xEncoderStatus==4 || xEncoderStatus==11 || xEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    encoder_counts ++;         // increase the encoder count by one
    // Serial.print("Positive ");
    // Serial.println(xEncoderStatus);
  } 
  else if (xEncoderStatus==1 || xEncoderStatus==7 || xEncoderStatus==8 || xEncoderStatus==14) {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    encoder_counts --;         // decrease the encoder count by one
    // Serial.print("Negative ");
    // Serial.println(xEncoderStatus);
  }
  isr_end_time = micros();
  last_isr_duration = isr_end_time - isr_start_time;
}
