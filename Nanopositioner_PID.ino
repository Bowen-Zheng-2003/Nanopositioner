//** PIN ASSIGNMENT: **//
// FOR POSITION
const int PIN_NR_ENCODER_A    = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B    = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int CS_PIN              = 4;  // Enables encoder A and B to start when set to LOW 
// FOR SIGNAL GENERATOR
const int SIGNAL              = 7; 
const int CURSOR              = 8; 
const int MODE                = 9; 
const int ADD                 = 10; 
const int SUBTRACT            = 11;

//** State Machine: **//
// CONSTANTS: 
// Definition of states in the state machine
const int CALIBRATE     = 1;
const int MOVE          = 2;
const int WAIT          = 3;
// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int  state = CALIBRATE;
int old_state = CALIBRATE;

//** VARIABLE: **//
// FOR POSITION
volatile int piezoPosition    = -15000; // [encoder counts] Current piezo position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int oldPiezoPosition = 200;
volatile int encoderStatus    = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).

// FOR CALIBRATION
const int calibrateRange      = 2;
unsigned long startTime       = 0;
const unsigned long duration  = 2000; // 2 seconds in milliseconds
bool calibrationDone          = false;
const float micronsToCount    = 0.513;
const float countsToMicrons   = 1.949;
const int range               = 5; // units are in encoder counts
bool messageDisplayed = false; // Global variable to track if message has been displayed

// FOR SIGNAL GEN
const int DELAY_ON            = 50; 
const int DELAY_OFF           = 100; 
bool TOGGLE_STATE             = false; // false is if signal generator is off, true is on
int cursor_location           = 6; // Ranging from 1 to 6 - tells you where the cursor is on LCD
long disp_freq                = 100000; // Default number that comes up when powering on device
long new_frequency            = 0;

// FOR PID CONTROL
unsigned long executionDuration = 0;  // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time
int  targetPosition  = 0;   // [encoder counts] desired motor position
float positionError  = 0;   // [encoder counts] Position error
float integralError  = 0;   // [encoder counts * seconds] Integrated position error
float velocityError  = 0;   // [encoder counts / seconds] Velocity error
float desiredVoltage = 0;   // [Volt] Desired motor voltage
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
float piezoVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 
const int  MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME  = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
const int speed = 100;
const float KP             = 5;  // [Volt / encoder counts] P-Gain
const float KD             = 0.05;  // [Volt * seconds / encoder counts] D-Gain
const float KI             = 0.5; // [Volt / (encoder counts * seconds)] I-Gain
// Timing:
const long  WAIT_TIME = 1000000; // [microseconds] Time waiting at each location
unsigned long startWaitTime; // [microseconds] System clock value at the moment the WAIT state started
const int TARGET_BAND = 5;                      // [encoder counts] "Close enough" range when moving towards a target.
bool direction = false; // false is reverse, true is forward

void setup() {
  // FOR POSITION
  pinMode(PIN_NR_ENCODER_A,               INPUT);
  pinMode(PIN_NR_ENCODER_B,               INPUT); 
  pinMode(CS_PIN,                         OUTPUT);
  digitalWrite(CS_PIN,                    LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  attachInterrupt(0, updatePiezoPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updatePiezoPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // FOR SIGNAL GEN
  pinMode(MODE,                           OUTPUT);
  pinMode(CURSOR,                         OUTPUT);
  pinMode(ADD,                            OUTPUT);
  pinMode(SUBTRACT,                       OUTPUT);
  pinMode(SIGNAL,                         OUTPUT);

  Serial.begin(115200);
}

void loop() {
  //******************************************************************************//
  // The state machine:
  switch (state) {
    case CALIBRATE:
      setSignal();
      signalOutput(); // turn it on
      setPosition();

      //Transition into WAIT state
      //Record which state you came from
      old_state = CALIBRATE;
      state = WAIT;
      startWaitTime = micros();
      break;

    case MOVE:
      if (piezoPosition>=targetPosition-TARGET_BAND && piezoPosition<=targetPosition+TARGET_BAND) {
        Serial.println("YOU FUCKING MADE IT !!!");
        if (TOGGLE_STATE){ // make sure the signal generator is off!
          signalOutput();
        }
        // We reached the position  
        // Start waiting timer:
        startWaitTime = micros();
        //Tranistion into WAIT state
        //Record which state you came from
        old_state = MOVE;
        state = WAIT;
      } 
      // Otherwise we continue moving towards the position
      break;
    
    case WAIT:
      if (micros()-startWaitTime>WAIT_TIME){

        if (old_state == CALIBRATE){
          Serial.println("Waiting for USER INPUT (from CALIBRATE)");
          state = MOVE;
          targetPosition = userInput();
          Serial.println("State transition from CALIBRATE to MOVE");
        }

        if (old_state == MOVE){
          Serial.println("Waiting for USER INPUT (from MOVE)");
          state = MOVE;
          targetPosition = userInput();
          Serial.println("State transition from MOVE to MOVE");
        }
      }
      break;

    default: 
      Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1); // infinite loop to halt the program
    break;

  }
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(piezoPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ... 
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    piezoVelocity = (double)(piezoPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = piezoPosition;
    previousVelCompTime   = micros();
  }
  // Serial.println(executionDuration);

  //** PID control: **//  
  // Compute the position error [encoder counts]
  positionError = targetPosition - piezoPosition;
  // Compute the integral of the position error  [encoder counts * seconds]
  integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
  // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
  velocityError = 0 - piezoVelocity;
  // This is the actual controller function that uses the error in 
  // position and velocity and the integrated error and computes a
  // desired voltage that should be sent to the motor:
  desiredVoltage = KP * positionError +  
                   KI * integralError +
                   KD * velocityError;
  
  if (desiredVoltage >= 900){
    desiredVoltage = 900;
  }
  else if (desiredVoltage <= -900){
    desiredVoltage = -900;
  }

  if ((desiredVoltage >= 0) && (state==MOVE)){
    if (TOGGLE_STATE){
      signalOutput();
    }
    changeSpeed(int(desiredVoltage));
    reverse();
    signalOutput();
  }
  else if ((desiredVoltage < 0) && (state==MOVE)){
    if (TOGGLE_STATE){
      signalOutput();
    }
    changeSpeed(int(abs(desiredVoltage)));
    forward();
    signalOutput();
  }
  
  Serial.print("CURRENT POSITION: ");
  Serial.println(piezoPosition);
  Serial.print("Target position: ");
  Serial.println(targetPosition);
}

void mode() {
  digitalWrite(MODE, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(MODE, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("Changing MODE");
}

void cursor() {
  digitalWrite(CURSOR, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(CURSOR, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("Changing CURSOR");
  cursor_location = cursor_location-1;
  if (cursor_location == 0){ // help loop it back
    cursor_location = 6;
  }
}

void increment() {
  digitalWrite(ADD, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(ADD, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("INCREASING");
}

void decrement() {
  digitalWrite(SUBTRACT, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SUBTRACT, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // Serial.println("DECREASING");
}

void signalOutput() {
  digitalWrite(SIGNAL, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SIGNAL, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  TOGGLE_STATE = !TOGGLE_STATE;
  if (!TOGGLE_STATE){
    cursor_location = 6;
  }
  // Serial.print("Turning signal ");
  // Serial.println(TOGGLE_STATE ? "ON" : "OFF");
}

void forward() {
  if (!direction){ // if it's in reverse mode, do this
    for (int i = 0; i < 6; i++) {
      mode();
    }
    direction = true;
  } else{
    return;
  }
}

void reverse() {
  if (direction){
    mode();
    direction = false;
  } else{
    return;
  }
}

void setSignal() {
  for (int i = 0; i < 3; i++) {
    mode();
    direction = true; // making it move forward now
  }
  changeSpeed(1000);
  Serial.println("Finished CALIBRATION");
  Serial.println("Starting in 1 sec");
  delay(1000);
}

void setPosition() {  
  while((piezoPosition != 0) && (!calibrationDone)){
    if ((piezoPosition >= oldPiezoPosition - calibrateRange) && 
        (piezoPosition <= oldPiezoPosition + calibrateRange)) {
      if (startTime == 0) {
        startTime = millis();
      } else if (millis() - startTime >= duration) {
        signalOutput();
        piezoPosition = 0;
        calibrationDone = true;
        Serial.println("Finished calibrating position");
        Serial.print("The starting position is: ");
        Serial.println(piezoPosition);
        break;
      }
    } else {
      startTime = 0;
    }
    oldPiezoPosition = piezoPosition;
    Serial.print("The calibrating encoder value is ");
    Serial.println(piezoPosition);
  }
}

void changeSpeed(long frequency) {
  String old_freq = String(disp_freq);
  String new_freq = String(frequency);
  if (old_freq.length() > new_freq.length()){
    int new_cursor_position = cursor_location - old_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = old_freq.length(); i > 0; i--) {
      if (i <= new_freq.length()){ // once the old and new frequency have same digits
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        int diff = new_freq_i - old_freq_i;
        if (diff > 0){
          for (int i = 0; i < diff; i++){
            increment();
          }
        }
        else if (diff < 0){
          diff = abs(diff);
          for (int i = 0; i < diff; i++){
            decrement();
          }
        }
      }
      else {
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        if (old_freq_i > 0){
          for (int i = 0; i < old_freq_i; i++){
            decrement();
          }
        }
      }
      cursor();
    }
  }
  else if (old_freq.length() == new_freq.length()){
    int new_cursor_position = cursor_location - old_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = old_freq.length(); i > 0; i--) {
      char old_freq_c = old_freq[old_freq.length()-i];
      int old_freq_i = old_freq_c - '0';
      char new_freq_c = new_freq[new_freq.length()-i];
      int new_freq_i = new_freq_c - '0';
      int diff = new_freq_i - old_freq_i;
      if (diff > 0){
        for (int i = 0; i < diff; i++){
          increment();
        }
      }
      else if (diff < 0){
        diff = abs(diff);
        for (int i = 0; i < diff; i++){
          decrement();
        }
      }
      cursor();
    }
  }
  else if (new_freq.length() > old_freq.length()){
    int new_cursor_position = cursor_location - new_freq.length();
    for (int i = 0; i < new_cursor_position; i++){
      cursor();
    }
    for (int i = new_freq.length(); i > 0; i--) {
      if (i <= old_freq.length()){ // once the old and new frequency have same digits
        char old_freq_c = old_freq[old_freq.length()-i];
        int old_freq_i = old_freq_c - '0';
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        int diff = new_freq_i - old_freq_i;
        if (diff > 0){
          for (int i = 0; i < diff; i++){
            increment();
          }
        }
        else if (diff < 0){
          diff = abs(diff);
          for (int i = 0; i < diff; i++){
            decrement();
          }
        }
      }
      else {
        char new_freq_c = new_freq[new_freq.length()-i];
        int new_freq_i = new_freq_c - '0';
        if (new_freq_i > 0){
          for (int i = 0; i < new_freq_i; i++){
            increment();
          }
        }
      }
      cursor();
    }
  }
  disp_freq = frequency;
  // Serial.print("The position is now ");
  // Serial.println(cursor_location);
  // Serial.print("The new displayed frequency is ");
  // Serial.println(disp_freq);
}

float userInput() {
  float value = 0; 
  if (!messageDisplayed) {
    Serial.println("Type in the position (microns) you want to move to!");
    messageDisplayed = true;
  }  
  while (Serial.available() == 0) {
    // Wait for user input
  }
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    value = input.toFloat()*micronsToCount;
  }
  return value;
}

//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updatePiezoPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(PIN_NR_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(PIN_NR_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    piezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    piezoPosition--;         // decrease the encoder count by one
  }
}
