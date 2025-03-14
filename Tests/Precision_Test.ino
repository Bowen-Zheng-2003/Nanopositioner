//** PIN ASSIGNMENT: **//
// FOR POSITION
const int Y_PIN_NR_ENCODER_A      = 18;  // Never change these, since the interrupts are attached to pin 2 and 3
const int Y_PIN_NR_ENCODER_B      = 19;  // Never change these, since the interrupts are attached to pin 2 and 3
const int Y_CS_PIN                = 17;  // Enables encoder A and B to start when set to LOW 

const int X_PIN_NR_ENCODER_A      = 20;  // Never change these, since the interrupts are attached to pin 2 and 3
const int X_PIN_NR_ENCODER_B      = 21;  // Never change these, since the interrupts are attached to pin 2 and 3
const int X_CS_PIN                = 16;  // Enables encoder A and B to start when set to LOW 

// FOR SIGNAL GENERATOR
const int MODE                  = 53; 
const int CURSOR                = 52; 
const int ADD                   = 51; 
const int SUBTRACT              = 50;
const int SIGNAL                = 49;
const int X_STAGE               = 10;
const int Y_STAGE               = 11; 

//** VARIABLE: **//
// STATE MACHINE:
// Definition of states in the state machine
const int CALIBRATE_X           = 1;
const int CALIBRATE_Y           = 2;
const int AXIS                  = 3;
const int MOVE                  = 4;
const int WAIT                  = 5;
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int  state                      = CALIBRATE_Y;
int old_state                   = CALIBRATE_Y;
bool axis_state                 = 0; // 0 is X axis, 1 is Y axis

// FOR POSITION
volatile int xPiezoPosition      = -15000; // [encoder counts] Current piezo position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int oldXPiezoPosition   = 200;
volatile int xEncoderStatus      = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)

volatile int yPiezoPosition      = -15000; // [encoder counts] Current piezo position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int oldYPiezoPosition   = 200;
volatile int yEncoderStatus      = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).

// FOR CALIBRATION
const int calibrateRange        = 2;
unsigned long startTime         = 0;
const unsigned long duration    = 2000;   // 2 seconds in milliseconds
bool calibrationDone            = false;
const float micronsToCount      = 0.513;
const float countsToMicrons     = 1.949;
const int range                 = 5;      // units are in encoder counts
const int useFinePosition       = 1;
const int useCoarsePosition     = 1;

// FOR SIGNAL GEN
const int DELAY_ON              = 50; 
const int DELAY_OFF             = 100; 
bool TOGGLE_STATE               = false;  // false is if signal generator is off, true is on
int cursor_location             = 6;      // Ranging from 1 to 6 - tells you where the cursor is on LCD
long disp_freq                  = 100000; // Default number that comes up when powering on device
long new_frequency              = 0;
bool direction                  = false;  // false is reverse, true is forward
bool defaultGains               = true;   // true is using default values, false is using new set

// FOR PID CONTROL
unsigned long executionDuration = 0;      // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;      // [microseconds] System clock value at the moment the loop was started the last time
int  targetPosition             = 0;      // [encoder counts] desired piezo position
float positionError             = 0;      // [encoder counts] Position error
float integralError             = 0;      // [encoder counts * seconds] Integrated position error
float velocityError             = 0;      // [encoder counts / seconds] Velocity error
float desiredFrequency          = 0;      // [Hz] Desired frequency for piezo actuator
float piezoVelocity             = 0;      // [encoder counts / seconds] Current piezo velocity 
int previousPiezoPosition       = 0;      // [encoder counts] Piezo position the last time a velocity was computed 
long previousVelCompTime        = 0;      // [microseconds] System clock value the last time a velocity was computed 
const int  MIN_VEL_COMP_COUNT   = 2;      // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME    = 10000;  // [microseconds] Minimal time that must pass between two velocity measurements
int numOfPresses                = 0;
const float KP                  = 2.5;    // [Volt / encoder counts] P-Gain
const float KD                  = 0.005;  // [Volt * seconds / encoder counts] D-Gain
const float KI                  = 0.005;  // [Volt / (encoder counts * seconds)] I-Gain

// Timing:
unsigned long startWaitTime;              // [microseconds] System clock value at the moment the WAIT state started
const long  WAIT_TIME           = 1000000;// [microseconds] Time waiting at each location
const int TARGET_BAND           = 0;      // [encoder counts] "Close enough" range when moving towards a target.

// USER INPUT:
const int NUM_POSITIONS = 10;
struct Position {
  bool axis;  // 0 = X, 1 = Y
  float positionMicrons;  // Target position in microns
  int positionCounts;     // Converted to encoder counts
};

Position targetPositions[NUM_POSITIONS] = {
  {1, 5.0, 0},
  {1, 10.0, 0},  
  {1, 15.0, 0},
  {1, 20.0, 0},
  {1, 25.0, 0},  
  {1, 20.0, 0},
  {1, 15.0, 0},
  {1, 10.0, 0}, 
  {1, 5.0, 0},
};

// Index to track current position in the sequence
int currentPositionIndex = 0;

void setup() {
  // FOR POSITION
  pinMode(X_PIN_NR_ENCODER_A,               INPUT);
  pinMode(X_PIN_NR_ENCODER_B,               INPUT); 
  pinMode(X_CS_PIN,                         OUTPUT);
  digitalWrite(X_CS_PIN,                    LOW);

  pinMode(Y_PIN_NR_ENCODER_A,               INPUT);
  pinMode(Y_PIN_NR_ENCODER_B,               INPUT); 
  pinMode(Y_CS_PIN,                         OUTPUT);
  digitalWrite(Y_CS_PIN,                    LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  attachInterrupt(digitalPinToInterrupt(X_PIN_NR_ENCODER_A), updateXPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(digitalPinToInterrupt(X_PIN_NR_ENCODER_B), updateXPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  attachInterrupt(digitalPinToInterrupt(Y_PIN_NR_ENCODER_A), updateYPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(digitalPinToInterrupt(Y_PIN_NR_ENCODER_B), updateYPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // FOR SIGNAL GEN
  pinMode(MODE,                           OUTPUT);
  pinMode(CURSOR,                         OUTPUT);
  pinMode(ADD,                            OUTPUT);
  pinMode(SUBTRACT,                       OUTPUT);
  pinMode(SIGNAL,                         OUTPUT);
  pinMode(X_STAGE,                        OUTPUT);
  pinMode(Y_STAGE,                        OUTPUT);
  Serial.begin(115200);
  
  // Convert all positions from microns to encoder counts
  for (int i = 0; i < NUM_POSITIONS; i++) {
    targetPositions[i].positionCounts = targetPositions[i].positionMicrons * micronsToCount;
  }

  setSignal();
  changeSpeed(10);
  reverse();
  digitalWrite(Y_STAGE, HIGH);
  Serial.println("Starting in 10 seconds");
  delay(10000);
}

void loop() {
  for (int i = 0; i < 20; i++){
    signalOutput();  
    signalOutput();
    delay(5000);
  }
  forward();
  for (int i = 0; i < 20; i++){
    signalOutput();  
    signalOutput();
    delay(5000);
  }
  
}

// void loop() {
//   switch (state) {
//     case CALIBRATE_X:
//       digitalWrite(X_STAGE, HIGH);
//       setSignal();
//       signalOutput(); // turn it on
//       setPosition(xPiezoPosition, oldXPiezoPosition);
//       digitalWrite(X_STAGE, LOW);

//       //Transition into WAIT state
//       //Record which state you came from
//       old_state = CALIBRATE_X;
//       state = WAIT;
//       startWaitTime = micros();
//       break;

//     case CALIBRATE_Y:
//       calibrationDone = false;
//       digitalWrite(Y_STAGE, HIGH);
//       // signalOutput(); // turn it on
//       // setPosition(yPiezoPosition, oldYPiezoPosition);
//       // digitalWrite(Y_STAGE, LOW);
//       setSignal();
//       signalOutput(); // turn it on
//       setPosition(yPiezoPosition, oldYPiezoPosition);
//       digitalWrite(Y_STAGE, LOW);

//       //Transition into WAIT state
//       //Record which state you came from
//       old_state = CALIBRATE_Y;
//       state = WAIT;
//       startWaitTime = micros();
//       break;

//     case AXIS:
//       // axis_state = userDirection();
//       axis_state = targetPositions[currentPositionIndex].axis;
//       old_state = AXIS;
//       state = WAIT;
//       startWaitTime = micros();
//       break;

//     case MOVE:
//       if (axis_state == 0){
//         enableX();
//       } else{
//         enableY();
//       }
//       targetPosition = targetPositions[currentPositionIndex].positionCounts;
//       // Serial.print("Your new target position is: ");
//       // Serial.println(targetPosition);
//       if ((axis_state == 0 ? xPiezoPosition : yPiezoPosition)>=targetPosition-TARGET_BAND && (axis_state == 0 ? xPiezoPosition : yPiezoPosition)<=targetPosition+TARGET_BAND) { // We reached the position
//         Serial.println("YOU REACHED THE POSITION");
//         if (TOGGLE_STATE){ // make sure the signal generator is off!
//           signalOutput();
//         }  
//         // Start waiting timer:
//         startWaitTime = micros();
//         old_state = MOVE; // Record which state you came from
//         state = WAIT; // Transition into WAIT state
//       } 
//       // Otherwise we continue moving towards the position
//       break;
    
//     case WAIT:
//       if (micros()-startWaitTime>WAIT_TIME){ // enter WAIT after a certain amount of time
//         if (old_state == CALIBRATE_X){
//           // Serial.println("Finished calibrating X-Stage");
//           state = CALIBRATE_Y;
//           // Serial.println("State transition from CALIBRATE_X to CALIBRATE_Y");
//         }
//         if (old_state == CALIBRATE_Y){
//           // Serial.println("Finished calibrating Y-Stage");
//           state = AXIS;
//           currentPositionIndex = 0;
//           // Serial.println("State transition from CALIBRATE_Y to AXIS");
//         }
//         if (old_state == AXIS){
//           // Serial.print("The axis you chose is: ");
//           // Serial.println(axis_state == 0 ? "X-Stage" : "Y-Stage");
//           // targetPosition = userInput();
//           state = MOVE;
//           integralError = 0;
//           // Serial.println("State transition from AXIS to MOVE");
//         }
//         if (old_state == MOVE){
//           currentPositionIndex++;
//           if (currentPositionIndex < NUM_POSITIONS) {
//             state = AXIS;  // Move to next position
//             // Serial.println("State transition from MOVE to AXIS");
//           } else {
//             Serial.println("All positions completed!");
//             while (1);  // Stop after all positions are done
//           }
//         }
//       }
//       break;

//     default: 
//       // Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
//       // Serial.print("Found the following unknown state: ");
//       // Serial.println(state);
//       while (1); // infinite loop to halt the program
//     break;

//   }
//   executionDuration = micros() - lastExecutionTime;
//   lastExecutionTime = micros();

//   // Compute the position error [encoder counts]
//   positionError = targetPosition - (axis_state == 0 ? xPiezoPosition : yPiezoPosition);
  
//   if ((abs(positionError) < useFinePosition) && (state == MOVE)){
//     numOfPresses = abs(positionError/2);
//     // Serial.print("numOfPresses is ");
//     // Serial.println(numOfPresses);
//     if (positionError >= 0){
//       if (TOGGLE_STATE){
//         signalOutput();
//       }
//       desiredFrequency = 5;
//       changeSpeed(int(desiredFrequency));
//       reverse();
//       for (int i = 0; i < numOfPresses; i++) {
//         signalOutput();
//         delay(50);
//         signalOutput();
//         delay(100);
//         // Serial.println("Entering positive fine control now");
//         // Serial.print("CURRENT POSITION: ");
//         // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
//         // Serial.print("Target position: ");
//         // Serial.println(targetPosition);
//       }
//     } 
//     else{ // you need to go the opposite direction
//       if (TOGGLE_STATE){
//         signalOutput();
//       }
//       desiredFrequency = 5;
//       changeSpeed(int(desiredFrequency));
//       forward();
//       for (int i = 0; i < numOfPresses; i++) {
//         signalOutput();
//         delay(50);
//         signalOutput();
//         delay(100);
//         // Serial.println("Entering negative fine control now");
//         // Serial.print("CURRENT POSITION: ");
//         // Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
//         // Serial.print("Target position: ");
//         // Serial.println(targetPosition);
//       }
//     }
//   }

//   else if ((abs(positionError) > useFinePosition) && (state == MOVE)){
//     if (positionError >= 0){
//       if (desiredFrequency != useCoarsePosition){
//         desiredFrequency = useCoarsePosition;
//         changeSpeed(int(desiredFrequency));
//         reverse();
//         signalOutput();
//       }
//     }
//     else { // you need to go the opposite direction
//       if (desiredFrequency != useCoarsePosition){
//         desiredFrequency = useCoarsePosition;
//         changeSpeed(int(desiredFrequency));
//         forward();
//         signalOutput();
//       }
//     }
//     // // Speed Computation:
//     // if ((abs((axis_state == 0 ? xPiezoPosition : yPiezoPosition) - previousPiezoPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME) {

//     //   // If at least a minimum time interval has elapsed or
//     //   // a minimum distance has been traveled, compute a new value for speed:
//     //   // (speed = delta encoder counts divided by delta time [seconds])
//     //   piezoVelocity = (double)((axis_state == 0 ? xPiezoPosition : yPiezoPosition) - previousPiezoPosition) * 1000000 / (micros() - previousVelCompTime);
//     //   // Remember this encoder count and time for the next iteration:
//     //   previousPiezoPosition = (axis_state == 0 ? xPiezoPosition : yPiezoPosition);
//     //   previousVelCompTime   = micros();
//     // }

//     // //** PID control: **//  
//     // // Compute the integral of the position error  [encoder counts * seconds]
//     // integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
//     // // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
//     // velocityError = 0 - piezoVelocity;
//     // // This is the actual controller function that uses the error in 
//     // // position and velocity and the integrated error and computes a
//     // // desired frequency that should be sent to the piezo:
//     // desiredFrequency = KP * positionError +  
//     //                   KI * integralError +
//     //                   KD * velocityError;

//     // // Sets range for acceptable frequency for piezo
//     // if (desiredFrequency >= 900){
//     //   desiredFrequency = 900;
//     // }
//     // else if (desiredFrequency <= -900){
//     //   desiredFrequency = -900;
//     // }
//     // else if ((desiredFrequency >= 0) && (desiredFrequency <= 10)){
//     //   desiredFrequency = 10;
//     // }
//     // else if ((desiredFrequency <= 0) && (desiredFrequency >= -10)){
//     //   desiredFrequency = -10;
//     // }

//     // // Changes the speeds here and involves direction (+ or -)
//     // if ((desiredFrequency >= 0) && (state==MOVE)){
//     //   if (TOGGLE_STATE){
//     //     signalOutput();
//     //   }
//     //   changeSpeed(int(desiredFrequency));
//     //   reverse();
//     //   signalOutput();
//     // }
//     // else if ((desiredFrequency < 0) && (state==MOVE)){
//     //   if (TOGGLE_STATE){
//     //     signalOutput();
//     //   }
//     //   changeSpeed(int(abs(desiredFrequency)));
//     //   forward();
//     //   signalOutput();
//     // }
//   }
//   // else{
//   //   desiredFrequency = 0;
//   //   Serial.println("Error reached - please check bang bang and PID control");
//   // }

//   if (state == MOVE){
//     Serial.print("CURRENT POSITION: ");
//     Serial.println(axis_state == 0 ? xPiezoPosition : yPiezoPosition);
//     // Serial.print("Target position: ");
//     // Serial.println(targetPosition);
//   }
// }

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
  // Serial.println("Finished CALIBRATION");
  // Serial.println("Starting in 1 sec");
  delay(1000);
}

void setPosition(volatile int &piezoPosition, volatile int &oldPiezoPosition) {
  while((piezoPosition != 0) && (!calibrationDone)){
    if ((piezoPosition >= oldPiezoPosition - calibrateRange) && 
        (piezoPosition <= oldPiezoPosition + calibrateRange)) {
      if (startTime == 0) {
        startTime = millis();
      } else if (millis() - startTime >= duration) {
        signalOutput();
        piezoPosition = 0;
        calibrationDone = true;
        // Serial.println("Finished calibrating position");
        // Serial.print("The starting position is: ");
        // Serial.println(piezoPosition);
        break;
      }
    } else {
      startTime = 0;
    }
    oldPiezoPosition = piezoPosition;
    // Serial.print("The calibrating encoder value is ");
    // Serial.println(piezoPosition);
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
}

bool userDirection() {
  bool value = 0; // 0 = X, 1 = Y
  Serial.println("Direction: X or Y?");  
  while (Serial.available() == 0) {
    // Wait for user input
  }
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if ((input == "Y") || (input == "y")){
      value = 1;
    }
  }
  return value;
}

float userInput() {
  float value = 0; 
  Serial.println("Type in the position (microns) you want to move to!");
  while (Serial.available() == 0) {
    // Wait for user input
  }
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    value = input.toFloat()*micronsToCount;
  }
  return value;
}

void enableX() {
  digitalWrite(Y_STAGE, LOW);
  digitalWrite(X_STAGE, HIGH);
}

void enableY() {
  digitalWrite(X_STAGE, LOW);
  digitalWrite(Y_STAGE, HIGH);
}

//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateXPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_PIN_NR_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_PIN_NR_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  xEncoderStatus &= 15;
  if (xEncoderStatus==2 || xEncoderStatus==4 || xEncoderStatus==11 || xEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    xPiezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    xPiezoPosition--;         // decrease the encoder count by one
  }
}

void updateYPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  yEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  yEncoderStatus |= digitalRead(Y_PIN_NR_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  yEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  yEncoderStatus |= digitalRead(Y_PIN_NR_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  yEncoderStatus &= 15;
  if (yEncoderStatus==2 || yEncoderStatus==4 || yEncoderStatus==11 || yEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    yPiezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    yPiezoPosition--;         // decrease the encoder count by one
  }
}
