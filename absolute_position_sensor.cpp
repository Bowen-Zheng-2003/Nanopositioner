#include <Arduino.h>
#include "timers.h"
#include "SPI.h"

#define DEADBAND 1.0F

// POSITIONING
#define X_ENCODER_A A3
#define X_ENCODER_B A4
// #define X_CS_PIN A5
volatile int xEncoderStatus     = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)

// #define Y_ENCODER_A SCK
// #define Y_ENCODER_B MOSI
// #define Y_CS_PIN MISO
volatile int yEncoderStatus     = 0;      // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)

// STATE MACHINE
const int CALIBRATE             = 1;
const int AXIS                  = 2;
const int MOVE                  = 3;
const int WAIT                  = 4;
int state                       = CALIBRATE;
int old_state                   = CALIBRATE;
bool axis_state                 = 0; // 0 is X axis, 1 is Y axis

// CALIBRATION
struct AxisStatus {
  volatile int piezoPosition;
  volatile int oldPiezoPosition;
  bool calibrationDone;
  unsigned long startTime;
};

#define NUM_AXES 2 // X, Y, Z
AxisStatus axes[NUM_AXES] = {
  {-15000, 200, false, 0}, // X
  {-15000, 200, false, 0}, // Y
  // {-15000, 200, false, 0}  // Z, add more as needed
};

const int calibrateRange        = 2;
unsigned long startTime         = 0;
const unsigned long duration    = 2000;   // 2 seconds in milliseconds
const int startingFrequencyX    = 700;
const int startingFrequencyY    = -700;
const float micronsToCount      = 0.512;
const float countsToMicrons     = 1.953125; // 2000 microns/2^10 counts

// FOR PID CONTROL
unsigned long executionDuration = 0;      // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;      // [microseconds] System clock value at the moment the loop was started the last time
int targetXPosition = 0, targetYPosition = 0;
float positionErrorX = 0, positionErrorY = 0;
float integralErrorX = 0, integralErrorY = 0;
float velocityErrorX = 0, velocityErrorY = 0;
float desiredFrequencyX = 0, desiredFrequencyY = 0;
float piezoVelocityX = 0, piezoVelocityY = 0;
int previousPiezoPositionX = 0, previousPiezoPositionY = 0;
long previousVelCompTimeX = 0, previousVelCompTimeY = 0;
const int  MIN_VEL_COMP_COUNT   = 2;      // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME    = 10000;  // [microseconds] Minimal time that must pass between two velocity measurements
float KP                        = 500;    // [Volt / encoder counts] P-Gain
float KD                        = 0.005;  // [Volt * seconds / encoder counts] D-Gain
float KI                        = 0.005;  // [Volt / (encoder counts * seconds)] I-Gain

// Timing:
unsigned long startWaitTime;              // [microseconds] System clock value at the moment the WAIT state started
const long  WAIT_TIME           = 1000000;// [microseconds] Time waiting at each location
const int TARGET_BAND           = 0;      // [encoder counts] "Close enough" range when moving towards a target.

// USER INPUT:
struct Coordinate {
  float xMicrons;
  float yMicrons;
  int xCounts;
  int yCounts;
};

#define NUM_COORDINATES 2
Coordinate coordinates[NUM_COORDINATES] = {
  {10000, 10000, 0, 0},    // First target: X=10000, Y=10000
  {5000, 5000, 0, 0}       // Second target: X=5000,  Y=5000 (add as many as needed)
};
// Track which coordinate we're at:
int currentCoordinateIndex = 0;

// Index to track current position in the sequence
int currentPositionIndex = 0;

void updateXPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_ENCODER_A);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  xEncoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  xEncoderStatus |= digitalRead(X_ENCODER_B);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  xEncoderStatus &= 15;
  if (xEncoderStatus==2 || xEncoderStatus==4 || xEncoderStatus==11 || xEncoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    axes[0].piezoPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    axes[0].piezoPosition--;         // decrease the encoder count by one
  }
}

// void updateYPosition() {
//   // Bitwise shift left by one bit, to make room for a bit of new data:
//   yEncoderStatus <<= 1;   
//   // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
//   // and put that value into the rightmost bit of encoderStatus:
//   yEncoderStatus |= digitalRead(Y_ENCODER_A);   
//   // Bitwise shift left by one bit, to make room for a bit of new data:
//   yEncoderStatus <<= 1;
//   // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
//   // and put that value into the rightmost bit of encoderStatus:
//   yEncoderStatus |= digitalRead(Y_ENCODER_B);
//   // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
//   // bitwise AND operator on mstatus and 15(=1111):
//   yEncoderStatus &= 15;
//   if (yEncoderStatus==2 || yEncoderStatus==4 || yEncoderStatus==11 || yEncoderStatus==13) {
//     // the encoder status matches a bit pattern that requires counting up by one
//     axes[1].piezoPosition++;         // increase the encoder count by one
//   } else {
//     // the encoder status does not match a bit pattern that requires counting up by one.  
//     // Since this function is only called if something has changed, we have to count downwards
//     axes[1].piezoPosition--;         // decrease the encoder count by one
//   }
// }

void write_freqs(float hz_0, float hz_1){
  // TODO: deadband... around zero... 
  if(hz_0 < DEADBAND && hz_0 > -DEADBAND){
    hz_0 = hz_0 < 0.0F ? -DEADBAND : DEADBAND;
  }
  if(hz_1 < DEADBAND && hz_1 > -DEADBAND){
    hz_1 = hz_1 < 0.0F ? -DEADBAND : DEADBAND;
  }
  // 
  float per_0 = 120000000.0F / (hz_0 * 4096.0F);
  float per_1 = 120000000.0F / (hz_1 * 4096.0F);
  // 
  write_periods((int16_t)per_0, (int16_t)per_1);
}

void calibrateAxis(int axisIndex) {
  AxisStatus &axis = axes[axisIndex];
  if (axis.calibrationDone) return;

  if ((axis.piezoPosition >= axis.oldPiezoPosition - calibrateRange) &&
      (axis.piezoPosition <= axis.oldPiezoPosition + calibrateRange)) {
    if (axis.startTime == 0) {
      axis.startTime = millis();
    } else if (millis() - axis.startTime >= duration) {
      if (axisIndex == 0){
        write_freqs(0, startingFrequencyY);
        axes[0].piezoPosition = 0;
      }
      else if (axisIndex == 1){
        write_freqs(0, 0);
        axes[1].piezoPosition = 0;
      }
      axis.calibrationDone = true;
      Serial.print("Finished calibrating axis ");
      Serial.println(axisIndex);
    }
  } else {
    axis.startTime = 0;
  }
  
  axis.oldPiezoPosition = axis.piezoPosition;
  delay(500);
}

void positionReached(volatile int &piezoPosition, int target) {
  if (piezoPosition>=target-TARGET_BAND && piezoPosition<=target+TARGET_BAND) { // We reached the position
    Serial.println("YOU REACHED THE POSITION");
    write_freqs(0, 0);
    // Start waiting timer:
    startWaitTime = micros();
    old_state = MOVE; // Record which state you came from
    state = WAIT; // Transition into WAIT state
  } 
}

uint16_t SPI_transact(void) {
  digitalWrite(A5, LOW);
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  uint8_t data0 = SPI.transfer(0); // does 8 pulses
  uint8_t data1 = SPI.transfer(0);
  uint8_t data2 = SPI.transfer(0); // do this 3 times to get 24 pulses
  // each SPI.transfer returns a byte
  SPI.endTransaction();
  digitalWrite(A5, HIGH);

  // Bit-wise Right Shift
  uint16_t output = (uint16_t)((uint16_t)data0 << 5) | (uint16_t)((uint16_t)data1 >> 4);
  return output;
}

void setup(){
  // start 
  // setup_timers();
  // le blink 
  pinMode(LED_BUILTIN, OUTPUT);

  // pinMode(X_ENCODER_A,   INPUT);
  // pinMode(X_ENCODER_B,   INPUT); 
  // pinMode(X_CS_PIN,      OUTPUT);
  // digitalWrite(X_CS_PIN, LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  // attachInterrupt(digitalPinToInterrupt(X_ENCODER_A), updateXPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  // attachInterrupt(digitalPinToInterrupt(X_ENCODER_B), updateXPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // pinMode(Y_ENCODER_A,   INPUT);
  // pinMode(Y_ENCODER_B,   INPUT); 
  // pinMode(Y_CS_PIN,      OUTPUT);
  // digitalWrite(Y_CS_PIN, LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  // attachInterrupt(digitalPinToInterrupt(Y_ENCODER_A), updateYPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  // attachInterrupt(digitalPinToInterrupt(Y_ENCODER_B), updateYPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3
  
  pinMode(A5, OUTPUT); // Chip Select
  pinMode(SCK, OUTPUT); // CLK
  pinMode(MISO, INPUT); // MISO
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);

  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for Serial to connect (important for native USB boards)
  }

  // Convert all positions from microns to encoder counts
  // for (int i = 0; i < NUM_COORDINATES; i++) {
  //   coordinates[i].xCounts = coordinates[i].xMicrons * micronsToCount;
  //   coordinates[i].yCounts = coordinates[i].yMicrons * micronsToCount;
  // }
}

uint32_t last_blink = 0;
float phase = 0.0F;
// speed scaling, then dir flip ? 

int frequency_test = -1000;
void loop() {
  uint16_t position = SPI_transact();
  Serial.print("Position of y is: ");
  Serial.println(position);

  // delay(100);

  // digitalWrite(X_CS_PIN, HIGH);
  // digitalWrite(Y_CS_PIN, HIGH);

  // switch (state){
  //   case CALIBRATE: {
  //     write_freqs(startingFrequencyX, startingFrequencyY); // Or update to support more axes if needed

  //     // delay(5000);
  //     bool allDone = true;
  //     for (int i = 0; i < NUM_AXES; i++) {
  //       calibrateAxis(i);
  //       allDone = allDone && axes[i].calibrationDone;
  //     }
  //     if (allDone) {
  //       Serial.println("All axes calibrated!");
  //       Serial.print("X position is ");
  //       Serial.println(axes[0].piezoPosition);
  //       Serial.print("Y position is ");
  //       Serial.println(axes[1].piezoPosition);
  //       state = WAIT;
  //       startWaitTime = micros();
  //     }
  //     // delay(500); // Small delay for serial and not to be too aggressive
  //     break;
  //   }
    
  //   case MOVE: {
  //     // Set targets
  //     targetXPosition = coordinates[currentCoordinateIndex].xCounts;
  //     targetYPosition = coordinates[currentCoordinateIndex].yCounts;

  //     // PID for X:
  //     executionDuration = micros() - lastExecutionTime;
  //     lastExecutionTime = micros();

  //     // --------- X Axis ---------
  //     if ((abs(axes[0].piezoPosition - previousPiezoPositionX) > MIN_VEL_COMP_COUNT) ||
  //         (micros() - previousVelCompTimeX) > MIN_VEL_COMP_TIME) {
  //       piezoVelocityX = (double)(axes[0].piezoPosition - previousPiezoPositionX) * 1000000 / (micros() - previousVelCompTimeX);
  //       previousPiezoPositionX = axes[0].piezoPosition;
  //       previousVelCompTimeX   = micros();
  //     }
  //     positionErrorX = targetXPosition - axes[0].piezoPosition;
  //     integralErrorX += positionErrorX * (float)(executionDuration) / 1000000;
  //     velocityErrorX = 0 - piezoVelocityX;
  //     desiredFrequencyX = KP * positionErrorX + KI * integralErrorX + KD * velocityErrorX;

  //     // Clamp X as in axis code:
  //     if      (desiredFrequencyX >= 800)  desiredFrequencyX = 800;
  //     else if (desiredFrequencyX <= -800) desiredFrequencyX = -800;
  //     else if ((desiredFrequencyX >= 0) && (desiredFrequencyX <= 10)) desiredFrequencyX = 10;
  //     else if ((desiredFrequencyX <= 0) && (desiredFrequencyX >= -10)) desiredFrequencyX = -10;

  //     // --------- Y Axis ---------
  //     if ((abs(axes[1].piezoPosition - previousPiezoPositionY) > MIN_VEL_COMP_COUNT) ||
  //         (micros() - previousVelCompTimeY) > MIN_VEL_COMP_TIME) {
  //       piezoVelocityY = (double)(axes[1].piezoPosition - previousPiezoPositionY) * 1000000 / (micros() - previousVelCompTimeY);
  //       previousPiezoPositionY = axes[1].piezoPosition;
  //       previousVelCompTimeY   = micros();
  //     }
  //     positionErrorY = targetYPosition - axes[1].piezoPosition;
  //     integralErrorY += positionErrorY * (float)(executionDuration) / 1000000;
  //     velocityErrorY = 0 - piezoVelocityY;
  //     desiredFrequencyY = KP * positionErrorY + KI * integralErrorY + KD * velocityErrorY;

  //     if      (desiredFrequencyY >= 800)  desiredFrequencyY = 800;
  //     else if (desiredFrequencyY <= -800) desiredFrequencyY = -800;
  //     else if ((desiredFrequencyY >= 0) && (desiredFrequencyY <= 10)) desiredFrequencyY = 10;
  //     else if ((desiredFrequencyY <= 0) && (desiredFrequencyY >= -10)) desiredFrequencyY = -10;

  //     // Write to both channels SIMULTANEOUSLY
  //     write_freqs(-desiredFrequencyX, desiredFrequencyY);

  //     // Target position check (both axes within TARGET_BAND)
  //     bool xReached = abs(axes[0].piezoPosition - targetXPosition) <= TARGET_BAND;
  //     bool yReached = abs(axes[1].piezoPosition - targetYPosition) <= TARGET_BAND;
  //     if(xReached && yReached) {
  //       Serial.println("YOU REACHED THE COORDINATE");
  //       write_freqs(0, 0);
  //       startWaitTime = micros();
  //       old_state = MOVE;
  //       state = WAIT;
  //     }
  //     Serial.print("CURRENT X: "); Serial.println(axes[0].piezoPosition);
  //     Serial.print("TARGET X: "); Serial.println(targetXPosition);
  //     Serial.print("CURRENT Y: "); Serial.println(axes[1].piezoPosition);
  //     Serial.print("TARGET Y: "); Serial.println(targetYPosition);
  //     break;
  //   }
    
  //   case WAIT:
  //     if (micros()-startWaitTime>WAIT_TIME){ // enter WAIT after a certain amount of time
  //       if (old_state == CALIBRATE){
  //         Serial.println("Finished calibrating X-Stage");
  //         state = MOVE;
  //         currentPositionIndex = 0;
  //         Serial.println("State transition from CALIBRATE to AXIS");
  //       }
  //       if (old_state == MOVE) {
  //         integralErrorX = 0;
  //         integralErrorY = 0;
  //         currentCoordinateIndex++;
  //         if (currentCoordinateIndex < NUM_COORDINATES) {
  //           state = MOVE;  // Jump to next coordinate
  //           Serial.println("State transition to next coordinate");
  //         } else {
  //           Serial.println("All coordinates completed!");
  //           while (1); // Stop program
  //         }
  //       }
  //     }
  //     break;

  //   default: 
  //     Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
  //     Serial.print("Found the following unknown state: ");
  //     Serial.println(state);
  //     while (1); // infinite loop to halt the program
  //   break;

  // }
}


