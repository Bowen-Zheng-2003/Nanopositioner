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
const float countsToMicrons    = 1.949;
const int range               = 5; // units are in encoder counts
bool messageDisplayed = false; // Global variable to track if message has been displayed

// FOR SIGNAL GEN
const int DELAY_ON            = 50; 
const int DELAY_OFF           = 100; 
bool TOGGLE_STATE             = false; // false is if signal generator is off, true is on

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
  setSignal();
  Serial.println("Type '0' to change output");
  signalOutput(); // turn it on
}

void loop() {
  // if (Serial.available() > 0) {
  //   switch (Serial.read()) {
  //     case '0':
  //       signalOutput();
  //       break;
  //   }
  // }
  if ((piezoPosition != 0) && (!calibrationDone)){
    setPosition();
  } else{
    userInput();
  }
  
}

void mode() {
  digitalWrite(MODE, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(MODE, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("Changing MODE");
}

void cursor() {
  digitalWrite(CURSOR, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(CURSOR, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("Changing CURSOR");
}

void increment() {
  digitalWrite(ADD, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(ADD, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("INCREASING");
}

void decrement() {
  digitalWrite(SUBTRACT, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SUBTRACT, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("DECREASING");
}

void signalOutput() {
  digitalWrite(SIGNAL, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SIGNAL, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  TOGGLE_STATE = !TOGGLE_STATE;
  Serial.print("Turning signal ");
  Serial.println(TOGGLE_STATE ? "ON" : "OFF");
}

void forward() {
  for (int i = 0; i < 6; i++) {
    mode();
  }
}

void reverse() {
  mode();
}

void setSignal() {
  for (int i = 0; i < 3; i++) {
    mode();
  }
  decrement();
  for (int i = 0; i < 2; i++) {
    cursor();
  }
  increment();
  Serial.println("Finished CALIBRATION");
  Serial.println("Starting in 5 sec");
  delay(5000);
}

void setPosition() {  
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
      return;
    }
  } else {
    startTime = 0;
  }
  
  oldPiezoPosition = piezoPosition;
  Serial.print("The calibrating encoder value is ");
  Serial.println(piezoPosition);
}

void userInput() {
  if (!messageDisplayed) {
    Serial.println("Type in the position (microns) you want to move to!");
    messageDisplayed = true;
  }  
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float value = input.toFloat()*micronsToCount;
    while (piezoPosition < value) {
      if (!TOGGLE_STATE) {
        reverse();
        signalOutput();
      }
    }
    // while (piezoPosition > input + range) {
    //   reverse():
    //     signalOutput();
    // }
    // while (!((piezoPosition < input - range) && (piezoPosition > input + range))) {
    //   if (piezoPosition < input - range) {
    //     reverse();
    //     signalOutput();
    //   } else if (piezoPosition > input + range) {
    //     forward();
    //     signalOutput();
    //   } else {
    //     if (TOGGLE_STATE){
    //       signalOutput();
    //     }
    //     Serial.println("HELP!! userInput() function has gone wrong!");
    //   }
    // }
    if (TOGGLE_STATE) {
      signalOutput();
    }
    updatePiezoPosition();
    Serial.print("You're at ");
    Serial.print(piezoPosition);
    Serial.print(" counts,");
    Serial.print(" which is the same as: ");
    Serial.print(piezoPosition*countsToMicrons);
    Serial.println(" microns!");
    delay(10000);
  }
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
