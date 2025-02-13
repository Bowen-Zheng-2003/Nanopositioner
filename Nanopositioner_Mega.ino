//** PIN ASSIGNMENT: **//
// FOR POSITION
// const int PIN_NR_ENCODER_A      = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
// const int PIN_NR_ENCODER_B      = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
// const int CS_PIN                = 4;  // Enables encoder A and B to start when set to LOW 
// FOR SIGNAL GENERATOR
const int MODE_X                  = 53; 
const int CURSOR_X                = 52; 
const int ADD_X                   = 51; 
const int SUBTRACT_X              = 50;
const int SIGNAL_X                = 49; 


// FOR SIGNAL GEN
const int DELAY_ON              = 50; 
const int DELAY_OFF             = 100; 

void setup() {
  // FOR POSITION
  // pinMode(PIN_NR_ENCODER_A,               INPUT);
  // pinMode(PIN_NR_ENCODER_B,               INPUT); 
  // pinMode(CS_PIN,                         OUTPUT);
  // digitalWrite(CS_PIN,                    LOW);
  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updatePiezoPosition' is called:
  // attachInterrupt(0, updatePiezoPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  // attachInterrupt(1, updatePiezoPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // FOR SIGNAL GEN
  pinMode(MODE_X,                           OUTPUT);
  pinMode(CURSOR_X,                         OUTPUT);
  pinMode(ADD_X,                            OUTPUT);
  pinMode(SUBTRACT_X,                       OUTPUT);
  pinMode(SIGNAL_X,                         OUTPUT);

  Serial.begin(115200);

  decrement(SUBTRACT_X);
}

void loop() {

}

void mode(int modePin) {
  digitalWrite(modePin, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(modePin, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("Changing MODE");
}

void cursor(int cursorPin) {
  digitalWrite(cursorPin, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(cursorPin, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("Changing CURSOR");
  // cursor_location = cursor_location-1;
  // if (cursor_location == 0){ // help loop it back
  //   cursor_location = 6;
  // }
}

void increment(int addPin) {
  digitalWrite(addPin, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(addPin, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("INCREASING");
}

void decrement(int subtractPin) {
  digitalWrite(subtractPin, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(subtractPin, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  Serial.println("DECREASING");
}

void signalOutput(int signalPin) {
  digitalWrite(signalPin, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(signalPin, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
  // TOGGLE_STATE = !TOGGLE_STATE;
  // if (!TOGGLE_STATE){
  //   cursor_location = 6;
  // }
  Serial.print("Turning signal ");
  // Serial.println(TOGGLE_STATE ? "ON" : "OFF");
}