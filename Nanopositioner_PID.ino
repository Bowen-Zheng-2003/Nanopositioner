// FOR SIGNAL GENERATOR CONTROL
const int SIGNAL    = 7; 
const int CURSOR    = 8; 
const int MODE      = 9; 
const int ADD       = 10; 
const int SUBTRACT  = 11;
const int DELAY_ON  = 50; 
const int DELAY_OFF = 100; 
bool TOGGLE_STATE   = false; // false is if signal generator is off, true is on

// FOR POSITION TRACKING
#define CLK_PIN 13
#define DO_PIN 12
#define CS_PIN 2
#define BUFFER_SIZE 10  // Buffer size for storing position values
#define CHECK_SIZE 10 // Size for array containing new position values that changed
bool magINC, magDEC, lin;
bool turnOn              = 1;
uint16_t positionBuffer[BUFFER_SIZE];  // Buffer to store position values
uint16_t positionCheck[BUFFER_SIZE]; // Separate buffer to store position values that are scraps
int j                    = 0;
int ignore               = 0; // only start doing calculations once the list is full
float mean               = 0;
uint8_t bufferIndex      = 0;  // Index for the position buffer
double averageDifference = 0.0;  // Average difference between consecutive position values
// uint8_t resolution       = 2.0475;  // Room for error due to sensor sensitivity
uint8_t resolution       = 5;
uint16_t currentPosition;
int calibratePosition    = 100;
int oldCalibratePosition = 200;
const int calibrateRange = 5;
double finalPosition    = -100;
unsigned long startTime = 0;
const unsigned long duration = 2000; // 2 seconds in milliseconds

void setup() {
  pinMode(MODE, OUTPUT);
  pinMode(CURSOR, OUTPUT);
  pinMode(ADD, OUTPUT);
  pinMode(SUBTRACT, OUTPUT);
  pinMode(SIGNAL, OUTPUT);

  pinMode(CLK_PIN, OUTPUT);
  pinMode(DO_PIN, INPUT);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Deselect chip initially  

  Serial.begin(115200);
  setSignal();
  Serial.println("Type '0' to change output");
  signalOutput(); // turn it on
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    switch (input) {
      case '0':
        signalOutput();
        break;
    }
  }
  if (finalPosition == -100){
    int timeStart = millis(); 
    setPosition(timeStart);
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
  for (int i = 0; i < 4; i++) {
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

void setPosition(unsigned long timeStart) {
  calibratePosition = calculatePosition();
  
  if ((calibratePosition >= oldCalibratePosition - calibrateRange) && 
      (calibratePosition <= oldCalibratePosition + calibrateRange)) {
    if (startTime == 0) {
      startTime = millis();
    } else if (millis() - startTime >= duration) {
      signalOutput();
      finalPosition = 0;
      Serial.println("Finished calibrating position");
      Serial.print("The starting position is: ");
      Serial.println(finalPosition);
      return;
    }
  } else {
    startTime = 0;
  }
  
  oldCalibratePosition = calibratePosition;
  Serial.print("The calibrating encoder value is ");
  Serial.println(calibratePosition);
}

uint16_t readAS5311() {
  uint16_t data = 0;
  int delayTime = 1;
  
  digitalWrite(CS_PIN, LOW); // Select chip
  delayMicroseconds(1); // Wait for at least 500ns
  
  // Read 12 bits of data
  for (int i = 0; i < 12; i++) {
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delayTime);
    data = (data << 1) | digitalRead(DO_PIN); // Shift in the next bit
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delayTime);
  }
  
  digitalWrite(CS_PIN, HIGH); // Deselect chip
  
  return data;
}

float calculateMean(uint16_t* array, int size) {
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += array[i];
  }
  return (float)sum / size;
}

float calculatePosition() {
  uint16_t position = readAS5311();
  if (turnOn == 1){
    currentPosition = positionBuffer[0];
    turnOn = 0;
  }
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if(ignore >= 10){
    mean = calculateMean(positionBuffer, BUFFER_SIZE);
    if(position > mean + resolution || position < mean - resolution){ // if it falls outside the acceptable range
      positionCheck[j] = position;
      j++;
      if(j==CHECK_SIZE){ // if nanopositioner position changes, update the new position
        float meanCheck = calculateMean(positionCheck, CHECK_SIZE);
        if(positionCheck[0] < meanCheck + resolution || positionCheck[0] > meanCheck - resolution){
          j = 0;
          // Copy contents of positionCheck to positionBuffer
          for(int k = 0; k < BUFFER_SIZE; k++) {
              positionBuffer[k] = positionCheck[k];
          }
        }
      }
    }
    else{
      // Store the position in the buffer
      positionBuffer[bufferIndex] = position;
      j = 0;
      for(int l = 0; l < BUFFER_SIZE; l++) {
        positionCheck[l] = 0;
      }
    }
  }
  else{
    // Store the position in the buffer
    positionBuffer[bufferIndex] = position;
  }
  currentPosition = mean;
  // Serial.print("Current position: ");
  // Serial.println(currentPosition);

  delay(5); // Adjust delay as necessary
  ignore++;
  return currentPosition;
}