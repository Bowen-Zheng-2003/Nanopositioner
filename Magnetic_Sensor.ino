// THIS CODE IS FOR ARDUINO UNO
#include <Wire.h>
#define CLK_PIN 13
#define DO_PIN 12
#define CS_PIN 2
#define BUFFER_SIZE 5  // Buffer size for storing position values
#define CHECK_SIZE 5 // Size for array containing new position values that changed

bool magINC, magDEC, lin;
bool turnOn = 1;
uint16_t positionBuffer[BUFFER_SIZE];  // Buffer to store position values
uint16_t positionCheck[BUFFER_SIZE]; // Separate buffer to store position values that are scraps
int j = 0;
int ignore = 0; // only start doing calculations once the list is full
float mean = 0;
uint8_t bufferIndex = 0;  // Index for the position buffer
double averageDifference = 0.0;  // Average difference between consecutive position values
// uint8_t resolution = 2.0475;  // Room for error due to sensor sensitivity
uint8_t resolution = 5;
int currentPosition;

// Tracking actual measurement (also utilizing user input)
double currentLength = -1;
int sampleRate = 15; // When position falls + or - outside of this, then I'll take a new measurement
double difference = 0;
double newDiff1 = 0;
double newDiff2 = 0;
double conversion = 0.48828; // Converts from bits to microns (so 2/4096*1000)
int previousPosition;
int diffRange = 1000; // Changes calculation of measurement since it signals that it's cycling (ie 4096 to 0 or vice versa)

void setup() {
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DO_PIN, INPUT);
  pinMode(CS_PIN, OUTPUT);

  Wire.begin(); // Start the I2C Bus as Master
  Serial.begin(9600);
  
  digitalWrite(CS_PIN, HIGH); // Deselect chip initially
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

void loop() {
  if (currentLength == -1){ // check to make sure that it is homed
    Serial.println("Please home your positioner. Then, type '0' to start");

    while (Serial.available() == 0) {
      // Wait for user input
    }
    currentLength = Serial.parseInt();
    
    // Optional: Add a delay to prevent rapid looping
    delay(1000);
  }
  else{ // you can now start the process for reading the sensor data
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

    Serial.println(" ");
    // Serial.println(currentPosition);
    // Serial.print("Previous position: ");
    // Serial.println(previousPosition);
    
    if (((previousPosition > currentPosition+sampleRate) || (previousPosition < currentPosition-sampleRate)) && (previousPosition > 0)) {
      difference = currentPosition-previousPosition;

      if (abs(difference) > diffRange){ // this is to counteract the cycling from 4096 to 0
        newDiff1 = 4096-currentPosition;
        newDiff2 = 4096-previousPosition;
        if (newDiff1 > newDiff2){ // this means that previousPosition is closer to 4096
          difference = newDiff2 + currentPosition;
        }
        else {
          difference = newDiff1 + previousPosition;
        }
      }

      difference = difference*conversion;

      Serial.print("Current position: ");
      Serial.println(currentPosition);
      Serial.print("Previous position: ");
      Serial.println(previousPosition);

      currentLength = currentLength+difference;
      Serial.print("The current length is: ");
      Serial.print(currentLength);
      Serial.println(" microns");
      previousPosition = currentPosition;
    }

    if (ignore == 10){ // need to first initialize previousPosition when currentPosition is determined
      previousPosition = currentPosition;
    }

    // delay(100); // Adjust delay as necessary

    ignore++;

    // Convert the bits to actual measurements

  }
  
}

float calculateMean(uint16_t* array, int size) {
  uint32_t sum = 0;
  for (int i = 0; i < size; i++) {
    sum += array[i];
  }
  return (float)sum / size;
}
