#define CLK_PIN 13
#define DO_PIN 12
#define CS_PIN 2
#define BUFFER_SIZE 3  // Buffer size for storing position values

bool magINC, magDEC, lin;
uint8_t trigger = 0;
bool backup = false;
uint16_t positionBuffer[BUFFER_SIZE];  // Buffer to store position values
uint16_t newPositionBuffer[BUFFER_SIZE];  // Buffer to store new position values if the sensor moves
uint8_t bufferIndex = 0;  // Index for the position buffer
uint8_t newBufferIndex = 0;  // Index for the new position buffer
double averageDifference = 0.0;  // Average difference between consecutive position values
// uint8_t resolution = 2.0475;  // Room for error due to sensor sensitivity
uint8_t resolution = 10;
int holder = 0;
double currentDifference = 0.0;

void setup() {
  pinMode(CLK_PIN, OUTPUT);
  pinMode(DO_PIN, INPUT);
  pinMode(CS_PIN, OUTPUT);
  
  Serial.begin(9600);
  
  digitalWrite(CS_PIN, HIGH); // Deselect chip initially
}

uint16_t readAS5311() {
  uint16_t data = 0;
  int delay = 1;
  
  digitalWrite(CS_PIN, LOW); // Select chip
  delayMicroseconds(1); // Wait for at least 500ns
  
  // Read 12 bits of data
  for (int i = 0; i < 12; i++) {
    digitalWrite(CLK_PIN, HIGH);
    delayMicroseconds(delay);
    data = (data << 1) | digitalRead(DO_PIN); // Shift in the next bit
    digitalWrite(CLK_PIN, LOW);
    delayMicroseconds(delay);
  }
  
  digitalWrite(CS_PIN, HIGH); // Deselect chip

  return data;
}

void loop() {
  uint16_t position = readAS5311();
  Serial.print("Position:");
  Serial.println(position);
  // Serial.print("bufferIndex:");
  // Serial.println(bufferIndex);

  double mean = 0.0;
  double sum = 0.0;
  for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
    sum += positionBuffer[i];
  }
  mean = sum/BUFFER_SIZE;
  
  Serial.print("Content of old position buffer:");
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.print(positionBuffer[i]);
    Serial.print(", ");
  }
  Serial.println(" .");

  double new_mean = 0.0;
  double new_sum = 0.0;
  if ((position > mean + resolution) || (position < mean - resolution)) {
    newPositionBuffer[newBufferIndex] = position;
    newBufferIndex ++;
    trigger ++;
    if (trigger == 2) {
      trigger = 0;
      backup = true;
    }
  } else {
      // Store the position value in the buffer
      // Slide everything to the left in the array by 1 and push the new value at the end
      positionBuffer[0] = positionBuffer[1];
      positionBuffer[1] = positionBuffer[2];
      positionBuffer[2] = position;
      for (uint8_t i = 0; i < BUFFER_SIZE; i++) {
        new_sum += positionBuffer[i];
      }
      new_mean = new_sum/BUFFER_SIZE;
  }

  if (backup) { // noise also appears once in a while, so if the new positional value doesn't consistently change, restart the new position buffer list
    backup = false;
    newBufferIndex --;
  }

  if (newBufferIndex == BUFFER_SIZE) { // if the new position buffer array is filled up, check it against the new positional value
    newBufferIndex = 0;
    if ((position > mean + resolution) || (position < mean - resolution)) {
        for (int i = 0; i < BUFFER_SIZE; i++) {
          positionBuffer[i] = newPositionBuffer[i]; // replace the old with the new if conditions meet
        }
    }
  }

  Serial.print("Content of new position buffer:");
  for (int i = 0; i < BUFFER_SIZE; i++){
    Serial.print(newPositionBuffer[i]);
    Serial.print(", ");
  }
  Serial.println(" .");
  
  if (new_mean == 0) {
    Serial.println("Position is currently being determined");
  } else{
      Serial.print("Current Position:");
      Serial.println(new_mean);
  }

  // holder = positionBuffer[bufferIndex];

  // bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;  // Wrap around the buffer index

  // // Calculate the average difference between consecutive position values
  // double sum = 0.0;
  // for (uint8_t i = 0; i < BUFFER_SIZE - 1; i++) {
  //   sum += abs(static_cast<double>(positionBuffer[i]) - static_cast<double>(positionBuffer[i + 1]));
  // }
  // averageDifference = sum / (BUFFER_SIZE - 1);
  // Serial.print("old average Difference: ");
  // Serial.println(averageDifference);

  // Serial.print("Content of new position buffer:");
  // for (int i = 0; i < BUFFER_SIZE; i++){
  //   Serial.print(positionBuffer[i]);
  //   Serial.print(", ");
  // }
  // Serial.println(" .");

  // // Check if the difference between the current and previous position is within 2.0475 of the average
  // if (bufferIndex == BUFFER_SIZE-1) { // if bufferIndex = 2, but position is at bufferIndex = 1 so check at bufferIndex = 0
  //   currentDifference = abs(static_cast<double>(position) - static_cast<double>(positionBuffer[bufferIndex - 2]));
  // }
  // else if (bufferIndex == BUFFER_SIZE-2) { // if bufferIndex = 1, but position is at bufferIndex = 0 so check at bufferIndex = 2
  //   currentDifference = abs(static_cast<double>(position) - static_cast<double>(positionBuffer[bufferIndex + 1]));    
  // }
  // else if (bufferIndex == BUFFER_SIZE-3) { // if bufferIndex = 0, but position is at bufferIndex = 2 so check at bufferIndex = 1
  //   currentDifference = abs(static_cast<double>(position) - static_cast<double>(positionBuffer[bufferIndex + 1]));     
  // }

  // if (currentDifference > averageDifference + resolution || currentDifference < averageDifference - resolution) {
  //   // Serial.println("Difference exceeds the threshold!");
  //   newPositionBuffer[newBufferIndex] = position;
  //   newBufferIndex ++;
  // }
  // else{
  //   // Serial.println("Not exceeded");
  //   positionBuffer[bufferIndex-1] = holder;
  // }

  //   // Calculate the average difference between consecutive position values
  // double new_sum = 0.0;
  // for (uint8_t i = 0; i < BUFFER_SIZE - 1; i++) {
  //   new_sum += abs(static_cast<double>(positionBuffer[i]) - static_cast<double>(positionBuffer[i + 1]));
  // }
  // averageDifference = new_sum / (BUFFER_SIZE - 1);

  // Serial.print("average Difference: ");
  // Serial.println(averageDifference);

  // Serial.print("Content of old position buffer:");
  // for (int i = 0; i < BUFFER_SIZE; i++){
  //   Serial.print(positionBuffer[i]);
  //   Serial.print(", ");
  // }
  // Serial.println(" .");

  // Serial.print("Content of new position buffer:");
  // for (int i = 0; i < BUFFER_SIZE; i++){
  //   Serial.print(newPositionBuffer[i]);
  //   Serial.print(", ");
  // }
  // Serial.println(" .");

  // if (newBufferIndex == BUFFER_SIZE){
  //   newBufferIndex = 0;
  // }

  delay(1000); // Delay for readability
}
