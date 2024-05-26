#define CLK_PIN 13
#define DO_PIN 12
#define CS_PIN 2
#define BUFFER_SIZE 10  // Buffer size for storing position values
bool magINC, magDEC, lin;
uint16_t positionBuffer[BUFFER_SIZE];  // Buffer to store position values
uint16_t newPositionBuffer[BUFFER_SIZE];  // Buffer to store new position values if the sensor moves
uint8_t bufferIndex = 0;  // Index for the position buffer
uint8_t newBufferIndex = 0;  // Index for the new position buffer
double averageDifference = 0.0;  // Average difference between consecutive position values
// uint8_t resolution = 2.0475;  // Room for error due to sensor sensitivity
uint8_t resolution = 10;

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
  
  // // Read 6 status bits
  // magINC = digitalRead(DO_PIN);
  // digitalWrite(CLK_PIN, HIGH);
  // delayMicroseconds(delay);
  // digitalWrite(CLK_PIN, LOW);
  // delayMicroseconds(delay);

  // magDEC = digitalRead(DO_PIN);
  // digitalWrite(CLK_PIN, HIGH);
  // delayMicroseconds(delay);
  // digitalWrite(CLK_PIN, LOW);
  // delayMicroseconds(delay);

  // lin = digitalRead(DO_PIN);
  // digitalWrite(CLK_PIN, HIGH);
  // delayMicroseconds(delay);
  // digitalWrite(CLK_PIN, LOW);
  // delayMicroseconds(delay);

  // Skip the remaining 3 status bits for now

  digitalWrite(CS_PIN, HIGH); // Deselect chip

  return data;
}

void loop() {
  uint16_t position = readAS5311();
  Serial.print("Position:");
  Serial.println(position);

  // Store the position value in the buffer
  positionBuffer[bufferIndex] = position;
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;  // Wrap around the buffer index


  // Calculate the average difference between consecutive position values
  double sum = 0.0;
  for (uint8_t i = 0; i < BUFFER_SIZE - 1; i++) {
    sum += abs(static_cast<double>(positionBuffer[i]) - static_cast<double>(positionBuffer[i + 1]));
  }
  averageDifference = sum / (BUFFER_SIZE - 1);

  // Check if the difference between the current and previous position is within 2.0475 of the average
  if (bufferIndex > 0) {
    double currentDifference = abs(static_cast<double>(position) - static_cast<double>(positionBuffer[bufferIndex - 2]));

    if (currentDifference > averageDifference + resolution || currentDifference < averageDifference - resolution) {
      // Serial.println("Difference exceeds the threshold!");
      newPositionBuffer[newBufferIndex] = position;
      newBufferIndex ++;
    }
    else{
      Serial.println("Not exceeded");
    }
  }

  // // Check the magnetic field strength status
  // if (magINC == 0 && magDEC == 0 && lin == 0) {
  //   Serial.println("Magnetic field OK (GREEN range)");
  // } else if (magINC == 1 && magDEC == 1 && lin == 0) {
  //   Serial.println("Magnetic field in YELLOW range");
  // } else if (lin == 1) {
  //   Serial.println("Magnetic field in RED range (not recommended)");
  // }

  delay(100); // Delay for readability
}
