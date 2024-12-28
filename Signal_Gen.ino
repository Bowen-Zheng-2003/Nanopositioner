const int SIGNAL   = 7; 
const int CURSOR   = 8; 
const int MODE     = 9; 
const int ADD      = 10; 
const int SUBTRACT = 11; 

void setup() {
  pinMode(MODE, OUTPUT);
  pinMode(CURSOR, OUTPUT);
  pinMode(ADD, OUTPUT);
  pinMode(SUBTRACT, OUTPUT);
  pinMode(SIGNAL, OUTPUT);

  Serial.begin(115200);
  calibration();
}

void loop() {
  
  Serial.println("Type '0' to change cursor, '1', to increment, '2' to decrement, and '3' to output");

  if (Serial.available() > 0) {
    char input = Serial.read();
    switch (input) {
      case '0':
        cursor();
        break;
      case '1':
        increment();
        break;
      case '2':
        decrement();
        break;
      case '3':
        signalOutput();
        break;
    }
  }
}

void mode() {
  digitalWrite(MODE, HIGH); // Turn on the MOSFET
  delay(100);
  digitalWrite(MODE, LOW);  // Turn off the MOSFET
  delay(1000);
}

void cursor() {
  digitalWrite(CURSOR, HIGH); // Turn on the MOSFET
  delay(100);
  digitalWrite(CURSOR, LOW);  // Turn off the MOSFET
  delay(500);
}

void increment() {
  digitalWrite(ADD, HIGH); // Turn on the MOSFET
  delay(100);
  digitalWrite(ADD, LOW);  // Turn off the MOSFET
  delay(500);
}

void decrement() {
  digitalWrite(SUBTRACT, HIGH); // Turn on the MOSFET
  delay(100);
  digitalWrite(SUBTRACT, LOW);  // Turn off the MOSFET
  delay(500);
}

void signalOutput() {
  digitalWrite(SIGNAL, HIGH); // Turn on the MOSFET
  delay(100);
  digitalWrite(SIGNAL, LOW);  // Turn off the MOSFET
  delay(500);
}

void calibration() {
  decrement();
  for (int i = 0; i < 3; i++) {
    mode();
  }
  for (int i = 0; i < 2; i++) {
    cursor();
  }
  increment();
}