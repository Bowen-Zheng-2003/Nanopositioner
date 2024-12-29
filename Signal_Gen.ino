const int SIGNAL    = 7; 
const int CURSOR    = 8; 
const int MODE      = 9; 
const int ADD       = 10; 
const int SUBTRACT  = 11;
const int DELAY_ON  = 50; 
const int DELAY_OFF = 100; 

void setup() {
  pinMode(MODE, OUTPUT);
  pinMode(CURSOR, OUTPUT);
  pinMode(ADD, OUTPUT);
  pinMode(SUBTRACT, OUTPUT);
  pinMode(SIGNAL, OUTPUT);

  Serial.begin(115200);
  calibration();
  Serial.println("Type '0' to change cursor, '1', to increment, '2' to decrement, and '3' to output");
}

void loop() {
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
      case '4':
        mode();
        break;
    }
  }
  signalOutput(); // turn it on
  delay(5000);
  signalOutput(); // turn it off
  reverse();
  signalOutput(); // turn it on
  delay(5000);
  signalOutput(); // turn it off
  forward();
}

void mode() {
  digitalWrite(MODE, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(MODE, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
}

void cursor() {
  digitalWrite(CURSOR, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(CURSOR, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
}

void increment() {
  digitalWrite(ADD, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(ADD, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
}

void decrement() {
  digitalWrite(SUBTRACT, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SUBTRACT, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
}

void signalOutput() {
  digitalWrite(SIGNAL, HIGH); // Turn on the MOSFET
  delay(DELAY_ON);
  digitalWrite(SIGNAL, LOW);  // Turn off the MOSFET
  delay(DELAY_OFF);
}

void forward() {
  for (int i = 0; i < 6; i++) {
    mode();
  }
}

void reverse() {
  mode();
}

void calibration() {
  for (int i = 0; i < 3; i++) {
    mode();
  }
  decrement();
  for (int i = 0; i < 2; i++) {
    cursor();
  }
  increment();
}
