const int X_DO = A4;
const int X_CLK = A3;
const int X_CS_PIN = A2;

uint16_t readBits(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t numBits) {
	uint16_t value = 0;
	uint16_t i;
	for (i = 0; i < numBits; i++) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST){
      delayMicroseconds(1);
			value |= digitalRead(dataPin) << i;
    }
		else{
      delayMicroseconds(1);
			value |= digitalRead(dataPin) << (numBits -1 - i);
    }
		digitalWrite(clockPin, LOW);
	}
	return value;
}

void setup() {
  pinMode(X_CS_PIN, OUTPUT);
  pinMode(X_CLK, OUTPUT);
  pinMode(X_DO, INPUT);
  
  digitalWrite(X_CLK, 1);  //CLK
  digitalWrite(X_CS_PIN, 1);   //Csn
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup over");
}

void loop() {
  digitalWrite(X_CLK, 1);                            //position-> CLK=1 avant CS=0
  digitalWrite(X_CS_PIN, LOW);       // Start transfer
  digitalWrite(X_CLK, 0);
  uint16_t pos = readBits(X_DO, X_CLK, MSBFIRST, 12);  
  digitalWrite(X_CS_PIN, HIGH);      // End transfer

  Serial.println(pos);
  delay(1);
}
