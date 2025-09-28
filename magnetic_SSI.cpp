#include <Arduino.h>
#include <timers.h>
#define DEADBAND 1.0F

const int X_CS = A2;
const int X_CLK = A3;
const int X_DO = A4;
const int Y_CS = A5;
const int Y_CLK = 25;
const int Y_DO = 24;

int position_x = 0;
int position_y = 0;
int old_x_pos = 0;
int old_y_pos = 0;
int new_x_pos = 0;
int new_y_pos = 0;
double counts_microns = 2000.0/4096;

uint16_t read_bits(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t numBits) {
	uint16_t value = 0;
	uint16_t i;
	for (i = 0; i < numBits; i++) {
		digitalWrite(clockPin, HIGH);
		if (bitOrder == LSBFIRST){
      delayMicroseconds(50);
			value |= digitalRead(dataPin) << i;
    }
		else{
      delayMicroseconds(50);
			value |= digitalRead(dataPin) << (numBits -1 - i);
    }
		digitalWrite(clockPin, LOW);
	}
	return value;
}

int position_read(uint8_t csPin, uint8_t clockPin, uint8_t dataPin){
  int position = 0;
  digitalWrite(clockPin, 1);           
  digitalWrite(csPin, LOW);       // Start transfer
  digitalWrite(clockPin, 0);
  uint16_t read_position = read_bits(dataPin, clockPin, MSBFIRST, 12);  
  uint8_t error = read_bits(dataPin, clockPin, MSBFIRST, 3);
  digitalWrite(csPin, HIGH);      // End transfer

  if(error == 4){ // expect binary to be 100 to be good, which is 4 in decimal
    position = read_position;
    return position;
  }
  else{
    return position;
  }
}

int track_position(int current_pos, int new_pos, int old_pos){
  int diff = new_pos-old_pos;
  if(abs(diff) > 4096/4) { // wrap around occurred
    if(diff > 0) { // backward motion occurred
      current_pos = current_pos - (4096-new_pos) - old_pos; 
    }
    else { // forward motion occured
      current_pos = current_pos + (4096-old_pos) + new_pos; 
    }
  }
  else { // no wrap around
    current_pos = current_pos + diff;
  }

  return current_pos;
}

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

void setup() {
  setup_timers();

  // POSITION TRACKING
  pinMode(X_CS, OUTPUT);
  pinMode(X_CLK, OUTPUT);
  pinMode(X_DO, INPUT);
  pinMode(Y_CS, OUTPUT);
  pinMode(Y_CLK, OUTPUT);
  pinMode(Y_DO, INPUT);
  digitalWrite(X_CLK, 1);      //CLK
  digitalWrite(X_CS, 1);   //Csn
  digitalWrite(Y_CLK, 1);      //CLK
  digitalWrite(Y_CS, 1);   //Csn

  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (! Serial) {
    // wait until Serial is connected
  }
  
  // Start DAC
  // write_freqs(600, 600);

  // Start position tracking
  old_x_pos = position_read(X_CS, X_CLK, X_DO);
  old_y_pos = position_read(Y_CS, Y_CLK, Y_DO);
}

int freq = 500;
unsigned long startTime = millis();

void loop() {
  unsigned long elapsed = millis() - startTime;
  if((elapsed < 10000)) {
    write_freqs(freq, freq);
  }
  else if((elapsed >= 10000)&(elapsed < 20000)) {
    write_freqs(-freq, -freq);
  }
  else{
    startTime = millis();
  }

  new_x_pos = position_read(X_CS, X_CLK, X_DO);
  position_x = track_position(position_x, new_x_pos, old_x_pos);
  Serial.print("X Pos: ");
  Serial.println(position_x*counts_microns);
  old_x_pos = new_x_pos;

  new_y_pos = position_read(Y_CS, Y_CLK, Y_DO);
  position_y = track_position(position_y, new_y_pos, old_y_pos);
  Serial.print("Y Pos: ");
  Serial.println(position_y*counts_microns);
  old_y_pos = new_y_pos;
  delay(100);
}
