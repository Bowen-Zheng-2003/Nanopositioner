#include <Arduino.h>
#include <timers.h>
#define DEADBAND 1.0F

// MAGNETIC SENSOR INITIALIZATION
const int X_CS = A2;
const int X_CLK = A3;
const int X_DO = A4;
const int Y_CS = A5;
const int Y_CLK = 25;
const int Y_DO = 24;

// STATE MACHINE
const int CALIBRATE = 1;
const int READ = 2;
const int MOVE = 3;
const int WAIT = 4;
int state = CALIBRATE;
int old_state = CALIBRATE;
bool axis_state = 0;       // 0 is X axis, 1 is Y axis
bool unusedXFlag = true;   // For tracking if offset is used
bool unusedYFlag = true;   // For tracking if offset is used
bool readComplete = false; // Checks if reading is complete before moving stages

// CALIBRATION
struct AxisStatus {
  volatile int piezoPosition;
  volatile int oldPiezoPosition;
  volatile int newPiezoPosition;
  volatile int offsetVal;
  bool calibrationDone;
  unsigned long startTime;
};

// INITIALIZING POSITION TRACKING
#define NUM_AXES 2  // X, Y
AxisStatus axes[NUM_AXES] = {
  { -15000, 0, 0, 0, false, 0 },  // X
  { -15000, 0, 0, 0, false, 0 },  // Y
};

const int calibrateRange = 2;
// unsigned long startTime         = 0;
const unsigned long duration = 2000;  // 2 seconds in milliseconds
const int startingFrequencyX = 700;
const int startingFrequencyY = -700;
const float countsToMicrons = 0.48828125;  // 2000 microns/2^12 counts

// FOR PID CONTROL
unsigned long executionDuration = 0;  // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time
int targetXPosition = 0, targetYPosition = 0;
float positionErrorX = 0, positionErrorY = 0;
float integralErrorX = 0, integralErrorY = 0;
float velocityErrorX = 0, velocityErrorY = 0;
float desiredFrequencyX = 0, desiredFrequencyY = 0;
float piezoVelocityX = 0, piezoVelocityY = 0;
int previousPiezoPositionX = 0, previousPiezoPositionY = 0;
long previousVelCompTimeX = 0, previousVelCompTimeY = 0;
const int MIN_VEL_COMP_COUNT = 2;      // [encoder counts] Minimal change in piezo position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME = 10000;  // [microseconds] Minimal time that must pass between two velocity measurements
float KP = 500;                        // [Volt / encoder counts] P-Gain
float KD = 0.005;                      // [Volt * seconds / encoder counts] D-Gain
float KI = 0.005;                      // [Volt / (encoder counts * seconds)] I-Gain

// Timing:
unsigned long startWaitTime;     // [microseconds] System clock value at the moment the WAIT state started
const long WAIT_TIME = 10000;  // [microseconds] Time waiting at each location
const int TARGET_BAND = 10;       // [encoder counts] "Close enough" range when moving towards a target.

// USER INPUT:
struct Coordinate {
  float xMicrons;
  float yMicrons;
  int xCounts;
  int yCounts;
};

#define NUM_COORDINATES 200
Coordinate coordinates[NUM_COORDINATES] = {
  {0, 0, 0, 0},
  {9276, 21084, 0, 0},
  {9276, 20684, 0, 0},
  {9276, 20284, 0, 0},
  {9276, 19884, 0, 0},
  {9276, 19484, 0, 0},
  {9276, 19084, 0, 0},
  {9276, 18684, 0, 0},
  {9276, 18284, 0, 0},
  {9276, 17884, 0, 0},
  {9276, 17484, 0, 0},
  {9276, 17084, 0, 0},
  {9276, 16684, 0, 0},
  {9276, 16284, 0, 0},
  {9276, 15884, 0, 0},
  {9276, 15484, 0, 0},
  {9276, 15084, 0, 0},
  {9276, 14684, 0, 0},
  {9276, 14284, 0, 0},
  {9276, 13884, 0, 0},
  {9276, 13484, 0, 0},
  {9276, 13084, 0, 0},
  {9276, 13084, 0, 0},
  {9383, 12564, 0, 0},
  {9490, 12044, 0, 0},
  {9596, 11525, 0, 0},
  {9703, 11005, 0, 0},
  {9810, 10485, 0, 0},
  {9917, 9965, 0, 0},
  {10024, 9446, 0, 0},
  {10131, 8926, 0, 0},
  {10238, 8406, 0, 0},
  {10345, 7886, 0, 0},
  {10452, 7366, 0, 0},
  {10559, 6847, 0, 0},
  {10666, 6327, 0, 0},
  {10773, 5807, 0, 0},
  {10880, 5287, 0, 0},
  {10987, 4767, 0, 0},
  {11094, 4248, 0, 0},
  {11201, 3728, 0, 0},
  {11308, 3208, 0, 0},
  {11415, 2688, 0, 0},
  {11415, 2688, 0, 0},
  {11605, 3273, 0, 0},
  {11794, 3857, 0, 0},
  {11984, 4442, 0, 0},
  {12174, 5027, 0, 0},
  {12364, 5611, 0, 0},
  {12554, 6196, 0, 0},
  {12743, 6780, 0, 0},
  {12933, 7365, 0, 0},
  {13123, 7949, 0, 0},
  {13313, 8534, 0, 0},
  {13502, 9119, 0, 0},
  {13692, 9703, 0, 0},
  {13882, 10288, 0, 0},
  {14072, 10872, 0, 0},
  {14262, 11457, 0, 0},
  {14451, 12041, 0, 0},
  {14641, 12626, 0, 0},
  {14831, 13211, 0, 0},
  {15021, 13795, 0, 0},
  {15210, 14380, 0, 0},
  {15210, 14380, 0, 0},
  {15210, 14715, 0, 0},
  {15210, 15050, 0, 0},
  {15210, 15385, 0, 0},
  {15210, 15721, 0, 0},
  {15210, 16056, 0, 0},
  {15210, 16391, 0, 0},
  {15210, 16726, 0, 0},
  {15210, 17061, 0, 0},
  {15210, 17397, 0, 0},
  {15210, 17732, 0, 0},
  {15210, 18067, 0, 0},
  {15210, 18402, 0, 0},
  {15210, 18738, 0, 0},
  {15210, 19073, 0, 0},
  {15210, 19408, 0, 0},
  {15210, 19743, 0, 0},
  {15210, 20078, 0, 0},
  {15210, 20414, 0, 0},
  {15210, 20749, 0, 0},
  {15210, 21084, 0, 0},
  {15210, 21084, 0, 0},
  {15360, 21084, 0, 0},
  {15510, 21084, 0, 0},
  {15660, 21084, 0, 0},
  {15810, 21084, 0, 0},
  {15960, 21084, 0, 0},
  {16110, 21084, 0, 0},
  {16260, 21084, 0, 0},
  {16410, 21084, 0, 0},
  {16560, 21084, 0, 0},
  {16710, 21084, 0, 0},
  {16860, 21084, 0, 0},
  {17010, 21084, 0, 0},
  {17160, 21084, 0, 0},
  {17310, 21084, 0, 0},
  {17460, 21084, 0, 0},
  {17610, 21084, 0, 0},
  {17760, 21084, 0, 0},
  {17910, 21084, 0, 0},
  {18060, 21084, 0, 0},
  {18210, 21084, 0, 0},
  {18210, 21084, 0, 0},
  {17614, 21084, 0, 0},
  {17017, 21084, 0, 0},
  {16420, 21084, 0, 0},
  {15823, 21084, 0, 0},
  {15227, 21084, 0, 0},
  {14630, 21084, 0, 0},
  {14033, 21084, 0, 0},
  {13436, 21084, 0, 0},
  {12840, 21084, 0, 0},
  {12243, 21084, 0, 0},
  {11646, 21084, 0, 0},
  {11050, 21084, 0, 0},
  {10453, 21084, 0, 0},
  {9856, 21084, 0, 0},
  {9259, 21084, 0, 0},
  {8663, 21084, 0, 0},
  {8066, 21084, 0, 0},
  {7469, 21084, 0, 0},
  {6872, 21084, 0, 0},
  {6276, 21084, 0, 0},
  {6276, 21084, 0, 0},
  {6426, 21084, 0, 0},
  {6576, 21084, 0, 0},
  {6726, 21084, 0, 0},
  {6876, 21084, 0, 0},
  {7026, 21084, 0, 0},
  {7176, 21084, 0, 0},
  {7326, 21084, 0, 0},
  {7476, 21084, 0, 0},
  {7626, 21084, 0, 0},
  {7776, 21084, 0, 0},
  {7926, 21084, 0, 0},
  {8076, 21084, 0, 0},
  {8226, 21084, 0, 0},
  {8376, 21084, 0, 0},
  {8526, 21084, 0, 0},
  {8676, 21084, 0, 0},
  {8826, 21084, 0, 0},
  {8976, 21084, 0, 0},
  {9126, 21084, 0, 0},
  {9276, 21084, 0, 0}
};
// Track which coordinate we're at:
int currentCoordinateIndex = 1; // Ignoring first one

uint16_t read_bits(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t numBits) {
  uint16_t value = 0;
  uint16_t i;
  for (i = 0; i < numBits; i++) {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST) {
      delayMicroseconds(50);
      value |= digitalRead(dataPin) << i;
    } else {
      delayMicroseconds(50);
      value |= digitalRead(dataPin) << (numBits - 1 - i);
    }
    digitalWrite(clockPin, LOW);
  }
  return value;
}

int position_read(uint8_t csPin, uint8_t clockPin, uint8_t dataPin) {
  int position = 0;
  digitalWrite(clockPin, 1);
  digitalWrite(csPin, LOW);  // Start transfer
  digitalWrite(clockPin, 0);
  uint16_t read_position = read_bits(dataPin, clockPin, MSBFIRST, 12);
  uint8_t error = read_bits(dataPin, clockPin, MSBFIRST, 3);
  digitalWrite(csPin, HIGH);  // End transfer

  if (error == 4) {  // expect binary to be 100 to be good, which is 4 in decimal
    position = read_position;
    return position;
  } else {
    return position;
  }
}

int track_position(int current_pos, int new_pos, int old_pos, int index, bool offsetFlag, int offset) {
  int diff = new_pos - old_pos;
  if (offsetFlag & (unusedXFlag | unusedYFlag)) {
    current_pos = current_pos - offset;
    if (index == 0) {
      unusedXFlag = false;  // X is offsetted
    } else if (index == 1) {
      unusedYFlag = false;  // Y is offsetted
    }
  }
  if (abs(diff) > 4096 / 4) {  // wrap around occurred
    if (diff > 0) {            // backward motion occurred
      current_pos = current_pos - (4096 - new_pos) - old_pos;
    } else {  // forward motion occured
      current_pos = current_pos + (4096 - old_pos) + new_pos;
    }
  } else {  // no wrap around
    current_pos = current_pos + diff;
  }

  return current_pos;
}

void write_freqs(float hz_0, float hz_1) {
  // TODO: deadband... around zero...
  if (hz_0 < DEADBAND && hz_0 > -DEADBAND) {
    hz_0 = hz_0 < 0.0F ? -DEADBAND : DEADBAND;
  }
  if (hz_1 < DEADBAND && hz_1 > -DEADBAND) {
    hz_1 = hz_1 < 0.0F ? -DEADBAND : DEADBAND;
  }
  //
  float per_0 = 120000000.0F / (hz_0 * 4096.0F);
  float per_1 = 120000000.0F / (hz_1 * 4096.0F);
  //
  write_periods((int16_t)per_0, (int16_t)per_1);
}

void calibrateAxis(int axisIndex) {
  AxisStatus &axis = axes[axisIndex];
  if (axis.calibrationDone) return;

  if (axisIndex == 0) {
    axis.piezoPosition = position_read(X_CS, X_CLK, X_DO);
  } else if (axisIndex == 1) {
    axis.piezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
  }
  if ((axis.piezoPosition >= axis.oldPiezoPosition - calibrateRange) && (axis.piezoPosition <= axis.oldPiezoPosition + calibrateRange)) {
    if (axis.startTime == 0) {
      axis.startTime = millis();
    } else if (millis() - axis.startTime >= duration) {
      Serial.println("Positioner stopped moving");
      if (axisIndex == 0) {  // X-Axis
        write_freqs(0, startingFrequencyY);
        axes[0].offsetVal = axis.piezoPosition;
      } else if (axisIndex == 1) {  // Y-Axis
        write_freqs(0, 0);
        axes[1].offsetVal = axis.piezoPosition;
      }
      axis.calibrationDone = true;
      write_freqs(0, 0);  // Temporarily stops movements - calibration is done
      Serial.print("Finished calibrating axis ");
      Serial.println(axisIndex);
    }
  } else {
    axis.startTime = 0;
  }

  axis.oldPiezoPosition = axis.piezoPosition;
  delay(100);
}

bool receivingCoords = false;
int coordCount = 0;

void readSerialInput() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "START_COORDS") {
      Serial.println("Ready to receive coordinates");
      receivingCoords = true;
      coordCount = 0;
      return;
    }

    if (line == "END_COORDS") {
      Serial.print("Received ");
      Serial.print(coordCount);
      Serial.println(" coordinates total.");
      receivingCoords = false;
      readComplete = true;  // Coordinate reading done 
      return;
    }
    if (receivingCoords) {
      int commaIndex = line.indexOf(',');
      if (commaIndex > 0) {
        String xStr = line.substring(0, commaIndex);
        String yStr = line.substring(commaIndex + 1);
        float xVal = xStr.toFloat();
        float yVal = yStr.toFloat();

        // Store or process coordinate
        coordinates[coordCount].xMicrons = xVal;
        coordinates[coordCount].yMicrons = yVal;
        coordinates[coordCount].xCounts = xVal / countsToMicrons;
        coordinates[coordCount].yCounts = yVal / countsToMicrons;

        coordCount++;
        Serial.println("ACK");
      }
    }
    
  }
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
  digitalWrite(X_CLK, 1);  //CLK
  digitalWrite(X_CS, 1);   //Csn
  digitalWrite(Y_CLK, 1);  //CLK
  digitalWrite(Y_CS, 1);   //Csn

  // INITIALIZE SERIAL MONITOR
  Serial.begin(115200);
  while (!Serial) {
    // wait until Serial is connected
  }

  // Start DAC
  // write_freqs(600, 600);
  write_freqs(0, 0);

  // Start position tracking
  axes[0].oldPiezoPosition = position_read(X_CS, X_CLK, X_DO);
  axes[1].oldPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);

  // Convert all positions from microns to encoder counts
  for (int i = 0; i < NUM_COORDINATES; i++) {
    coordinates[i].xCounts = coordinates[i].xMicrons / countsToMicrons;
    coordinates[i].yCounts = coordinates[i].yMicrons / countsToMicrons;
  }
  Serial.println("Arduino ready");
}

unsigned long startTime = millis();
String incomingString;

void loop() {
  switch (state) {
    case CALIBRATE:
      {
        write_freqs(startingFrequencyX, startingFrequencyY);  // Or update to support more axes if needed
        bool allDone = true;
        for (int i = 0; i < NUM_AXES; i++) {
          calibrateAxis(i);
          allDone = allDone && axes[i].calibrationDone;
        }
        if (allDone) {
          Serial.println("All axes calibrated!");
          Serial.print("X position is ");
          Serial.println(axes[0].piezoPosition);
          Serial.print("Y position is ");
          Serial.println(axes[1].piezoPosition);
          state = WAIT;
          startWaitTime = micros();
        }
        break;
      }
    case READ:
    {
      readSerialInput();
      // Wait until all coordinates have been received before moving on
      if (readComplete) {
        Serial.println("All coordinates received, moving to MOVE state");
        readComplete = false;  // reset for next round, if needed
        // currentCoordinateIndex = 0; // reset index for movement
        state = MOVE;
      }
      break;
    }
    case MOVE:
      {
        // Set targets
        targetXPosition = coordinates[currentCoordinateIndex].xCounts;
        targetYPosition = coordinates[currentCoordinateIndex].yCounts;

        Serial.print("Coordinate Index: ");
        Serial.println(currentCoordinateIndex);
        // Serial.print("Target Y: ");
        // Serial.println(targetYPosition);

        // PID for X:
        executionDuration = micros() - lastExecutionTime;
        lastExecutionTime = micros();

        // --------- X Axis ---------
        if ((abs(axes[0].piezoPosition - previousPiezoPositionX) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTimeX) > MIN_VEL_COMP_TIME) {
          piezoVelocityX = (double)(axes[0].piezoPosition - previousPiezoPositionX) * 1000000 / (micros() - previousVelCompTimeX);
          previousPiezoPositionX = axes[0].piezoPosition;
          previousVelCompTimeX = micros();
        }
        positionErrorX = targetXPosition - axes[0].piezoPosition;
        integralErrorX += positionErrorX * (float)(executionDuration) / 1000000;
        velocityErrorX = 0 - piezoVelocityX;
        desiredFrequencyX = KP * positionErrorX + KI * integralErrorX + KD * velocityErrorX;

        // Clamp X as in axis code:
        int limit = 400;
        if (desiredFrequencyX >= limit) desiredFrequencyX = limit;
        else if (desiredFrequencyX <= -limit) desiredFrequencyX = -limit;
        else if ((desiredFrequencyX >= 0) && (desiredFrequencyX <= 10)) desiredFrequencyX = 10;
        else if ((desiredFrequencyX <= 0) && (desiredFrequencyX >= -10)) desiredFrequencyX = -10;

        // --------- Y Axis ---------
        if ((abs(axes[1].piezoPosition - previousPiezoPositionY) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTimeY) > MIN_VEL_COMP_TIME) {
          piezoVelocityY = (double)(axes[1].piezoPosition - previousPiezoPositionY) * 1000000 / (micros() - previousVelCompTimeY);
          previousPiezoPositionY = axes[1].piezoPosition;
          previousVelCompTimeY = micros();
        }
        positionErrorY = targetYPosition - axes[1].piezoPosition;
        integralErrorY += positionErrorY * (float)(executionDuration) / 1000000;
        velocityErrorY = 0 - piezoVelocityY;
        desiredFrequencyY = KP * positionErrorY + KI * integralErrorY + KD * velocityErrorY;

        if (desiredFrequencyY >= limit) desiredFrequencyY = limit;
        else if (desiredFrequencyY <= -limit) desiredFrequencyY = -limit;
        else if ((desiredFrequencyY >= 0) && (desiredFrequencyY <= 10)) desiredFrequencyY = 10;
        else if ((desiredFrequencyY <= 0) && (desiredFrequencyY >= -10)) desiredFrequencyY = -10;

        // Write to both channels SIMULTANEOUSLY
        // Serial.print("X Freq: ");
        // Serial.println(desiredFrequencyX);
        // Serial.print("Y Freq: ");
        // Serial.println(desiredFrequencyY);

        write_freqs(-desiredFrequencyX, desiredFrequencyY);

        // Target position check (both axes within TARGET_BAND)
        bool xReached = abs(axes[0].piezoPosition - targetXPosition) <= TARGET_BAND;
        bool yReached = abs(axes[1].piezoPosition - targetYPosition) <= TARGET_BAND;
        if (xReached && yReached) {
          Serial.println("YOU REACHED THE COORDINATE");
          write_freqs(0, 0);
          startWaitTime = micros();
          old_state = MOVE;
          state = WAIT;
        }
        break;
      }

    case WAIT:
      if (micros() - startWaitTime > WAIT_TIME) {  // enter WAIT after a certain amount of time
        if (old_state == CALIBRATE) {
          state = MOVE;
          Serial.println("State transition from CALIBRATE to READ");
        }
        if (old_state == READ) {
          state = MOVE;
          Serial.println("State transition from READ to MOVE");
        }
        if (old_state == MOVE) {
          integralErrorX = 0;
          integralErrorY = 0;
          currentCoordinateIndex++;
          if (currentCoordinateIndex < NUM_COORDINATES) {
            state = MOVE;  // Jump to next coordinate
            Serial.println("State transition to next coordinate");
          } else {
            Serial.println("All coordinates completed!");
            while (1)
              ;  // Stop program
          }
        }
      }
      break;

    default:
      Serial.println("State machine reached a state that it cannot handle.  ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1)
        ;  // infinite loop to halt the program
      break;
  }

  if (state == MOVE) {
    // TRACKING X POSITION
    axes[0].newPiezoPosition = position_read(X_CS, X_CLK, X_DO);
    axes[0].piezoPosition = track_position(axes[0].piezoPosition, axes[0].newPiezoPosition, axes[0].oldPiezoPosition, 0, true, axes[0].offsetVal);
    Serial.print("X: ");
    Serial.println(axes[0].piezoPosition * countsToMicrons);
    axes[0].oldPiezoPosition = axes[0].newPiezoPosition;

    // TRACKING Y POSITION
    axes[1].newPiezoPosition = position_read(Y_CS, Y_CLK, Y_DO);
    axes[1].piezoPosition = track_position(axes[1].piezoPosition, axes[1].newPiezoPosition, axes[1].oldPiezoPosition, 1, true, axes[1].offsetVal);
    Serial.print("Y: ");
    Serial.println(axes[1].piezoPosition * countsToMicrons);
    axes[1].oldPiezoPosition = axes[1].newPiezoPosition;
  }
}
