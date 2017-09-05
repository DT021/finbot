#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4ButtonC buttonC;

// Serial
void printToSerial(const char* fmt, ...) {
  static char buffer[128];
  va_list argp;
	va_start(argp, fmt);
  vsprintf(buffer, fmt, argp);
	va_end(argp);
  Serial.print(buffer);
}

// Buzzer
Zumo32U4Buzzer buzzer;
const char CHARGE_MELODY[] PROGMEM = "O4 T100 V12 L4 MS g12>c12>e12>G6>E12 ML>G2";
//

// Sensor readings
enum class SensorReading {
  None,
  Left,
  Right,
  Center,
  FrontLeft,
  FrontRight
};

char sensorReadingToChar(SensorReading reading) {
  switch (reading) {
    case SensorReading::Left: return 'l';
    case SensorReading::Right: return 'r';
    case SensorReading::Center: return 'C';
    case SensorReading::FrontLeft: return 'L';
    case SensorReading::FrontRight: return 'R';
    default: return ' ';
  }
}

// Line Sensing
Zumo32U4LineSensors lineSensors;
const uint16_t LINE_SENSOR_THRESHOLD = 1000;
const bool useEmitters = true;
bool whiteLines;
uint16_t lineSensorRight;
uint16_t lineSensorCenter;
uint16_t lineSensorLeft;

void setupLineSensor(bool useWhiteLines) {
  whiteLines = useWhiteLines;
  lineSensors.initThreeSensors();
  /*
  for (uint8_t i = 0; i < 5; i++) {
    lineSensors.calibrate();
    delay(20);
  }
  */
}

SensorReading readLineSensor() {
  static unsigned int lineSensorValues[3];
  lineSensors.read(lineSensorValues, useEmitters ? QTR_EMITTERS_ON : QTR_EMITTERS_OFF);
  lineSensorRight = lineSensorValues[0];
  lineSensorCenter = lineSensorValues[1];
  lineSensorLeft = lineSensorValues[2];
  SensorReading reading = SensorReading::None;
  if (whiteLines) {
    if ((lineSensorRight < LINE_SENSOR_THRESHOLD && lineSensorLeft < LINE_SENSOR_THRESHOLD) || lineSensorCenter < LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Center;
    } else if (lineSensorRight < LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Right;
    } else if (lineSensorLeft < LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Left;
    }
  } else {
    if ((lineSensorRight > LINE_SENSOR_THRESHOLD && lineSensorLeft > LINE_SENSOR_THRESHOLD) || lineSensorCenter > LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Center;
    } else if (lineSensorRight > LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Right;
    } else if (lineSensorLeft > LINE_SENSOR_THRESHOLD) {
      reading = SensorReading::Left;
    }
  }

  lcd.gotoXY(7, 1);
  lcd.print(sensorReadingToChar(reading));
  return reading;
}
//

// Proximity Sensing
Zumo32U4ProximitySensors proximitySensors;
const uint8_t PROXIMITY_SENSOR_THRESHOLD = 5;
uint8_t proximitySensorLeft;
uint8_t proximitySensorFrontLeft;
uint8_t proximitySensorFrontRight;
uint8_t proximitySensorRight;

void setupProximitySensor() {
  proximitySensors.initThreeSensors();
}

SensorReading readProximitySensor() {
  proximitySensors.read();
  proximitySensorLeft = proximitySensors.countsLeftWithLeftLeds();
  proximitySensorFrontLeft = proximitySensors.countsFrontWithLeftLeds();
  proximitySensorFrontRight = proximitySensors.countsFrontWithRightLeds();
  proximitySensorRight = proximitySensors.countsRightWithRightLeds();

  SensorReading reading = SensorReading::None;
  // Ignore left and right for now, seem fishy
  /*
  if (proximitySensorRight >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorRight > proximitySensorFrontRight && proximitySensorRight > proximitySensorFrontLeft && proximitySensorRight > proximitySensorLeft) {
    reading = SensorReading::Right;
  } else if (proximitySensorLeft >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorLeft > proximitySensorFrontLeft && proximitySensorLeft > proximitySensorFrontRight && proximitySensorLeft > proximitySensorRight) {
    reading = SensorReading::Left;
  }
  */
  if (proximitySensorFrontRight >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorFrontLeft >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorFrontRight == proximitySensorFrontLeft) {
    reading = SensorReading::Center;
  } else if (proximitySensorFrontRight >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorFrontRight > proximitySensorFrontLeft) {
    reading = SensorReading::FrontRight;
  } else if (proximitySensorFrontLeft >= PROXIMITY_SENSOR_THRESHOLD && proximitySensorFrontLeft > proximitySensorFrontRight) {
    reading = SensorReading::FrontLeft;
  }

  lcd.gotoXY(0, 1);
  lcd.print(sensorReadingToChar(reading));
  lcd.print(' ');
  lcd.print(proximitySensorLeft);
  lcd.print(proximitySensorFrontLeft);
  lcd.print(proximitySensorFrontRight);
  lcd.print(proximitySensorRight);
  return reading;
}
//

// Turn Sensing
L3G gyro;
const int32_t turnAngle45 = 0x20000000; // This constant represents a turn of 45 degrees.
const int32_t turnAngle90 = turnAngle45 * 2; // This constant represents a turn of 90 degrees.
const int32_t turnAngle1 = (turnAngle45 + 22) / 45; // This constant represents a turn of approximately 1 degree.
int16_t gyroOffset;
uint32_t turnAngle;
int16_t turnRate;
uint64_t gyroLastUpdate;

void setupTurnSensing() {
  Wire.begin();
  gyro.init();
  gyro.writeReg(L3G::CTRL1, 0b11111111); // 800 Hz output data rate, low-pass filter cutoff 100 Hz
  gyro.writeReg(L3G::CTRL4, 0b00100000); // 2000 dps full scale
  gyro.writeReg(L3G::CTRL5, 0b00000000); // High-pass filter disabled
  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++) {
    while(!gyro.readReg(L3G::STATUS_REG) & 0x08); // Wait for new data to be available, then read it.
    gyro.read();
    total += gyro.g.z; // Add the Z axis reading to the total.
  }
  gyroOffset = total / 1024;
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void resetTurnSensor() {
  gyroLastUpdate = micros();
  turnAngle = 0;
}

int32_t readTurnSensor() {
  gyro.read();
  turnRate = gyro.g.z - gyroOffset;
  uint64_t now = micros();
  uint64_t dt = now - gyroLastUpdate;
  gyroLastUpdate = now;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle -= (int64_t)d * 14680064 / 17578125;
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

// Motor
Zumo32U4Motors motors;
const int16_t fastSpeed = 400;
const int16_t mediumSpeed = 200;
const int16_t slowSpeed = 100;
const int16_t veerSpeedProwl = 350;
const int16_t veerSpeedBorderFollow = 250;
const int16_t veerSpeedBorderFollowDetection = 100;

void setTurnLeft() {
  motors.setSpeeds(-mediumSpeed, mediumSpeed);
}

void setTurnRight() {
  motors.setSpeeds(mediumSpeed, -mediumSpeed);
}

void setFastForward() {
  motors.setSpeeds(fastSpeed, fastSpeed);
}

void setMediumForward() {
  motors.setSpeeds(mediumSpeed, mediumSpeed);
}

void setSlowForward() {
  motors.setSpeeds(slowSpeed, slowSpeed);
}

void setFastBackward() {
  motors.setSpeeds(-fastSpeed, -fastSpeed);
}

void setSlowBackward() {
  motors.setSpeeds(-slowSpeed, -slowSpeed);
}

void setStop() {
  motors.setSpeeds(0, 0);
}

void turn(int16_t degrees) {
  degrees = max(min(degrees, 179), -179); // Turn sensor handles +-179
  resetTurnSensor();
  if (degrees > 0) {
    setTurnRight();
    while (readTurnSensor() < degrees); // Turn for max one sec
  } else if (degrees < 0) {
    setTurnLeft();
    while (readTurnSensor() > degrees); // Turn for max one sec
  }
  setStop();
}
//

// State Machine
enum class State {
  Scan,
  Prowl,
  Rotate2Target,
  ChargeTarget,
  EvasiveAction,
  EdgeRecovery,
  BorderFollow,
  Debug
};

State currentState;
uint64_t currentStateTime;

void changeState(State state)  {
  currentState = state;
  currentStateTime = micros();
  lcd.clear();
  lcd.gotoXY(0, 0);
  switch (currentState) {
    case State::Scan: lcd.print(F("Scan"));
    break;
    case State::Prowl: lcd.print(F("Prowl"));
    break;
    case State::Rotate2Target: lcd.print(F("Rotate"));
    break;
    case State::ChargeTarget: lcd.print(F("Charge"));
    break;
    case State::EvasiveAction: lcd.print(F("Evasive"));
    break;
    case State::EdgeRecovery: lcd.print(F("Edge"));
    break;
    case State::BorderFollow: lcd.print(F("Border"));
    break;
    case State::Debug: lcd.print(F("Debug"));
    break;
    default: lcd.print(F("Unknown"));
    break;
  }
}

uint64_t getStateDuration() {
  return (micros() - currentStateTime) / 1000;
}
//

// Charge bot

enum class VeerState {
  Left,
  Right
};

VeerState currentVeerState;

void modeALoop() {
  SensorReading lineReading = readLineSensor();
  SensorReading proximityReading = readProximitySensor();
  switch (currentState) {

    case State::Prowl: {
      bool lineFound = lineReading != SensorReading::None;
      bool targetFound = proximityReading != SensorReading::None;

      if (lineFound) {
        if (lineReading == SensorReading::Left) {
          currentVeerState = VeerState::Right;
          motors.setSpeeds(veerSpeedBorderFollowDetection, fastSpeed);
        } else if (lineReading == SensorReading::Right) {
          currentVeerState = VeerState::Left;
          motors.setSpeeds(fastSpeed, veerSpeedBorderFollowDetection);
        } else if (lineReading == SensorReading::Center) {
          changeState(State::EdgeRecovery);
        }
      } else if (targetFound) {
        changeState(State::ChargeTarget);
      } else {
        if (currentVeerState == VeerState::Left) {
          motors.setSpeeds(veerSpeedBorderFollow, fastSpeed);
        } else {
          motors.setSpeeds(fastSpeed, veerSpeedBorderFollow);
        }
      }
    }
    break;

    case State::ChargeTarget: {
      bool lineFound = lineReading != SensorReading::None;
      bool targetFound = proximityReading != SensorReading::None;
      if (targetFound) {
        if (proximityReading == SensorReading::FrontLeft) {
          motors.setSpeeds(veerSpeedProwl, fastSpeed);
        } else if (proximityReading == SensorReading::FrontRight) {
          motors.setSpeeds(fastSpeed, veerSpeedProwl);
        } else {
          motors.setSpeeds(fastSpeed, fastSpeed);
        }
      } else if (lineFound) {
        changeState(State::EdgeRecovery);
      }
    }
    break;

    case State::EdgeRecovery: {
      if (lineReading == SensorReading::Left) {
        setFastBackward();
        delay(200);
        turn(45);
        currentVeerState = VeerState::Right;
      } else if (lineReading == SensorReading::Right) {
        setFastBackward();
        delay(200);
        turn(-45);
        currentVeerState = VeerState::Left;
      } else if (lineReading == SensorReading::Center) {
        setFastBackward();
        delay(200);
        turn(90);
        currentVeerState = VeerState::Right;
      } else {
        changeState(State::BorderFollow);
      }
    }
    break;

    default: {
      changeState(State::Prowl);
    }
    break;
  }
}

// Flee bot
void modeBLoop() {
  SensorReading lineReading = readLineSensor();
  SensorReading proximityReading = readProximitySensor();
  switch (currentState) {
    case State::Scan: {
      setStop();
      bool targetFound = false;
      for (uint8_t i = 0; i < 8; i++) {
        turn(45);
        targetFound = proximityReading != SensorReading::None;
        if (targetFound) {
          break;
        }
      }
      if (targetFound) {
        changeState(State::EdgeRecovery);
      } else {
        changeState(State::EvasiveAction);
      }
    }
    break;

    case State::EvasiveAction: {
      bool lineFound = lineReading != SensorReading::None;
      bool escapeLeft = proximityReading == SensorReading::FrontLeft;
      bool escapeCenter = proximityReading == SensorReading::Center;
      bool escapeRight = proximityReading == SensorReading::FrontRight;
      if (lineFound) {
        changeState(State::EdgeRecovery);
      } else if (escapeLeft) {
        turn(45);
      } else if (escapeRight) {
        turn(-45);
      } else if (escapeCenter) {
        turn(90);
      }
      setSlowForward();
    }
    break;

    case State::EdgeRecovery: {
      setSlowBackward();
      delay(1000);
      turn(180);
      changeState(State::EvasiveAction);
    }
    break;

    default:
    break;
  }
}

// Debug mode
void modeCLoop() {
  SensorReading lineReading = readLineSensor();
  SensorReading proximityReading = readProximitySensor();
  printToSerial("Line sensors: %c %4d %4d %4d %c\n", sensorReadingToChar(lineReading), lineSensorLeft, lineSensorCenter, lineSensorRight, useEmitters ? 'E' : 'e');
  printToSerial("Proximity sensors: %c %4d %4d %4d %4d\n", sensorReadingToChar(proximityReading), proximitySensorLeft, proximitySensorFrontLeft, proximitySensorFrontRight, proximitySensorRight);
  printToSerial("Turn sensor: %4d %4d\n", readTurnSensor());
  delay(1000);
}


// Setup and mode selection
char mode = 'A';

void setup() {
  randomSeed(analogRead(0));
  setupLineSensor(true);
  setupProximitySensor();
  setupTurnSensing();

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Can haz");
  lcd.gotoXY(0, 1);
  lcd.print("A B or C");

  while (true) {
    if (buttonA.getSingleDebouncedPress()) {
      mode = 'A';
      changeState(State::Prowl);
      break;
    } else if (buttonB.getSingleDebouncedPress()) {
      mode = 'B';
      changeState(State::Prowl);
      break;
    } else if (buttonC.getSingleDebouncedPress()) {
      mode = 'C';
      break;
    }
    delay(100);
  }
  lcd.clear();
  delay(2000);
  buzzer.playFromProgramSpace(CHARGE_MELODY);
}

void loop() {
  switch (mode) {
    case 'A': modeALoop(); break;
    case 'B': modeBLoop(); break;
    case 'C': modeCLoop(); break;
    default: break;
  }
}
//
