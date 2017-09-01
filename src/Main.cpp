#include <Arduino.h>
#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD lcd;
Zumo32U4ButtonA buttonA;

// Buzzer
Zumo32U4Buzzer buzzer;
const char CHARGE_MELODY[] PROGMEM = "O4 T100 V12 L4 MS g12>c12>e12>G6>E12 ML>G2";
//

// Line Sensing
Zumo32U4LineSensors lineSensors;
const unsigned int LINE_SENSOR_THRESHOLD = 1000;
unsigned int lineSensorValues[3];

enum class LineReading {
  None,
  Right,
  Center,
  Left
};

void setupLineSensor() {
  lineSensors.initThreeSensors();
  for (uint8_t i = 0; i < 5; i++) {
    lineSensors.calibrate();
    delay(20);
  }
}

LineReading readLineSensor() {
  lineSensors.read(lineSensorValues);
  LineReading reading = LineReading::None;
  if ((lineSensorValues[0] < LINE_SENSOR_THRESHOLD && lineSensorValues[2] < LINE_SENSOR_THRESHOLD) || lineSensorValues[1] < LINE_SENSOR_THRESHOLD) {
    reading = LineReading::Center;
  } else if (lineSensorValues[0] < LINE_SENSOR_THRESHOLD) {
    reading = LineReading::Right;
  } else if (lineSensorValues[2] < LINE_SENSOR_THRESHOLD) {
    reading = LineReading::Left;
  }
  lcd.gotoXY(7, 1);
  switch (reading) {
    case LineReading::Right: lcd.print(F("R"));
    break;
    case LineReading::Left: lcd.print(F("L"));
    break;
    case LineReading::Center: lcd.print(F("C"));
    break;
    case LineReading::None: lcd.print(F("N"));
    break;
    default: lcd.print(F(" "));
    break;
  }
  return reading;
}
//

// Proximity Sensing
Zumo32U4ProximitySensors proximitySensors;
const uint8_t PROXIMITY_SENSOR_THRESHOLD = 5;

enum class ProximityReading {
  None,
  Right,
  FrontRight,
  FrontLeft,
  Left,
  Center
};

void setupProximitySensor() {
  proximitySensors.initThreeSensors();
}

ProximityReading readProximitySensor() {
  proximitySensors.read();
  uint8_t left = proximitySensors.countsLeftWithLeftLeds();
  uint8_t frontLeft = proximitySensors.countsFrontWithLeftLeds();
  uint8_t frontRight = proximitySensors.countsFrontWithRightLeds();
  uint8_t right = proximitySensors.countsRightWithRightLeds();

  ProximityReading reading = ProximityReading::None;
  if (right >= PROXIMITY_SENSOR_THRESHOLD && right > frontRight && right > frontLeft && right > left) {
    reading = ProximityReading::Right;
  } else if (left >= PROXIMITY_SENSOR_THRESHOLD && left > frontLeft && left > frontRight && left > right) {
    reading = ProximityReading::Left;
  } else if (frontRight >= PROXIMITY_SENSOR_THRESHOLD && frontRight > frontLeft) {
    reading = ProximityReading::FrontRight;
  } else if (frontLeft >= PROXIMITY_SENSOR_THRESHOLD && frontLeft > frontRight) {
    reading = ProximityReading::FrontLeft;
  } else if (frontRight + frontLeft >= PROXIMITY_SENSOR_THRESHOLD * 2) {
    reading = ProximityReading::Center;
  }

  lcd.gotoXY(0, 1);
  switch (reading) {
    case ProximityReading::Right: lcd.print(F("R"));
    break;
    case ProximityReading::FrontRight: lcd.print(F("FR"));
    break;
    case ProximityReading::FrontLeft: lcd.print(F("FL"));
    break;
    case ProximityReading::Left: lcd.print(F("L "));
    break;
    case ProximityReading::Center: lcd.print(F("C "));
    break;
    case ProximityReading::None: lcd.print(F("N "));
    break;
    default: lcd.print(F("  "));
    break;
  }
  lcd.print(left);
  lcd.print(frontLeft);
  lcd.print(frontRight);
  lcd.print(right);
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

int32_t readTurnSensor() {
  gyro.read();
  turnRate = gyro.g.z - gyroOffset;
  uint64_t now = micros();
  uint64_t dt = now - gyroLastUpdate;
  gyroLastUpdate = now;
  int32_t d = (int32_t)turnRate * dt;
  turnAngle += (int64_t)d * 14680064 / 17578125;
  return (((int32_t)turnAngle >> 16) * 360) >> 16;
}

// Motor
Zumo32U4Motors motors;
const int16_t fastSpeed = 400;
const int16_t mediumSpeed = 200;
const int16_t slowSpeed = 100;

void setTurnLeft() {
  motors.setSpeeds(-mediumSpeed, mediumSpeed);
}

void setTurnRight() {
  motors.setSpeeds(mediumSpeed, -mediumSpeed);
}

void setFastForward() {
  motors.setSpeeds(fastSpeed, fastSpeed);
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
//

// State Machine
enum class State {
  Scan,
  Wander,
  Rotate2Target,
  ChargeTarget,
  EvasiveAction,
  EdgeRecovery
};

State currentState;

void changeState(State state)  {
  currentState = state;
  lcd.clear();
  lcd.gotoXY(0, 0);
  switch (currentState) {
    case State::Scan: lcd.print(F("Scan"));
    break;
    case State::Rotate2Target: lcd.print(F("Rotate"));
    break;
    case State::ChargeTarget: lcd.print(F("Charge"));
    break;
    case State::EvasiveAction: lcd.print(F("Evasive"));
    break;
    case State::EdgeRecovery: lcd.print(F("Edge"));
    break;
    case State::Wander: lcd.print(F("Wander"));
    break;
    default: lcd.print(F("Unknown"));
    break;
  }
}
//

void setup() {
  setupLineSensor();
  setupProximitySensor();

  lcd.clear();
  lcd.gotoXY(0, 0);
  lcd.print("Press A");
  buttonA.waitForButton();
  lcd.clear();
  delay(2000);

  buzzer.playFromProgramSpace(CHARGE_MELODY);
  changeState(State::Scan);
}

void loop() {
  LineReading lineReading = readLineSensor();
  ProximityReading proximityReading = readProximitySensor();
  switch (currentState) {
    case State::Scan: {
      bool targetFound = proximityReading != ProximityReading::None;
      if (targetFound) {
        changeState(State::Rotate2Target);
      } else {
        changeState(State::Wander);
      }
    }
    case State::Wander: {
      bool lineFound = lineReading != LineReading::None;
      bool targetFound = proximityReading != ProximityReading::None;
      if (lineFound) {
        changeState(State::EdgeRecovery);
      } else if (targetFound) {
        changeState(State::EdgeRecovery);
      }
      setSlowForward();
    }
    break;

    case State::Rotate2Target: {
      changeState(State::ChargeTarget);
    }
    break;

    case State::EvasiveAction: {
      changeState(State::Scan);
    }
    break;

    case State::ChargeTarget: {
      changeState(State::EdgeRecovery);
    }
    break;

    case State::EdgeRecovery: {
      setSlowBackward();
      delay(1000);
      setTurnLeft();
      delay(500);
      changeState(State::Scan);
    }
    break;
  }

}
