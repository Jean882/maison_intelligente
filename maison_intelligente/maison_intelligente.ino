#include <AccelStepper.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <HCSR04.h>

// Define Pins
#define TRIGGER_PIN 2
#define ECHO_PIN 3
#define IN_1 8
#define IN_2 9
#define IN_3 10
#define IN_4 11
#define MOTOR_INTERFACE_TYPE 4


AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);
LCD_I2C lcd(0x27, 16, 2);
HCSR04 distanceSensor(TRIGGER_PIN, ECHO_PIN);

// States
enum DoorState { CLOSED,
                 OPENING,
                 OPEN,
                 CLOSING };
DoorState state = CLOSED;

// Global variables
unsigned long lastMeasureTime = 0;
unsigned long lastSerialTime = 0;
const long closedPosition = 0;
const long openPosition = 1000;

// Constant variables
const int distanceThresholdOpen = 30;
const int distanceThresholdClose = 60;
const unsigned long measureInterval = 50;
const unsigned long serialInterval = 100;
const unsigned long minAngle = 10;
const unsigned long maxAngle = 170;

#pragma region

int getCurrentAngle() {  // return map angle
  return map(myStepper.currentPosition(), closedPosition, openPosition, minAngle, maxAngle);
}

double measureDistance() {  // return distance
  return distanceSensor.dist();
}

void displayLCD(double distance) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist: ");
  lcd.print(distance);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  switch (state) {
    case CLOSED:
      lcd.print("Porte: Closed");
      break;
    case OPEN:
      lcd.print("Porte: Open");
      break;
    default:
      lcd.print("Porte: ");
      lcd.print(getCurrentAngle());
      lcd.print(" deg");
      break;
  }
}

void displaySerial(double distance) {
  Serial.print("etd:2206160");
  Serial.print(",dist:");
  Serial.print(distance);
  Serial.print(",deg:");
  Serial.println(getCurrentAngle());
}

#pragma endregion

void updateState(double distance) {
  switch (state) {
    case CLOSED:
      if (distance < distanceThresholdOpen) {
        state = OPENING;
        myStepper.enableOutputs();
        myStepper.moveTo(openPosition);
      }
      break;

    case OPENING:
      if (myStepper.distanceToGo() == 0) {
        state = OPEN;
        myStepper.disableOutputs();
      }
      break;

    case OPEN:
      if (distance > distanceThresholdClose) {
        state = CLOSING;
        myStepper.enableOutputs();
        myStepper.moveTo(closedPosition);
      }
      break;

    case CLOSING:
      if (myStepper.distanceToGo() == 0) {
        state = CLOSED;
        myStepper.disableOutputs();
      }
      break;
  }
}

#pragma region setup - loop

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(250);
  myStepper.setCurrentPosition(closedPosition);
  myStepper.disableOutputs();

  while (millis() <= 2000) {

    lcd.setCursor(0, 0);
    lcd.print("2206160      ");
    lcd.setCursor(0, 1);
    lcd.print("Labo 4a      ");
  }
}

void loop() {
  unsigned long currentTime = millis();
  static double distance = 0;

  if (currentTime - lastMeasureTime >= measureInterval) {
    lastMeasureTime = currentTime;
    distance = measureDistance();
  }
  updateState(distance);
  myStepper.run();

  if (currentTime - lastSerialTime >= serialInterval) {
    lastSerialTime = currentTime;
    displayLCD(distance);
    displaySerial(distance);
  }
}

#pragma endregion
