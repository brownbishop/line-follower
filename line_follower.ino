#include <QTRSensors.h>

// Motor pins
#define leftMotorIN1 12   // Direction control pin for left motor
#define leftMotorIN2 3    // PWM control pin for left motor
#define rightMotorIN1 13  // Direction control pin for right motor
#define rightMotorIN2 11  // PWM control pin for right motor
#define pwmMotorStanga 3    // PWM left
#define pwmMotorDreapta 11  // PWM right
//#define DEBUG 12

// Senzor de linie QTR-8RC
QTRSensors qtr;
unsigned char sensorPins[] = { 2, 4, 5, 6, 7, 8, 9, 10 };
unsigned int sensorValues[8];

// PID variables
constexpr float Kp = 0.04;
constexpr float Ki = 0.0001;
constexpr float Kd = 0.4;
constexpr float setpoint = 3500;  // center

float lastError = 0;
float integral = 0;
float baseSpeed = 30;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  pinMode(leftMotorIN1, OUTPUT);
  pinMode(leftMotorIN2, OUTPUT);
  pinMode(rightMotorIN1, OUTPUT);
  pinMode(rightMotorIN2, OUTPUT);

  qtr.setTypeRC();
  qtr.setSensorPins(sensorPins, 8);

  calibrateSensors();
}

void loop() {
  // black line position
  int position = qtr.readLineBlack(sensorValues);

  float error = setpoint - position;

  // PID computation
  float proportional = error;
  // integral += error;
  float derivative = error - lastError;
  float turn = Kp * proportional + Kd * derivative;

  //if (derivative == 0) {
  //  turn += 10;
  //}

#ifdef DEBUG
  Serial.print("position: ");
  Serial.print(position);
  Serial.print("derivative: ");
  Serial.print(derivative);
  Serial.print(" error: ");
  Serial.print(error);
  Serial.print(" turn: ");
  Serial.print(turn);
  Serial.print("\n");
#endif

  // Compute speed
  int leftMotorSpeed = baseSpeed + turn;
  int rightMotorSpeed = baseSpeed - turn;

  // limit the speed else the robot goes nuts
  leftMotorSpeed = constrain(leftMotorSpeed, -50, 50);
  rightMotorSpeed = constrain(rightMotorSpeed, -50, 50);

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);

  lastError = error;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(leftMotorIN1, HIGH);
    digitalWrite(leftMotorIN2, LOW);
  } else {
    digitalWrite(leftMotorIN1, LOW);
    digitalWrite(leftMotorIN2, HIGH);
  }

  if (rightSpeed > 0) {
    digitalWrite(rightMotorIN1, HIGH);
    digitalWrite(rightMotorIN2, LOW);
  } else {
    digitalWrite(rightMotorIN1, LOW);
    digitalWrite(rightMotorIN2, HIGH);
  }

  analogWrite(pwmMotorStanga, abs(leftSpeed));
  analogWrite(pwmMotorDreapta, abs(rightSpeed));
}

void calibrateSensors() {
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  int turnSpeed = 30;
  setMotorSpeed(turnSpeed, -turnSpeed);
  delay(50);
  for (uint16_t i = 0; i < 9; i++) {
    turnSpeed = -turnSpeed;
    setMotorSpeeds(turnSpeed, -turnSpeed);
    qtr.calibrate();
    delay(100);
  }

  turnSpeed = -turnSpeed;
  setMotorSpeeds(turnSpeed, -turnSpeed);
  delay(50);
  setMotorSpeeds(0, 0);
}
