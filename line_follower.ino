#include <QTRSensors.h>  // Includem librăria QTRSensors

// Definire motor
#define motorStangaIN1 12   // Pin de control direcție pentru motorul stâng.
#define motorStangaIN2 3    // Pin de control PWM pentru motorul stâng.
#define motorDreaptaIN1 13  // Pin de control direcție pentru motorul drept.
#define motorDreaptaIN2 11  // Pin de control PWM pentru motorul drept.
#define pwmMotorStanga 3    // PWM motor stâng
#define pwmMotorDreapta 11  // PWM motor drept
//#define DEBUG 12

// Senzor de linie QTR-8RC
QTRSensors qtr;
unsigned char sensorPins[] = { 2, 4, 5, 6, 7, 8, 9, 10 };
unsigned int sensorValues[8];

// Variabile PID
constexpr float Kp = 0.04;
constexpr float Ki = 0.0001;
constexpr float Kd = 0.4;
constexpr float setpoint = 3500;  // Valoare pentru centru
float lastError = 0;
float integral = 0;
float baseSpeed = 30;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif
  // Setare pinii pentru motoare
  pinMode(motorStangaIN1, OUTPUT);
  pinMode(motorStangaIN2, OUTPUT);
  pinMode(motorDreaptaIN1, OUTPUT);
  pinMode(motorDreaptaIN2, OUTPUT);

  // Setare tip senzori și pinii aferenți
  qtr.setTypeRC();                   // Setăm tipul de senzor QTR-8RC
  qtr.setSensorPins(sensorPins, 8);  // Setăm pinii senzorilor

  // Calibrare senzori
  calibrateSensors();
}

void loop() {
  // Citirea valorilor senzorilor și obținerea poziției liniei negre
  int position = qtr.readLineBlack(sensorValues);  // Citim poziția liniei negre

  float error = setpoint - position;

  // Calcul PID
  float proportional = error;
  integral += error;
  float derivative = error - lastError;
  float turn = Kp * proportional + Ki * integral + Kd * derivative;

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
  // Calculare viteze motoare
  int leftMotorSpeed = baseSpeed + turn;
  int rightMotorSpeed = baseSpeed - turn;

  // Limitare viteze motoare
  leftMotorSpeed = constrain(leftMotorSpeed, -50, 50);
  rightMotorSpeed = constrain(rightMotorSpeed, -50, 50);

  // Setare viteze motoare
  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);

  lastError = error;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(motorStangaIN1, HIGH);
    digitalWrite(motorStangaIN2, LOW);
  } else {
    digitalWrite(motorStangaIN1, LOW);
    digitalWrite(motorStangaIN2, HIGH);
  }

  if (rightSpeed > 0) {
    digitalWrite(motorDreaptaIN1, HIGH);
    digitalWrite(motorDreaptaIN2, LOW);
  } else {
    digitalWrite(motorDreaptaIN1, LOW);
    digitalWrite(motorDreaptaIN2, HIGH);
  }

  analogWrite(pwmMotorStanga, abs(leftSpeed));
  analogWrite(pwmMotorDreapta, abs(rightSpeed));
}

void calibrateSensors() {
  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    if (i > 100 && i <= 300) {
        setMotorSpeeds(-10, 10);
    } else {
        setMotorSpeeds(10, -10);
    }
    qtr.calibrate();
  }
}
