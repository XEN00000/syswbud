#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <MPU6050_light.h>

#define DEG_TO_RAD 0.01745329251

SoftwareSerial BTSerial(0, 1); // RX, TX — unikamy 0 i 1 (USB)

Servo esc1, esc2, esc3, esc4;
MPU6050 mpu(Wire);

// Funkcja ustawiania PWM dla wszystkich ESC
void setAllMotors(int pwmMicroseconds);

char command;
const int minPower = 1000;
const int maxPower = 2000;
int currentPower = minPower;
bool upPressed = false;

const int ledPin = 8;

float angleX = 0;
float angleY = 0;
float angleZ = 0;
float kP = 3.0; // Wzmocnienie korekcji

// Obrót danych IMU o -45° względem drona
void transformAxes(float x, float y, float &xOut, float &yOut) {
  float angleRad = -45.0 * DEG_TO_RAD;
  float cosA = cos(angleRad);
  float sinA = sin(angleRad);

  xOut = x * cosA - y * sinA;
  yOut = x * sinA + y * cosA;
}

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  esc1.attach(5);
  esc2.attach(6);
  esc3.attach(9);
  esc4.attach(10);
  pinMode(ledPin, OUTPUT);

  Wire.begin();
  mpu.begin();
  mpu.calcGyroOffsets();

  Serial.println("Kalibracja poziomu...");
  delay(2000);

  setAllMotors(minPower); // Rozruch ESC
  delay(3000); // Czas na uzbrojenie ESC
}

void loop() {
  // Bluetooth input
  if (BTSerial.available()) {
    command = BTSerial.read();
    if (command == 'U') upPressed = true;
    else if (command == 'u') upPressed = false;
  }

  // Sterowanie mocą
  if (upPressed && currentPower < maxPower) {
    currentPower += 2;
    if (currentPower > maxPower) currentPower = maxPower;
  } else if (!upPressed && currentPower > minPower) {
    currentPower -= 1;
    if (currentPower < minPower) currentPower = minPower;
  }

  // Odczyt IMU
  mpu.update();

  float rawX = mpu.getAngleX();
  float rawY = mpu.getAngleY();
  angleZ = mpu.getAngleZ();

  // Transformacja osi X i Y
  transformAxes(rawX, rawY, angleX, angleY);

  // Korekcja tylko dla osi X i Y (pitch i roll)
  int correctionX = kP * angleX;
  int correctionY = kP * angleY;

  // Wyznaczenie mocy dla każdego silnika
  int esc1Power = constrain(currentPower - correctionX - correctionY, minPower, maxPower); // Front-Left
  int esc2Power = constrain(currentPower - correctionX + correctionY, minPower, maxPower); // Front-Right
  int esc3Power = constrain(currentPower + correctionX - correctionY, minPower, maxPower); // Back-Left
  int esc4Power = constrain(currentPower + correctionX + correctionY, minPower, maxPower); // Back-Right

  // Ustawienie silników
  esc1.writeMicroseconds(esc1Power);
  esc2.writeMicroseconds(esc2Power);
  esc3.writeMicroseconds(esc3Power);
  esc4.writeMicroseconds(esc4Power);

  // LED status
  digitalWrite(ledPin, currentPower > minPower ? HIGH : LOW);

  // Debug: X, Y, Z po transformacji
  Serial.print("Power: "); Serial.print(currentPower);
  Serial.print(" | X: "); Serial.print(angleX, 1);
  Serial.print(" | Y: "); Serial.print(angleY, 1);
  Serial.print(" | Z: "); Serial.println(angleZ, 1);

  delay(10);
}

void setAllMotors(int pwmMicroseconds) {
  esc1.writeMicroseconds(pwmMicroseconds);
  esc2.writeMicroseconds(pwmMicroseconds);
  esc3.writeMicroseconds(pwmMicroseconds);
  esc4.writeMicroseconds(pwmMicroseconds);
}