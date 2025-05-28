#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // RX, TX

Servo esc1, esc2, esc3, esc4;

const int minPower = 1000;         // Minimalna moc (dla bezpieczeństwa)
const int maxPower = 1800;         // Maksymalna moc silników
const int flightMinPower = 1200;   // Minimalna moc podczas lotu (żeby śmigła się nie zatrzymywały)
int currentPower = minPower;

bool upPressed = false;
bool armed = false;

const int ledPin = 8;

Adafruit_MPU6050 mpu;
char command;

float roll = 0.0;
float pitch = 0.0;
float rollOffset = 0.0;
float pitchOffset = 0.0;

const float alpha = 0.98; // filtr komplementarny

// PID - współczynniki
float Kp_roll = 5.0;
float Ki_roll = 0.2;
float Kd_roll = 0.8;
float integralRoll = 0.0;
float lastRoll = 0.0;

float Kp_pitch = 5.0;
float Ki_pitch = 0.2;
float Kd_pitch = 0.8;
float integralPitch = 0.0;
float lastPitch = 0.0;

unsigned long lastTime = 0;
unsigned long armTime = 0;
const unsigned long STARTUP_DURATION = 2000; // ms

void setup() {
  Serial.begin(115200);
  BTSerial.begin(9600);

  esc1.attach(5);   // Prawy dolny
  esc2.attach(6);   // Lewy górny
  esc3.attach(10);  // Prawy górny
  esc4.attach(9);   // Lewy dolny

  pinMode(ledPin, OUTPUT);
  setAllMotors(minPower);
  delay(3000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 ready.");

  delay(1000); // stabilizacja MPU6050

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  rollOffset = roll_acc;
  pitchOffset = pitch_acc;

  Serial.print("Roll offset: "); Serial.println(rollOffset);
  Serial.print("Pitch offset: "); Serial.println(pitchOffset);

  lastTime = micros();
}

void loop() {
  // Odczyt komend z Bluetooth
  if (BTSerial.available()) {
    command = BTSerial.read();
    if (command == 'U') {
      armed = true;
      upPressed = true;
      armTime = millis(); // zapisz czas uzbrojenia
      Serial.println("ARMED");
    }
    if (command == 'u') {
      upPressed = false;
    }
    if (command == 'Q') {
      armed = false;
      upPressed = false;
      currentPower = minPower;
      setAllMotors(minPower);
      Serial.println("DISARMED");
    }
  }

  // Sterowanie mocą
  if (armed) {
    if (upPressed && currentPower < maxPower) {
      currentPower += 2;
    } else if (!upPressed && currentPower > minPower) {
      currentPower -= 1;
    }
  } else {
    currentPower = minPower;
    setAllMotors(minPower);
  }

  // LED: włączony gdy uzbrojony i z mocą
  digitalWrite(ledPin, (armed && currentPower > minPower) ? HIGH : LOW);

  // Odczyt MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180 / PI;

  // Integracja gyro i filtr komplementarny
  roll += g.gyro.x * 180 / PI * dt;
  pitch += g.gyro.y * 180 / PI * dt;

  roll = alpha * roll + (1 - alpha) * (roll_acc - rollOffset);
  pitch = alpha * pitch + (1 - alpha) * (pitch_acc - pitchOffset);

  // PID dla roll
  integralRoll += roll * dt;
  float derivativeRoll = (roll - lastRoll) / dt;
  lastRoll = roll;
  int rollCorrection = (int)(-(Kp_roll * roll + Ki_roll * integralRoll + Kd_roll * derivativeRoll));
  rollCorrection = constrain(rollCorrection, -300, 300);

  // PID dla pitch
  integralPitch += pitch * dt;
  float derivativePitch = (pitch - lastPitch) / dt;
  lastPitch = pitch;
  int pitchCorrection = (int)(Kp_pitch * pitch + Ki_pitch * integralPitch + Kd_pitch * derivativePitch);
  pitchCorrection = constrain(pitchCorrection, -300, 300);

  // Obliczenie mocy na silniki (konfiguracja X)
  int powerRightBottom = constrain(currentPower - rollCorrection - pitchCorrection, flightMinPower, maxPower); // esc1 - pin 5
  int powerLeftTop     = constrain(currentPower + rollCorrection - pitchCorrection, flightMinPower, maxPower); // esc2 - pin 6
  int powerRightTop    = constrain(currentPower - rollCorrection + pitchCorrection, flightMinPower, maxPower); // esc3 - pin 10
  int powerLeftBottom  = constrain(currentPower + rollCorrection + pitchCorrection, flightMinPower, maxPower); // esc4 - pin 9

  // Start - równa moc przez chwilę po uzbrojeniu
  if (armed && (millis() - armTime < STARTUP_DURATION)) {
    setAllMotors(currentPower);
  } else if (armed) {
    esc1.writeMicroseconds(powerRightBottom); // pin 5
    esc2.writeMicroseconds(powerLeftTop);     // pin 6
    esc3.writeMicroseconds(powerRightTop);    // pin 10
    esc4.writeMicroseconds(powerLeftBottom);  // pin 9
  }

  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(" | Pitch: "); Serial.print(pitch, 2);
  Serial.print(" || RB: "); Serial.print(powerRightBottom);
  Serial.print(" LT: "); Serial.print(powerLeftTop);
  Serial.print(" RT: "); Serial.print(powerRightTop);
  Serial.print(" LB: "); Serial.println(powerLeftBottom);

  delay(15);
}

void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}