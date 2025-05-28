#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(2, 3); // RX, TX

Servo esc1, esc2, esc3, esc4;

const int minPower = 1000;
const int maxPower = 1750;
const int flightMinPower = 1200;
int currentPower = minPower;

int powerChange = 0; // 1 = zwiększ moc, -1 = zmniejsz moc, 0 = stabilna

bool armed = false;

const int ledPin = 8;

Adafruit_MPU6050 mpu;

float roll = 0.0;
float pitch = 0.0;
float rollOffset = 0.0;
float pitchOffset = 0.0;

const float alpha = 0.98;

// PID parametry dla roll
float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8;
float integralRoll = 0.0, lastRoll = 0.0;

// PID parametry dla pitch
float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8;
float integralPitch = 0.0, lastPitch = 0.0;

unsigned long lastTime = 0;
unsigned long armTime = 0;
const unsigned long STARTUP_DURATION = 2000;

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);

  esc1.attach(5); // prawy dol 
  esc2.attach(6); //lewa gora 
  esc3.attach(10); // prawa gora
  esc4.attach(9); // lewa dol

  pinMode(ledPin, OUTPUT);

  // setAllMotors(minPower);
  // delay(3000);
  for (int i = 0; i < 50; i++) {
    setAllMotors(minPower);
    delay(20);  // 20ms – typowy czas dla sygnału PWM (50Hz)
  }
  delay(1000); 

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Kalibracja offsetów (średnia z 100 pomiarów)
  const int calibrationSamples = 100;
  float rollSum = 0.0;
  float pitchSum = 0.0;
  Serial.println("Calibrating sensors...");
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    rollSum += roll_acc;
    pitchSum += pitch_acc;

    delay(10);
  }
  rollOffset = rollSum / calibrationSamples;
  pitchOffset = pitchSum / calibrationSamples;

  Serial.print("rollOffset: ");
  Serial.println(rollOffset);
  Serial.print("pitchOffset: ");
  Serial.println(pitchOffset);

  lastTime = micros();
}

void loop() {
  // Odczyt znaków z modułu Bluetooth
  while (Bluetooth.available()) {
    char c = Bluetooth.read();

    if (c == 'U') {
      // Przytrzymanie przycisku zwiększania mocy
      powerChange = 1;
      if (!armed) {
        armed = true;
        armTime = millis();
        Serial.println("ARMED (U)");
      }
    } else if (c == 'u') {
      // Puszczenie przycisku zwiększania mocy
      powerChange = 0;
      if (armed && currentPower < flightMinPower) {
        currentPower = flightMinPower;
        Serial.println("Power set to flightMinPower on U release");
      }
    } else if (c == 'D') {
      // Przytrzymanie przycisku zmniejszania mocy
      powerChange = -1;
      if (!armed) {
        armed = true;
        armTime = millis();
        Serial.println("ARMED (D)");
      }
    } else if (c == 'd') {
      // Puszczenie przycisku zmniejszania mocy
      powerChange = 0;
      if (armed && currentPower < flightMinPower) {
        currentPower = flightMinPower;
        Serial.println("Power set to flightMinPower on D release");
      }
   } else if (c == 'Q' || c == 'q') {
      // Disarm
      armed = false;
      powerChange = 0;
      currentPower = minPower;
      setAllMotors(minPower);

      // Reset PID
      integralRoll = 0.0;
      lastRoll = 0.0;
      integralPitch = 0.0;
      lastPitch = 0.0;
      roll = 0.0;
      pitch = 0.0;

      Serial.println("DISARMED & PID reset");
    }
  }

  // Zmiana mocy jeśli uzbrojony
  if (armed) {
    if (powerChange == 1 && currentPower < maxPower) {
      currentPower += 2; // zwiększ moc
    } else if (powerChange == -1 && currentPower > minPower) {
      currentPower -= 2; // zmniejsz moc
    }
  } else {
    currentPower = minPower;
    setAllMotors(minPower);
  }

  digitalWrite(ledPin, (armed && currentPower > minPower) ? HIGH : LOW);

  // Odczyt danych z MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  roll += g.gyro.x * 180.0 / PI * dt;
  pitch += g.gyro.y * 180.0 / PI * dt;

  roll = alpha * roll + (1.0 - alpha) * (roll_acc - rollOffset);
  pitch = alpha * pitch + (1.0 - alpha) * (pitch_acc - pitchOffset);

  // PID roll
  integralRoll += roll * dt;
  float derivativeRoll = (roll - lastRoll) / dt;
  lastRoll = roll;
  int rollCorrection = (int)(-(Kp_roll * roll + Ki_roll * integralRoll + Kd_roll * derivativeRoll));
  rollCorrection = constrain(rollCorrection, -300, 300);

  // PID pitch
  integralPitch += pitch * dt;
  float derivativePitch = (pitch - lastPitch) / dt;
  lastPitch = pitch;
  int pitchCorrection = (int)(Kp_pitch * pitch + Ki_pitch * integralPitch + Kd_pitch * derivativePitch);
  pitchCorrection = constrain(pitchCorrection, -300, 300);

  int powerRightBottom = constrain(currentPower - rollCorrection - pitchCorrection, flightMinPower, maxPower);
  int powerLeftTop     = constrain(currentPower + rollCorrection - pitchCorrection, flightMinPower, maxPower);
  int powerRightTop    = constrain(currentPower - rollCorrection + pitchCorrection, flightMinPower, maxPower);
  int powerLeftBottom  = constrain(currentPower + rollCorrection + pitchCorrection, flightMinPower, maxPower);

  if (armed && (millis() - armTime < STARTUP_DURATION)) {
    // Startowe ustawienie mocy
    setAllMotors(currentPower);
  } else if (armed) {
    esc1.writeMicroseconds(powerRightBottom);
    esc2.writeMicroseconds(powerLeftTop);
    esc3.writeMicroseconds(powerRightTop);
    esc4.writeMicroseconds(powerLeftBottom);
  }
    // Wysyłanie danych diagnostycznych
  Serial.print("roll:"); Serial.print(roll);
  Serial.print(", pitch:"); Serial.print(pitch);
  Serial.print(", rollCorrection:"); Serial.print(rollCorrection);
  Serial.print(", pitchCorrection:"); Serial.print(pitchCorrection);
  Serial.print(", powerRB:"); Serial.print(powerRightBottom);
  Serial.print(", powerLT:"); Serial.print(powerLeftTop);
  Serial.print(", powerRT:"); Serial.print(powerRightTop);
  Serial.print(", powerLB:"); Serial.println(powerLeftBottom);

  delay(15);
}

void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}