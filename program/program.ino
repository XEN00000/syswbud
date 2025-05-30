#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// === KONFIGURACJA ===
const int RX_PIN = 2;
const int TX_PIN = 3;
const int LED_PIN = 8;

// ESC Piny
const int ESC1_PIN = 5; // prawy dol
const int ESC2_PIN = 6; // lewa gora
const int ESC3_PIN = 10; // prawa gora
const int ESC4_PIN = 9; // lewa dol

// Moc silników
const int MIN_POWER = 1000;
const int MAX_POWER = 1750;
const int FLIGHT_MIN_POWER = 1200;
const int STABLE_FLIGHT_POWER = 1650;

// PID
const float ALPHA = 0.98;
const float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8;
const float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8;
const unsigned long STARTUP_DURATION = 500;

//ZMIENNE 
SoftwareSerial Bluetooth(RX_PIN, TX_PIN);
Servo esc1, esc2, esc3, esc4;
Adafruit_MPU6050 mpu;

int currentPower = MIN_POWER;
int powerChange = 0;
int powerStep = 10;
bool armed = false;

float rollOffset = 0.0, pitchOffset = 0.0;
float roll = 0.0, pitch = 0.0;
float integralRoll = 0.0, lastRoll = 0.0;
float integralPitch = 0.0, lastPitch = 0.0;

unsigned long lastTime = 0;
unsigned long armTime = 0;

// Deklaracje funkcji
void initializeESCs();
void calibrateIMU();
void readIMU(float &rollOut, float &pitchOut, sensors_event_t &a, sensors_event_t &g, float dt);
int computePIDRoll(float rollValue, float dt);
int computePIDPitch(float pitchValue, float dt);
void applyMotorPower(int rollCorr, int pitchCorr); 
void resetPID();
void handleBluetoothCommand(char c);
void setAllMotors(int pwm);


// === SETUP ===
void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);
  pinMode(LED_PIN, OUTPUT);

  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);

  initializeESCs();
  calibrateIMU();
  lastTime = micros();
}

// === GŁÓWNA PĘTLA ===
void loop() {
  while (Bluetooth.available()) {
    char c = Bluetooth.read();
    handleBluetoothCommand(c);
  }

  if (armed) {
    if (powerChange == 1 && currentPower < MAX_POWER) currentPower += powerStep;
    else if (powerChange == -1 && currentPower > MIN_POWER) currentPower -= powerStep;
  } else {
    currentPower = MIN_POWER;
    setAllMotors(MIN_POWER);
  }

  digitalWrite(LED_PIN, (armed && currentPower > MIN_POWER) ? HIGH : LOW);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6;
  lastTime = now;

  readIMU(roll, pitch, a, g, dt);

  int rollCorrection = computePIDRoll(roll, dt);
  int pitchCorrection = computePIDPitch(pitch, dt);

  applyMotorPower(rollCorrection, pitchCorrection);

  Serial.print("roll:"); Serial.print(roll);
  Serial.print(", pitch:"); Serial.print(pitch);
  Serial.print(", rollCorrection:"); Serial.print(rollCorrection);
  Serial.print(", pitchCorrection:"); Serial.print(pitchCorrection);
  Serial.print(", power:"); Serial.println(currentPower);

  delay(15);
}

// === FUNKCJE POMOCNICZE ===
void initializeESCs() {
  for (int i = 0; i < 50; i++) {
    setAllMotors(MIN_POWER);
    delay(20);
  }
  delay(1000);
}

void calibrateIMU() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  const int samples = 100;
  float rollSum = 0.0, pitchSum = 0.0;
  Serial.println("Calibrating sensors...");

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch_acc = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

    rollSum += roll_acc;
    pitchSum += pitch_acc;

    delay(10);
  }

  rollOffset = rollSum / samples;
  pitchOffset = pitchSum / samples;

  Serial.print("rollOffset: "); Serial.println(rollOffset);
  Serial.print("pitchOffset: "); Serial.println(pitchOffset);
}

void readIMU(float &rollOut, float &pitchOut, sensors_event_t &a, sensors_event_t &g, float dt) {
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

  roll += g.gyro.x * 180.0 / PI * dt;
  pitch += g.gyro.y * 180.0 / PI * dt;

  rollOut = ALPHA * roll + (1 - ALPHA) * (roll_acc - rollOffset);
  pitchOut = ALPHA * pitch + (1 - ALPHA) * (pitch_acc - pitchOffset);
}

int computePIDRoll(float rollValue, float dt) {
  integralRoll += rollValue * dt;
  float derivative = (rollValue - lastRoll) / dt;
  lastRoll = rollValue;
  return constrain(-(Kp_roll * rollValue + Ki_roll * integralRoll + Kd_roll * derivative), -300, 300);
}

int computePIDPitch(float pitchValue, float dt) {
  integralPitch += pitchValue * dt;
  float derivative = (pitchValue - lastPitch) / dt;
  lastPitch = pitchValue;
  return constrain(Kp_pitch * pitchValue + Ki_pitch * integralPitch + Kd_pitch * derivative, -300, 300);
}

void applyMotorPower(int rollCorr, int pitchCorr) {
  if (armed && (millis() - armTime < STARTUP_DURATION)) {
    setAllMotors(currentPower);
  } else if (armed) {
    esc1.writeMicroseconds(constrain(currentPower - rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // RB
    esc2.writeMicroseconds(constrain(currentPower + rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // LT
    esc3.writeMicroseconds(constrain(currentPower - rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // RT
    esc4.writeMicroseconds(constrain(currentPower + rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // LB
  }
}

void resetPID() {
  integralRoll = 0.0; lastRoll = 0.0;
  integralPitch = 0.0; lastPitch = 0.0;
  roll = 0.0; pitch = 0.0;
}

void handleBluetoothCommand(char c) {
  if (c == 'U') {
    powerChange = 1;
    if (!armed) {
      armed = true;
      armTime = millis();
      Serial.println("ARMED (U)");
    }
  } else if (c == 'u') {
    powerChange = 0;
    if (armed) {
      currentPower = STABLE_FLIGHT_POWER;
      Serial.println("Power locked on U release");
    }
  } else if (c == 'D') {
    powerChange = -1;
    if (!armed) {
      armed = true;
      armTime = millis();
      Serial.println("ARMED (D)");
    }
  } else if (c == 'd') {
    powerChange = 0;
    if (armed) {
      currentPower = STABLE_FLIGHT_POWER;
      Serial.println("Power locked on D release");
    }
  } else if (c == 'Q' || c == 'q') {
    armed = false;
    powerChange = 0;
    currentPower = MIN_POWER;
    setAllMotors(MIN_POWER);
    resetPID();
    Serial.println("DISARMED & PID reset");
  }
}

void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}
