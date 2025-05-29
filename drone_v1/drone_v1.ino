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
const int powetToStayInAir = 1650; // Zwiększona moc bazowa
int currentPower = minPower;

bool holdPositionMode = false;
int powerChange = 0; // 1 = zwiększ moc, -1 = zmniejsz moc, 0 = stabilna
int powerStep = 10;

bool armed = false;

const int ledPin = 8;

Adafruit_MPU6050 mpu;

float roll = 0.0;
float pitch = 0.0;
float rollOffset = 0.0;
float pitchOffset = 0.0;

// Trim offsety do kompensacji mechanicznych niesymetrii
float rollTrim = 5.0;  // Kompensacja drифtu w prawo (możesz dostosować)
float pitchTrim = 0.0;

const float alpha = 0.95; // Zwiększone filtrowanie

// Poprawione parametry PID dla roll - bardziej konserwatywne
float Kp_roll = 1.8, Ki_roll = 0.02, Kd_roll = 0.6;
float integralRoll = 0.0, lastRoll = 0.0;

// Poprawione parametry PID dla pitch - bardziej konserwatywne  
float Kp_pitch = 1.8, Ki_pitch = 0.02, Kd_pitch = 0.6;
float integralPitch = 0.0, lastPitch = 0.0;

// Ograniczenia dla składników PID
const float MAX_INTEGRAL = 50.0;
const float MAX_CORRECTION = 250;

unsigned long lastTime = 0;
unsigned long armTime = 0;
const unsigned long STARTUP_DURATION = 1000; // Wydłużony czas startu

void setup() {
  Serial.begin(9600);
  Bluetooth.begin(9600);

  esc1.attach(5); // prawy dol 
  esc2.attach(6); // lewa gora 
  esc3.attach(10); // prawa gora
  esc4.attach(9); // lewa dol

  pinMode(ledPin, OUTPUT);

  // Inicjalizacja ESC - wydłużona i ulepszona
  Serial.println("Inicjalizacja ESC...");
  for (int i = 0; i < 100; i++) {
    setAllMotors(minPower);
    delay(20);
  }
  delay(2000); 

  if (!mpu.begin()) {
    Serial.println("MPU6050 nie znaleziony!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ); // Lepsze filtrowanie

  // Ulepszona kalibracja offsetów
  const int calibrationSamples = 200; // Więcej próbek
  float rollSum = 0.0;
  float pitchSum = 0.0;
  Serial.println("Kalibracja czujników... Upewnij się że dron jest poziomo!");
  
  // Odczekaj 2 sekundy przed kalibracją
  delay(2000);
  
  for (int i = 0; i < calibrationSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

    rollSum += roll_acc;
    pitchSum += pitch_acc;

    if (i % 20 == 0) {
      Serial.print("Kalibracja: ");
      Serial.print((i * 100) / calibrationSamples);
      Serial.println("%");
    }

    delay(10);
  }
  
  rollOffset = rollSum / calibrationSamples;
  pitchOffset = pitchSum / calibrationSamples;

  Serial.print("rollOffset: ");
  Serial.println(rollOffset);
  Serial.print("pitchOffset: ");
  Serial.println(pitchOffset);
  Serial.println("Kalibracja zakończona!");

  lastTime = micros();
}

void loop() {
  // Odczyt komend z Bluetooth
  while (Bluetooth.available()) {
    char c = Bluetooth.read();

    if (c == 'U') {
      powerChange = 1;
      if (!armed) {
        armed = true;
        armTime = millis();
        resetPID(); // Reset PID przy uzbrójeniu
        Serial.println("UZBROJONY (U)");
      }
      holdPositionMode = false;
    } else if (c == 'u') {
      powerChange = 0;
      if (armed) {
        currentPower = powetToStayInAir;
        holdPositionMode = true;
        Serial.println("Moc zablokowana na " + String(powetToStayInAir) + " - tryb utrzymania pozycji WŁĄCZONY");
      }
    } else if (c == 'D') {
      powerChange = -1;
      if (!armed) {
        armed = true;
        armTime = millis();
        resetPID(); // Reset PID przy uzbrójeniu
        Serial.println("UZBROJONY (D)");
      }
      holdPositionMode = false;
    } else if (c == 'd') {
      powerChange = 0;
      if (armed) {
        currentPower = powetToStayInAir;
        holdPositionMode = true;
        Serial.println("Moc zablokowana na " + String(powetToStayInAir) + " - tryb utrzymania pozycji WŁĄCZONY");
      }
    } else if (c == 'Q' || c == 'q') {
      disarmDrone();
    } else if (c == 'L') {
      // Trim w lewo (zmniejsz rollTrim)
      rollTrim -= 0.5;
      Serial.println("Trim w lewo: " + String(rollTrim));
    } else if (c == 'R') {
      // Trim w prawo (zwiększ rollTrim)
      rollTrim += 0.5;
      Serial.println("Trim w prawo: " + String(rollTrim));
    } else if (c == 'F') {
      // Trim do przodu (zmniejsz pitchTrim)
      pitchTrim -= 0.5;
      Serial.println("Trim do przodu: " + String(pitchTrim));
    } else if (c == 'B') {
      // Trim do tyłu (zwiększ pitchTrim)
      pitchTrim += 0.5;
      Serial.println("Trim do tyłu: " + String(pitchTrim));
    } else if (c == 'T') {
      // Wyzeruj trim
      rollTrim = 0.0;
      pitchTrim = 0.0;
      Serial.println("Trim wyzerowany");
    }
  }

  // Kontrola mocy
  if (armed) {
    if (powerChange == 1 && currentPower < maxPower) {
      currentPower += powerStep;
    } else if (powerChange == -1 && currentPower > flightMinPower) {
      currentPower -= powerStep;
    }
    
    // Ograniczenie minimalnej mocy podczas lotu
    if (currentPower < flightMinPower) {
      currentPower = flightMinPower;
    }
  } else {
    currentPower = minPower;
    setAllMotors(minPower);
  }

  // Kontrola LED
  digitalWrite(ledPin, (armed && currentPower > minPower) ? HIGH : LOW);

  // Odczyt z MPU6050 i obliczenia PID tylko gdy uzbrojony
  if (armed) {
    updateIMUAndPID();
  }

  delay(10); // Zmniejszony delay dla lepszej responsywności
}

void updateIMUAndPID() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Ograniczenie dt dla stabilności
  if (dt > 0.1) dt = 0.1;

  // Obliczenie kątów z akcelerometru
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Integracja żyroskopu
  roll += g.gyro.x * 180.0 / PI * dt;
  pitch += g.gyro.y * 180.0 / PI * dt;

  // Filtr komplementarny z poprawkami offsetu
  roll = alpha * roll + (1.0 - alpha) * (roll_acc - rollOffset);
  pitch = alpha * pitch + (1.0 - alpha) * (pitch_acc - pitchOffset);

  // Kontroler PID dla roll z trim
  float rollError = roll - rollTrim;  // Zastosowanie trim
  integralRoll += rollError * dt;
  integralRoll = constrain(integralRoll, -MAX_INTEGRAL, MAX_INTEGRAL); // Ochrona przed integral windup
  
  float derivativeRoll = (rollError - lastRoll) / dt;
  lastRoll = rollError;
  
  int rollCorrection = (int)(-(Kp_roll * rollError + Ki_roll * integralRoll + Kd_roll * derivativeRoll));
  rollCorrection = constrain(rollCorrection, -MAX_CORRECTION, MAX_CORRECTION);

  // Kontroler PID dla pitch z trim
  float pitchError = pitch - pitchTrim;  // Zastosowanie trim
  integralPitch += pitchError * dt;
  integralPitch = constrain(integralPitch, -MAX_INTEGRAL, MAX_INTEGRAL); // Ochrona przed integral windup
  
  float derivativePitch = (pitchError - lastPitch) / dt;
  lastPitch = pitchError;
  
  int pitchCorrection = (int)(Kp_pitch * pitchError + Ki_pitch * integralPitch + Kd_pitch * derivativePitch);
  pitchCorrection = constrain(pitchCorrection, -MAX_CORRECTION, MAX_CORRECTION);

  // Obliczenie mocy dla każdego silnika
  int powerRightBottom = constrain(currentPower - rollCorrection - pitchCorrection, flightMinPower, maxPower);
  int powerLeftTop     = constrain(currentPower + rollCorrection - pitchCorrection, flightMinPower, maxPower);
  int powerRightTop    = constrain(currentPower - rollCorrection + pitchCorrection, flightMinPower, maxPower);
  int powerLeftBottom  = constrain(currentPower + rollCorrection + pitchCorrection, flightMinPower, maxPower);

  // Stopniowe uruchamianie - pierwszą sekundę jednakowa moc na wszystkie silniki
  if (millis() - armTime < STARTUP_DURATION) {
    setAllMotors(currentPower);
    Serial.println("Rozruch - moc: " + String(currentPower));
  } else {
    // Normalne działanie z PID
    esc1.writeMicroseconds(powerRightBottom);
    esc2.writeMicroseconds(powerLeftTop);
    esc3.writeMicroseconds(powerRightTop);
    esc4.writeMicroseconds(powerLeftBottom);
  }

  // Logowanie w trybie utrzymania pozycji (zmniejszona częstotliwość)
  static unsigned long lastLogTime = 0;
  if (holdPositionMode && (millis() - lastLogTime > 100)) { // Log co 100ms
    lastLogTime = millis();
    Serial.print("R:"); Serial.print(roll, 1);
    Serial.print(", P:"); Serial.print(pitch, 1);
    Serial.print(", RC:"); Serial.print(rollCorrection);
    Serial.print(", PC:"); Serial.print(pitchCorrection);
    Serial.print(", PWR[RB:"); Serial.print(powerRightBottom);
    Serial.print(", LT:"); Serial.print(powerLeftTop);
    Serial.print(", RT:"); Serial.print(powerRightTop);
    Serial.print(", LB:"); Serial.print(powerLeftBottom);
    Serial.println("]");
    
    // Dodatkowe ostrzeżenia
    if (abs(roll) > 20 || abs(pitch) > 20) {
      Serial.println("UWAGA: Duże odchylenie kątowe!");
    }
  }
}

void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}

void resetPID() {
  integralRoll = 0.0;
  lastRoll = 0.0;
  integralPitch = 0.0;
  lastPitch = 0.0;
  roll = 0.0;
  pitch = 0.0;
  Serial.println("PID zresetowany");
}

void disarmDrone() {
  armed = false;
  powerChange = 0;
  currentPower = minPower;
  setAllMotors(minPower);
  resetPID();
  holdPositionMode = false;
  Serial.println("ROZBROJONY - wszystkie systemy zresetowane");
}