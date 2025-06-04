#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;

Servo esc1, esc2, esc3, esc4;

// PID zmienne
float pitch = 0.0, roll = 0.0;
float accAngleX = 0.0, accAngleY = 0.0;
float gyroXrate = 0.0, gyroYrate = 0.0;

float pitchZero = 0.0, rollZero = 0.0;            // <- zapamiętana "zerowa" pozycja
float pitchSetpoint = 0.0, rollSetpoint = 0.0;    // <- ustawiane później w setup()

float errorPitch = 0.0, errorRoll = 0.0;
float previousErrorPitch = 0.0, previousErrorRoll = 0.0;
float integralPitch = 0.0, integralRoll = 0.0;
float derivativePitch = 0.0, derivativeRoll = 0.0;

float Kp = 2.5, Ki = 0.0, Kd = 1.2;

float pidPitch = 0.0, pidRoll = 0.0;

unsigned long previousTime = 0;
float elapsedTime = 0.0;

int baseSpeed = 1000; // wartość ustawiana z Serial
int m1 = 0, m2 = 0, m3 = 0, m4 = 0;


void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  esc1.attach(5);
  esc2.attach(6);
  esc3.attach(9);
  esc4.attach(10);

  // Armowanie ESC
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  delay(2000); // czas na inicjalizację ESC

  // --- Pierwszy odczyt danych z MPU ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accAngleX = atan2(ax, az) * 180 / PI;       // poprawione dla obróconego MPU
  accAngleY = -atan2(ay, az) * 180 / PI;

  gyroXrate = gy / 131.0;
  gyroYrate = -gx / 131.0;

  // Inicjalny filtr
  pitch = accAngleX;
  roll  = accAngleY;

  // Zapisujemy tę pozycję jako "poziomą"
  pitchZero = pitch;
  rollZero  = roll;

  pitchSetpoint = pitchZero;
  rollSetpoint  = rollZero;

  Serial.println("Zerowanie ustawione:");
  Serial.print("PitchZero: "); Serial.print(pitchZero);
  Serial.print(" | RollZero: "); Serial.println(rollZero);

  previousTime = millis();
}

void loop() {
  // Odczyt z Serial: nowy throttle?
  if (Serial.available() > 0) {
    int input = Serial.parseInt();
    if (input >= 1000 && input <= 1700) {
      baseSpeed = input;
      Serial.print("Ustawiono bazową moc (baseSpeed): ");
      Serial.println(baseSpeed);
    } else {
      Serial.println("Nieprawidłowa wartość (dozwolone 1000–1700)");
    }
  }

  // 1. Czas
  unsigned long currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // 2. Odczyt MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // poprawka na obróconego MPU o 90° w prawo
  accAngleX = atan2(ax, az) * 180 / PI; // był ay → ax
  accAngleY = -atan2(ay, az) * 180 / PI; // był ax → ay z minusem

  gyroXrate = gy / 131.0;  // był gx → gy
  gyroYrate = -gx / 131.0; // był gy → gx z minusem

  // 3. Komplementarny filtr
  pitch = 0.98 * (pitch + gyroXrate * elapsedTime) + 0.02 * accAngleX;
  roll  = 0.98 * (roll  + gyroYrate * elapsedTime) + 0.02 * accAngleY;

  // 4. PID Pitch
  errorPitch = pitchSetpoint - pitch;
  integralPitch += errorPitch * elapsedTime;
  derivativePitch = (errorPitch - previousErrorPitch) / elapsedTime;
  pidPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
  previousErrorPitch = errorPitch;

  // 5. PID Roll
  errorRoll = rollSetpoint - roll;
  integralRoll += errorRoll * elapsedTime;
  derivativeRoll = (errorRoll - previousErrorRoll) / elapsedTime;
  pidRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;
  previousErrorRoll = errorRoll;

  // 6. Mikser – tylko pitch i roll (bez yaw)
  m1 = baseSpeed + pidPitch + pidRoll;  // M1 (CW)
  m2 = baseSpeed + pidPitch - pidRoll;  // M2 (CCW)
  m3 = baseSpeed - pidPitch + pidRoll;  // M3 (CCW)
  m4 = baseSpeed - pidPitch - pidRoll;  // M4 (CW)

  // 7. Ograniczenia i wysyłanie do ESC
  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  esc1.writeMicroseconds(m1);
  esc2.writeMicroseconds(m2);
  esc3.writeMicroseconds(m3);
  esc4.writeMicroseconds(m4);

  // 8. Debug
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print(" | Setpoint: "); Serial.print(pitchSetpoint);
  Serial.print(" | Roll: "); Serial.print(roll);
  Serial.print(" | baseSpeed: "); Serial.println(baseSpeed);

  //delay(); // 100 Hz
}
