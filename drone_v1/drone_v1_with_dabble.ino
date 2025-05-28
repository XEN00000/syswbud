// #define CUSTOM_SETTINGS
// #define INCLUDE_GAMEPAD_MODULE
// #include <Dabble.h>

// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Servo.h>
// #include <SoftwareSerial.h>

// SoftwareSerial Bluetooth(2, 3); // RX, TX
// Servo esc1, esc2, esc3, esc4;

// const int minPower = 1000;
// const int maxPower = 1800;
// const int flightMinPower = 1200;
// int currentPower = minPower;

// int powerChange = 0;  // 1 = moc rośnie (triangle), -1 = moc maleje (cross), 0 = moc stabilna

// bool armed = false;

// const int ledPin = 8;

// Adafruit_MPU6050 mpu;

// float roll = 0.0;
// float pitch = 0.0;
// float rollOffset = 0.0;
// float pitchOffset = 0.0;

// const float alpha = 0.98;

// // PID - roll
// float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8;
// float integralRoll = 0.0, lastRoll = 0.0;

// // PID - pitch
// float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8;
// float integralPitch = 0.0, lastPitch = 0.0;

// unsigned long lastTime = 0;
// unsigned long armTime = 0;
// const unsigned long STARTUP_DURATION = 2000;

// void setup() {
//   Serial.begin(9600);
//   Bluetooth.begin(9600);
//   Dabble.begin(Bluetooth);

//   esc1.attach(5);
//   esc2.attach(6);
//   esc3.attach(10);
//   esc4.attach(9);

//   pinMode(ledPin, OUTPUT);
//   setAllMotors(minPower);
//   delay(3000);

//   if (!mpu.begin()) {
//     Serial.println("MPU6050 not found!");
//     while (1);
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   // Kalibracja offsetów - średnia z 100 pomiarów
//   const int calibrationSamples = 100;
//   float rollSum = 0.0;
//   float pitchSum = 0.0;

//   Serial.println("Calibrating sensors...");
//   for (int i = 0; i < calibrationSamples; i++) {
//     sensors_event_t a, g, temp;
//     mpu.getEvent(&a, &g, &temp);

//     float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
//     float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

//     rollSum += roll_acc;
//     pitchSum += pitch_acc;

//     delay(10);
//   }
//   rollOffset = rollSum / calibrationSamples;
//   pitchOffset = pitchSum / calibrationSamples;

//   Serial.print("rollOffset: ");
//   Serial.println(rollOffset);
//   Serial.print("pitchOffset: ");
//   Serial.println(pitchOffset);

//   lastTime = micros();
// }

// void loop() {
//   Dabble.processInput();

//   // Obsługa przycisków z Dabble Gamepad
//   if (GamePad.isTrianglePressed()) {
//     powerChange = 1;
//     if (!armed) {
//       armed = true;
//       armTime = millis();
//       Serial.println("ARMED (triangle)");
//     }
//   } else if (GamePad.isCrossPressed()) {
//     powerChange = -1;
//   } else {
//     powerChange = 0;
//   }

//   if (GamePad.isSelectPressed()) {
//     armed = false;
//     powerChange = 0;
//     currentPower = minPower;
//     setAllMotors(minPower);
//     Serial.println("DISARMED (select)");
//   }

//   if (armed) {
//     if (powerChange == 1 && currentPower < maxPower) {
//       currentPower += 2;  // zwiększ moc
//     } else if (powerChange == -1 && currentPower > minPower) {
//       currentPower -= 2;  // zmniejsz moc
//     }
//     // jeśli powerChange == 0 moc pozostaje na ostatnim poziomie (utrzymanie wysokości)
//   } else {
//     currentPower = minPower;
//     setAllMotors(minPower);
//   }

//   digitalWrite(ledPin, (armed && currentPower > minPower) ? HIGH : LOW);

//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   unsigned long now = micros();
//   float dt = (now - lastTime) / 1000000.0;
//   lastTime = now;

//   float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
//   float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

//   roll += g.gyro.x * 180.0 / PI * dt;
//   pitch += g.gyro.y * 180.0 / PI * dt;

//   roll = alpha * roll + (1.0 - alpha) * (roll_acc - rollOffset);
//   pitch = alpha * pitch + (1.0 - alpha) * (pitch_acc - pitchOffset);

//   // PID - roll
//   integralRoll += roll * dt;
//   float derivativeRoll = (roll - lastRoll) / dt;
//   lastRoll = roll;
//   int rollCorrection = (int)(-(Kp_roll * roll + Ki_roll * integralRoll + Kd_roll * derivativeRoll));
//   rollCorrection = constrain(rollCorrection, -300, 300);

//   // PID - pitch
//   integralPitch += pitch * dt;
//   float derivativePitch = (pitch - lastPitch) / dt;
//   lastPitch = pitch;
//   int pitchCorrection = (int)(Kp_pitch * pitch + Ki_pitch * integralPitch + Kd_pitch * derivativePitch);
//   pitchCorrection = constrain(pitchCorrection, -300, 300);

//   int powerRightBottom = constrain(currentPower - rollCorrection - pitchCorrection, flightMinPower, maxPower);
//   int powerLeftTop     = constrain(currentPower + rollCorrection - pitchCorrection, flightMinPower, maxPower);
//   int powerRightTop    = constrain(currentPower - rollCorrection + pitchCorrection, flightMinPower, maxPower);
//   int powerLeftBottom  = constrain(currentPower + rollCorrection + pitchCorrection, flightMinPower, maxPower);

//   if (armed && (millis() - armTime < STARTUP_DURATION)) {
//     setAllMotors(currentPower);
//   } else if (armed) {
//     esc1.writeMicroseconds(powerRightBottom);
//     esc2.writeMicroseconds(powerLeftTop);
//     esc3.writeMicroseconds(powerRightTop);
//     esc4.writeMicroseconds(powerLeftBottom);
//   }

//   delay(15);
// }

// void setAllMotors(int pwm) {
//   esc1.writeMicroseconds(pwm);
//   esc2.writeMicroseconds(pwm);
//   esc3.writeMicroseconds(pwm);
//   esc4.writeMicroseconds(pwm);
// }