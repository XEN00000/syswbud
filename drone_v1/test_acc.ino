// #include <Wire.h>
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Servo.h>
// #include <SoftwareSerial.h>

// SoftwareSerial BTSerial(2, 3); // RX, TX

// Servo esc1, esc2, esc3, esc4;
// const int minPower = 1000;
// const int maxPower = 1800;
// int currentPower = 1300;
// bool upPressed = false;

// const int ledPin = 8;

// Adafruit_MPU6050 mpu;

// char command;

// // Filtr komplementarny - do obliczania kąta z gyro i accel
// float roll = 0.0;
// float pitch = 0.0;

// const float alpha = 0.98;  // współczynnik filtra komplementarnego

// // Współczynniki P do korekcji kąta (dostosuj)
// float Kp_roll = 20.0;
// float Kp_pitch = 20.0;

// unsigned long lastTime = 0;

// void setup() {
//   Serial.begin(115200);
//   BTSerial.begin(9600);

//   esc1.attach(5);   // Prawy dol
//   esc2.attach(6);   // Lewy gora
//   esc3.attach(10);   // Prawy gora
//   esc4.attach(9);  // Lewy dol

//   pinMode(ledPin, OUTPUT);
//   setAllMotors(minPower);
//   delay(3000);

//   if (!mpu.begin()) {
//     Serial.println("MPU6050 not found!");
//     while (1) delay(10);
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   Serial.println("MPU6050 ready.");
//   lastTime = micros();
// }

// void loop() {
//   if (BTSerial.available()) {
//     command = BTSerial.read();
//     if (command == 'U') upPressed = true;
//     if (command == 'u') upPressed = false;
//   }

//   if (upPressed && currentPower < maxPower) {
//     currentPower += 2;
//   } else if (!upPressed && currentPower > minPower) {
//     currentPower -= 1;
//   }

//   digitalWrite(ledPin, currentPower > minPower ? HIGH : LOW);

//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   unsigned long now = micros();
//   float dt = (now - lastTime) / 1000000.0; // czas w sekundach
//   lastTime = now;

//   // Kąty z akcelerometru (w stopniach)
//   float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
//   float pitch_acc = atan2(-a.acceleration.x, sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * 180 / PI;

//   // Integracja gyro do kąta (w stopniach)
//   roll += g.gyro.x * 180 / PI * dt;  
//   pitch += g.gyro.y * 180 / PI * dt;

//   // Filtr komplementarny - łączymy gyro i accel
//   roll = alpha * roll + (1 - alpha) * roll_acc;
//   pitch = alpha * pitch + (1 - alpha) * pitch_acc;

//   // Obliczamy korekcję na podstawie kąta
// int rollCorrection = (int)(-Kp_roll * roll);
// int pitchCorrection = (int)(Kp_pitch * pitch);  // <-- odwrócony znak

// // Moc silników z korekcją
// int powerRightTop    = constrain(currentPower - rollCorrection - pitchCorrection, minPower, maxPower); // pin 9
// int powerRightBottom = constrain(currentPower - rollCorrection + pitchCorrection, minPower, maxPower); // pin 5
// int powerLeftTop     = constrain(currentPower + rollCorrection - pitchCorrection, minPower, maxPower); // pin 6
// int powerLeftBottom  = constrain(currentPower + rollCorrection + pitchCorrection, minPower, maxPower); // pin 10

//   esc1.writeMicroseconds(powerRightBottom); // pin 5
//   esc2.writeMicroseconds(powerLeftTop);     // pin 6
//   esc3.writeMicroseconds(powerRightTop);    // pin 9
//   esc4.writeMicroseconds(powerLeftBottom);  // pin 10

//   // Debug info
//   Serial.print("Roll: "); Serial.print(roll);
//   Serial.print(" | Pitch: "); Serial.print(pitch);
//   Serial.print(" || RTop: "); Serial.print(powerRightTop);
//   Serial.print(" RBot: "); Serial.print(powerRightBottom);
//   Serial.print(" | LTop: "); Serial.print(powerLeftTop);
//   Serial.print(" LBot: "); Serial.println(powerLeftBottom);

//   delay(30);
// }

// void setAllMotors(int pwm) {
//   esc1.writeMicroseconds(pwm);
//   esc2.writeMicroseconds(pwm);
//   esc3.writeMicroseconds(pwm);
//   esc4.writeMicroseconds(pwm);
// }