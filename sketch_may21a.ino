#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(0, 1); // RX, TX — UWAGA: lepiej użyj np. 2 i 3!

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

char command;
const int minPower = 1000;
const int maxPower = 2000;
int currentPower = minPower;
bool upPressed = false;

const int ledPin = 8; // dioda podłączona do pinu 8

void setup() {
  Serial.begin(9600);
  BTSerial.begin(9600);

  esc1.attach(5);
  esc2.attach(6);
  esc3.attach(9);
  esc4.attach(10);

  pinMode(ledPin, OUTPUT);

  setAllMotors(minPower);
  delay(3000);
}

void loop() {
  if (BTSerial.available()) {
    command = BTSerial.read();
    Serial.print("Received: ");
    Serial.println(command);

    if (command == 'U') {
      upPressed = true;
    } else if (command == 'u') {
      upPressed = false;
    }
  }

  if (upPressed && currentPower < maxPower) {
    currentPower += 2;
    if (currentPower > maxPower) currentPower = maxPower;
    setAllMotors(currentPower);
  } else if (!upPressed && currentPower > minPower) {
    currentPower -= 1;
    if (currentPower < minPower) currentPower = minPower;
    setAllMotors(currentPower);
  }

  // Zapal diodę jeśli silniki się kręcą
  digitalWrite(ledPin, currentPower > minPower ? HIGH : LOW);

  delay(30);
}

void setAllMotors(int pwmMicroseconds) {
  esc1.writeMicroseconds(pwmMicroseconds);
  esc2.writeMicroseconds(pwmMicroseconds-35);
  esc3.writeMicroseconds(pwmMicroseconds);
  esc4.writeMicroseconds(pwmMicroseconds);

  Serial.print("ESC1: "); Serial.print(pwmMicroseconds); Serial.print(" | ");
  Serial.print("ESC2: "); Serial.print(pwmMicroseconds); Serial.print(" | ");
  Serial.print("ESC3: "); Serial.print(pwmMicroseconds); Serial.print(" | ");
  Serial.print("ESC4: "); Serial.println(pwmMicroseconds);
}