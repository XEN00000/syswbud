#include <Servo.h>

Servo esc1;
Servo esc2;
Servo esc3;
Servo esc4;

void setup() {
  esc1.attach(3);
  esc2.attach(5);
  esc3.attach(6);
  esc4.attach(9);

  esc1.writeMicroseconds(1000); // na start silniki wyłączone
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  delay(3000); // poczekaj na uzbrojenie ESC
}

void loop() {
  esc1.writeMicroseconds(1200); // minimalne obroty
  esc2.writeMicroseconds(1200);
  esc3.writeMicroseconds(1200);
  esc4.writeMicroseconds(1200);
  
  delay(10000); // silniki kręcą się 10 sekund
}
