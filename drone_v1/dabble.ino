#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#define INCLUDE_TERMINAL_MODULE
#include <Dabble.h>
#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(2, 3); // RX, TX (do HC-05)

void setup() {
  Serial.begin(9600);        // Monitor szeregowy (USB)
  Bluetooth.begin(9600);     // Bluetooth HC-05
  Dabble.begin(Bluetooth);   // Inicjalizacja Dabble
}

void loop() {
  Dabble.processInput();     // Odbiór danych z aplikacji

  // Terminal
  if (Terminal.available()) {
    String data = Terminal.readString();
    Serial.print("Terminal: ");
    Serial.println(data);
  }

  // Gamepad - przyciski
  if (GamePad.isUpPressed())
    Serial.println("Góra (↑) wciśnięta");
  if (GamePad.isDownPressed())
    Serial.println("Dół (↓) wciśnięta");
  if (GamePad.isLeftPressed())
    Serial.println("Lewo (←) wciśnięte");
  if (GamePad.isRightPressed())
    Serial.println("Prawo (→) wciśnięte");

  if (GamePad.isSquarePressed())
    Serial.println("Kwadrat (☐) wciśnięty");
  if (GamePad.isCirclePressed())
    Serial.println("Kółko (○) wciśnięte");
  if (GamePad.isCrossPressed())
    Serial.println("Krzyżyk (X) wciśnięty");
  if (GamePad.isTrianglePressed())
    Serial.println("Trójkąt (△) wciśnięty");

  if (GamePad.isStartPressed())
    Serial.println("Start wciśnięty");
  if (GamePad.isSelectPressed())
    Serial.println("Select wciśnięty");

  delay(100); // Dla czytelności wyjścia
}
