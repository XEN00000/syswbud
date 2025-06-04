#include <Wire.h>          // Wymagane do komunikacji I2C
#include <MPU6050_light.h> // Biblioteka dla czujnika IMU MPU6050
#include <Servo.h>         // Biblioteka do sterowania ESC (Electronic Speed Controllers)
// #include <SoftwareSerial.h> // Usunięta - używamy HardwareSerial (Serial)

// === KONFIGURACJA ===
// Definicje pinów
// Komunikacja Bluetooth będzie teraz używać sprzętowego UART na pinach 0 (RX) i 1 (TX).
// Pamiętaj o dzielnikach napięcia dla HC-05 TX -> Arduino RX (Pin 0) i HC-05 RX -> Arduino TX (Pin 1)!
const int LED_PIN = 8;       // Pin diody LED (dla wizualizacji stanu)
const int BT_STATE_PIN = 2;  // Pin STATE z HC-05 podłączony do pinu 2 Arduino (dla przerwania zewnętrznego INT0).
                             // Pamiętaj o dzielniku napięcia dla HC-05 STATE -> Arduino Pin 2!

// Przypisania pinów ESC (dostosuj w zależności od układu silników drona)
// Te komentarze opisują fizyczne położenie silników na dronie
// zakładając konfigurację 'X'.
const int ESC1_PIN = 5;  // Tylny Prawy (prawy dol)
const int ESC2_PIN = 6;  // Przedni Lewy (lewa gora)
const int ESC3_PIN = 10; // Przedni Prawy (prawa gora)
const int ESC4_PIN = 9;  // Tylny Lewy (lewa dol)

// Ustawienia Mocy Silników (w mikrosekundach, dla ESC)
const int MIN_POWER = 1000;         // Minimalny sygnał do ESC (silniki wyłączone/jałowe)
const int MAX_POWER = 1350;         // Maksymalny sygnał do ESC (pełny gaz)
const int FLIGHT_MIN_POWER = 1200;  // Minimalna moc dla stabilnego lotu (silniki się kręcą)

// PID (Proportional-Integral-Derivative) Stałe Kontrolera dla Trybu Lotu
const float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8;   // Stałe PID dla osi Roll
const float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8; // Stałe PID dla osi Pitch

// Stałe Trybu Demonstracji Akcelerometru
const int DEMO_BASE_POWER = 1150;    // Podstawowa moc dla silników w trybie demo
const float DEMO_KP_ROLL = 15.0;     // Współczynnik P dla roll w trybie demo
const float DEMO_KP_PITCH = 15.0;    // Współczynnik P dla pitch w trybie demo (odwrócony znak)

// Uruchamianie i Uzbrajanie
const unsigned long STARTUP_DURATION = 500; // Czas trwania po uzbrojeniu przed pełnym sterowaniem PID

// === ZMIENNE GLOBALNE ===
// Usunięto: SoftwareSerial Bluetooth(RX_PIN, TX_PIN); - używamy wbudowanego obiektu Serial
Servo esc1, esc2, esc3, esc4;             // Obiekty Servo do sterowania ESC
MPU6050 mpu(Wire);                        // Obiekt czujnika MPU6050

// Zmienne sterowania lotem
int currentPower = MIN_POWER; // Aktualny poziom gazu dla wszystkich silników
int powerChange = 0;          // -1 dla zmniejszenia, 0 dla braku zmiany, 1 dla zwiększenia
int powerStep = 10;           // Krok zwiększania/zmniejszania gazu
bool armed = false;           // Prawda, jeśli silniki są uzbrojone i gotowe do lotu (tylko dla FLIGHT_MODE)
bool demo_motors_active = false; // Prawda, jeśli silniki są aktywne w trybie demo

// Zmienne stanu IMU i PID
float roll = 0.0, pitch = 0.0;             // Aktualne kąty roll i pitch (stopnie)
float integralRoll = 0.0, lastRoll = 0.0;  // Całka i poprzedni błąd dla PID Roll
float integralPitch = 0.0, lastPitch = 0.0; // Całka i poprzedni błąd dla PID Pitch

unsigned long lastTime = 0; // Znacznik czasu ostatniej iteracji pętli (do obliczania dt)
unsigned long armTime = 0;  // Znacznik czasu uzbrojenia drona

// Zmienne dla funkcji Fail-Safe (bezpieczne lądowanie) z IRQ
volatile boolean bluetooth_connected_ISR = false; // Aktualny stan połączenia Bluetooth (zmienna dla ISR)
const int LANDING_POWER_DECREASE_STEP = 5; // Krok zmniejszania mocy podczas lądowania awaryjnego

// Tryby Działania
#define FLIGHT_MODE 0       // Normalny tryb lotu ze stabilizacją PID
#define ACCEL_DEMO_MODE 1   // Tryb demonstracji akcelerometru
int currentOperatingMode = ACCEL_DEMO_MODE; // Domyślny tryb po uruchomieniu

// === DEKLARACJE FUNKCJI ===
void initializeESCs();
void calibrateIMU();
void readIMU(float &rollOut, float &pitchOut, float dt);
int computePIDRoll(float rollValue, float dt);
int computePIDPitch(float pitchValue, float dt);
void applyMotorPower(int rollCorr, int pitchCorr);
void resetPID();
void handleBluetoothCommand(char c);
void setAllMotors(int pwm);
void switchMode(int newMode);
void disarmMotors(); // Funkcja do rozbrajania silników
void initiateFailSafeLanding(); // Funkcja do inicjowania bezpiecznego lądowania

// === FUNKCJA OBSŁUGI PRZERWANIA (ISR) ===
// Ta funkcja jest wywoływana automatycznie, gdy stan na BT_STATE_PIN (pin 2) się zmienia.
void bluetoothStateChange() {
  bluetooth_connected_ISR = digitalRead(BT_STATE_PIN) == HIGH; // Odczytaj nowy stan
  // Używamy tylko dla debugowania podczas rozwoju.
  if (bluetooth_connected_ISR) {
    // Serial.println(F("ISR: BLUETOOTH: Ponownie połączono!"));
  } else {
    // Serial.println(F("ISR: BLUETOOTH: Utracono połączenie!"));
  }
}

// === SETUP ===
void setup() {
  // Inicjalizacja sprzętowego UART (Serial) dla komunikacji z komputerem (debugowanie)
  // i jednocześnie dla modułu Bluetooth.
  // Pamiętaj, że port jest współdzielony!
  Serial.begin(9600); // Standardowa prędkość dla debugowania na monitorze szeregowym
  
  // Ustawienie prędkości dla modułu Bluetooth.
  // UWAGA: Upewnij się, że Twój moduł HC-05 jest skonfigurowany na tę samą prędkość (np. 9600, 19200, 38400).
  // Prędkość 9900 jest bardzo nietypowa i prawdopodobnie będziesz musiał ją zmienić.
  Serial.begin(9900); // Prędkość komunikacji z modułem HC-05

  pinMode(LED_PIN, OUTPUT);     // Ustaw pin LED jako wyjście
  pinMode(BT_STATE_PIN, INPUT); // Ustaw pin BT_STATE jako wejście

  // Konfiguracja przerwania zewnętrznego dla pinu BT_STATE_PIN (pin 2)
  // Przerwanie będzie wyzwolone przy każdej zmianie stanu (CHANGE) pinu.
  attachInterrupt(digitalPinToInterrupt(BT_STATE_PIN), bluetoothStateChange, CHANGE);

  // Podłącz ESC do odpowiednich pinów serwo
  esc1.attach(ESC1_PIN);
  esc2.attach(ESC2_PIN);
  esc3.attach(ESC3_PIN);
  esc4.attach(ESC4_PIN);

  // Inicjalizacja ESC poprzez wysłanie niskiego sygnału na krótki okres
  initializeESCs();

  // Inicjalizacja i kalibracja czujnika MPU6050
  calibrateIMU();

  lastTime = micros();

  Serial.println(F("System drona gotowy!"));
  Serial.print(F("Aktualny tryb: "));
  Serial.println(currentOperatingMode == FLIGHT_MODE ? F("FLIGHT_MODE") : F("ACCEL_DEMO_MODE"));
  Serial.println(F("Wyślij 'F' dla Trybu Lotu, 'A' dla Trybu Demonstracji Akcelerometru."));
  Serial.println(F("W Trybie Lotu: 'U' aby zwiększyć moc, 'u' aby zablokować moc (lub zmniejszać), 'D' aby zmniejszyć moc, 'd' aby zablokować moc (lub zwiększać)."));
  Serial.println(F("W Trybie Demonstracji Akcelerometru: 'U' aby WŁĄCZYĆ silniki, 'D' aby WYŁĄCZYĆ silniki."));
  Serial.println(F("Wyślij 'Q' aby rozbroić silniki (działa w obu trybach)."));
  Serial.println(F("Pin STATE HC-05 podłączony do pinu 2 Arduino (przerwanie INT0)."));
  Serial.println(F("Komunikacja Bluetooth przez sprzętowy UART (piny 0 i 1)."));
  
  // Początkowy odczyt stanu BT_STATE_PIN
  bluetooth_connected_ISR = digitalRead(BT_STATE_PIN) == HIGH;
  Serial.print(F("Początkowy stan Bluetooth: "));
  Serial.println(bluetooth_connected_ISR ? F("POŁĄCZONO") : F("ROZŁĄCZONO"));
}

// === GŁÓWNA PĘTLA ===
void loop() {
  // Odczytaj dane z modułu Bluetooth za pomocą sprzętowego UART (obiekt Serial)
  while (Serial.available()) {
    char c = Serial.read();
    handleBluetoothCommand(c);
  }

  // Odczytaj stan połączenia Bluetooth ze zmiennej volatile
  noInterrupts(); // Wyłącz przerwania na krótki czas, aby bezpiecznie odczytać zmienną volatile
  bool current_bt_state = bluetooth_connected_ISR; // Skopiuj wartość do zmiennej lokalnej
  interrupts(); // Włącz przerwania z powrotem

  // NOWA KONTROLA DIODY LED: Świeci tylko w trybie FLIGHT_MODE
  digitalWrite(LED_PIN, currentOperatingMode == FLIGHT_MODE ? HIGH : LOW);

  // Jeśli połączenie Bluetooth zostało utracone, zainicjuj bezpieczne lądowanie
  if (currentOperatingMode == FLIGHT_MODE && armed && !current_bt_state) {
    initiateFailSafeLanding();
  }
  
  // Oblicz czas, który upłynął od ostatniej iteracji pętli (dt)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6; // dt w sekundach
  lastTime = now;

  // Odczytaj i zaktualizuj kąty roll i pitch z MPU6050_light
  mpu.update(); // Zaktualizuj dane IMU

  roll = mpu.getAngleX();
  pitch = mpu.getAngleY();

  int rollCorrection = 0;
  int pitchCorrection = 0;

  // Oblicz korekcje PID tylko w Trybie Lotu, jeśli jest uzbrojony i połączony
  if (currentOperatingMode == FLIGHT_MODE && armed && current_bt_state) {
    if (powerChange == 1 && currentPower < MAX_POWER) currentPower += powerStep;
    else if (powerChange == -1 && currentPower > MIN_POWER) currentPower -= powerStep;
    
    rollCorrection = computePIDRoll(roll, dt);
    pitchCorrection = computePIDPitch(pitch, dt);
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    currentPower = DEMO_BASE_POWER; // W trybie demo moc bazowa jest stała
  } else {
    // Jeśli nie jesteśmy w trybie lotu i uzbrojeni, lub nie jesteśmy połączeni,
    // silniki powinny być na minimum (albo w stanie lądowania awaryjnego)
    setAllMotors(currentPower);
  }

  // Zastosuj moc silników w zależności od aktualnego trybu i korekcji
  applyMotorPower(rollCorrection, pitchCorrection);

  // Wyjście debugowania do Monitora Szeregowego
  // Pamiętaj, że te komunikaty będą widoczne TYLKO, gdy moduł Bluetooth jest odłączony
  // i Arduino jest podłączone do komputera przez USB.
  Serial.print(F("Tryb: ")); Serial.print(currentOperatingMode == FLIGHT_MODE ? F("F") : F("A"));
  Serial.print(F(", Roll:")); Serial.print(roll, 2);
  Serial.print(F(", Pitch:")); Serial.print(pitch, 2);
  if (currentOperatingMode == FLIGHT_MODE) {
    Serial.print(F(", Korekcja Roll:")); Serial.print(rollCorrection);
    Serial.print(F(", Korekcja Pitch:")); Serial.print(pitchCorrection);
    Serial.print(F(", Moc:")); Serial.print(currentPower);
    Serial.print(F(", BT Connected: ")); Serial.println(current_bt_state ? F("TAK") : F("NIE"));
  } else { // ACCEL_DEMO_MODE
    Serial.print(F(", Demo Active: ")); Serial.print(demo_motors_active ? F("TAK") : F("NIE"));
    Serial.print(F(", ESC1:")); Serial.print(esc1.readMicroseconds());
    Serial.print(F(", ESC2:")); Serial.print(esc2.readMicroseconds());
    Serial.print(F(", ESC3:")); Serial.print(esc3.readMicroseconds());
    Serial.print(F(", ESC4:")); Serial.println(esc4.readMicroseconds());
  }
  
  delay(15);
}

// === FUNKCJE POMOCNICZE ===

void initializeESCs() {
  Serial.println(F("Inicjalizacja ESC..."));
  setAllMotors(MIN_POWER);
  delay(3000);
  Serial.println(F("ESCs zainicjalizowane."));
}

void calibrateIMU() {
  Serial.println(F("Inicjalizacja MPU6050..."));
  Wire.begin();
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print(F("MPU6050 nie znaleziono! Błąd statusu: "));
    Serial.println(status);
    while (1) delay(10);
  }
  Serial.println(F("MPU6050 podłączony."));
  Serial.println(F("Kalibracja MPU6050... Trzymaj drona nieruchomo."));
  mpu.calcOffsets(true, true);
  Serial.println(F("Kalibracja zakończona."));
}

void readIMU(float &rollOut, float &pitchOut, float dt) {
  // mpu.update() jest wywoływane bezpośrednio w loop()
  rollOut = mpu.getAngleX();
  pitchOut = mpu.getAngleY();
}

int computePIDRoll(float rollValue, float dt) {
  integralRoll += rollValue * dt;
  integralRoll = constrain(integralRoll, -200, 200);

  float derivative = (rollValue - lastRoll) / dt;
  lastRoll = rollValue;

  return constrain(-(Kp_roll * rollValue + Ki_roll * integralRoll + Kd_roll * derivative), -300, 300);
}

int computePIDPitch(float pitchValue, float dt) {
  integralPitch += pitchValue * dt;
  integralPitch = constrain(integralPitch, -200, 200);

  float derivative = (pitchValue - lastPitch) / dt;
  lastPitch = pitchValue;

  return constrain(Kp_pitch * pitchValue + Ki_pitch * integralPitch + Kd_pitch * derivative, -300, 300);
}

void applyMotorPower(int rollCorr, int pitchCorr) {
  noInterrupts();
  bool current_bt_state_local = bluetooth_connected_ISR;
  interrupts();

  if (currentOperatingMode == FLIGHT_MODE) {
    if (armed && current_bt_state_local) {
      if (millis() - armTime < STARTUP_DURATION) {
        setAllMotors(currentPower);
      } else {
        int powerRightTop    = constrain(currentPower - rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER);
        int powerRightBottom = constrain(currentPower - rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER);
        int powerLeftTop     = constrain(currentPower + rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER);
        int powerLeftBottom  = constrain(currentPower + rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER);

        esc1.writeMicroseconds(powerRightBottom);
        esc2.writeMicroseconds(powerLeftTop);
        esc3.writeMicroseconds(powerRightTop);
        esc4.writeMicroseconds(powerLeftBottom);
      }
    } else {
      setAllMotors(currentPower);
    }
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    if (demo_motors_active) {
      int demoRollCorrection = (int)(DEMO_KP_ROLL * roll);
      int demoPitchCorrection = (int)(DEMO_KP_PITCH * pitch);

      int power1 = constrain(DEMO_BASE_POWER + demoRollCorrection + demoPitchCorrection, MIN_POWER, MAX_POWER);
      int power2 = constrain(DEMO_BASE_POWER - demoRollCorrection - demoPitchCorrection, MIN_POWER, MAX_POWER);
      int power3 = constrain(DEMO_BASE_POWER + demoRollCorrection - demoPitchCorrection, MIN_POWER, MAX_POWER);
      int power4 = constrain(DEMO_BASE_POWER - demoRollCorrection + demoPitchCorrection, MIN_POWER, MAX_POWER);

      esc1.writeMicroseconds(power1);
      esc2.writeMicroseconds(power2);
      esc3.writeMicroseconds(power3);
      esc4.writeMicroseconds(power4);
    } else {
      setAllMotors(MIN_POWER);
    }
  }
}

void resetPID() {
  integralRoll = 0.0;
  lastRoll = 0.0;
  integralPitch = 0.0;
  lastPitch = 0.0;
  roll = 0.0;
  pitch = 0.0;
  calibrateIMU();
}

void disarmMotors() {
  armed = false;
  demo_motors_active = false;
  powerChange = 0;
  currentPower = MIN_POWER;
  setAllMotors(MIN_POWER);
  resetPID();
  Serial.println(F("SILNIKI ROZBROJONE I ZRESETOWANY PID (Awaryjne zatrzymanie)"));
}

void handleBluetoothCommand(char c) {
  noInterrupts();
  bool current_bt_state_local = bluetooth_connected_ISR;
  interrupts();

  if (c == 'Q' || c == 'q') {
    disarmMotors();
    return;
  }

  if (c == 'F') {
    switchMode(FLIGHT_MODE);
  } else if (c == 'A') {
    switchMode(ACCEL_DEMO_MODE);
  } else if (currentOperatingMode == FLIGHT_MODE) {
    if (current_bt_state_local) {
      if (c == 'U') {
        powerChange = 1;
        if (!armed) {
          armed = true;
          armTime = millis();
          Serial.println(F("UZBROJONY (U)"));
        }
      } else if (c == 'u') {
        powerChange = 0;
        Serial.println(F("Moc zablokowana po zwolnieniu U"));
      } else if (c == 'D') {
        powerChange = -1;
        if (!armed) {
          armed = true;
          armTime = millis();
          Serial.println(F("UZBROJONY (D)"));
        }
      } else if (c == 'd') {
        powerChange = 0;
        Serial.println(F("Moc zablokowana po zwolnieniu D"));
      }
    } else {
      Serial.println(F("Brak połączenia Bluetooth - nie można sterować w trybie lotu."));
    }
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    if (c == 'U' || c == 'u') {
      demo_motors_active = true;
      Serial.println(F("Tryb Demo: Silniki WŁĄCZONE (przycisk U)"));
    } else if (c == 'D' || c == 'd') {
      demo_motors_active = false;
      setAllMotors(MIN_POWER);
      Serial.println(F("Tryb Demo: Silniki WYŁĄCZONE (przycisk D)"));
    }
  } else {
    Serial.print(F("Nieznane polecenie lub polecenie nieprawidłowe w bieżącym trybie: "));
    Serial.println(c);
  }
}

void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}

void switchMode(int newMode) {
  if (currentOperatingMode != newMode) {
    currentOperatingMode = newMode;
    disarmMotors();
    Serial.print(F("Przełączono na "));
    Serial.println(newMode == FLIGHT_MODE ? F("FLIGHT_MODE") : F("ACCEL_DEMO_MODE"));
  } else {
    Serial.print(F("Już w trybie "));
    Serial.println(newMode == FLIGHT_MODE ? F("FLIGHT_MODE") : F("ACCEL_DEMO_MODE"));
  }
}

void initiateFailSafeLanding() {
  if (currentPower > MIN_POWER) {
    currentPower -= LANDING_POWER_DECREASE_STEP;
    if (currentPower < MIN_POWER) {
      currentPower = MIN_POWER;
    }
    setAllMotors(currentPower);
    Serial.print(F("Ladowanie awaryjne! Moc: "));
    Serial.println(currentPower);
  } else {
    if (armed) {
      Serial.println(F("Ladowanie awaryjne zakończone - rozbrojenie silników."));
      disarmMotors();
    }
  }
}