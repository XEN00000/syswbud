#include <Wire.h>          // Wymagane do komunikacji I2C
#include <MPU6050_light.h> // Biblioteka dla czujnika IMU MPU6050
#include <Servo.h>         // Biblioteka do sterowania ESC (Electronic Speed Controllers)
#include <SoftwareSerial.h> // Biblioteka do programowej komunikacji szeregowej (np. moduł Bluetooth)

// === KONFIGURACJA ===
// Definicje pinów
const int RX_PIN = 2; // Pin RX modułu Bluetooth (łączy się z TX Arduino)
const int TX_PIN = 3; // Pin TX modułu Bluetooth (łączy się z RX Arduino)
const int LED_PIN = 8; // Pin diody LED

// Przypisania pinów ESC (dostosuj w zależności od układu silników drona)
// Te komentarze opisują fizyczne położenie silników na dronie
// zakładając konfigurację 'X'.
const int ESC1_PIN = 5;  // Tylny Prawy (prawy dol)
const int ESC2_PIN = 6;  // Przedni Lewy (lewa gora)
const int ESC3_PIN = 10; // Przedni Prawy (prawa gora)
const int ESC4_PIN = 9;  // Tylny Lewy (lewa dol)

// Ustawienia Mocy Silników (w mikrosekundach, dla ESC)
const int MIN_POWER = 1000;         // Minimalny sygnał do ESC (silniki wyłączone/jałowe)
const int MAX_POWER = 1750;         // Maksymalny sygnał do ESC (pełny gaz)
const int FLIGHT_MIN_POWER = 1200;  // Minimalna moc dla stabilnego lotu (silniki się kręcą)
const int STABLE_FLIGHT_POWER = 1650; // Docelowa moc dla stabilnego zawisu (można dostosować)

// PID (Proportional-Integral-Derivative) Stałe Kontrolera dla Trybu Lotu
const float MPU_FILTER_ALPHA = 0.98; // Współczynnik filtra komplementarnego dla MPU6050_light (jeśli używany ręcznie)
const float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8; // Stałe PID dla osi Roll
const float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8; // Stałe PID dla osi Pitch

// Stałe Trybu Demonstracji Akcelerometru
const int DEMO_BASE_POWER = 1150;    // Podstawowa moc dla silników w trybie demo (jak w Twoim przykładzie)
const float DEMO_KP_ROLL = 15.0;     // Współczynnik P dla roll w trybie demo
const float DEMO_KP_PITCH = 15.0;    // Współczynnik P dla pitch w trybie demo (odwrócony znak)

// Uruchamianie i Uzbrajanie
const unsigned long STARTUP_DURATION = 500; // Czas trwania po uzbrojeniu przed pełnym sterowaniem PID

// === ZMIENNE GLOBALNE ===
SoftwareSerial Bluetooth(RX_PIN, TX_PIN); // Obiekt SoftwareSerial do komunikacji Bluetooth
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

// === SETUP ===
void setup() {
  Serial.begin(9600);     // Inicjalizacja komunikacji szeregowej do debugowania
  Bluetooth.begin(9900);  // Inicjalizacja programowej komunikacji szeregowej dla modułu Bluetooth (Upewnij się, że ta prędkość jest zgodna z Twoim modułem BT)
  pinMode(LED_PIN, OUTPUT); // Ustaw pin LED jako wyjście

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

  Serial.println("System drona gotowy!");
  Serial.print("Aktualny tryb: ");
  Serial.println(currentOperatingMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  Serial.println("Wyślij 'F' dla Trybu Lotu, 'A' dla Trybu Demonstracji Akcelerometru.");
  Serial.println("W Trybie Lotu: 'U' aby zwiększyć moc, 'u' aby zablokować moc (lub zmniejszać), 'D' aby zmniejszyć moc, 'd' aby zablokować moc (lub zwiększać).");
  Serial.println("W Trybie Demonstracji Akcelerometru: 'U' aby WŁĄCZYĆ silniki, 'D' aby WYŁĄCZYĆ silniki.");
  Serial.println("Wyślij 'Q' aby rozbroić silniki (działa w obu trybach).");
}

// === GŁÓWNA PĘTLA ===
void loop() {
  // Sprawdź przychodzące polecenia Bluetooth
  while (Bluetooth.available()) {
    char c = Bluetooth.read();
    handleBluetoothCommand(c);
  }

  // Kontroluj diodę LED w zależności od stanu uzbrojenia i aktywności silników
  digitalWrite(LED_PIN, (armed && currentPower > MIN_POWER) || (currentOperatingMode == ACCEL_DEMO_MODE && demo_motors_active) ? HIGH : LOW);

  // Oblicz czas, który upłynął od ostatniej iteracji pętli (dt)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6; // dt w sekundach
  lastTime = now;

  // Odczytaj i zaktualizuj kąty roll i pitch z MPU6050_light
  readIMU(roll, pitch, dt);

  int rollCorrection = 0;
  int pitchCorrection = 0;

  // Oblicz korekcje PID tylko w Trybie Lotu
  if (currentOperatingMode == FLIGHT_MODE) {
    // Zaktualizuj currentPower w zależności od stanu uzbrojenia i poleceń gazu
    if (armed) {
      if (powerChange == 1 && currentPower < MAX_POWER) currentPower += powerStep;
      else if (powerChange == -1 && currentPower > MIN_POWER) currentPower -= powerStep;
    } else {
      currentPower = MIN_POWER;
      setAllMotors(MIN_POWER);
    }
    rollCorrection = computePIDRoll(roll, dt);
    pitchCorrection = computePIDPitch(pitch, dt);
  } else { // ACCEL_DEMO_MODE
    currentPower = DEMO_BASE_POWER; // W trybie demo moc bazowa jest stała
  }

  // Zastosuj moc silników w zależności od aktualnego trybu i korekcji
  applyMotorPower(rollCorrection, pitchCorrection);

  // Wyjście debugowania do Monitora Szeregowego
  Serial.print("Tryb: "); Serial.print(currentOperatingMode == FLIGHT_MODE ? "F" : "A");
  Serial.print(", Roll:"); Serial.print(roll, 2);
  Serial.print(", Pitch:"); Serial.print(pitch, 2);
  if (currentOperatingMode == FLIGHT_MODE) {
    Serial.print(", Korekcja Roll:"); Serial.print(rollCorrection);
    Serial.print(", Korekcja Pitch:"); Serial.print(pitchCorrection);
    Serial.print(", Moc:"); Serial.println(currentPower);
  } else { // ACCEL_DEMO_MODE
    Serial.print(", Demo Active: "); Serial.print(demo_motors_active ? "TAK" : "NIE");
    // W trybie demo wyświetlamy aktualne moce silników, aby zobaczyć reakcję na przechylenie
    Serial.print(", ESC1:"); Serial.print(esc1.readMicroseconds());
    Serial.print(", ESC2:"); Serial.print(esc2.readMicroseconds());
    Serial.print(", ESC3:"); Serial.print(esc3.readMicroseconds());
    Serial.print(", ESC4:"); Serial.println(esc4.readMicroseconds());
  }
  
  delay(15); // Opóźnienie jak w oryginalnym kodzie
}

// === FUNKCJE POMOCNICZE ===

/**
 * @brief Inicjalizuje ESC poprzez wysłanie niskiego sygnału na krótki czas.
 * Jest to często wymagane do sekwencji kalibracji/uzbrajania ESC.
 */
void initializeESCs() {
  Serial.println("Inicjalizacja ESC...");
  setAllMotors(MIN_POWER); // Upewnij się, że wszystkie silniki są na minimalnej mocy
  delay(3000); // Poczekaj, aż ESC się uzbroją (niektóre ESC potrzebują dłuższego opóźnienia)
  Serial.println("ESCs zainicjalizowane.");
}

/**
 * @brief Kalibruje czujnik MPU6050_light.
 * Ta funkcja używa wbudowanych metod kalibracji MPU6050_light.
 */
void calibrateIMU() {
  Serial.println("Inicjalizacja MPU6050...");
  Wire.begin(); // Rozpocznij komunikację I2C
  byte status = mpu.begin();
  if (status != 0) { // 0 = wszystko ok!
    Serial.print("MPU6050 nie znaleziono! Błąd statusu: ");
    Serial.println(status);
    while (1) delay(10); // Zatrzymaj wykonanie
  }
  Serial.println("MPU6050 podłączony.");

  // MPU6050_light domyślnie wykonuje pewne ustawienia i offsety przy pierwszym uruchomieniu.
  // Jeśli potrzebujesz ręcznej kalibracji (np. zerowania offsetów),
  // możesz użyć mpu.calcOffsets(true, true); z włączonymi akcelerometrem i żyroskopem.
  // Należy umieścić drona w pozycji poziomej podczas kalibracji.
  Serial.println("Kalibracja MPU6050... Trzymaj drona nieruchomo.");
  mpu.calcOffsets(true, true); // Kalibracja akcelerometru i żyroskopu (trujące true)
  Serial.println("Kalibracja zakończona.");
}

/**
 * @brief Odczytuje dane z MPU6050_light i aktualizuje kąty roll i pitch.
 * MPU6050_light wewnętrznie stosuje filtr komplementarny.
 * @param rollOut Referencja do globalnej zmiennej roll.
 * @param pitchOut Referencja do globalnej zmiennej pitch.
 * @param dt Czas, który upłynął od ostatniego odczytu w sekundach.
 */
void readIMU(float &rollOut, float &pitchOut, float dt) {
  mpu.update(); // Pobierz nowe dane i zaktualizuj wewnętrzne kąty
  
  // MPU6050_light używa getAngleX() i getAngleY() do zwracania kątów roll i pitch.
  rollOut = mpu.getAngleX();
  pitchOut = mpu.getAngleY();
}


/**
 * @brief Oblicza korekcję PID dla osi roll.
 * @param rollValue Aktualny kąt roll.
 * @param dt Czas, który upłynął od ostatniego obliczenia w sekundach.
 * @return Wartość korekcji PID dla roll.
 */
int computePIDRoll(float rollValue, float dt) {
  integralRoll += rollValue * dt; // Skumuluj błąd całkowy
  // Ogranicz narastanie całki, aby zapobiec dużym przeregulowaniom
  integralRoll = constrain(integralRoll, -200, 200);

  float derivative = (rollValue - lastRoll) / dt; // Oblicz błąd różniczkowy
  lastRoll = rollValue; // Zapisz aktualny roll do obliczeń różniczkowych w następnej iteracji

  // Oblicz wyjście PID i ogranicz je w bezpiecznym zakresie
  // Znak ujemny, ponieważ pozytywny roll wymaga ujemnej korekcji dla prawych silników
  return constrain(-(Kp_roll * rollValue + Ki_roll * integralRoll + Kd_roll * derivative), -300, 300);
}

/**
 * @brief Oblicza korekcję PID dla osi pitch.
 * @param pitchValue Aktualny kąt pitch.
 * @param dt Czas, który upłynął od ostatniego obliczenia w sekundach.
 * @return Wartość korekcji PID dla pitch.
 */
int computePIDPitch(float pitchValue, float dt) {
  integralPitch += pitchValue * dt; // Skumuluj błąd całkowy
  // Ogranicz narastanie całki
  integralPitch = constrain(integralPitch, -200, 200);

  float derivative = (pitchValue - lastPitch) / dt; // Oblicz błąd różniczkowy
  lastPitch = pitchValue; // Zapisz aktualny pitch do obliczeń różniczkowych w następnej iteracji

  // Oblicz wyjście PID i ogranicz je w bezpiecznym zakresie
  // Znak dodatni, ponieważ pozytywny pitch wymaga dodatniej korekcji dla tylnych silników
  return constrain(Kp_pitch * pitchValue + Ki_pitch * integralPitch + Kd_pitch * derivative, -300, 300);
}

/**
 * @brief Stosuje obliczoną moc silników w zależności od aktualnego trybu działania.
 * W FLIGHT_MODE używa korekcji PID. W ACCEL_DEMO_MODE reaguje na przechylenie,
 * aktywując silniki na "najniższej" stronie (lub stronach).
 * @param rollCorr Korekcja PID dla roll (używana w FLIGHT_MODE).
 * @param pitchCorr Korekcja PID dla pitch (używana w FLIGHT_MODE).
 */
void applyMotorPower(int rollCorr, int pitchCorr) {
  if (currentOperatingMode == FLIGHT_MODE) {
    if (armed && (millis() - armTime < STARTUP_DURATION)) {
      setAllMotors(currentPower);
    } else if (armed) {
      // Mapowanie silników dla konfiguracji 'X':
      // ESC1 (Tylny Prawy) - pin 5
      // ESC2 (Przedni Lewy) - pin 6
      // ESC3 (Przedni Prawy) - pin 10
      // ESC4 (Tylny Lewy) - pin 9

      // Moc silników z korekcją
      int powerRightTop    = constrain(currentPower - rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER); // ESC3 (pin 10)
      int powerRightBottom = constrain(currentPower - rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER); // ESC1 (pin 5)
      int powerLeftTop     = constrain(currentPower + rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER); // ESC2 (pin 6)
      int powerLeftBottom  = constrain(currentPower + rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER); // ESC4 (pin 9)

      esc1.writeMicroseconds(powerRightBottom); // Tylny Prawy (pin 5)
      esc2.writeMicroseconds(powerLeftTop);     // Przedni Lewy (pin 6)
      esc3.writeMicroseconds(powerRightTop);    // Przedni Prawy (pin 10)
      esc4.writeMicroseconds(powerLeftBottom);  // Tylny Lewy (pin 9)
    } else {
      setAllMotors(MIN_POWER);
    }
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    if (demo_motors_active) {
      // W Trybie Demonstracji Akcelerometru, silniki reagują na przechylenie
      // Moc jest modyfikowana w zależności od kąta przechylenia.
      // Używamy Kp z trybu demo.

      // Obliczamy "korekcję" podobnie jak w PID, ale tylko z członem proporcjonalnym
      // i z osobnymi stałymi dla trybu demo.
      // Ważne: znaki korekcji są odwrócone, bo chcemy, aby silniki z "niższej" strony zwiększały moc.
      // W trybie lotu PID zmniejsza moc silnika, który jest "za wysoko".
      // W trybie demo chcemy, aby silnik z "niższej" strony ZWIĘKSZAŁ moc.
      int demoRollCorrection = (int)(DEMO_KP_ROLL * roll);
      int demoPitchCorrection = (int)(DEMO_KP_PITCH * pitch); // Odwrócony znak dla pitch, aby zachować logikę "dolnej strony"

      // Zastosuj moc do każdego silnika, gdzie bazowa moc to DEMO_BASE_POWER
      // Wartość power jest zwiększana, gdy silnik znajduje się na "niższej" stronie.
      // constrain zapewnia, że moc mieści się w dopuszczalnym zakresie (MIN_POWER do MAX_POWER)
      
      // ESC1 (Tylny Prawy) - pin 5: moc zwiększa się, gdy roll jest dodatni (prawa strona w dół) i pitch jest ujemny (tył w dół)
      int power1 = constrain(DEMO_BASE_POWER + demoRollCorrection + demoPitchCorrection, MIN_POWER, MAX_POWER);
      // ESC2 (Przedni Lewy) - pin 6: moc zwiększa się, gdy roll jest ujemny (lewa strona w dół) i pitch jest dodatni (przód w dół)
      int power2 = constrain(DEMO_BASE_POWER - demoRollCorrection - demoPitchCorrection, MIN_POWER, MAX_POWER);
      // ESC3 (Przedni Prawy) - pin 10: moc zwiększa się, gdy roll jest dodatni (prawa strona w dół) i pitch jest dodatni (przód w dół)
      int power3 = constrain(DEMO_BASE_POWER + demoRollCorrection - demoPitchCorrection, MIN_POWER, MAX_POWER);
      // ESC4 (Tylny Lewy) - pin 9: moc zwiększa się, gdy roll jest ujemny (lewa strona w dół) i pitch jest ujemny (tył w dół)
      int power4 = constrain(DEMO_BASE_POWER - demoRollCorrection + demoPitchCorrection, MIN_POWER, MAX_POWER);

      // Zastosuj obliczoną moc do każdego silnika
      esc1.writeMicroseconds(power1); // Tylny Prawy (pin 5)
      esc2.writeMicroseconds(power2); // Przedni Lewy (pin 6)
      esc3.writeMicroseconds(power3); // Przedni Prawy (pin 10)
      esc4.writeMicroseconds(power4); // Tylny Lewy (pin 9)
    } else {
      // Jeśli demo_motors_active jest false, wyłącz wszystkie silniki w trybie demo
      setAllMotors(MIN_POWER);
    }
  }
}

/**
 * @brief Resetuje człony całkowe i różniczkowe PID oraz oszacowania kątów.
 * Wywoływane podczas rozbrajania lub przełączania trybów, aby zapewnić czysty start.
 */
void resetPID() {
  integralRoll = 0.0;
  lastRoll = 0.0;
  integralPitch = 0.0;
  lastPitch = 0.0;
  // Kąty roll i pitch są resetowane poprzez ponowną kalibrację MPU
  roll = 0.0;
  pitch = 0.0;
  calibrateIMU(); // Ponownie skalibruj IMU, aby uzyskać świeże zerowe odniesienie
}

/**
 * @brief Rozbraja silniki, ustawia moc na MIN_POWER i resetuje PID.
 * Używana dla awaryjnego zatrzymania.
 */
void disarmMotors() {
  armed = false;
  demo_motors_active = false; // Rozbrajamy też tryb demo
  powerChange = 0;
  currentPower = MIN_POWER;
  setAllMotors(MIN_POWER); // Upewnij się, że silniki się zatrzymały
  resetPID(); // Zresetuj stan PID i skalibruj IMU
  Serial.println("SILNIKI ROZBROJONE I ZRESETOWANY PID (Awaryjne zatrzymanie)");
}

/**
 * @brief Obsługuje przychodzące polecenia Bluetooth do sterowania stanem i trybem drona.
 * @param c Znak odebrany z Bluetooth.
 */
void handleBluetoothCommand(char c) {
  // Awaryjne rozbrojenie działa w obu trybach
  if (c == 'Q' || c == 'q') {
    disarmMotors();
    return; // Zakończ funkcję po rozbrojeniu
  }

  if (c == 'F') { // Przełącz na Tryb Lotu
    switchMode(FLIGHT_MODE);
  } else if (c == 'A') { // Przełącz na Tryb Demonstracji Akcelerometru
    switchMode(ACCEL_DEMO_MODE);
  } else if (currentOperatingMode == FLIGHT_MODE) {
    // Przetwarzaj polecenia sterowania lotem tylko w FLIGHT_MODE
    if (c == 'U') { // Zwiększ gaz
      powerChange = 1;
      // Ustawienie armed na true tylko przy pierwszym naciśnięciu 'U' lub 'D'
      if (!armed) {
        armed = true;
        armTime = millis(); // Zapisz czas uzbrojenia
        Serial.println("UZBROJONY (U)");
      }
    } else if (c == 'u') { // Przestań zwiększać gaz (lub pozwól na zmniejszanie)
      powerChange = 0; // Zablokuj zmianę mocy, pozostawiając ją na aktualnym poziomie lub pozwalając na spadek
      Serial.println("Moc zablokowana po zwolnieniu U");
    } else if (c == 'D') { // Zmniejsz gaz
      powerChange = -1;
      // Ustawienie armed na true tylko przy pierwszym naciśnięciu 'U' lub 'D'
      if (!armed) {
        armed = true;
        armTime = millis(); // Zapisz czas uzbrojenia
        Serial.println("UZBROJONY (D)");
      }
    } else if (c == 'd') { // Przestań zmniejszać gaz (lub pozwól na zwiększanie)
      powerChange = 0; // Zablokuj zmianę mocy, pozostawiając ją na aktualnym poziomie lub pozwalając na wzrost
      Serial.println("Moc zablokowana po zwolnieniu D");
    }
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    // Polecenia dla trybu demonstracyjnego
    if (c == 'U' || c == 'u') {
      demo_motors_active = true;
      Serial.println("Tryb Demo: Silniki WŁĄCZONE (przycisk U)");
    } else if (c == 'D' || c == 'd') {
      demo_motors_active = false;
      setAllMotors(MIN_POWER); // Upewnij się, że silniki są wyłączone natychmiast
      Serial.println("Tryb Demo: Silniki WYŁĄCZONE (przycisk D)");
    }
  } else {
    Serial.print("Nieznane polecenie lub polecenie nieprawidłowe w bieżącym trybie: ");
    Serial.println(c);
  }
}

/**
 * @brief Ustawia wszystkie silniki na określoną wartość PWM (w mikrosekundach).
 * @param pwm Wartość PWM do wysłania do wszystkich ESC.
 */
void setAllMotors(int pwm) {
  esc1.writeMicroseconds(pwm);
  esc2.writeMicroseconds(pwm);
  esc3.writeMicroseconds(pwm);
  esc4.writeMicroseconds(pwm);
}

/**
 * @brief Przełącza tryb działania drona.
 * Resetuje odpowiednie stany podczas przełączania trybów.
 * @param newMode Tryb, na który należy się przełączyć (FLIGHT_MODE lub ACCEL_DEMO_MODE).
 */
void switchMode(int newMode) {
  if (currentOperatingMode != newMode) {
    currentOperatingMode = newMode;
    disarmMotors(); // Zawsze rozbrajaj podczas przełączania trybów dla bezpieczeństwa
    Serial.print("Przełączono na ");
    Serial.println(newMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  } else {
    Serial.print("Już w trybie ");
    Serial.println(newMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  }
}