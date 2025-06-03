#include <Wire.h>          // Wymagane do komunikacji I2C z MPU6050
#include <Adafruit_MPU6050.h> // Biblioteka dla czujnika IMU MPU6050
#include <Adafruit_Sensor.h> // Podstawowa biblioteka czujników dla czujników Adafruit
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
const float ALPHA = 0.98;           // Współczynnik filtra komplementarnego (do fuzji danych IMU)
const float Kp_roll = 5.0, Ki_roll = 0.2, Kd_roll = 0.8; // Stałe PID dla osi Roll (przechyłu bocznego)
const float Kp_pitch = 5.0, Ki_pitch = 0.2, Kd_pitch = 0.8; // Stałe PID dla osi Pitch (przechyłu przód-tył)

// Stałe Trybu Demonstracji Akcelerometru
const int DEMO_BASE_POWER = 1100;    // Podstawowa moc dla silników w trybie demo (nieco powyżej MIN_POWER)
const float DEMO_TILT_THRESHOLD = 15.0; // Stopnie przechylenia, które wywołują redukcję mocy w trybie demo
const int DEMO_REDUCTION_AMOUNT = 100; // Kwota redukcji mocy w trybie demo po przechyleniu

// Uruchamianie i Uzbrajanie
const unsigned long STARTUP_DURATION = 500; // Czas trwania po uzbrojeniu przed pełnym sterowaniem PID

// === ZMIENNE GLOBALNE ===
SoftwareSerial Bluetooth(RX_PIN, TX_PIN); // Obiekt SoftwareSerial do komunikacji Bluetooth
Servo esc1, esc2, esc3, esc4;             // Obiekty Servo do sterowania ESC
Adafruit_MPU6050 mpu;                     // Obiekt czujnika MPU6050

// Zmienne sterowania lotem
int currentPower = MIN_POWER; // Aktualny poziom gazu dla wszystkich silników
int powerChange = 0;          // -1 dla zmniejszenia, 0 dla braku zmiany, 1 dla zwiększenia
int powerStep = 10;           // Krok zwiększania/zmniejszania gazu
bool armed = false;           // Prawda, jeśli silniki są uzbrojone i gotowe do lotu

// Zmienne stanu IMU i PID
float rollOffset = 0.0, pitchOffset = 0.0; // Offsety kalibracyjne dla IMU
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
void readIMU(float &rollOut, float &pitchOut, sensors_event_t &a, sensors_event_t &g, float dt);
int computePIDRoll(float rollValue, float dt);
int computePIDPitch(float pitchValue, float dt);
void applyMotorPower(int rollCorr, int pitchCorr); // Teraz przyjmuje korekcje i stosuje w zależności od trybu
void resetPID();
void handleBluetoothCommand(char c);
void setAllMotors(int pwm);
void switchMode(int newMode); // Nowa funkcja do obsługi przełączania trybów

// === SETUP ===
void setup() {
  Serial.begin(9600);     // Inicjalizacja komunikacji szeregowej do debugowania
  Bluetooth.begin(9600);  // Inicjalizacja programowej komunikacji szeregowej dla modułu Bluetooth
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

  lastTime = micros(); // Zapisz aktualny czas do obliczania dt w pętli

  Serial.println("System drona gotowy!");
  Serial.print("Aktualny tryb: ");
  Serial.println(currentOperatingMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  Serial.println("Wyślij 'F' dla Trybu Lotu, 'A' dla Trybu Demonstracji Akcelerometru.");
  Serial.println("W Trybie Lotu: 'U' aby zwiększyć moc, 'D' aby zmniejszyć moc, 'Q' aby rozbroić.");
}

// === GŁÓWNA PĘTLA ===
void loop() {
  // Sprawdź przychodzące polecenia Bluetooth
  while (Bluetooth.available()) {
    char c = Bluetooth.read();
    handleBluetoothCommand(c);
  }

  // Zaktualizuj currentPower w zależności od stanu uzbrojenia i poleceń gazu
  if (currentOperatingMode == FLIGHT_MODE) {
    if (armed) {
      if (powerChange == 1 && currentPower < MAX_POWER) currentPower += powerStep;
      else if (powerChange == -1 && currentPower > MIN_POWER) currentPower -= powerStep;
    } else {
      currentPower = MIN_POWER; // Jeśli rozbrojony, silniki są na minimalnej mocy
      setAllMotors(MIN_POWER);
    }
  } else { // ACCEL_DEMO_MODE
    currentPower = DEMO_BASE_POWER; // W trybie demo moc jest stała
  }


  // Kontroluj diodę LED w zależności od stanu uzbrojenia i aktywności silników
  digitalWrite(LED_PIN, (armed && currentPower > MIN_POWER) ? HIGH : LOW);

  // Pobierz nowe zdarzenia z czujnika MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Oblicz czas, który upłynął od ostatniej iteracji pętli (dt)
  unsigned long now = micros();
  float dt = (now - lastTime) / 1e6; // dt w sekundach
  lastTime = now;

  // Odczytaj i połącz dane IMU, aby uzyskać aktualne kąty roll i pitch
  readIMU(roll, pitch, a, g, dt);

  int rollCorrection = 0;
  int pitchCorrection = 0;

  // Oblicz korekcje PID tylko w Trybie Lotu
  if (currentOperatingMode == FLIGHT_MODE) {
    rollCorrection = computePIDRoll(roll, dt);
    pitchCorrection = computePIDPitch(pitch, dt);
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
  }
  Serial.print(", Moc:"); Serial.println(currentPower);

  delay(15); // Małe opóźnienie, aby umożliwić stabilne odczyty czujników i wykonanie pętli
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
 * @brief Kalibruje czujnik MPU6050 poprzez odczytanie początkowych wartości akcelerometru
 * w celu określenia offsetów poziomu dla roll i pitch.
 */
void calibrateIMU() {
  Serial.println("Inicjalizacja MPU6050...");
  if (!mpu.begin()) {
    Serial.println("MPU6050 nie znaleziono! Sprawdź okablowanie.");
    while (1) delay(10); // Zatrzymaj wykonanie, jeśli MPU6050 nie zostanie wykryty
  }
  Serial.println("MPU6050 podłączony.");

  // Skonfiguruj ustawienia MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // Ustaw cyfrowy filtr dolnoprzepustowy

  const int samples = 100; // Liczba próbek do kalibracji
  float rollSum = 0.0, pitchSum = 0.0;
  Serial.println("Kalibracja czujników... Trzymaj drona nieruchomo.");

  // Zbierz próbki, aby obliczyć średnie offsety
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); // Pobierz nowe dane z czujnika

    // Oblicz roll i pitch z danych akcelerometru (w stopniach)
    float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
    float pitch_acc = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

    rollSum += roll_acc;
    pitchSum += pitch_acc;

    delay(10); // Małe opóźnienie między próbkami
  }

  // Oblicz średnie offsety
  rollOffset = rollSum / samples;
  pitchOffset = pitchSum / samples;

  Serial.print("Kalibracja zakończona. Offset Roll: "); Serial.print(rollOffset, 2);
  Serial.print(", Offset Pitch: "); Serial.println(pitchOffset, 2);
}

/**
 * @brief Odczytuje surowe dane IMU i stosuje filtr komplementarny do oszacowania
 * dokładnych kątów roll i pitch.
 * @param rollOut Referencja do przechowywania obliczonego kąta roll.
 * @param pitchOut Referencja do przechowywania obliczonego kąta pitch.
 * @param a Dane zdarzenia akcelerometru.
 * @param g Dane zdarzenia żyroskopu.
 * @param dt Czas, który upłynął od ostatniego odczytu w sekundach.
 */
void readIMU(float &rollOut, float &pitchOut, sensors_event_t &a, sensors_event_t &g, float dt) {
  // Oblicz roll i pitch z akcelerometru (statyczne odniesienie)
  float roll_acc = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch_acc = atan2(-a.acceleration.x, sqrt(pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2))) * 180.0 / PI;

  // Zintegruj dane żyroskopu, aby uzyskać zmianę kąta (dynamiczne odniesienie)
  // Wartości żyroskopu są w rad/s, przekonwertuj na deg/s i pomnóż przez dt
  roll += g.gyro.x * 180.0 / PI * dt;
  pitch += g.gyro.y * 180.0 / PI * dt;

  // Zastosuj filtr komplementarny:
  // Żyroskop zapewnia szybką, krótkoterminową dokładność. Akcelerometr zapewnia wolną, długoterminową dokładność.
  rollOut = ALPHA * roll + (1 - ALPHA) * (roll_acc - rollOffset);
  pitchOut = ALPHA * pitch + (1 - ALPHA) * (pitch_acc - pitchOffset);

  // Zaktualizuj globalne zmienne roll i pitch przefiltrowanymi wartościami
  roll = rollOut;
  pitch = pitchOut;
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
 * W FLIGHT_MODE używa korekcji PID. W ACCEL_DEMO_MODE reaguje na przechylenie.
 * @param rollCorr Korekcja PID dla roll (używana w FLIGHT_MODE).
 * @param pitchCorr Korekcja PID dla pitch (używana w FLIGHT_MODE).
 */
void applyMotorPower(int rollCorr, int pitchCorr) {
  if (currentOperatingMode == FLIGHT_MODE) {
    if (armed && (millis() - armTime < STARTUP_DURATION)) {
      // Podczas początkowej fazy uruchamiania, uruchom wszystkie silniki z currentPower
      setAllMotors(currentPower);
    } else if (armed) {
      // Zastosuj korekcje PID do poszczególnych silników
      // Mapowanie silników dla konfiguracji 'X':
      // esc1 (Tylny Prawy)
      // esc2 (Przedni Lewy)
      // esc3 (Przedni Prawy)
      // esc4 (Tylny Lewy)

      // Dostosuj moc w zależności od korekcji roll i pitch
      // Korekcja roll: zmniejsz moc prawych silników, zwiększ moc lewych silników (dla pozytywnego roll)
      // Korekcja pitch: zmniejsz moc przednich silników, zwiększ moc tylnych silników (dla pozytywnego pitch)
      esc1.writeMicroseconds(constrain(currentPower - rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // Tylny Prawy
      esc2.writeMicroseconds(constrain(currentPower + rollCorr - pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // Przedni Lewy
      esc3.writeMicroseconds(constrain(currentPower - rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // Przedni Prawy
      esc4.writeMicroseconds(constrain(currentPower + rollCorr + pitchCorr, FLIGHT_MIN_POWER, MAX_POWER)); // Tylny Lewy
    } else {
      // Jeśli nie uzbrojony, upewnij się, że wszystkie silniki są na minimalnej mocy
      setAllMotors(MIN_POWER);
    }
  } else if (currentOperatingMode == ACCEL_DEMO_MODE) {
    // W Trybie Demonstracji Akcelerometru, silniki pracują z podstawową mocą i reagują na przechylenie
    int power1 = DEMO_BASE_POWER; // Tylny Prawy
    int power2 = DEMO_BASE_POWER; // Przedni Lewy
    int power3 = DEMO_BASE_POWER; // Przedni Prawy
    int power4 = DEMO_BASE_POWER; // Tylny Lewy

    // Zastosuj redukcję mocy w zależności od przechylenia roll
    if (roll > DEMO_TILT_THRESHOLD) { // Przechylony w prawo
      // Zmniejsz moc prawych silników (Tylny Prawy, Przedni Prawy)
      power1 = constrain(power1 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
      power3 = constrain(power3 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
    } else if (roll < -DEMO_TILT_THRESHOLD) { // Przechylony w lewo
      // Zmniejsz moc lewych silników (Przedni Lewy, Tylny Lewy)
      power2 = constrain(power2 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
      power4 = constrain(power4 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
    }

    // Zastosuj redukcję mocy w zależności od przechylenia pitch
    if (pitch > DEMO_TILT_THRESHOLD) { // Przechylony do przodu
      // Zmniejsz moc przednich silników (Przedni Lewy, Przedni Prawy)
      // Uwaga: Ponownie ograniczamy, więc jeśli roll już to zmniejszył, nie spadnie poniżej MIN_POWER
      power2 = constrain(power2 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
      power3 = constrain(power3 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
    } else if (pitch < -DEMO_TILT_THRESHOLD) { // Przechylony do tyłu
      // Zmniejsz moc tylnych silników (Tylny Prawy, Tylny Lewy)
      power1 = constrain(power1 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
      power4 = constrain(power4 - DEMO_REDUCTION_AMOUNT, MIN_POWER, DEMO_BASE_POWER);
    }

    // Zastosuj obliczoną moc do każdego silnika
    esc1.writeMicroseconds(power1);
    esc2.writeMicroseconds(power2);
    esc3.writeMicroseconds(power3);
    esc4.writeMicroseconds(power4);
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
  roll = 0.0;
  pitch = 0.0;
  // Ponownie skalibruj IMU, aby uzyskać świeże offsety dla aktualnej orientacji
  calibrateIMU();
}

/**
 * @brief Obsługuje przychodzące polecenia Bluetooth do sterowania stanem i trybem drona.
 * @param c Znak odebrany z Bluetooth.
 */
void handleBluetoothCommand(char c) {
  if (c == 'F') { // Przełącz na Tryb Lotu
    switchMode(FLIGHT_MODE);
  } else if (c == 'A') { // Przełącz na Tryb Demonstracji Akcelerometru
    switchMode(ACCEL_DEMO_MODE);
  } else if (currentOperatingMode == FLIGHT_MODE) {
    // Przetwarzaj polecenia sterowania lotem tylko w FLIGHT_MODE
    if (c == 'U') { // Zwiększ gaz
      powerChange = 1;
      if (!armed) {
        armed = true;
        armTime = millis(); // Zapisz czas uzbrojenia
        Serial.println("UZBROJONY (U)");
      }
    } else if (c == 'u') { // Przestań zwiększać gaz, zablokuj moc
      powerChange = 0;
      if (armed) {
        // Opcjonalnie, zablokuj moc na STABLE_FLIGHT_POWER lub aktualnym poziomie
        // currentPower = STABLE_FLIGHT_POWER;
        Serial.println("Moc zablokowana po zwolnieniu U");
      }
    } else if (c == 'D') { // Zmniejsz gaz
      powerChange = -1;
      if (!armed) {
        armed = true;
        armTime = millis(); // Zapisz czas uzbrojenia
        Serial.println("UZBROJONY (D)");
      }
    } else if (c == 'd') { // Przestań zmniejszać gaz, zablokuj moc
      powerChange = 0;
      if (armed) {
        // Opcjonalnie, zablokuj moc na STABLE_FLIGHT_POWER lub aktualnym poziomie
        // currentPower = STABLE_FLIGHT_POWER;
        Serial.println("Moc zablokowana po zwolnieniu D");
      }
    } else if (c == 'Q' || c == 'q') { // Rozbrój silniki
      armed = false;
      powerChange = 0;
      currentPower = MIN_POWER;
      setAllMotors(MIN_POWER); // Upewnij się, że silniki się zatrzymały
      resetPID(); // Zresetuj stan PID
      Serial.println("ROZBROJONY i zresetowany PID");
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
    armed = false; // Zawsze rozbrajaj podczas przełączania trybów
    powerChange = 0;
    currentPower = MIN_POWER;
    setAllMotors(MIN_POWER); // Upewnij się, że silniki są wyłączone
    resetPID(); // Zresetuj PID i ponownie skalibruj IMU dla nowego trybu

    Serial.print("Przełączono na ");
    Serial.println(newMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  } else {
    Serial.print("Już w trybie ");
    Serial.println(newMode == FLIGHT_MODE ? "FLIGHT_MODE" : "ACCEL_DEMO_MODE");
  }
}
