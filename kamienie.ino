#include <stdint.h>

// --- KONFIGURACJA PINÓW ---
const uint8_t LEWY_PWM_PIN = 5;      // Silnik lewy
const uint8_t PRAWY_PWM_PIN = 6;     // Silnik prawy
const uint8_t LEWY_ENCODER_PIN = 2;  // Lewy enkoder
const uint8_t PRAWY_ENCODER_PIN = 3; // Prawy enkoder

const float WSPOLCZYNNIK = 4114.28;

// --- PARAMETRY DO ZADANIA 3 (ODLEGŁOŚĆ) ---
// Parametry mechaniczne do przeliczenia cm na impulsy
const float SREDNICA_KOLA_CM = 4.5;       
const float PRZEKLADNIA = 14.58;          // Wyliczone z WSPOLCZYNNIK (4114.28 ~= 60000/14.58)
const float ZMIANY_NA_OBROT_ENKODERA = 4.0; // 4 zmiany stanu na obrót trybu enkodera

// Współczynnik: ile impulsów przypada na 1 cm drogi
// Wzór: (ImpulsyNaObrotKola) / ObwodKola
// ImpulsyNaObrotKola = ZMIANY_NA_OBROT_ENKODERA * PRZEKLADNIA
const float IMPULSY_NA_CM = (ZMIANY_NA_OBROT_ENKODERA * PRZEKLADNIA) / (SREDNICA_KOLA_CM * 3.14159);

// Zmienna do ustawienia zadanej odległości
float ZADANY_DYSTANS_CM = 30.0; // <--- TU WPISZ ODLEGŁOŚĆ W CM
bool cel_osiagniety = false;

// --- ZMIENNE PID ---
// Wspólne nastawy (Kp, Ki, Kd)
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// Osobne prędkości zadane dla kół
float predkoscZadanaL = 100.0; // Prędkość początkowa jazdy
float predkoscZadanaR = 100.0;

// --- ZMIENNE POMIAROWE (LEWA STRONA) ---
volatile uint32_t czas_ost_impulsu_L = 0;
volatile uint16_t pomiary_L[4] = {0};
volatile uint8_t numer_L = 0;
volatile long licznik_impulsow_L = 0; // Całkowita liczba impulsów lewego koła

// --- ZMIENNE POMIAROWE (PRAWA STRONA) ---
volatile uint32_t czas_ost_impulsu_R = 0;
volatile uint16_t pomiary_R[4] = {0};
volatile uint8_t numer_R = 0;
volatile long licznik_impulsow_R = 0; // Całkowita liczba impulsów prawego koła

// Deklaracje funkcji
float predkoscMierzonaLewa();
float predkoscMierzonaPrawa();
void przerwanieLewe();
void przerwaniePrawe();

void setup() {
  pinMode(LEWY_PWM_PIN, OUTPUT);
  pinMode(PRAWY_PWM_PIN, OUTPUT);
  
  pinMode(LEWY_ENCODER_PIN, INPUT_PULLUP);
  pinMode(PRAWY_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEWY_ENCODER_PIN), przerwanieLewe, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PRAWY_ENCODER_PIN), przerwaniePrawe, CHANGE);
  
  Serial.begin(9600);
  
  // Obliczamy ile impulsów to cel
  Serial.print("Impulsy na cm: "); Serial.println(IMPULSY_NA_CM);
  Serial.print("Cel w impulsach: "); Serial.println(ZADANY_DYSTANS_CM * IMPULSY_NA_CM);
}

void loop() {
  // --- KONTROLA DYSTANSU (ZADANIE 3) ---
  // Oblicz średni przebyty dystans na podstawie liczników z obu kół
  long srednia_impulsow = (licznik_impulsow_L + licznik_impulsow_R) / 2;
  float przebyty_dystans_cm = (float)srednia_impulsow / IMPULSY_NA_CM;
  
  if (!cel_osiagniety) {
    if (przebyty_dystans_cm >= ZADANY_DYSTANS_CM) {
      // Jeśli osiągnięto cel -> zatrzymaj robota
      predkoscZadanaL = 0;
      predkoscZadanaR = 0;
      cel_osiagniety = true;
    } else {
      // Jeśli nie osiągnięto celu -> jedź prosto
      predkoscZadanaL = 100.0;
      predkoscZadanaR = 100.0;
    }
  }

  // --- REGULATOR PID (cykl 10ms) ---
  static uint32_t last_pid = 0;
  if (millis() - last_pid >= 10) {
    last_pid = millis();

    // Pamięć regulatorów
    static float sum_e_L = 0;
    static float prev_e_L = 0;
    static float pid_raw_L = 0;

    static float sum_e_R = 0;
    static float prev_e_R = 0;
    static float pid_raw_R = 0;

    // 1. Dobór parametrów (Gain Scheduling)
    // Decyzja na podstawie średniej prędkości obu kół
    float srednia_zadana = (predkoscZadanaL + predkoscZadanaR) / 2.0;
    float stare_Ki = Ki;

    if (srednia_zadana <= 50.0) {
      // Dla małych prędkości (również dla 0 - hamowanie/postój)
      Kp = 0.7;
      Ki = 0.1;
      Kd = 0.7;
    } else {
      // Dla dużych prędkości
      Kp = 2.0;
      Ki = 0.5;
      Kd = 0.8;
    }

    // Reset całki przy zmianie parametrów lub zatrzymaniu (opcjonalne, ale pomaga przy starcie)
    if (stare_Ki != Ki) {
      sum_e_L = 0;
      sum_e_R = 0;
    }

    // ================= LEWY SILNIK =================
    float aktualna_L = predkoscMierzonaLewa();
    float e_L = predkoscZadanaL - aktualna_L;
    
    // Anti-windup Lewy
    bool nasycenie_L = (pid_raw_L >= 200.0 && e_L > 0) || (pid_raw_L <= 50.0 && e_L < 0);
    if (!nasycenie_L) sum_e_L += e_L;
    
    // PID Lewy
    float P_L = Kp * e_L;
    float I_L = Ki * sum_e_L;
    float D_L = Kd * (e_L - prev_e_L);
    pid_raw_L = P_L + I_L + D_L;
    
    uint8_t pwm_L = 0;
    if (predkoscZadanaL > 0) {
      pwm_L = (uint8_t)constrain(pid_raw_L, 60, 200);
    } else {
      pwm_L = 0;
      pid_raw_L = 0; // Reset wyjścia PID przy postoju
      sum_e_L = 0;   // Reset całki przy postoju
    }
    analogWrite(LEWY_PWM_PIN, pwm_L);
    prev_e_L = e_L;

    // ================= PRAWY SILNIK =================
    float aktualna_R = predkoscMierzonaPrawa();
    float e_R = predkoscZadanaR - aktualna_R;
    
    // Anti-windup Prawy
    bool nasycenie_R = (pid_raw_R >= 200.0 && e_R > 0) || (pid_raw_R <= 50.0 && e_R < 0);
    if (!nasycenie_R) sum_e_R += e_R;
    
    // PID Prawy
    float P_R = Kp * e_R;
    float I_R = Ki * sum_e_R;
    float D_R = Kd * (e_R - prev_e_R);
    pid_raw_R = P_R + I_R + D_R;
    
    uint8_t pwm_R = 0;
    if (predkoscZadanaR > 0) {
      pwm_R = (uint8_t)constrain(pid_raw_R, 60, 200);
    } else {
      pwm_R = 0;
      pid_raw_R = 0; // Reset wyjścia PID przy postoju
      sum_e_R = 0;   // Reset całki przy postoju
    }
    analogWrite(PRAWY_PWM_PIN, pwm_R);
    prev_e_R = e_R;

    // 6. Wykres (Format: ZadanaL MierzonaL ZadanaR MierzonaR DystansCM)
    Serial.print(0); Serial.print(" ");
    Serial.print(predkoscZadanaL); Serial.print(" ");
    Serial.print(aktualna_L);      Serial.print(" ");
    Serial.print(predkoscZadanaR); Serial.print(" ");
    Serial.print(aktualna_R);      Serial.print(" ");
    Serial.println(przebyty_dystans_cm); // Dodatkowo wyśw. dystans
  }
}

// --- OBSŁUGA PRZERWAŃ ---

void przerwanieLewe() {
  uint32_t now = millis();
  pomiary_L[numer_L] = (uint16_t)(now - czas_ost_impulsu_L);
  czas_ost_impulsu_L = now;
  numer_L++; if (numer_L > 3) numer_L = 0;
  
  licznik_impulsow_L++; // Zliczanie drogi
}

void przerwaniePrawe() {
  uint32_t now = millis();
  pomiary_R[numer_R] = (uint16_t)(now - czas_ost_impulsu_R);
  czas_ost_impulsu_R = now;
  numer_R++; if (numer_R > 3) numer_R = 0;
  
  licznik_impulsow_R++; // Zliczanie drogi
}

// --- FUNKCJE POMIAROWE ---

float predkoscMierzonaLewa() {
  uint32_t now = millis();
  noInterrupts();
  uint32_t last = czas_ost_impulsu_L; 
  interrupts();
  
  if (now - last > 300) return 0.0;
  
  float suma = 0;
  noInterrupts();
  for(uint8_t i=0; i<4; i++) suma += pomiary_L[i];
  interrupts();
  
  return (suma == 0) ? 0.0 : (WSPOLCZYNNIK / suma);
}

float predkoscMierzonaPrawa() {
  uint32_t now = millis();
  noInterrupts();
  uint32_t last = czas_ost_impulsu_R; 
  interrupts();
  
  if (now - last > 300) return 0.0;
  
  float suma = 0;
  noInterrupts();
  for(uint8_t i=0; i<4; i++) suma += pomiary_R[i]; 
  interrupts();
  
  return (suma == 0) ? 0.0 : (WSPOLCZYNNIK / suma);
}