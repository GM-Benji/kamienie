#include <stdint.h>

// --- KONFIGURACJA PINÓW ---
const uint8_t LEWY_PWM_PIN = 5;      // Silnik lewy
const uint8_t PRAWY_PWM_PIN = 6;     // Silnik prawy
const uint8_t LEWY_ENCODER_PIN = 2;  // Lewy enkoder
const uint8_t PRAWY_ENCODER_PIN = 3; // Prawy enkoder

const float WSPOLCZYNNIK = 4114.28;

// --- ZMIENNE PID ---
// Wspólne nastawy (Kp, Ki, Kd) dla uproszczenia,
// ale dobierane na podstawie średniej prędkości
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// Osobne prędkości zadane dla kół
float predkoscZadanaL = 200.0; 
float predkoscZadanaR = 200.0;

// --- ZMIENNE POMIAROWE (LEWA STRONA) ---
volatile uint32_t czas_ost_impulsu_L = 0;
volatile uint16_t pomiary_L[4] = {0};
volatile uint8_t numer_L = 0;

// --- ZMIENNE POMIAROWE (PRAWA STRONA) ---
volatile uint32_t czas_ost_impulsu_R = 0;
volatile uint16_t pomiary_R[4] = {0};
volatile uint8_t numer_R = 0;

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
}

void loop() {
  // Usunięto sekwencję testową (zmianę prędkości w czasie)
  
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
      Kp = 0.7;
      Ki = 0.1;
      Kd = 0.7;
    } else {
      Kp = 2.0;
      Ki = 0.5;
      Kd = 0.8;
    }

    // Reset całki przy zmianie parametrów
    if (stare_Ki != Ki) {
      sum_e_L = 0;
      sum_e_R = 0;
    }

    // ================= LEWY SILNIK =================
    float aktualna_L = predkoscMierzonaLewa();
    float e_L = predkoscZadanaL - aktualna_L;
    
    // Anti-windup Lewy (Limit 200 zamiast 255)
    bool nasycenie_L = (pid_raw_L >= 200.0 && e_L > 0) || (pid_raw_L <= 50.0 && e_L < 0);
    if (!nasycenie_L) sum_e_L += e_L;

    // PID Lewy
    float P_L = Kp * e_L;
    float I_L = Ki * sum_e_L;
    float D_L = Kd * (e_L - prev_e_L);
    pid_raw_L = P_L + I_L + D_L;
    
    uint8_t pwm_L = 0;
    if (predkoscZadanaL > 0) {
      // Ograniczenie PWM do 200
      pwm_L = (uint8_t)constrain(pid_raw_L, 60, 200);
    } else {
      pwm_L = 0;
    }
    analogWrite(LEWY_PWM_PIN, pwm_L);
    prev_e_L = e_L;

    // ================= PRAWY SILNIK =================
    float aktualna_R = predkoscMierzonaPrawa();
    float e_R = predkoscZadanaR - aktualna_R;
    
    // Anti-windup Prawy (Limit 200 zamiast 255)
    bool nasycenie_R = (pid_raw_R >= 200.0 && e_R > 0) || (pid_raw_R <= 50.0 && e_R < 0);
    if (!nasycenie_R) sum_e_R += e_R;

    // PID Prawy
    float P_R = Kp * e_R;
    float I_R = Ki * sum_e_R;
    float D_R = Kd * (e_R - prev_e_R);
    pid_raw_R = P_R + I_R + D_R;
    
    uint8_t pwm_R = 0;
    if (predkoscZadanaR > 0) {
      // Ograniczenie PWM do 200
      pwm_R = (uint8_t)constrain(pid_raw_R, 60, 200);
    } else {
      pwm_R = 0;
    }
    analogWrite(PRAWY_PWM_PIN, pwm_R);
    prev_e_R = e_R;

    // 6. Wykres (Format: ZadanaL MierzonaL ZadanaR MierzonaR)
    Serial.print(predkoscZadanaL); Serial.print(" ");
    Serial.print(aktualna_L);      Serial.print(" ");
    Serial.print(predkoscZadanaR); Serial.print(" ");
    Serial.println(aktualna_R);
  }
}

// --- OBSŁUGA PRZERWAŃ ---

void przerwanieLewe() {
  uint32_t now = millis();
  pomiary_L[numer_L] = (uint16_t)(now - czas_ost_impulsu_L);
  czas_ost_impulsu_L = now;
  numer_L++; if (numer_L > 3) numer_L = 0;
}

void przerwaniePrawe() {
  uint32_t now = millis();
  pomiary_R[numer_R] = (uint16_t)(now - czas_ost_impulsu_R);
  czas_ost_impulsu_R = now;
  numer_R++; if (numer_R > 3) numer_R = 0;
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