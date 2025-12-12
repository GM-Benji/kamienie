#include <stdint.h>

// --- KONFIGURACJA PINÓW ---
const uint8_t LEWY_PWM_PIN = 5;
const uint8_t PRAWY_PWM_PIN = 6;
const uint8_t LEWY_ENCODER_PIN = 2;
const uint8_t PRAWY_ENCODER_PIN = 3;

const float WSPOLCZYNNIK = 4114.28;

// --- PARAMETRY KINEMATYCZNE ROBOTA ---
// UWAGA: Zmierz rzeczywisty rozstaw kół (D) swojego robota! 
const float ROZSTAW_KOL = 90.0; // mm (przykładowa wartość, do zmiany)

// --- ZMIENNE PID ---
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// Osobne prędkości zadane dla kół
float predkoscZadanaL = 0.0; 
float predkoscZadanaR = 0.0;

// --- ZMIENNE POMIAROWE ---
volatile uint32_t czas_ost_impulsu_L = 0;
volatile uint16_t pomiary_L[4] = {0};
volatile uint8_t numer_L = 0;

volatile uint32_t czas_ost_impulsu_R = 0;
volatile uint16_t pomiary_R[4] = {0};
volatile uint8_t numer_R = 0;

// Deklaracje funkcji
float predkoscMierzonaLewa();
float predkoscMierzonaPrawa();
void przerwanieLewe();
void przerwaniePrawe();
void jazdaPoLuku(float predkoscSrodka, float promienSkretu);

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
  // --- REALIZACJA RUCHU PO ŁUKU ---
  // Przykład: Prędkość 60 RPM, Promień skrętu 300 mm
  // Jeśli Promień jest dodatni -> Skręt w PRAWO (Lewe koło szybsze)
  // Jeśli chcesz skręcać w lewo, logika w funkcji musiałaby zostać odwrócona 
  // lub promienSkretu potraktowany odpowiednio.
  // Tutaj zakładamy konfigurację z Rys. 23 (skręt w prawo).
  
  jazdaPoLuku(100.0, 500.0); 

  // --- REGULATOR PID (cykl 10ms) ---
  static uint32_t last_pid = 0;
  if (millis() - last_pid >= 10) {
    last_pid = millis();

    // Pamięć regulatorów
    static float sum_e_L = 0, prev_e_L = 0, pid_raw_L = 0;
    static float sum_e_R = 0, prev_e_R = 0, pid_raw_R = 0;

    // 1. Dobór parametrów (Gain Scheduling)
    float srednia_zadana = (predkoscZadanaL + predkoscZadanaR) / 2.0;
    float stare_Ki = Ki;

    if (srednia_zadana <= 50.0) {
      Kp = 0.7; Ki = 0.1; Kd = 0.7;
    } else {
      Kp = 2.0; Ki = 0.5; Kd = 0.8;
    }

    if (stare_Ki != Ki) { sum_e_L = 0; sum_e_R = 0; }

    // ================= LEWY SILNIK =================
    float aktualna_L = predkoscMierzonaLewa();
    float e_L = predkoscZadanaL - aktualna_L;
    
    bool nasycenie_L = (pid_raw_L >= 200.0 && e_L > 0) || (pid_raw_L <= 50.0 && e_L < 0);
    if (!nasycenie_L) sum_e_L += e_L;

    float P_L = Kp * e_L;
    float I_L = Ki * sum_e_L;
    float D_L = Kd * (e_L - prev_e_L);
    pid_raw_L = P_L + I_L + D_L;
    
    uint8_t pwm_L = 0;
    if (predkoscZadanaL > 0) pwm_L = (uint8_t)constrain(pid_raw_L, 60, 200);
    analogWrite(LEWY_PWM_PIN, pwm_L);
    prev_e_L = e_L;

    // ================= PRAWY SILNIK =================
    float aktualna_R = predkoscMierzonaPrawa();
    float e_R = predkoscZadanaR - aktualna_R;
    
    bool nasycenie_R = (pid_raw_R >= 200.0 && e_R > 0) || (pid_raw_R <= 50.0 && e_R < 0);
    if (!nasycenie_R) sum_e_R += e_R;

    float P_R = Kp * e_R;
    float I_R = Ki * sum_e_R;
    float D_R = Kd * (e_R - prev_e_R);
    pid_raw_R = P_R + I_R + D_R;
    
    uint8_t pwm_R = 0;
    if (predkoscZadanaR > 0) pwm_R = (uint8_t)constrain(pid_raw_R, 60, 200);
    analogWrite(PRAWY_PWM_PIN, pwm_R);
    prev_e_R = e_R;

    // 6. Wykres
    Serial.print(0);Serial.print(" ");
    Serial.print(predkoscZadanaL); Serial.print(" ");
    Serial.print(aktualna_L);      Serial.print(" ");
    Serial.print(predkoscZadanaR); Serial.print(" ");
    Serial.println(aktualna_R);
  }
}

// --- KINEMATYKA (Zadanie 4) ---
/*
 * Funkcja oblicza prędkości kół dla zadanego promienia skrętu R_c
 * i prędkości środka robota v.
 * Bazuje na wzorach (2) i (3) z instrukcji.
 * Dla skrętu w prawo:
 * Lewe koło (zewnętrzne) musi jechać szybciej.
 * Prawe koło (wewnętrzne) musi jechać wolniej.
 */
void jazdaPoLuku(float predkoscSrodka, float promienSkretu) {
  // Zabezpieczenie przed dzieleniem przez zero (promień 0 to obrót w miejscu - wymaga innej logiki)
  if (promienSkretu < ROZSTAW_KOL) promienSkretu = ROZSTAW_KOL; 

  // Wzór wyprowadzony z sekcji 4.2 Kinematyka ruchu po okręgu 
  // v_L = v * (R + D/2) / R
  predkoscZadanaL = predkoscSrodka * (promienSkretu + (ROZSTAW_KOL / 2.0)) / promienSkretu;
  
  // v_R = v * (R - D/2) / R
  predkoscZadanaR = predkoscSrodka * (promienSkretu - (ROZSTAW_KOL / 2.0)) / promienSkretu;
}

// --- POZOSTAŁE FUNKCJE (BEZ ZMIAN) ---
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
float predkoscMierzonaLewa() {
  uint32_t now = millis();
  noInterrupts(); uint32_t last = czas_ost_impulsu_L; interrupts();
  if (now - last > 300) return 0.0;
  float suma = 0;
  noInterrupts(); for(uint8_t i=0; i<4; i++) suma += pomiary_L[i]; interrupts();
  return (suma == 0) ? 0.0 : (WSPOLCZYNNIK / suma);
}
float predkoscMierzonaPrawa() {
  uint32_t now = millis();
  noInterrupts(); uint32_t last = czas_ost_impulsu_R; interrupts();
  if (now - last > 300) return 0.0;
  float suma = 0;
  noInterrupts(); for(uint8_t i=0; i<4; i++) suma += pomiary_R[i]; interrupts();
  return (suma == 0) ? 0.0 : (WSPOLCZYNNIK / suma);
}