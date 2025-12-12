#include <stdint.h>

// --- KONFIGURACJA PINÓW ---
const uint8_t LEWY_PWM_PIN = 5;      // Silnik lewy
const uint8_t PRAWY_PWM_PIN = 6;     // Silnik prawy
const uint8_t LEWY_ENCODER_PIN = 2;  // Lewy enkoder
const uint8_t PRAWY_ENCODER_PIN = 3; // Prawy enkoder

const float WSPOLCZYNNIK = 4114.28;

// --- PARAMETRY MECHANICZNE I GEOMETRIA ---
const float SREDNICA_KOLA_CM = 4.2;       // Typowa średnica koła (w cm)
const float ROZSTAW_KOL_CM = 9.0;        // D: Rozstaw kół w cm (zmienna D z instrukcji) 
const float PRZEKLADNIA = 14.58;          
const float ZMIANY_NA_OBROT_ENKODERA = 4.0; 

// Współczynnik: ile impulsów przypada na 1 cm drogi
const float IMPULSY_NA_CM = (ZMIANY_NA_OBROT_ENKODERA * PRZEKLADNIA) / (SREDNICA_KOLA_CM * 3.14159);

// --- KONFIGURACJA ZADANIA (ŁUK) ---
// Parametry ruchu zadanego w zadaniu 5 (sekcja 4.3) [cite: 746]
float PROMIEN_LUKU_CM = 20.0;   // Rc: Promień skrętu [cite: 723]
float DLUGOSC_LUKU_CM = PROMIEN_LUKU_CM*3.14*2;  // Zadana długość łuku do przejechania
float PREDKOSC_BAZOWA = 100.0;  // Prędkość szybszego koła (zewnętrznego)

bool cel_osiagniety = false;

// --- ZMIENNE PID ---
float Kp = 0.0, Ki = 0.0, Kd = 0.0;
float predkoscZadanaL = 0.0;
float predkoscZadanaR = 0.0;

// --- ZMIENNE POMIAROWE ---
volatile uint32_t czas_ost_impulsu_L = 0, czas_ost_impulsu_R = 0;
volatile uint16_t pomiary_L[4] = {0}, pomiary_R[4] = {0};
volatile uint8_t numer_L = 0, numer_R = 0;
volatile long licznik_impulsow_L = 0, licznik_impulsow_R = 0;

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
  
  // --- OBLICZENIE PRĘDKOŚCI DLA ŁUKU (Kinematyka) ---
  // Wzory (2) i (3) z instrukcji 
  // vR = omega * (Rc - D/2)
  // vL = omega * (Rc + D/2)
  // Ponieważ prędkość obrotowa koła jest proporcjonalna do liniowej, stosujemy te same proporcje.
  
  float D = ROZSTAW_KOL_CM;
  float Rc = PROMIEN_LUKU_CM;
  
  // Obliczamy prędkości teoretyczne dla lewego i prawego koła
  float v_theory_L = Rc + (D / 2.0);
  float v_theory_R = Rc - (D / 2.0);
  
  // Skalujemy tak, aby szybsze koło miało PREDKOSC_BAZOWA
  if (v_theory_L >= v_theory_R) {
    // Skręt w prawo (lewe koło po zewnętrznej)
    predkoscZadanaL = PREDKOSC_BAZOWA;
    predkoscZadanaR = PREDKOSC_BAZOWA * (v_theory_R / v_theory_L);
  } else {
    // Skręt w lewo (prawe koło po zewnętrznej - np. ujemny promień lub inna logika)
    predkoscZadanaR = PREDKOSC_BAZOWA;
    predkoscZadanaL = PREDKOSC_BAZOWA * (v_theory_L / v_theory_R);
  }

  Serial.print("Start luku. Promien: "); Serial.print(Rc);
  Serial.print(" cm. Dystans: "); Serial.println(DLUGOSC_LUKU_CM);
  Serial.print("Predkosc L: "); Serial.print(predkoscZadanaL);
  Serial.print(" Predkosc R: "); Serial.println(predkoscZadanaR);
}

void loop() {
  // --- KONTROLA DYSTANSU ---
  // Droga środka robota to średnia z dróg kół
  long srednia_impulsow = (licznik_impulsow_L + licznik_impulsow_R) / 2;
  float przebyty_dystans_cm = (float)srednia_impulsow / IMPULSY_NA_CM;
  
  if (!cel_osiagniety) {
    if (przebyty_dystans_cm >= DLUGOSC_LUKU_CM) {
      // STOP
      predkoscZadanaL = 0;
      predkoscZadanaR = 0;
      cel_osiagniety = true;
    } 
    // Jeśli nie osiągnięto celu, prędkości pozostają takie, jak wyliczono w setup()
  }

  // --- REGULATOR PID (cykl 10ms) ---
  static uint32_t last_pid = 0;
  if (millis() - last_pid >= 10) {
    last_pid = millis();

    // Pamięć regulatorów
    static float sum_e_L = 0, prev_e_L = 0, pid_raw_L = 0;
    static float sum_e_R = 0, prev_e_R = 0, pid_raw_R = 0;

    // Dobór nastaw (Gain Scheduling)
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
    
    // Anti-windup L
    bool nasycenie_L = (pid_raw_L >= 200.0 && e_L > 0) || (pid_raw_L <= 50.0 && e_L < 0);
    if (!nasycenie_L) sum_e_L += e_L;
    
    float P_L = Kp * e_L;
    float I_L = Ki * sum_e_L;
    float D_L = Kd * (e_L - prev_e_L);
    pid_raw_L = P_L + I_L + D_L;
    
    uint8_t pwm_L = 0;
    if (predkoscZadanaL > 0) {
      pwm_L = (uint8_t)constrain(pid_raw_L, 60, 200);
    } else {
      pwm_L = 0; pid_raw_L = 0; sum_e_L = 0;
    }
    analogWrite(LEWY_PWM_PIN, pwm_L);
    prev_e_L = e_L;

    // ================= PRAWY SILNIK =================
    float aktualna_R = predkoscMierzonaPrawa();
    float e_R = predkoscZadanaR - aktualna_R;
    
    // Anti-windup R
    bool nasycenie_R = (pid_raw_R >= 200.0 && e_R > 0) || (pid_raw_R <= 50.0 && e_R < 0);
    if (!nasycenie_R) sum_e_R += e_R;
    
    float P_R = Kp * e_R;
    float I_R = Ki * sum_e_R;
    float D_R = Kd * (e_R - prev_e_R);
    pid_raw_R = P_R + I_R + D_R;
    
    uint8_t pwm_R = 0;
    if (predkoscZadanaR > 0) {
      pwm_R = (uint8_t)constrain(pid_raw_R, 60, 200);
    } else {
      pwm_R = 0; pid_raw_R = 0; sum_e_R = 0;
    }
    analogWrite(PRAWY_PWM_PIN, pwm_R);
    prev_e_R = e_R;

    // Wykres
    Serial.print(0); Serial.print(" ");
    Serial.print(predkoscZadanaL); Serial.print(" ");
    Serial.print(aktualna_L);      Serial.print(" ");
    Serial.print(predkoscZadanaR); Serial.print(" ");
    Serial.print(aktualna_R);      Serial.print(" ");
    Serial.println(przebyty_dystans_cm);
  }
}

// --- OBSŁUGA PRZERWAŃ ---
void przerwanieLewe() {
  uint32_t now = millis();
  pomiary_L[numer_L] = (uint16_t)(now - czas_ost_impulsu_L);
  czas_ost_impulsu_L = now;
  numer_L++; if (numer_L > 3) numer_L = 0;
  licznik_impulsow_L++;
}

void przerwaniePrawe() {
  uint32_t now = millis();
  pomiary_R[numer_R] = (uint16_t)(now - czas_ost_impulsu_R);
  czas_ost_impulsu_R = now;
  numer_R++; if (numer_R > 3) numer_R = 0;
  licznik_impulsow_R++;
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