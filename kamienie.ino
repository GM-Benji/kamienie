#include <stdint.h>

// --- Konfiguracja ---
const uint8_t PWM_PIN = 9; 
const uint8_t ENCODER_PIN = 2;
const uint32_t SAMPLING_TIME = 10; // 10ms
const float WSPOLCZYNNIK = 4114.28; // Kalibracja z Lab 2

// --- Zmienne Globalne ---
float Kp = 1.0;          // Wzmocnienie P (na start małe)
float predkoscZadana = 50.0; 

// --- Zmienne Enkodera ---
volatile uint32_t czas_ost_impulsu = 0; 
volatile uint16_t pomiary[4] = {0}; 
volatile uint8_t numer = 0;

// Prototypy
float predkoscMierzona();
void przerwanie();

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), przerwanie, CHANGE);
  
  Serial.begin(9600);
  Serial.println(F("Zadanie 3.6.1 - Regulator P. Komendy: S=Predkosc, P=Kp"));
}

void loop() {
  // --- Obsługa Komunikacji (Zadanie 3.6.2) --- [cite: 561-576]
  if (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    float val = Serial.parseFloat();
    switch(cmd) {
      case 'S': case 's': predkoscZadana = val; break;
      case 'P': case 'p': Kp = val; break;
    }
  }

  // --- Pętla Regulacji ---
  static uint32_t last_pid = 0;
  if (millis() - last_pid >= SAMPLING_TIME) {
    last_pid = millis();

    float aktualna = predkoscMierzona();
    float e = predkoscZadana - aktualna;

    // Tylko człon P [cite: 498]
    float sterowanie = Kp * e;

    uint8_t pwm = (uint8_t)constrain(sterowanie, 0, 255);
    analogWrite(PWM_PIN, pwm);

    // Wykres [cite: 421-424]
    Serial.print(0); Serial.print(" ");
    Serial.print(predkoscZadana); Serial.print(" ");
    Serial.println(aktualna);
  }
}

// --- Obsługa Enkodera ---
void przerwanie() {
  uint32_t now = millis();
  pomiary[numer] = (uint16_t)(now - czas_ost_impulsu);
  czas_ost_impulsu = now;
  numer++; if (numer > 3) numer = 0;
}

float predkoscMierzona() {
  uint32_t now = millis();
  noInterrupts(); uint32_t last = czas_ost_impulsu; interrupts();
  if (now - last > 300) return 0.0;
  
  float suma = 0;
  noInterrupts(); for(uint8_t i=0; i<4; i++) suma += pomiary[i]; interrupts();
  return (suma == 0) ? 0.0 : (WSPOLCZYNNIK / suma);
}
