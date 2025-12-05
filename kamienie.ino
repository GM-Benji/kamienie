#include <stdint.h>

const uint8_t PWM_PIN = 9;
const uint8_t ENCODER_PIN = 2;
const float WSPOLCZYNNIK = 4114.28;

float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;
float predkoscZadana = 30.0;

volatile uint32_t czas_ost_impulsu = 0;
volatile uint16_t pomiary[4] = {0};
volatile uint8_t numer = 0;

float predkoscMierzona();
void przerwanie();

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), przerwanie, CHANGE);
  Serial.begin(9600);
}

void loop() {
  static uint32_t timer_sekwencji = 0;
  static uint8_t etap = 0;

  if (millis() - timer_sekwencji > 4000) {
    timer_sekwencji = millis();
    etap++;
    if (etap > 5) etap = 0;

    switch(etap) {
      case 0: predkoscZadana = 30.0; break;
      case 1: predkoscZadana = 100.0; break;
      case 2: predkoscZadana = 20.0; break;
      case 3: predkoscZadana = 40.0; break;
      case 4: predkoscZadana = 120.0; break;
      case 5: predkoscZadana = 70.0; break;


    }
  }

  static uint32_t last_pid = 0;
  if (millis() - last_pid >= 10) {
    last_pid = millis();

    // Deklaracja zmiennych pamięci regulatora przeniesiona na górę bloku,
    // aby można było zresetować sum_e w sekcji doboru parametrów.
    static float sum_e = 0;
    static float prev_e = 0;
    static float pid_raw = 0;

    // 1. Dobór parametrów (Gain Scheduling) z wykrywaniem zmiany
    float stare_Ki = Ki; // Zapamiętaj poprzednie Kp

    if (predkoscZadana <= 50.0) {
      Kp = 0.7;
      Ki = 0.1;
      Kd = 0.7;
    } else {
      Kp = 2.0;
      Ki = 0.5;
      Kd = 0.8;
    }

    // Jeśli Kp uległo zmianie (zmieniliśmy tryb), zerujemy całkę
    if (stare_Ki != Ki) {
      sum_e = 0;
    }

    // 2. Pomiary i uchyb
    float aktualna = predkoscMierzona();
    float e = predkoscZadana - aktualna;
    
    // 3. Anti-windup
    bool nasycenie = (pid_raw >= 255.0 && e > 0) || (pid_raw <= 50.0 && e < 0);
    if (!nasycenie) sum_e += e;

    // 4. Obliczenie PID
    float P = Kp * e;
    float I = Ki * sum_e;
    float D = Kd * (e - prev_e);
    pid_raw = P + I + D;
    uint8_t pwm;
    // 5. Wykonanie
    if (predkoscZadana>0) {
    pwm= (uint8_t)constrain(pid_raw, 60, 255);
    }
    else {
    pwm =0;
    }
    
    analogWrite(PWM_PIN, pwm);
    prev_e = e;

    // 6. Wykres
    Serial.print(0); Serial.print(" ");
    Serial.print(predkoscZadana); Serial.print(" ");
    Serial.println(aktualna);
  }
}

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