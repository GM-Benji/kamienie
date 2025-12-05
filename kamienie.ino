#include <stdint.h>

const uint8_t PWM_PIN = 9;
const uint8_t ENCODER_PIN = 2;
const float WSPOLCZYNNIK = 4114.28;

float Kp = 2.0;
float Ki = 0.5;
float Kd = 0.1;
float predkoscZadana = 50.0;

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
  if (Serial.available()) {
    char cmd = Serial.read();
    float val = Serial.parseFloat();
    switch(cmd) {
      case 'S': case 's': predkoscZadana = val; break;
      case 'P': case 'p': Kp = val; break;
      case 'i': case 'I': Ki = val; break;
      case 'd': case 'D': Kd = val; break;
    }
  }

  static uint32_t last_pid = 0;
  if (millis() - last_pid >= 10) {
    last_pid = millis();

    float aktualna = predkoscMierzona();
    float e = predkoscZadana - aktualna;

    static float sum_e = 0;
    static float prev_e = 0;
    static float pid_raw = 0;

    bool nasycenie = (pid_raw >= 255.0 && e > 0) || (pid_raw <= 0.0 && e < 0);
    
    if (!nasycenie) {
      sum_e += e;
    }
    if (Ki == 0) sum_e = 0;

    float P = Kp * e;
    float I = Ki * sum_e;
    float D = Kd * (e - prev_e);

    pid_raw = P + I + D;

    uint8_t pwm = (uint8_t)constrain(pid_raw, 0, 255);
    analogWrite(PWM_PIN, pwm);
    
    prev_e = e;

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