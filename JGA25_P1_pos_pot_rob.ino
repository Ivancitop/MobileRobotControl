/*
  CONTROLADOR PID PARA REGULACIÓN DE LA POSICIÓN
  DE UN MOTOR DE CD MODELO JGA25-175

  Trayectoria senoidal:
  th_des(t) = amplitud * sin(2*pi*frecuencia*t)

  TECNOLÓGICO DE MONTERREY CAMPUS GUADALAJARA
  PROFESOR: DR. JOSE LUIS LUNA PINEDA
*/

// ----- TIEMPO DE MUESTREO -----
const int dt_us = 4000;
const float dt = dt_us * 0.000001;
unsigned long t1 = 0, t2 = 0;

// ----- TRAYECTORIA SENOIDAL -----
float amplitud = 45.0;
float frecuencia = 0.05;
float t_trayectoria = 0.0;
unsigned long t_inicio;

// ----- ENCODER -----
volatile int Np = 0;
const float R = 360.0 / (2.0 * 17.0 * 49.0);
float th = 0.0, thp = 0.0;
float error_threshold = 1.0 * R;

// ----- VELOCIDAD -----
float dth_d = 0.0, dth_f = 0.0;
float alpha = 0.03;

// ----- CONTROL PID -----
float kp = 0.0, kd = 0.0, ki = 0.0;
float e = 0.0, de = 0.0, inte = 0.0;
float u = 0.0, usat = 0.0;
float PWM = 0.0;
float th_des = 0.0;

// ----- AJUSTE DINÁMICO DE kp, kd y ki -----
const int pin_kp = 32;
const int pin_kd = 34;
const int pin_ki = 35;

// ----- PINES DE CONTROL -----
const int sen1 = 13;
const int sen2 = 27;

// ----- INTERRUPCIONES -----
void IRAM_ATTR CH_A();
void IRAM_ATTR CH_B();

void setup() {
  Serial.begin(115200);

  pinMode(25, INPUT_PULLUP);
  pinMode(26, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(25), CH_A, RISING);
  attachInterrupt(digitalPinToInterrupt(26), CH_B, RISING);

  ledcAttach(13, 10000, 12);
  ledcAttach(27, 10000, 12);

  t_inicio = micros();

  ledcWrite(sen1, 0);
  ledcWrite(sen2, 0);
}

void loop() {

  // ----- INICIO DEL PERIODO DE MUESTREO -----
  t1 = micros();

  // ----- PARADA DE EMERGENCIA -----
  if (Serial.available() > 0) {
    char tecla = Serial.read();

    if (tecla == 'q' || tecla == 'Q') {
      ledcWrite(sen1, 0);
      ledcWrite(sen2, 0);

      while (true) {
        ledcWrite(sen1, 0);
        ledcWrite(sen2, 0);
        delay(100);
      }
    }
  }

  // ----- TRAYECTORIA SENOIDAL -----
  t_trayectoria = (micros() - t_inicio) * 0.000001;
  th_des = amplitud * sin(2.0 * PI * frecuencia * t_trayectoria);

  // ----- AJUSTE DINÁMICO DE kp, kd y ki -----
  kp = analogRead(pin_kp) * (25.0 / 4095.0);
  kd = analogRead(pin_kd) * (2.0 / 4095.0);
  ki = analogRead(pin_ki) * (10.0 / 4095.0);

  // ----- POSICIÓN -----
  th = R * Np;

  // ----- VELOCIDAD -----
  dth_d = (th - thp) / dt;
  dth_f = alpha * dth_d + (1.0 - alpha) * dth_f;

  // ----- PID -----
  e = th_des - th;
  de = -dth_f;
  inte += e * dt;

  u = kp * e + kd * de + ki * inte;

  // ----- SATURACIÓN -----
  usat = constrain(u, -4095, 4095);
  PWM = usat;

  // ----- CONTROL PWM -----
  if (abs(e) < error_threshold) {
    ledcWrite(sen1, 0);
    ledcWrite(sen2, 0);
  } 
  else {

    if (PWM > 0) {
      ledcWrite(sen1, (int)PWM);
      ledcWrite(sen2, 0);
    }

    if (PWM < 0) {
      ledcWrite(sen1, 0);
      ledcWrite(sen2, (int)(-PWM));
    }

    if (PWM == 0) {
      ledcWrite(sen1, 0);
      ledcWrite(sen2, 0);
    }
  }

  // ----- ACTUALIZACIONES -----
  thp = th;

  // ----- SERIAL PLOTTER -----
  Serial.print(th_des);
  Serial.print(" ");
  Serial.print(th);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.println(PWM);

  // ----- ESPERA DE MUESTREO -----
  t2 = micros();

  while ((t2 - t1) < dt_us) {
    t2 = micros();
  }
}

// ----- INTERRUPCIÓN CANAL A -----
void IRAM_ATTR CH_A() {
  if (digitalRead(26) == LOW) Np++;
  if (digitalRead(26) == HIGH) Np--;
}

// ----- INTERRUPCIÓN CANAL B -----
void IRAM_ATTR CH_B() {
  if (digitalRead(25) == HIGH) Np++;
  if (digitalRead(25) == LOW) Np--;
}