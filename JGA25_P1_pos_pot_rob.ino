/* 
  CONTROLADOR PID PARA REGULACIÓN DE LA POSICIÓN DE UN MOTOR DE CD MODELO JGA25-175
  TECNOLÓGICO DE MONTERREY CAMPUS GUADALAJARA
  PROFESOR: DR. JOSE LUIS LUNA PINEDA

  Este programa implementa un control PID para posicionar un motor de corriente directa
  usando lectura de encoder en cuadratura doble (X2). La señal de control se calcula como:
  u = kP*E - kD*dth_f + kI*int(e)

  Se incluye filtrado de velocidad, saturación de señal PWM y lectura de consigna por monitor serial.
*/

// ----- TIEMPO DE MUESTREO -----
int dt_us = 4000;                      // Muestreo cada 4 ms----> para este caso puede ser entre 1 ms y 5 ms
float dt = dt_us * 0.000001;           // Tiempo en segundos para el cálculo de la derivada y la integral
unsigned long t1 = 0, t2 = 0;          // Estos tiempos son para crear un tiempo de muestreo constante "estable"

// ----- ENCODER -----
volatile int Np = 0;                   // Conteo de pulsos del encoder
// En este programa solo se toma dos flancos de subida, CUADRATURA doble
const float R = (360.0/(2.0*17.0*49.0));  // Resolución angular (R)=  360°/(cuadratura_doble*pulsos_encoder*gear ratio)
// Para el MOTOR DE CD MODELO JGA25-175: 360.0/(2.0*11.0*34.0)
float th = 0, thp = 0;                 // th es ángulo actual.---> thp es el valor del ángulo pasado.
float error_threshold = 1.0 * R;       // Umbral de seguridad: si el error es menor a 2 pulsos se apaga la señal PWM

// ----- VELOCIDAD -----
float dth_d = 0, dth_f = 0;
float alpha = 0.03;                    // Se usa para el filtro de velocidad entre 0 y 1
// alpha cercano a 1 → más reactivo (menos suave)    alpha cercano a 0 → más lento pero más suave

// ----- CONTROL PID -----
float kp = 0.0  , kd = 0.0, ki = 0.0;     // AJUSTE DE LAS GANANCIAS DEL CONTROLADOR PID
// RECUERDE USAR VALORES PEQUEÑOS DE K_d PARA NO AMPLIFICAR EL RUIDO DE MEDICIÓN
// SUGERENCIA: USAR ENTRADAS ANALÓGICAS PARA AJUSTARLAS USANDO POTENCIOMETROS, DELIMITE EL VALOR COMO SIGUE:
// 0<=K_p<=25----0<=K_d<=2-------0<=K_i<=10
float b_est= 155.696;
float a_est=34.970;
// ----- AJUSTE DINÁMICO DE kp kd y ki -----
const int pin_kp = 32;                 // Pin analógico para ajustar kp con potenciómetro
const int pin_kd = 34;                 // Pin analógico para ajustar kd con potenciómetro
const int pin_ki = 35;                 // Pin analógico para ajustar ki con potenciómetro

float e = 0, de = 0, inte = 0;
float u = 0, usat = 0;
float PWM = 0;
float th_des = 0;                      // EL VALOR DE REFERENCIA DESEADO INICIAL ES CERO. ESTABLEZCA LA REFERENCIA EN EL MONITOR SERIAL

// ----- PINES DE CONTROL ----- SIEMPRE VERIFIQUE LA DIRECCIÓN DEL MOTOR 
const int sen1 = 13;    // Dirección 1 EN EL CASO DE USAR EL PUENTE H BTS7986 CORRESPONDE LPWM O RPWM
const int sen2 = 27;    // Dirección 2 EN EL CASO DE USAR EL PUENTE H BTS7986 CORRESPONDE LPWM O RPWM

String consigna; // SIRVE PARA GUARDAR EL VALOR DEL MONITOR SERIAL COMO UN STRING

void IRAM_ATTR CH_A();
void IRAM_ATTR CH_B();

void setup() {
  Serial.begin(115200);

  pinMode(25, INPUT_PULLUP);  // Canal A del encoder.
  pinMode(26, INPUT_PULLUP);  // Canal B

  attachInterrupt(digitalPinToInterrupt(25), CH_A, RISING); // Generación de las interrupciones para el pin 2 cuando detecta un flanco de subida
  attachInterrupt(digitalPinToInterrupt(26), CH_B, RISING);

  // Asignación de los pines usados como PWM como salidas
  ledcAttach(13,10000,12);
  ledcAttach(27,10000,12);
}

void loop() {

   if (Serial.available() > 0) {
    char tecla = Serial.read();

    if (tecla == 'q' || tecla == 'Q') {
      Serial.end();  // Detiene la comunicación Serial
    }
  }
  
  t1 = micros();  // Marca de inicio

  // ----- LECTURA DE CONSIGNA -----
  if (Serial.available() > 0) {
    consigna = Serial.readStringUntil('\n');
    th_des = consigna.toFloat(); // Toma el valor del monitor serial y lo guarda en la variable, inicialmente es cero.
  }

// ----- AJUSTE DINÁMICO DE kp kd y ki -----
   kp = analogRead(pin_kp) * (200.0 / 4095.0); // Escala de 0 a 25 usando potenciómetro
   kd = analogRead(pin_kd) * (10.0 / 4095.0); // Escala de 0 a 1 usando potenciómetro
   ki = analogRead(pin_ki) * (130.0 / 4095.0); // Escala de 0 a 15 usando potenciómetro

  // ----- POSICIÓN Y VELOCIDAD -----
  th = R * Np; // Np se calcula con las rutinas de interrupción, nos da los pulsos, al multiplicarlo por R obtenemos el ángulo en grados
  // Si cambiamos 360 por 1, medimos revoluciones.
  dth_d = (th - thp) / dt;

  // Filtro de primer orden tipo IIR (filtro exponencial) o también conocido como filtro de media exponencial móvil
  // Ecuación general ----> y[n]=alpha*x[n]+(1-alpha)y[n-1]
  dth_f = alpha * dth_d + (1 - alpha) * dth_f;
  //   Si modelamos el filtro como:
// D_f(s) = \frac{α}{s + α}·D(s)
// donde:
// - D(s) es la velocidad angular sin filtrar (entrada),
// - D_f(s) es la salida filtrada,
// - α representa el coeficiente que regula la suavidad (en segundos⁻¹)

// Interpretación física
// - Este filtro atenua frecuencias altas (ruido) y pasa las bajas (variaciones reales).
// - La constante de tiempo es τ = \frac{1}{α}, lo que define qué tan rápido responde el sistema.
// - Si α es pequeño, el filtro es más lento (más suavizado). Si α es grande, responde más rápido pero suaviza menos.

// Es un equivalente del filtro analógico de primer orden transformado por el método de diferencia finita (Euler).

  // ----- PID ----- 
  e = th_des - th;         // PRIMERO CALCULAMOS EL ERROR PARA PODER GENERAR LA SEÑAL DE CONTROL U
  de = -dth_f;             // USAMOS LA DERIVADA DE LA SALIDA PERO FILTRADA
  inte += e * dt;          // CALCULAMOS EL VALOR DE LA INTEGRAL
   u =( kp * e + kd * de  + ki * inte); // OBTENEMOS EL VALOR DE LA SEÑAL DE CONTROL
  //u = (1.0 / b_est) * (kp * e - kd * dth_f - beta * dth_f + beta * kp * inte - beta * kd * th);
  //u =(1.0/b_est)*( kp * e + kd * de + ki * inte); // OBTENEMOS EL VALOR DE LA SEÑAL DE CONTROL
  // RECUERDE QUE TENEMOS VALORES LÍMITE ENTRE -255 Y 255 POR LO TANTO DEBEMOS CUIDAR EL VALOR
  // QUE USAMOS EN LAS GANANCIAS KP, KD Y KI PARA NO SATURAR EL SISTEMA

  usat = constrain(u, -4095, 4095); // Esto porque el PWM en Arduino es de 8 bits por lo tanto va de 0 a 255.
  PWM = usat;


  // ----- CONTROL PWM CON DETECCIÓN DE UMBRAL DE ERROR -----
  if (abs(e) < error_threshold) {
    // Error pequeño: apaga PWM
    ledcWrite(sen1, 0);
    ledcWrite(sen2, 0);
  } else {
    if (PWM > 0) {
      ledcWrite(sen1, PWM);
      ledcWrite(sen2, 0);
    }

    if (PWM < 0) {
      ledcWrite(sen1, 0);
      ledcWrite(sen2, -PWM);
    }

    if (PWM == 0) {
      ledcWrite(sen1, 0);
      ledcWrite(sen2, 0);
    }
  }

  // ----- ACTUALIZACIONES -----
  thp = th;

 Serial.print(kp); Serial.print(' '); 
 Serial.print(kd); Serial.print(' '); 
 Serial.print(ki); Serial.print(' '); 
 Serial.print(th_des); Serial.print(' '); 
 Serial.print(th); Serial.print(' '); 
 Serial.println(PWM);

  // Espera activa para mantener la frecuencia
  t2 = micros();
  while ((t2 - t1) < dt_us) {
    t2 = micros();
  }
}

// Función que se ejecuta al detectar una subida en el canal A
void IRAM_ATTR CH_A() {
  if (digitalRead(26) == LOW)
    Np = Np + 1;
  if (digitalRead(26) == HIGH)
    Np = Np - 1;
}

// Función que se ejecuta al detectar una subida en el canal B
void IRAM_ATTR CH_B() {
  if (digitalRead(25) == HIGH)
    Np = Np + 1;
  if (digitalRead(25) == LOW)
    Np = Np - 1;
}
