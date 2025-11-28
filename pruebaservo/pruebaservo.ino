#include <Servo.h>

// Objeto de tipo servo
Servo myservo;

// Intervalo de muestreo
int period = 20;

// Variables de tiempo de muestreo
float timePrev, timeNow;

// Constantes de ganancia
float kp = 7.5;
float ki = 0.03;
float kd = 31;

// Margen de error
float margen_error = 0.7;

// Setpoint inicial
float distance_setpoint = 22;

// Distancia actual
float distance = 0.0;

// Error actual de distancia
float distance_error = 0;

// Error anterior de distancia
float distance_previous_error = 0;

// Calculo de las acciones de control
float PID_p, PID_i, PID_d, PID_total;

void setup() {
  Serial.begin(9600);
  myservo.attach(11); // Pin para controlar el servo
  myservo.write(0);  // posición inicial
  timeNow = millis();
}

void loop() {
  // Lectura de comandos desde Serial
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    procesarComando(comando);
  }

  // Inicio de tiempo
  if (millis() > timeNow + period) {
    timeNow = millis();
    
    // Antiwindup
    if (PID_i > 5 || PID_i < 0) PID_i = 0;
    
    // Lectura del sharp
    distance = calcularDistancia(200);

    if (distance > 45.0) distance = 45;
    
    // Calculo del error
    distance_error = distance_setpoint - distance;

    // Margen de error
    if (abs(distance_error - margen_error) < 0.0) myservo.write(60);
    
    // Calculo de accion de control P
    PID_p = kp * distance_error;

    // Calculo de accion de control I
    float dist_difference = distance_error - distance_previous_error;
    PID_i += ki * distance_error;

    // Calculo de accion de control D
    PID_d = kd * dist_difference;
    distance_previous_error = distance_error;

    // Calculo de accion de control PID
    PID_total = PID_p + PID_i + PID_d;

    // Mapeo del angulo del servo
    PID_total = map(PID_total, -150, 150, 50, 125);

    // Limites del servo
    if (PID_total > 125) PID_total = 125;
    if (PID_total < 50) PID_total = 30;

    // Datos enviados a traves del puerto serial
    Serial.println(timeNow);
    Serial.print("Distancia IR: ");
    Serial.print(distance);
    Serial.println(" cm");
    Serial.print("Servo -> ");
    Serial.println(PID_total);
    Serial.print("Error -> ");
    Serial.println(distance_error);
    myservo.write(PID_total);
  }
}

// Procesar comando proveniente del puerto serial
void procesarComando(String cmd) {
  // Detectar si es comando de setpoint (empieza con 'S')
  if (cmd.startsWith("S")) {
    String setpoint_str = cmd.substring(1);
    distance_setpoint = setpoint_str.toFloat();
    Serial.print("Setpoint actualizado -> ");
    Serial.println(distance_setpoint);
    return;
  }
  
  int c1 = cmd.indexOf(',');
  int c2 = cmd.indexOf(',', c1 + 1);

  if (c1 > 0 && c2 > c1) {
    String kp_str = cmd.substring(0, c1);
    String ki_str = cmd.substring(c1 + 1, c2);
    String kd_str = cmd.substring(c2 + 1);

    kp = kp_str.toFloat();
    ki = ki_str.toFloat();
    kd = kd_str.toFloat();

    Serial.print("PID actualizado -> Kp=");
    Serial.print(kp);
    Serial.print(" Ki=");
    Serial.print(ki);
    Serial.print(" Kd=");
    Serial.println(kd);
  } else {
    Serial.println("Error: Formato invalido. Use 'kp,ki,kd' o 'Ssetpoint'");
  }
}

// Calculo de la distancia en CM usando los valores que da el sensor
float calcularDistancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma += analogRead(A0);
  }
  float adc = suma / n;
  float distancia_cm = 17569.7 * pow(adc, -1.207);
  return distancia_cm;
}
