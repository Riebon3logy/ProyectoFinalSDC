#include <Servo.h>

Servo myservo;

int period = 15;
float timePrev, timeNow;

float kp = 4.5;
float ki = 0;
float kd = 14;

// Setpoint inicial
float distance_setpoint = 22;

float distance = 0.0;
float distance_error = 0;
float distance_previous_error = 0;

float PID_p, PID_i, PID_d, PID_total;

void setup() {
  Serial.begin(9600);

  myservo.attach(11);
  myservo.write(125);  // posición inicial
  timeNow = millis();
}

void loop() {

  // --- Lectura de comandos desde Serial ---
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    procesarComando(comando);
  }

  if (millis() > timeNow + period) {
    timeNow = millis();

    // --- Lectura del sensor IR ---
    distance = calcularDistancia(20);

    // --- Cálculo del error ---
    distance_error = distance_setpoint - distance;

    // --- PID ---
    PID_p = kp * distance_error;

    float dist_difference = distance_error - distance_previous_error;
    PID_i += ki * distance_error;
    PID_d = kd * dist_difference;
    distance_previous_error = distance_error;

    PID_total = PID_p + PID_i + PID_d;

    // --- Mapeo a rango de servo ---
    PID_total = map(PID_total, -150, 150, 80, 180);

    // --- Límites del servo ---
    if (PID_total > 180) PID_total = 180;
    if (PID_total < 80) PID_total = 80;
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

void procesarComando(String cmd) {
  // Detectar si es comando de setpoint (empieza con 'S')
  if (cmd.startsWith("S")) {
    // Formato: "S22.0"
    String setpoint_str = cmd.substring(1);
    distance_setpoint = setpoint_str.toFloat();

    // Resetear integrador al cambiar setpoint
    PID_i = 0;

    Serial.print("Setpoint actualizado -> ");
    Serial.println(distance_setpoint);
    return;
  }

  // Si no, asumimos formato de PID: "kp,ki,kd"
  int c1 = cmd.indexOf(',');
  int c2 = cmd.indexOf(',', c1 + 1);

  if (c1 > 0 && c2 > c1) {
    // Formato: "kp,ki,kd"
    String kp_str = cmd.substring(0, c1);
    String ki_str = cmd.substring(c1 + 1, c2);
    String kd_str = cmd.substring(c2 + 1);

    kp = kp_str.toFloat();
    ki = ki_str.toFloat();
    kd = kd_str.toFloat();

    // Resetear integrador al cambiar parámetros
    PID_i = 0;

    // Confirmación
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

float calcularDistancia(int n) {
  long suma = 0;

  for (int i = 0; i < n; i++) {
    suma += analogRead(A0);
  }

  float adc = suma / n;

  float distancia_cm = 17569.7 * pow(adc, -1.2062);

  return distancia_cm;
}
