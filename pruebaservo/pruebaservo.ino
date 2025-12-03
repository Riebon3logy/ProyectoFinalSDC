#include <Servo.h>

Servo myServo;
int periodo = 15;
unsigned long tiempoAnterior = 0;

float kp = 3.111;
float ki = 0.11;
float kd = 23;

float referencia = 22;
float distancia = 0;
float error_distancia = 0;
float integral = 0;
float maxIntegral = 50;
float error_anterior = 0;
float PID_p, PID_i, PID_d, PID_total;

const int anguloMedio = 45;
const int anguloMax = 0;
const int anguloMin = 70;

void setup() {
  Serial.begin(9600);
  myServo.attach(11);
  myServo.write(anguloMedio);
}

void loop() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    procesarComando(comando);
  }

  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= periodo) {
    tiempoAnterior = tiempoActual;
    distancia = calcularDistancia(50);
    if (distancia > 42 && distancia < 47) distancia = 44;
    error_distancia = referencia - distancia;
    PID_p = kp * error_distancia;
    integral += error_distancia;
    integral = constrain(integral, -maxIntegral, maxIntegral);
    PID_i = ki * integral;
    PID_d = kd * (error_distancia - error_anterior);
    error_anterior = error_distancia;
    PID_total = PID_p + PID_i + PID_d + 45;
    int angulo = constrain(PID_total, anguloMax, anguloMin);
    Serial.println(tiempoActual);
    Serial.print("Distancia IR: ");
    Serial.print(distancia);
    Serial.println(" cm");
    Serial.print("Servo -> ");
    Serial.println(PID_total);
    Serial.print("Error -> ");
    Serial.println(error_distancia);
    myServo.write(angulo);
  }
}

void procesarComando(String cmd) {
  if (cmd.startsWith("S")) {
    String setpoint_str = cmd.substring(1);
    referencia = setpoint_str.toFloat();
    integral = 0;
    Serial.print("Setpoint -> ");
    Serial.println(referencia);
    return;
  }

  int c1 = cmd.indexOf(',');
  int c2 = cmd.indexOf(',', c1 + 1);

  if (c1 > 0 && c2 > c1) {
    kp = cmd.substring(0, c1).toFloat();
    ki = cmd.substring(c1 + 1, c2).toFloat();
    kd = cmd.substring(c2 + 1).toFloat();
    integral = 0;

    Serial.print("PID -> Kp="); Serial.print(kp);
    Serial.print(" Ki="); Serial.print(ki);
    Serial.print(" Kd="); Serial.println(kd);
  } else {
    Serial.println("Error: Use 'kp,ki,kd' o 'Ssetpoint'");
  }
}

float calcularDistancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma += analogRead(A0);
  }
  float adc = suma / (float)n;
  return 17569.7 * pow(adc, -1.2062);
}
