#include <Servo.h>

Servo myservo;

int Read = 0;
float distance = 0.0;
float elapsedTime, time, timePrev;
float distance_previous_error, distance_error;
int period = 50;

float kp = 1;
float ki = 0;
float kd = 0;
float distance_setpoint = 18;

float PID_p, PID_i, PID_d, PID_total;

void setup() {
  Serial.begin(9600);  
  myservo.attach(11);
  myservo.write(125);
  time = millis();
}

void loop() {
  // Verificar si hay datos disponibles en el puerto serial
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    data.trim(); // Eliminar espacios en blanco
    
    // Parsear los valores separados por comas
    int firstComma = data.indexOf(',');
    int secondComma = data.indexOf(',', firstComma + 1);
    
    if (firstComma > 0 && secondComma > firstComma) {
      float new_kp = data.substring(0, firstComma).toFloat();
      float new_ki = data.substring(firstComma + 1, secondComma).toFloat();
      float new_kd = data.substring(secondComma + 1).toFloat();
      
      // Actualizar los valores solo si son válidos
      if (new_kp >= 0 && new_ki >= 0 && new_kd >= 0) {
        kp = new_kp;
        ki = new_ki;
        kd = new_kd;
        
        // Reiniciar el integrador cuando cambien los parámetros
        PID_i = 0;
        
        // Confirmar recepción
        Serial.print("PID actualizado - Kp: ");
        Serial.print(kp, 4);
        Serial.print(", Ki: ");
        Serial.print(ki, 4);
        Serial.print(", Kd: ");
        Serial.println(kd, 4);
      }
    }
  }
  
  if (millis() > time + period)
  {
    long tiempo_lectura = millis();      // Tiempo antes de iniciar la lectura
    time = millis();    
    distance = distancia(10);            // Lectura de distancia con promedio de 10 muestras
    tiempo_lectura = millis() - tiempo_lectura;  // Milisegundos que duró la lectura
    
    distance_error = distance_setpoint - distance;   
    
    // Mostrar información de la lectura
    Serial.print("Tiempo de lectura: ");
    Serial.print(tiempo_lectura);
    Serial.print("ms  Distancia: ");
    Serial.print(distance);
    Serial.println(" cm");

    PID_p = kp * distance_error;
    float dist_diference = distance_error - distance_previous_error;     
    PID_d = kd * (dist_diference / period);
      
    if (-3 < distance_error && distance_error < 3)
      PID_i += ki * distance_error;
    else
      PID_i = 0;
  
    PID_total = PID_p + PID_i + PID_d;  
    PID_total = map(PID_total, -150, 150, 0, 150);
    Serial.print("PID: ");
    Serial.println(PID_total);
    if (PID_total < 0) PID_total = 0;
    if (PID_total > 180) PID_total = 180;
  
    myservo.write(PID_total);  
    distance_previous_error = distance_error;
  }
}

// Función para leer distancia del sensor analógico Sharp GP2Y0A21YK
float distancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(A0);
  }
  float adc = suma / n;
  float distancia_cm = 17569.7 * pow(adc, -1.2062);
  return (distancia_cm);
}