#include <Servo.h>
#include <Ultrasonic.h>

Servo servo;
Ultrasonic ultrasonic(7, 8);

// Variables del sistema
int distanciaMedida;
int referencia = 22;  // cm - distancia objetivo

// Variables PID
double errorAcumulado = 0;
double errorAnterior = 0;
unsigned long tiempoActual = 0;
unsigned long tiempoAnterior = 0;
unsigned long tiempoInicioError = 0;

// Variables para suavizar movimiento del servo
int anguloActual = 100;        // Posición actual del servo
int anguloObjetivo = 90;      // Posición objetivo del servo
unsigned long tiempoServo = 0;

// Parámetros PID - ajusta según tu sistema
double kp = 0.50;    // Ganancia proporcional (reducida)
double ki = 0;   // Ganancia integral (reducida)
double kd = 2.95;    // Ganancia derivativa (reducida)

// Control de tiempo
const unsigned long INTERVALO_LECTURA = 15;  // ms entre lecturas (aumentado)
const unsigned long INTERVALO_SERVO = 10;    // ms entre movimientos del servo
const unsigned long TIEMPO_RESET_ERROR = 2000;  // 2 segundos

// Configuración del servo
const int POSICION_EQUILIBRIO = 100;   // Posición central del servo
const int LIMITE_MIN = 30;            // Límite mínimo seguro
const int LIMITE_MAX = 180;           // Límite máximo seguro
const int RANGO_PID_MAX = 180;         // Máximo ajuste desde equilibrio (reducido)

void setup() {
  Serial.begin(9600);
  servo.attach(11);
  
  // Inicializar posición del servo suavemente
  anguloActual = POSICION_EQUILIBRIO;
  anguloObjetivo = POSICION_EQUILIBRIO;
  servo.write(anguloActual);
  //delay(1000);  // Dar tiempo al servo para posicionarse
  
  // Inicializar tiempo
  tiempoAnterior = millis();
  tiempoInicioError = millis();
  tiempoServo = millis();
  
  Serial.println("=== Sistema PID Balancín Iniciado ===");
  Serial.print("Posición de equilibrio: ");
  Serial.print(POSICION_EQUILIBRIO);
  Serial.println("°");
  Serial.print("Referencia de distancia: ");
  Serial.print(referencia);
  Serial.println(" cm");
  // Serial.print("Velocidad del servo: ");
  // Serial.print(VELOCIDAD_SERVO);
  Serial.println("°/iteración");
  Serial.println("Distancia | Error | P     | I     | D     | PID   | Objetivo | Actual");
  Serial.println("----------|-------|-------|-------|-------|-------|----------|-------");
}

void loop() {
  tiempoActual = millis();
  
  // Ejecutar control PID cada INTERVALO_LECTURA ms
  if (tiempoActual - tiempoAnterior >= INTERVALO_LECTURA) {
    
    // Leer distancia del sensor ultrasónico
    distanciaMedida = ultrasonic.read();
    
    // Filtrar lecturas inválidas del sensor
    if (distanciaMedida < 2 || distanciaMedida > 400) {
      // Lectura inválida, mantener la anterior o usar un valor por defecto
      return;
    }
    
    // Calcular error
    double error = referencia - distanciaMedida;
    
    // Calcular delta de tiempo en segundos
    double deltaTime = (tiempoActual - tiempoAnterior) / 1000.0;
    
    // Término Proporcional
    double P = kp * error;
    
    // Término Integral - acumular error con respecto al tiempo
    errorAcumulado += error * deltaTime;
    
    // Reset del error acumulado si ha pasado mucho tiempo
    if(ki>0){
      
      if (tiempoActual - tiempoInicioError >= TIEMPO_RESET_ERROR) {
        errorAcumulado = 0;
        tiempoInicioError = tiempoActual;
        Serial.println(">>> Error integral reseteado <<<");
      }
      
      // Limitador anti-windup para el término integral
      double maxIntegral = RANGO_PID_MAX / ki;
      if (errorAcumulado > maxIntegral) {
        errorAcumulado = maxIntegral;
      } else if (errorAcumulado < -maxIntegral) {
        errorAcumulado = -maxIntegral;
      }
    }
    double I = ki * errorAcumulado;
    
    // Término Derivativo - cambio del error con respecto al tiempo
    double D = 0;
    if (deltaTime > 0) {
      D = kd * (error - errorAnterior) / deltaTime;
    }
    
    // Salida PID total
    double PID = P + I + D;
    
    // Limitar la salida PID
    if (PID > RANGO_PID_MAX) {
      PID = RANGO_PID_MAX;
    } else if (PID < -RANGO_PID_MAX) {
      PID = -RANGO_PID_MAX;
    }
    
    // Convertir PID a ángulo objetivo del servo (centrado en equilibrio)
    anguloObjetivo = POSICION_EQUILIBRIO + (int)PID;
    
    // Aplicar límites absolutos del servo al objetivo
    if (anguloObjetivo > LIMITE_MAX) {
      anguloObjetivo = LIMITE_MAX;
    } else if (anguloObjetivo < LIMITE_MIN) {
      anguloObjetivo = LIMITE_MIN;
    }
    
    // Debug - imprimir valores
    Serial.print(distanciaMedida);
    Serial.print(" cm    | ");
    Serial.print(error, 1);
    Serial.print("   | ");
    Serial.print(P, 2);
    Serial.print(" | ");
    Serial.print(I, 2);
    Serial.print(" | ");
    Serial.print(D, 2);
    Serial.print(" | ");
    Serial.print(PID, 2);
    Serial.print(" | ");
    Serial.print(anguloObjetivo);
    Serial.print("°     | ");
    Serial.print(anguloActual);
    Serial.println("°");
    
    // Actualizar variables para próxima iteración
    errorAnterior = error;
    tiempoAnterior = tiempoActual;
  }
  
  // // Movimiento suave del servo (independiente del PID)
  // if (tiempoActual - tiempoServo >= INTERVALO_SERVO) {
  //   moverServoSuave();
  //   tiempoServo = tiempoActual;
  // }
}

// // Función para mover el servo suavemente
// void moverServoSuave() {
//   if (anguloActual != anguloObjetivo) {
//     if (anguloActual < anguloObjetivo) {
//       anguloActual += VELOCIDAD_SERVO;
//       if (anguloActual > anguloObjetivo) {
//         anguloActual = anguloObjetivo;  // No sobrepasar el objetivo
//       }
//     } else {
//       anguloActual -= VELOCIDAD_SERVO;
//       if (anguloActual < anguloObjetivo) {
//         anguloActual = anguloObjetivo;  // No sobrepasar el objetivo
//       }
//     }
//     servo.write(anguloActual);
//   }
// }

// Función para ajustar parámetros PID en tiempo real
void ajustarPID(double nuevo_kp, double nuevo_ki, double nuevo_kd) {
  kp = nuevo_kp;
  ki = nuevo_ki;
  kd = nuevo_kd;
  
  // Reset del error acumulado al cambiar parámetros
  errorAcumulado = 0;
  
  Serial.println(">>> Parámetros PID actualizados <<<");
  Serial.print("Kp: "); Serial.println(kp, 3);
  Serial.print("Ki: "); Serial.println(ki, 3);
  Serial.print("Kd: "); Serial.println(kd, 3);
}

// // Función para ajustar velocidad del servo
// void ajustarVelocidadServo(int nueva_velocidad) {
//   if (nueva_velocidad > 0 && nueva_velocidad <= 10) {
//     VELOCIDAD_SERVO = nueva_velocidad;
//     Serial.print(">>> Nueva velocidad del servo: ");
//     Serial.print(VELOCIDAD_SERVO);
//     Serial.println("°/iteración <<<");
//   }
// }