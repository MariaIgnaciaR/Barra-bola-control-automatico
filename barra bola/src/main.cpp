/*
 * ============================================================================
 * Actividad 2 — Accionamiento del servomotor SG90 (planta bola sobre barra)
 * CBM-422 Ingeniería de Control Automático — Universidad de Valparaíso
 * ----------------------------------------------------------------------------
 * VERSION PLATFORMIO — copiar este archivo a:  src/main.cpp
 *
 * platformio.ini:
 *     [env:uno]
 *     platform = atmelavr
 *     board = uno
 *     framework = arduino
 *     monitor_speed = 9600
 *
 * (La librería Servo ya viene incluida en el framework Arduino para AVR,
 *  NO hace falta instalar nada ni agregar lib_deps)
 *
 * Conexión SG90:  café/negro → GND,  rojo → 5V,  naranja → pin 9
 * Monitor serie / Python: 9600 baud, comandos terminados en Enter.
 *
 * Comandos:
 *   A <grados>   ir a un ángulo absoluto [°]         (punto a)
 *   M <delta>    moverse en relativo, ej: M -2
 *   H            ir a la posición de barra HORIZONTAL (punto b)
 *   Z            grabar el ángulo actual como horizontal (θ0)
 *   L            ver límites;  L <min> <max> para cambiarlos (punto c/e)
 *   S            barrido lento entre límites (inspección del recorrido)
 *   ?            ver estado completo
 * ============================================================================
 */

#include <Arduino.h>     // ← obligatorio en PlatformIO (el IDE de Arduino lo pone solo)
#include <Servo.h>

const int PIN_SERVO = 9;
Servo servo;

// ----- Valores iniciales: AJUSTAR tras la identificación experimental -----
float THETA0    = 120.0;   // ángulo del servo con barra horizontal   ← identificar (b)
float THETA_MIN = 80.0;   // límite inferior seguro [°]              ← identificar (c)
float THETA_MAX = 150.0;  // límite superior seguro [°]              ← identificar (c)

float angulo = THETA0;    // posición angular actual [°]

// Mueve el servo SUAVEMENTE hasta 'destino', sin salirse de los límites (e)
void moverSuave(float destino) {
  destino = constrain(destino, THETA_MIN, THETA_MAX);   // límite de seguridad
  while (fabs(destino - angulo) > 0.4) {
    angulo += (destino > angulo) ? 0.5 : -0.5;          // pasos de 0.5°
    servo.write(angulo);
    delay(15);                                          // ~33°/s, sin tirones
  }
  angulo = destino;
  servo.write(angulo);
}

void imprimirEstado() {
  Serial.print(F("angulo = "));      Serial.print(angulo, 1);
  Serial.print(F("°  |  theta0 = ")); Serial.print(THETA0, 1);
  Serial.print(F("°  |  limites = [")); Serial.print(THETA_MIN, 1);
  Serial.print(F(", "));             Serial.print(THETA_MAX, 1);
  Serial.println(F("]"));
}

void setup() {
  Serial.begin(9600);
  servo.attach(PIN_SERVO);
  servo.write(angulo);            // parte en THETA0 (barra ~horizontal)
  Serial.println(F("--- Actividad 2: control del SG90 listo ---"));
  Serial.println(F("Comandos: A <grados>, M <delta>, H, Z, L, S, ?"));
  imprimirEstado();
}

void loop() {
  if (!Serial.available()) return;

  String linea = Serial.readStringUntil('\n');
  linea.trim();
  if (linea.length() == 0) return;

  char cmd = toupper(linea.charAt(0));
  String resto = linea.substring(1);
  resto.trim();

  switch (cmd) {
    case 'A':                                // ángulo absoluto
      if (resto.length() > 0) moverSuave(resto.toFloat());
      Serial.print(F("OK -> ")); Serial.print(angulo, 1); Serial.println(F("°"));
      break;

    case 'M':                                // movimiento relativo
      if (resto.length() > 0) moverSuave(angulo + resto.toFloat());
      Serial.print(F("OK -> ")); Serial.print(angulo, 1); Serial.println(F("°"));
      break;

    case 'H':                                // ir a horizontal
      moverSuave(THETA0);
      Serial.println(F("OK -> horizontal"));
      break;

    case 'Z':                                // grabar horizontal actual
      THETA0 = angulo;
      Serial.print(F("OK -> theta0 = ")); Serial.print(THETA0, 1); Serial.println(F("°"));
      break;

    case 'L': {                              // ver/cambiar límites
      if (resto.length() > 0) {
        int espacio = resto.indexOf(' ');
        if (espacio > 0) {
          THETA_MIN = resto.substring(0, espacio).toFloat();
          THETA_MAX = resto.substring(espacio + 1).toFloat();
        }
      }
      Serial.print(F("limites = [")); Serial.print(THETA_MIN, 1);
      Serial.print(F(", ")); Serial.print(THETA_MAX, 1); Serial.println(F("]"));
      break;
    }

    case 'S':                                // barrido lento de inspección
      Serial.println(F("barrido..."));
      moverSuave(THETA_MIN);
      moverSuave(THETA_MAX);
      moverSuave(THETA0);
      Serial.println(F("barrido listo"));
      break;

    case '?':
      imprimirEstado();
      break;

    default:
      Serial.println(F("comando no reconocido (use ?)"));
  }
}