// Robogol 4WD. ESP32 Dev Module de Espressif + biblioteca RemoteXY.
// BLE, dos joysticks autocentrados. El inversor cambia el eje de avance.
// Toque corto: invertir frente. Mantener 1 segundo: siguiente nivel PWM.
#include <Arduino.h>
#include <math.h>
#define REMOTEXY_MODE__ESP32CORE_BLE
#define REMOTEXY_BLUETOOTH_NAME "Robogol"
#include <BLEDevice.h>
#include <RemoteXY.h>

// Panel horizontal de 200 x 84. Dos joysticks y un botón central.
uint8_t RemoteXY_CONF[] = {
  255,5,0,0,0,48,0,19,0,0,0,0,83,1,200,84,1,1,3,0,
  5,2,7,70,70,32,2,26,31,
  5,128,7,70,70,32,2,26,31,
  1,77,27,46,29,1,2,31,'I','n','v','e','r','t','i','r',0
};
#pragma pack(push,1)
struct {
  int8_t izqX, izqY, derX, derY;
  uint8_t boton, connect_flag;
} RemoteXY;
#pragma pack(pop)

const int standbyDel = 23, standbyTras = 25;
const int niveles[] = {60,75,90,100};
int nivel = 0;
bool invertido = false, armado = false;
bool botonAnterior = false, pulsacionLarga = false;
unsigned long inicioBoton = 0;

void detener() {
  digitalWrite(standbyDel, LOW);
  digitalWrite(standbyTras, LOW);
}

void motor(int pwm, int in1, int in2, float orden) {
  digitalWrite(in1, orden > 0);
  digitalWrite(in2, orden < 0);
  analogWrite(pwm, lroundf(255 * fabsf(orden)));
}

float eje(int lectura) {
  float v = fminf(fabsf(float(lectura)) / 100.0f, 1.0f);
  if (v <= 0.10f) return 0;
  v = (v - 0.10f) / 0.90f;
  return copysignf(v * (0.55f + 0.45f * v), lectura);
}

void leerBoton(bool pulsado) {
  if (pulsado && !botonAnterior) {
    inicioBoton = millis();
    pulsacionLarga = false;
  }
  if (pulsado && !pulsacionLarga && millis() - inicioBoton >= 1000) {
    nivel = (nivel + 1) % 4;
    pulsacionLarga = true;
    Serial.printf("Potencia %d %%\n", niveles[nivel]);
  }
  if (!pulsado && botonAnterior && !pulsacionLarga) {
    invertido = !invertido;
    Serial.println(invertido ? "Frente invertido" : "Frente normal");
  }
  botonAnterior = pulsado;
}

void setup() {
  pinMode(standbyDel, OUTPUT);
  pinMode(standbyTras, OUTPUT);
  detener();
  const int pines[] = {4,17,16,21,18,19,13,33,32,14,26,27};
  for (int pin : pines) pinMode(pin,OUTPUT);
  Serial.begin(115200);
  RemoteXY_Init();
}

void loop() {
  RemoteXY_Handler();
  if (!RemoteXY.connect_flag) {
    detener();
    armado = false;
    invertido = false;
    nivel = 0;
    botonAnterior = false;
    return;
  }

  float avance = -eje(RemoteXY.izqY);
  float giro = -eje(RemoteXY.derX);
  if (!armado) {
    detener();
    armado = avance == 0 && giro == 0 && !RemoteXY.boton;
    return;
  }

  leerBoton(RemoteXY.boton);
  if (invertido) avance = -avance;
  if (avance == 0 && giro == 0) {
    detener();
    return;
  }

  float izq = avance + giro, der = avance - giro;
  float escala = fmaxf(1, fmaxf(fabsf(izq), fabsf(der)));
  float factor = niveles[nivel] / (100.0f * escala);
  motor(4,17,16, izq * factor);     // M1 A delantero izquierdo
  motor(21,18,19,-der * factor);    // M1 B delantero derecho
  motor(13,33,32, der * factor);    // M2 A trasero derecho
  motor(14,26,27,-izq * factor);    // M2 B trasero izquierdo
  digitalWrite(standbyDel, HIGH);
  digitalWrite(standbyTras, HIGH);
}
