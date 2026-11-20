// Robogol 4WD
// ESP32-WROOM-32 + Bluepad32 + dos drivers TB6612FNG

#include <Bluepad32.h>
#include <initializer_list>
#include <math.h>

// Driver delantero M1
constexpr int M1_PWMA = 4,  M1_AIN1 = 17, M1_AIN2 = 16;
constexpr int M1_PWMB = 21, M1_BIN1 = 18, M1_BIN2 = 19;
constexpr int M1_STBY = 23;

// Driver trasero M2
constexpr int M2_PWMA = 13, M2_AIN1 = 33, M2_AIN2 = 32;
constexpr int M2_PWMB = 14, M2_BIN1 = 26, M2_BIN2 = 27;
constexpr int M2_STBY = 25;

// Polaridad de cada canal físico del driver
constexpr bool INVERTIR_M1_A = false;
constexpr bool INVERTIR_M1_B = true;
constexpr bool INVERTIR_M2_A = false;
constexpr bool INVERTIR_M2_B = true;

constexpr bool INVERTIR_GIRO = true;
constexpr float PWM_PRECISION = 0.30f;
constexpr unsigned long TIEMPO_SIN_DATOS_MS = 300;

ControllerPtr mando = nullptr;
int modoPorcentaje = 60;
bool armado = false;
bool frenteInvertido = false;
bool l1Anterior = false;
unsigned long ultimoDato = 0;

void parar() {
  digitalWrite(M1_STBY, LOW);
  digitalWrite(M2_STBY, LOW);

  analogWrite(M1_PWMA, 0);
  analogWrite(M1_PWMB, 0);
  analogWrite(M2_PWMA, 0);
  analogWrite(M2_PWMB, 0);

  armado = false;
  modoPorcentaje = 60;
  frenteInvertido = false;
  l1Anterior = false;
}

void conectado(ControllerPtr c) {
  if (!mando && c->isGamepad()) {
    mando = c;
    parar();
    mando->setColorLED(0, 120, 255);
    Serial.println("Mando conectado. Centra ambos sticks.");
  }
}

void desconectado(ControllerPtr c) {
  if (c == mando) {
    parar();
    mando = nullptr;
    Serial.println("Mando desconectado. Motores detenidos.");
  }
}

// Zona muerta del 10 % y respuesta progresiva del stick
float eje(int lectura) {
  float valor = fminf(fabsf((float)lectura) / 500.0f, 1.0f);

  if (valor <= 0.10f) return 0.0f;

  valor = (valor - 0.10f) / 0.90f;
  return copysignf(valor * (0.55f + 0.45f * valor), lectura);
}

void motor(int pwm, int in1, int in2, float orden, bool invertir) {
  if (invertir) orden = -orden;

  digitalWrite(in1, orden > 0.0f ? HIGH : LOW);
  digitalWrite(in2, orden < 0.0f ? HIGH : LOW);
  analogWrite(pwm, lroundf(255.0f * fabsf(orden)));
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_STBY, OUTPUT);
  pinMode(M2_STBY, OUTPUT);
  digitalWrite(M1_STBY, LOW);
  digitalWrite(M2_STBY, LOW);

  for (int pin : {M1_PWMA, M1_AIN1, M1_AIN2,
                  M1_PWMB, M1_BIN1, M1_BIN2,
                  M2_PWMA, M2_AIN1, M2_AIN2,
                  M2_PWMB, M2_BIN1, M2_BIN2}) {
    pinMode(pin, OUTPUT);
  }

  parar();
  BP32.setup(&conectado, &desconectado);
  BP32.enableVirtualDevice(false);

  Serial.println("Robogol 4WD listo. Esperando mando...");
}

void loop() {
  bool nuevo = BP32.update();

  if (mando && mando->isConnected() && nuevo && mando->hasData()) {
    ultimoDato = millis();

    // L1 cambia el frente una sola vez por pulsación
    bool l1Actual = mando->l1();
    if (l1Actual && !l1Anterior) {
      frenteInvertido = !frenteInvertido;

      if (frenteInvertido) {
        mando->setColorLED(255, 60, 0);
        Serial.println("Frente invertido.");
      } else {
        mando->setColorLED(0, 120, 255);
        Serial.println("Frente normal.");
      }
    }
    l1Anterior = l1Actual;

    // Stick izquierdo vertical: avance y retroceso
    // Stick derecho horizontal: giro
    float avance = eje(mando->axisY());
    float giro = eje(mando->axisRX());

    if (frenteInvertido) avance = -avance;
    if (INVERTIR_GIRO) giro = -giro;

    if (!armado && avance == 0.0f && giro == 0.0f) {
      armado = true;
      Serial.println("Motores habilitados.");
    }

    if (armado) {
      int nivelAnterior = modoPorcentaje;

      if      (mando->b()) modoPorcentaje = 100;  // Círculo
      else if (mando->y()) modoPorcentaje = 90;   // Triángulo
      else if (mando->x()) modoPorcentaje = 75;   // Cuadrado
      else if (mando->a()) modoPorcentaje = 60;   // Cruz

      if (modoPorcentaje != nivelAnterior) {
        Serial.printf("Nivel de potencia: %d%%\n", modoPorcentaje);
      }

      // L2 suelto permite usar todo el nivel seleccionado.
      // L2 a fondo reduce el límite al 30 %.
      float precision = mando->brake() / 1023.0f;
      float limite = modoPorcentaje / 100.0f;
      limite += (PWM_PRECISION - limite) * precision;

      // Combina avance y giro para poder doblar mientras se mueve.
      float izq = avance + giro;
      float der = avance - giro;
      float escala = fmaxf(1.0f, fmaxf(fabsf(izq), fabsf(der)));

      float potenciaIzq = limite * izq / escala;
      float potenciaDer = limite * der / escala;

      // Delanteros: canal A izquierdo, canal B derecho
      motor(M1_PWMA, M1_AIN1, M1_AIN2, potenciaIzq, INVERTIR_M1_A);
      motor(M1_PWMB, M1_BIN1, M1_BIN2, potenciaDer, INVERTIR_M1_B);

      // Traseros corregidos: canal A derecho, canal B izquierdo
      motor(M2_PWMA, M2_AIN1, M2_AIN2, potenciaDer, INVERTIR_M2_A);
      motor(M2_PWMB, M2_BIN1, M2_BIN2, potenciaIzq, INVERTIR_M2_B);

      digitalWrite(M1_STBY, HIGH);
      digitalWrite(M2_STBY, HIGH);
    }
  }

  if (armado && millis() - ultimoDato > TIEMPO_SIN_DATOS_MS) {
    parar();
    if (mando && mando->isConnected()) {
      mando->setColorLED(0, 120, 255);
    }
    Serial.println("Sin datos. Centra ambos sticks para reanudar.");
  }

  delay(5);
}