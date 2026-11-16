// ESP32-WROOM-32 + Bluepad32 + TB6612FNG. Canal A izquierdo, B derecho.
#include <initializer_list>
#include <Bluepad32.h>
#include <math.h>

constexpr int PWMA=4, AIN1=16, AIN2=17, PWMB=5, BIN1=18, BIN2=19, STBY=23;
// Un motor en false y el otro en true para compensar el montaje en espejo:
constexpr bool INVERTIR_A=false, INVERTIR_B=true;

// Al 100 % los motores reciben casi todo VM.
constexpr float PWM_PRECISION=0.30f;
int modoPorcentaje=50;
ControllerPtr mando=nullptr;
bool armado=false;
unsigned long ultimoDato=0;

// Variables para invertir el frente con L1
bool frenteInvertido=false;
bool l1Anterior=false;

void parar() {
  digitalWrite(STBY, LOW);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  armado=false;
  modoPorcentaje=50;
  frenteInvertido=false;
  l1Anterior=false;
}

void conectado(ControllerPtr c) {
  if (!mando && c->isGamepad()) {
    mando=c;
    parar();
    mando->setColorLED(0, 120, 255); // Azul: frente normal
    Serial.println("Mando conectado. Centra ambos sticks.");
  }
}

void desconectado(ControllerPtr c) {
  if (c==mando) {
    parar();
    mando=nullptr;
    Serial.println("Mando desconectado.");
  }
}

// Zona muerta y curva progresiva para corregir el rumbo cerca del balon.
float eje(int lectura) {
  float valor=fminf(abs(lectura)/500.0f, 1.0f);
  if (valor<=0.10f) return 0;
  valor=(valor-0.10f)/0.90f;
  return copysignf(valor*(0.55f+0.45f*valor), lectura);
}

void motor(int pwm, int in1, int in2, float orden, bool invertir) {
  if (invertir) orden=-orden;
  digitalWrite(in1, orden>0);
  digitalWrite(in2, orden<0);
  analogWrite(pwm, lroundf(255.0f*fabsf(orden)));
}

void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);
  for (int pin : {AIN1, AIN2, BIN1, BIN2, PWMA, PWMB}) pinMode(pin, OUTPUT);
  parar();
  Serial.begin(115200);
  BP32.setup(&conectado, &desconectado);
  BP32.enableVirtualDevice(false);
  Serial.println("Robogol listo. Esperando mando.");
}

void loop() {
  bool nuevo=BP32.update();
  if (mando && mando->isConnected() && nuevo && mando->hasData()) {
    ultimoDato=millis();

    // Detectar pulsacion unica de L1 para alternar el frente del carro
    bool l1Actual=mando->l1();
    if (l1Actual && !l1Anterior) {
      frenteInvertido=!frenteInvertido;
      if (frenteInvertido) {
        mando->setColorLED(255, 60, 0); // Naranja: frente invertido
        Serial.println("Frente INVERTIDO activo");
      } else {
        mando->setColorLED(0, 120, 255); // Azul: frente normal
        Serial.println("Frente NORMAL activo");
      }
    }
    l1Anterior=l1Actual;

    // Izquierda vertical invertida: arriba avanza, abajo retrocede.
    float avance=eje(mando->axisY());
    // Si activaste L1, la trasera pasa a ser la delantera:
    if (frenteInvertido) avance=-avance;

    // Menos sensibilidad al giro a alta demanda de avance.
    // Derecha horizontal: gira hacia el lado indicado.
    float giro=eje(mando->axisRX())*(1.0f-0.30f*fabsf(avance));
    if (!armado && avance==0 && giro==0) armado=true;
    if (armado) {
      float izq=avance+giro, der=avance-giro;
      float escala=fmaxf(1.0f, fmaxf(fabsf(izq), fabsf(der)));
      // Bluepad32: A = cruz, X = cuadrado, Y = triangulo.
      if (mando->a()) modoPorcentaje=50;
      else if (mando->x()) modoPorcentaje=75;
      else if (mando->y()) modoPorcentaje=100;
      float precision=mando->brake()/1023.0f;
      float limite=modoPorcentaje/100.0f;
      limite+=(PWM_PRECISION-limite)*precision;
      motor(PWMA, AIN1, AIN2, izq*limite/escala, INVERTIR_A);
      motor(PWMB, BIN1, BIN2, der*limite/escala, INVERTIR_B);
      digitalWrite(STBY, HIGH);
    }
  }
  if (armado && millis()-ultimoDato>300) parar();
  delay(5);
}