#include <Arduino.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// Pines Driver TB6612FNG
#define AIN1 4
#define AIN2 2
#define PWMA 15 // PWM Motor A (Izq)
#define BIN1 19
#define BIN2 18
#define PWMB 5  // PWM Motor B (Der)
#define STBY 21 // Standby

// Pines Sensor QTR-8A
#define S8 14
#define S7 27
#define S6 26
#define S5 25
#define S4 33
#define S3 32
#define S2 35
#define S1 34

const int qtrPins[8] = {S8, S7, S6, S5, S4, S3, S2, S1};

//timer declara el timer el handle 
hw_timer_t *timerGiro = NULL;
hw_timer_t *timerMuestreo = NULL;

//declaro segundo nucleo
TaskHandle_t TaskBT;
//TaskHandle_t TaskMotores;

volatile bool cambiarSentido = false;
volatile bool muestrear = false;
volatile bool listo_enviar = false;
static bool sentido = true;

int sensorValues[8];
int sensorValuesDig[8];
float referencia = 0;
float angulo = 0;
float ek = 0;
float ek_1 = 0;
float uk = 0;
float uk_1 = 0;
float pwmbase = 60;
float pwmsent = 0;

float deadband = 1;



float calcularAngulo(int sensorValuesDig[8])
{
  float y = 0;

  // Fila 1: 00000001 -> y = -8.75
  if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
      sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
      sensorValuesDig[6] == 0 && sensorValuesDig[7] == 1)
  {
    y = -8.75;
  }
  // Fila 2: 00000011 -> y = -7.5
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 1 && sensorValuesDig[7] == 1)
  {
    y = -7.5;
  }
  // Fila 3: 00000010 -> y = -6.25
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 1 && sensorValuesDig[7] == 0)
  {
    y = -6.25;
  }
  // Fila 4: 00000110 -> y = -5
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 1 &&
           sensorValuesDig[6] == 1 && sensorValuesDig[7] == 0)
  {
    y = -5;
  }
  // Fila 5: 00000100 -> y = -3.75
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 1 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = -3.75;
  }
  // Fila 6: 00001100 -> y = -2.5
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 1 && sensorValuesDig[5] == 1 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = -2.5;
  }
  // Fila 7: 00001000 -> y = -1.25
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 1 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = -1.25;
  }
  // Fila 8: 00011000 -> y = 0
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 1 && sensorValuesDig[4] == 1 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 0;
  }
  // Fila 9: 00010000 -> y = 1.25
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 1 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 1.25;
  }
  // Fila 10: 00110000 -> y = 2.5
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 1 &&
           sensorValuesDig[3] == 1 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 2.5;
  }
  // Fila 11: 00100000 -> y = 3.75
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 1 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 3.75;
  }
  // Fila 12: 01100000 -> y = 5
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 1 && sensorValuesDig[2] == 1 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 5;
  }
  // Fila 13: 01000000 -> y = 6.25 (tomamos esta como única para este patrón)
  else if (sensorValuesDig[0] == 0 && sensorValuesDig[1] == 1 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 6.25;
  }

  // Fila 14: 11000000 -> y = 7.5 (tomamos esta como única para este patrón)
  else if (sensorValuesDig[0] == 1 && sensorValuesDig[1] == 1 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 7.5;
  }

  // Fila 15: 10000000 -> y = 6.25 (tomamos esta como única para este patrón)
  else if (sensorValuesDig[0] == 1 && sensorValuesDig[1] == 0 && sensorValuesDig[2] == 0 &&
           sensorValuesDig[3] == 0 && sensorValuesDig[4] == 0 && sensorValuesDig[5] == 0 &&
           sensorValuesDig[6] == 0 && sensorValuesDig[7] == 0)
  {
    y = 8.75;
  }

  return y;
}

void moverDerecha(float pw, float us)
{
  // --- Motor Izquierdo ---
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, (pw - us));

  // --- Motor Derecho ---
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, (pw + us));
}

void moverIzquierda(float pw, float us)
{
  // --- Motor Izquierdo ---
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, (pw - us));

  // --- Motor Derecho ---
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, (pw + us));
}



void IRAM_ATTR onTimerGiro() {
  cambiarSentido = true; // Activar bandera
}

void IRAM_ATTR onTimerMuestreo() {
  muestrear = true; // Activar bandera de muestreo
}

void loopBlutu(void *parameter){for (;;){
  
  if (listo_enviar == true)
  {
    SerialBT.print(referencia,4);
    SerialBT.print("p");
    SerialBT.print(angulo),4;
    SerialBT.print("p");
    SerialBT.print(ek,4);
    SerialBT.print("p");
    SerialBT.println(uk,4);  
    listo_enviar == false;
  }
  
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');  // Igual que con Serial
    float nuevoValor = input.toFloat();
    referencia = nuevoValor;
  }
//vTaskDelay(10);
}}

//void loopMotores(void *parameter){for (;;){
 
//}}

void setup()
{
  SerialBT.begin("RB_13");

  delay(10000);

  Serial.begin(115200);
  Serial.println("Lectura QTR-8A inicializada...");

  xTaskCreatePinnedToCore(loopBlutu,"TaskBT",4096,NULL,1,&TaskBT,0);
  //xTaskCreatePinnedToCore(loopMotores,"TaskMotores",4096,NULL,1,&TaskMotores,1);

  // Configurar pines como salida
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // === Configurar Timers ===
  // Timer 0: cambio de giro cada 1s
  timerGiro = timerBegin(0, 80, true);  // 1 tick = 1µs
  timerAttachInterrupt(timerGiro, &onTimerGiro, true);
  timerAlarmWrite(timerGiro, 200000, true); // 1s
  timerAlarmEnable(timerGiro);

  // Timer 1: muestreo cada 10ms
  timerMuestreo = timerBegin(1, 80, true);
  timerAttachInterrupt(timerMuestreo, &onTimerMuestreo, true);
  timerAlarmWrite(timerMuestreo, 10000, true); // 10ms
  timerAlarmEnable(timerMuestreo);

  // Activar driver (STBY en HIGH)
  digitalWrite(STBY, HIGH);


}

void loop()
{
  // === Muestreo ===
  if (muestrear) {
    muestrear = false;

    for (int i = 0; i < 8; i++)
  {
    sensorValues[i] = analogRead(qtrPins[i]);
    if (sensorValues[i] > 2047.5)
    {
      sensorValuesDig[i] = 1;
    }
    else
    {
      sensorValuesDig[i] = 0;
    }
  }

   angulo = calcularAngulo(sensorValuesDig);
    ek = angulo - referencia;

    // --- Banda muerta ---  
    if (fabs(ek) < deadband) {
      ek = 0;
    }

    listo_enviar = true;

    // Control discreto
    //uk = 1.5436*(ek - 0.9553*ek_1) - 0.8477*uk_1;
    // Control discreto
    uk = 1.5436*(ek - 0.9553*ek_1) + 0.8477*uk_1;
    ek_1 = ek;
    uk_1 = uk;

    // Aplicar control solo si el error está fuera de la banda
    if (ek > 0) {
      moverIzquierda(pwmbase, uk * 255);
    } else if (ek < 0) {
      moverDerecha(pwmbase, uk * 255);
    } else {
      // dentro de la banda muerta -> motores en velocidad base sin corrección
      moverIzquierda(0, 0);
    }
  }

}
