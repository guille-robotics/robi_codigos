/*
Este codigo posee el Control PID para los dos motores.
*/
#include "motorControl.h"
////// Pines del L298N///////
//Pines direccion derecha
int in1=9;
int in2=8;
// Pines direccion izquierda
int in3=7;
int in4=6;
// Pin control de velocidad derecha
int ena=10;
// Pin control de velocidad izquierda
int enb=11;
//////// PINES ENCODER///////
// Pines Encoder Rueda Derecha
const int    Amarillo_Derecho = 2; // Entrada de la señal A del encoder.
const int    Verde_Derecho = 4; // Entrada de la señal B del encoder.
// Pines Encoder Rueda Izquierda/////
const int    Amarillo_Izquierdo = 3; // Entrada de la señal A del encoder.
const int    Verde_Izquierdo = 5; // Entrada de la señal B del encoder.


///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 2;
double data[dataLength];

///// Valor PWM/////
int cvDerecho=0;
int cvIzquierdo=0;

//// VALORES PARA CALCULO DEL CV WREF VS WMEDIDA
double wRefDerecho=0.0;
double wRefIzquierdo=0.0;

///// VALORES VELOCIDAD ANGULAR ////
double wDerecho=0.0;
double wIzquierdo=0.0;
double constValue=27.54;// (1000*2*pi)/R ---> R = 228 Resolucion encoder cuadruple
/// DATOS PARA LECTURA DE ENCODER////
volatile long  nDerecho = 0;
volatile long  nIzquierdo = 0;

bool Direction_derecho=false;
bool Direction_izquierdo=false;

//// CONTROL DEL TIEMPO /////
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

/// FUNCIONES PARA CONTROL DE LOS MOTORES USANDO PID
motorControl motorR(sampleTime);
motorControl motorL(sampleTime);

void setup()
{
  Serial.begin(9600);
  //// VALORES PID PARA CADA MOTOR ///
  motorR.setGains(0.23, 0.1, 0.05);
  motorL.setGains(0.22, 0.1, 0.04); // (Kc,Ti,Td) 0.55, 0.10, 0.05   0.25, 0.1, 0.05 Rueda Izquierda = 0.22, 0.21, 0.04
  /*
  -Aumentando Kc disminuye la estabilidad
  -El error decae mas rapido si se disminuye el Ti
  -Disminuyendo Ti disminuye la estabilidad
  -Aumentando Td mejora la estabilidad
  */
  ///// LIMITES DE LA PV(W [rad/s] y CV(PWM))
  motorR.setCvLimits(255,0);
  motorR.setPvLimits(41,0);

  motorL.setCvLimits(255,0); 
  motorL.setPvLimits(41,0);
 

  /////// DEFINICION DE LOS MODOS DE LOS PINES //////
  // Pines de los Encoder derecho
  pinMode(Amarillo_Derecho, INPUT);
  pinMode(Verde_Derecho, INPUT);
  // Pines de los Encoder izquierdo
  pinMode(Amarillo_Izquierdo, INPUT);
  pinMode(Verde_Izquierdo, INPUT);
  // Pines del L298N
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ena,OUTPUT);
  pinMode(enb,OUTPUT);

  //////////////////// MOTOR APAGADO //////////////////////
  digitalWrite(in1, false);       
  digitalWrite(in2, false); 
  digitalWrite(in3, false);       
  digitalWrite(in4, false);   

  /////// MOTORES EN 0 PARA INICIAR /////
  analogWrite(ena,cvDerecho);
  analogWrite(enb,cvIzquierdo);

  //// INTERRUPCIONES (Se generan en los pines de la Señal A)//////
  attachInterrupt(digitalPinToInterrupt(Amarillo_Derecho), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(Amarillo_Izquierdo), left_wheel_pulse, RISING);
  
  lastTime = millis();

}

void loop() {
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
     }
     
     wRefDerecho=data[0];
     wRefIzquierdo=data[1];
     
     inputString = "";
     stringComplete = false;
  }

  //Serial.print("Cuentas: ");Serial.println(n);
  if (millis() - lastTime >= sampleTime){  
     calculo_vel_angular_derecho();
     calculo_vel_angular_izquierdo(); 
     cvDerecho = motorR.compute(wRefDerecho,wDerecho);
     cvIzquierdo = motorL.compute(wRefIzquierdo,wIzquierdo); 
     if (cvDerecho > 0) anticlockwise(in1,in2,ena,cvDerecho); else clockwise(in1,in2,ena,abs(cvDerecho));
     if (cvIzquierdo > 0) anticlockwise(in3,in4,enb,cvIzquierdo); else clockwise(in3,in4,enb,abs(cvIzquierdo));       
     Serial.print("Derecho: ");
     Serial.print(wDerecho);
     Serial.print("--");
     Serial.print(cvDerecho);
     Serial.print("|||||");
     Serial.print("Izquierdo: ");
     Serial.print(wIzquierdo);
     Serial.print("--");
     Serial.println(cvIzquierdo);
    lastTime = millis(); //  Importante: Porque actuliza el valor de la W  
  } 
}

//// FUNCIONES CALCULO DE LAS VELOCIDADES ANGULARES /////

// Velocidad angular izquierda
void calculo_vel_angular_izquierdo(){
  wIzquierdo = constValue*nIzquierdo/(millis()-lastTime);
  //lastTime = millis(); //  Importante: Porque actuliza el valor de la W
  nIzquierdo=0;
}
// Velocidad angular derecha
void calculo_vel_angular_derecho(){
  wDerecho = constValue*nDerecho/(millis()-lastTime);
  //lastTime = millis(); //  Importante: Porque actuliza el valor de la W
  nDerecho=0;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
////// FUNCIONES CONTEO DE PUNTOS ENCODER //////
// Encoder Derecho
void right_wheel_pulse() {   
  int val_derecho = digitalRead(Verde_Derecho);
 
  if(val_derecho == LOW) {
    Direction_derecho = false; // Reverse
  }
  else {
    Direction_derecho = true; // Forward
  }
   
  if (Direction_derecho) {
    nDerecho++;
  }
  else {
    nDerecho--;
  }
}
// Encoder Izquierdo
void left_wheel_pulse() {   
  int val_izquierdo = digitalRead(Verde_Izquierdo);
 
  if(val_izquierdo == LOW) {
    Direction_izquierdo = false; // Reverse
  }
  else {
    Direction_izquierdo = true; // Forward
  }
 
  if (Direction_izquierdo) {
    nIzquierdo--;
  }
  else {
    nIzquierdo++;
  }
}

////// FUNCIONES MOVIMIENTO DEL MOTOR /////
// Movimiento hacia delante
void clockwise(int pin1, int pin2,int analogPin, int pwm)
{  
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}
// Movimiento hacia atras
void anticlockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}
