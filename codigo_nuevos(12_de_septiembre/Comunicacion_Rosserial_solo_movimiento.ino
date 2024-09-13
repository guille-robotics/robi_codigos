/*
Este codigo solo funciona correctamente al enviar datos desde ROS. Sin embargo, falta agregar la parte de PID y la tranformacion correcta de los datos 
desde ROS a los movimientos de los motores.
*/
/////LIBRERIAS ////
#include "PinChangeInterrupt.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>


//// NODO DE ROS /////
ros::NodeHandle nh;
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

///// Valor PWM/////
int cvDerecho=0;
int cvIzquierdo=0;

/// DATOS PARA LECTURA DE ENCODER////
volatile long  nDerecho = 0;
volatile long  nIzquierdo = 0;

bool Direction_derecha=false;
bool Direction_izquierda=false;

//// CONTROL DEL TIEMPO /////
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 30;  // Tiempo de muestreo
long currentMillis = 0;

//// VARIABLES PARA TRANFORMAR VELOCIDADES DEL CMD A VELOCIDADES DE CADA RUEDA ///
double wDerecha=0.0;
double wIzquierda=0.0;
const double R = 0.24; // radio de la llanta
const double d = 0.25; // Distancia entre llantas

/// VARIABLE PARA RECIBOR TOPICO CMD_VEL ///
double lastCmdVelReceived=0.0;
double linear=0.0;
double angular=0.0;

/// FUNCION QUE RECIBE EL TIPICO CMD_VEL////
void recibe_cmd(const geometry_msgs::Twist& cmdVel){
  //Almacena el ultimo comando cmd_vel recibido
  lastCmdVelReceived = (millis()/1000);
   
  ///VALORES PARA ALMACENAR DATOS ENVIADOS DESDE EL CMD_VEL////
  // Variable para la velocidad linear en x
  linear= cmdVel.linear.x ;
  // Variable para la velocidad angular en z
  angular= cmdVel.angular.z ;

}

void velocityMotor(double linear, double angular)
{
 wDerecha = (linear+(d*angular/2))/R; 
 wIzquierda = (linear-(d*angular/2))/R; 
}

void mover (){
    cvDerecho=wDerecha*200;
    cvIzquierdo=wIzquierda*200;
    if (cvDerecho > 0) {
      anticlockwise(in1, in2, ena, cvDerecho);
    } else {
      clockwise(in1, in2, ena, abs(cvDerecho));
    }
    
    if (cvIzquierdo > 0) {
      anticlockwise(in3, in4, enb, cvIzquierdo);
    } else {
      clockwise(in3, in4, enb, abs(cvIzquierdo));
    }
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &recibe_cmd );

void setup()
{
  Serial.begin(9600);

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
  //// INTERRUPCIONES (Se generan en los pines de la Señal A)//////
  attachInterrupt(digitalPinToInterrupt(Amarillo_Derecho), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(Amarillo_Izquierdo), left_wheel_pulse, RISING);
  
  ////CREACION DE SUSCRIPTOR////
  // Velocidad a la que se comunica Ros
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subCmdVel); 
  lastTime = millis();
}

void loop() {
  nh.spinOnce();
  currentMillis = millis();
  
  if (currentMillis - lastTime >= sampleTime){
    //Serial.println("Actualizando velocidad...");
    lastTime = currentMillis;
    velocityMotor(linear, angular);
    //cvDerecho = map(wDerecha, 0.0, 0.5, 0, 255);
    //cvIzquierdo = map(wIzquierda, 0.0, 0.5, 0, 255);

  }
mover();
}
void clockwise(int pin1, int pin2,int analogPin, int pwm)
{  
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}

void anticlockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}

void right_wheel_pulse() {   
  int val_derecho = digitalRead(Verde_Derecho);
 
  if(val_derecho == LOW) {
    Direction_derecha = false; // Reverse
  }
  else {
    Direction_derecha = true; // Forward
  }
   
  if (Direction_derecha) {
    nDerecho++;
  }
  else {
    nDerecho--;
  }
}

void left_wheel_pulse() {   
  int val_izquierdo = digitalRead(Verde_Izquierdo);
 
  if(val_izquierdo == LOW) {
    Direction_izquierda = false; // Reverse
  }
  else {
    Direction_izquierda = true; // Forward
  }
 
  if (Direction_izquierda) {
    nIzquierdo--;
  }
  else {
    nIzquierdo++;
  }
}
