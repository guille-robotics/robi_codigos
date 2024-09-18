/////LIBRERIAS ////
#include "motorControl.h"
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

//// VALORES PARA CALCULO DEL CV WREF VS WMEDIDA
double wRefDerecho=0.0;
double wRefIzquierdo=0.0;

///// VALORES VELOCIDAD ANGULAR ////
double wDerecho=0.0;
double wIzquierdo=0.0;
double constValue=20.33;//27.54;// (1000*2*pi)/R ---> R = 228 Resolucion encoder cuadruple

///// Valor PWM/////
int cvDerecho=0;
int cvIzquierdo=0;

/// DATOS PARA LECTURA DE ENCODER////
volatile long  nDerecho = 0;
volatile long  nIzquierdo = 0;

bool Direction_derecho=false;
bool Direction_izquierdo=false;

//// CONTROL DEL TIEMPO /////
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 30;  // Tiempo de muestreo
long currentMillis = 0;

///// VALORES PARA CALCULAR ODOMETRIA /////
const double R = 0.0335; // radio de la llanta
const double d = 0.25; // Distancia entre llantas

/// VARIABLE PARA RECIBOR TOPICO CMD_VEL ///
double lastCmdVelReceived=0.0;
double linear=0.0;
double angular=0.0;

/// FUNCIONES PARA CONTROL DE LOS MOTORES USANDO PID
motorControl motorR(sampleTime);
motorControl motorL(sampleTime);

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
 wRefDerecho = (linear+(d*angular/2))/R; 
 wRefIzquierdo = (linear-(d*angular/2))/R; 
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

void mover (){
    if (cvDerecho > 0) {
      anticlockwise(in1, in2, ena, cvDerecho);
    } else if (cvDerecho < 0) {
      clockwise(in1, in2, ena, abs(cvDerecho));
    }
    
    if (cvIzquierdo > 0) {
      anticlockwise(in3, in4, enb, cvIzquierdo);
    } else  {
      clockwise(in3, in4, enb, abs(cvIzquierdo));
    }
    
}
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &recibe_cmd );

void setup()
{
  Serial.begin(9600);
    //// VALORES PID PARA CADA MOTOR ///
  motorR.setGains(0.21, 0.19, 0.04);
  motorL.setGains(0.21, 0.19, 0.04);
  //motorR.setGains(0.23, 0.1, 0.05);
  //motorL.setGains(0.22, 0.1, 0.04); // (Kc,Ti,Td) 0.55, 0.10, 0.05   0.25, 0.1, 0.05 Rueda Izquierda = 0.22, 0.21, 0.04
  /*
  -Aumentando Kc disminuye la estabilidad
  -El error decae mas rapido si se disminuye el Ti
  -Disminuyendo Ti disminuye la estabilidad
  -Aumentando Td mejora la estabilidad
  */
  ///// LIMITES DE LA PV(W [rad/s] y CV(PWM))
  motorR.setCvLimits(80,20);
  motorR.setPvLimits(20,0);

  motorL.setCvLimits(80,20); 
  motorL.setPvLimits(20,0);
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
  
  if (millis() - lastTime >= sampleTime){
    calculo_vel_angular_derecho();
    calculo_vel_angular_izquierdo(); 
    velocityMotor(linear,angular);
    /*if (wRefDerecho==0.0) cvDerecho=0; else */cvDerecho = motorR.compute(wRefDerecho,wDerecho);
    /*if (wRefIzquierdo==0.0) cvIzquierdo=0; else */cvIzquierdo= motorL.compute(wRefIzquierdo,wIzquierdo); 
    //cvDerecho = motorR.compute(wRefDerecho,wDerecho);
    //cvIzquierdo = motorL.compute(wRefIzquierdo,wIzquierdo); 
    lastTime = millis();
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

void stop(int pin1, int pin2){

  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, LOW);
}
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
