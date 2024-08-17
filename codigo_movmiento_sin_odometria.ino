/* Este codigo funciona correctamente al enviarle datos desde Rosserial.
Falta agregar la parte de la publicacion de odometria*/
#include "PinChangeInterrupt.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "motorControl.h"

// Handles startup and shutdown of ROS
ros::NodeHandle nh;


/////////////////////////// CONTROLADOR PID //////////////////
unsigned long lastTime, sampleTime = 100;

motorControl motorR(sampleTime);
motorControl motorL(sampleTime);


//////////////////////MOTOR DERECHO///////////////////////////////

const int    C1R = 2;    // Entrada de la señal A del encoder.
const int    C2R = 4;    // Entrada de la señal B del encoder.
int cvR = 0;
boolean Direction_right;

//// Puente H L298N ////
const int    in1 = 9;                 
const int    in2 = 8;         
const int    ena = 10;  

volatile int nR = 0;
volatile int antR      = 0;
volatile int actR      = 0;

double w1Ref = 0.0;
double wR = 0.0;

//////////////////////MOTOR IZQUIERDO///////////////////////////////
const int    C1L = 3;                  // Entrada de la señal A del encoder.
const int    C2L = 5;                  // Entrada de la señal B del encoder.
int cvL = 0;
boolean Direction_left;

//// Puente H L298N ////
const int    in3 = 7;                  
const int    in4 = 6;                  
const int    enb = 11;

volatile int nL = 0;
volatile int antL      = 0;
volatile int actL      = 0; 

double w2Ref = 0.0; 
double wL = 0.0;

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////

double constValue = 20.33; // (1000*2*pi)/R ---> R = 1980 Resolucion encoder cuadruple

//////////////////////// ROBOT /////////////////////////
double uMeas_ori  = 0.0;
double wMeas_ori  = 0.0;
double xp = 0.0, yp = 0.0;
double x_ori = 0.0, y_ori = 0.0;
double phi_ori = 0.0;

const double R = 0.03; // radio de la llanta en metros
const double d = 0.34; // Distancia entre llantas en metros

//// Publicadores phi, x,y,uMeas y wMeas
double linear=0.0;
double angular=0.0;

std_msgs::Float32 phi;
ros::Publisher phiPub("phi", &phi);

std_msgs::Float32 x;
ros::Publisher xPub("x", &x);

std_msgs::Float32 y;
ros::Publisher yPub("y", &y);

std_msgs::Float32 uMeas;
ros::Publisher uMeasPub("uMeas", &uMeas);
 
std_msgs::Float32 wMeas;
ros::Publisher wMeasPub("wMeas", &wMeas); 
double lastCmdVelReceived=0.0;

void calc_pwm_values(const geometry_msgs::Twist& cmdVel){
    // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity 
  linear= cmdVel.linear.x ;
  angular= cmdVel.angular.z ;

}
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void setup()
{
  Serial.begin(9600);
  
  ////////////////// SINTONIA FINA PID //////////////////
  
  motorR.setGains(0.22, 0.18, 0.06); // (Kc,Ti,Td)
  motorL.setGains(0.22, 0.18, 0.06); // (Kc,Ti,Td)
  
  ////////////////// Limites de señales //////////////////
  motorR.setCvLimits(255,30);
  motorR.setPvLimits(20,0);  

  motorL.setCvLimits(255,30);
  motorL.setPvLimits(20,0);  
  
  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT);

  pinMode(C1L, INPUT_PULLUP);
  pinMode(C2L, INPUT);

  pinMode(in1, OUTPUT);       
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);       
  pinMode(in4, OUTPUT); 

  
  digitalWrite(in1, LOW);       
  digitalWrite(in2, LOW);   
  digitalWrite(in3, LOW);       
  digitalWrite(in4, LOW); 

  attachInterrupt(digitalPinToInte
