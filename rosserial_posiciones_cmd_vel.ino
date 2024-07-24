////Librerias/////
#include "PinChangeInterrupt.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
 
// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
boolean Direction_right = true;
boolean Direction_left = true;
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 2

// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_LEFT_A 18
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
// Motor A connections
const int enA = 9;
const int in1 = 5;//11;
const int in2 = 6;
  
// Motor B connections
const int enB = 10; 
const int in3 = 7;
const int in4 = 8;

// Encoder Derecho
volatile int nR = 0;
volatile int antR = 0;
volatile int actR = 0;

// Encoder Izquierdo
volatile int nL = 0;
volatile int antL = 0;
volatile int actL = 0;

unsigned long lastTime = 0;    // Tiempo anterior
unsigned long sampleTime = 100; // Tiempo de muestreo

/////////////////////// VELOCIDAD ANGULAR //////////////////////
double wR = 0.0;  // Velocidad angular Rueda Derecha en rad/s.
double wL = 0.0;  // Velocidad angular Rueda Izquierda en rad/s.

/////////////////////// Valores //////////////////////
double constValue = 7.040; //(1000*2*pi)/R, R= 25910 // R=10000 motor pequeño //R=892 robi

//////////////////////// ROBOT /////////////////////////
double uMeas_ori  = 0.0;
double wMeas_ori  = 0.0;
double xp = 0.0, yp = 0.0;
double x_ori = 0.0, y_ori = 0.0;
double phi_ori = 0.0;

const double R = 0.03; // radio de la llanta en metros
const double d = 0.34; // Distancia entre llantas en metros

//// Publicadores phi, x,y,uMeas y wMeas

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
 
////////////////// Motor Controller Variables and Constants ///////////////////

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 80; // about 0.1 m/s
const int PWM_MAX = 100; // about 0.172 m/s
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
 
/////////////////////// Motor Controller Functions ////////////////////////////
  
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
   
  // Calculate the PWM value given the desired velocity 
  pwmLeftReq = K_P * cmdVel.linear.x + b;
  pwmRightReq = K_P * cmdVel.linear.x + b;
 
  // Check if we need to turn 
  if (cmdVel.angular.z != 0.0) {
 
    // Turn left
    if (cmdVel.angular.z > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
  else {
     
    // Remove any differences in wheel velocities 
    // to make sure the robot goes straight
    static double prevDiff = 0;
    static double prevPrevDiff = 0;
    double currDifference = velLeftWheel - velRightWheel; 
    double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
    prevPrevDiff = prevDiff;
    prevDiff = currDifference;
 
    // Correct PWM values of both wheels to make the vehicle go straight
    pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
    pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
  }
 
  // Handle low PWM values
  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;  
  }  
}
 
void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW); 
  }
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && velLeftWheel == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && velRightWheel == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut); 
  analogWrite(enB, pwmRightOut); 
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
// Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING); 
   
  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  analogWrite(enA, 0); 
  analogWrite(enB, 0);

  /* Test Velocidad
  analogWrite(enA, 255);
  analogWrite(enB, 255); 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);*/
 
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(phiPub);
  nh.advertise(xPub);
  nh.advertise(yPub);
  nh.advertise(uMeasPub);
  nh.advertise(wMeasPub);
  nh.subscribe(subCmdVel);

  lastTime = millis();
}
 
void loop() {
   
  nh.spinOnce();
 
  if (millis() - lastTime >= sampleTime) { // Se actualiza cada sampleTime (milisegundos)
    computeWR();
    computeWL(); 
    velocityRobot(wR, wL);

    phi_ori = phi_ori+0.1*wMeas_ori;
    xp = uMeas_ori*cos(phi_ori);
    yp = uMeas_ori*sin(phi_ori);
    
    x_ori = x_ori + 0.1*xp;
    y_ori = y_ori + 0.1*yp; 

    phi.data=phi_ori;
    x.data=x_ori;
    y.data=y_ori;
    uMeas.data=uMeas_ori;
    wMeas.data=wMeas_ori;

    //// Publicación de los datos en ROS////
    phiPub.publish( &phi);
    xPub.publish( &x);
    yPub.publish( &y);
    uMeasPub.publish( &uMeas);
    wMeasPub.publish( &wMeas); 
    lastTime = millis();
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
}

// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    nR++;
  }
  else {
    nR--;
  }
}

// Increment the number of pulses by 1
void left_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val_l = digitalRead(ENC_IN_LEFT_B);
 
  if(val_l == HIGH) {
    Direction_left = false; // Reverse
  }
  else {
    Direction_left = true; // Forward
  }
   
  if (Direction_left) {
    nL++;
  }
  else {
    nL--;
  }
}

void computeWR(void) {
  wR = (constValue * nR) / (millis() - lastTime); // Calculamos velocidad Rueda Derecha rad/s
  nR = 0;  // Reiniciamos los pulsos.
}

void computeWL(void) {
  wL = (constValue * nL) / (millis() - lastTime); // Calculamos velocidad Rueda Izquierda rad/s
  nL = 0;  // Reiniciamos los pulsos.
}

void velocityRobot(double w1, double w2)
{
  uMeas_ori = (R*(w1+w2))/2.0; // w1=wR w2=Wl
  wMeas_ori = (R*(w1-w2))/d;
}
