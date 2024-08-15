#include "PinChangeInterrupt.h"
#include "motorControl.h"

/////////////////////////// CONTROLADOR PID //////////////////
unsigned long lastTime, sampleTime = 100;

motorControl motorR(sampleTime);
motorControl motorL(sampleTime);

///////////////////// COMUNICACION SERIAL ////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 2;
double data[dataLength];

//////////////////////MOTOR DERECHO///////////////////////////////
//// Ojo se ha invertido canales////////
const int    C1R = 2;    // Entrada de la señal A del encoder.
const int    C2R = 4;    // Entrada de la señal B del encoder.
int cvR = 0;

bool Direction_right;


//// Puente H L298N ////
const int    in1 = 9;                 
const int    in2 = 8;         
const int    ena = 11;  

volatile int nR = 0;
volatile int antR      = 0;
volatile int actR      = 0;

double w1Ref = 0;
double wR = 0;

//////////////////////MOTOR IZQUIERDO///////////////////////////////
const int    C1L = 3;                  // Entrada de la señal A del encoder.
const int    C2L = 5;                  // Entrada de la señal B del encoder.
int cvL = 0;

bool Direction_left;

//// Puente H L298N ////
const int    in3 = 7;                  
const int    in4 = 6;                  
const int    enb = 10;

volatile int nL = 0;
volatile int antL      = 0;
volatile int actL      = 0; 

double w2Ref = 0; 
double wL = 0;

//////// VARIABLES PARA CALCULAR VELOCIDADES ANGULARES /////////

double constValue = 20.33; // (1000*2*pi)/R ---> R = 1980 Resolucion encoder cuadruple


void setup()
{
  Serial.begin(9600);
  
  ////////////////// SINTONIA FINA PID //////////////////
  
  motorR.setGains(0.37, 0.1, 0.06); // (Kc,Ti,Td)
  motorL.setGains(0.37, 0.1, 0.06); // (Kc,Ti,Td)
  
  ////////////////// Limites de señales //////////////////
  motorR.setCvLimits(255,15);
  motorR.setPvLimits(13,0);  

  motorL.setCvLimits(255,15);
  motorL.setPvLimits(13,0);  
  
  pinMode(C1R, INPUT_PULLUP);
  pinMode(C2R, INPUT);
  pinMode(C1L, INPUT_PULLUP);
  pinMode(C2L, INPUT);

  pinMode(in1, OUTPUT);       
  pinMode(in2, OUTPUT);   
  pinMode(in3, OUTPUT);       
  pinMode(in4, OUTPUT); 

  
  digitalWrite(in1, false);       
  digitalWrite(in2, false);   
  digitalWrite(in3, false);       
  digitalWrite(in4, false); 

  attachInterrupt(digitalPinToInterrupt(C1R), right_wheel_pulse, RISING);
  //attachInterrupt(digitalPinToInterrupt(C2R), encoderR, CHANGE);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(C1L), left_wheel_pulse, RISING);
  //attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(C2L), encoderL, CHANGE);             

  lastTime = millis();
     
}
void loop() 
{
  ////////// SI RECIBE DATOS /////////////
  if (stringComplete) 
  {
    for (int i = 0; i < dataLength ; i++)
    {
      int index = inputString.indexOf(separator);
      data[i] = inputString.substring(0, index).toFloat();
      inputString = inputString.substring(index + 1);
     }
     
     w1Ref = data[0];
     w2Ref = data[1];

     inputString = "";
     stringComplete = false;
  }

  
  /////////////////// CONTROLADOR PID ////////////////
  if(millis()-lastTime >= sampleTime)
  {
    wR = constValue*nR/(millis()-lastTime);
    wL = constValue*nL/(millis()-lastTime);
    lastTime = millis();
    nR = 0;
    nL = 0;
    
    cvR = motorR.compute(w1Ref,wR);
    cvL = motorL.compute(w2Ref,wL);

    if (cvR > 0) clockwise(in2,in1,ena,cvR); else anticlockwise(in2,in1,ena,abs(cvR));     
    if (cvL > 0) anticlockwise(in3,in4,enb,cvL); else clockwise(in3,in4,enb,abs(cvL));

    Serial.println(wR); 
    Serial.println(wL); 

  }
  
}
/////////////// RECEPCION DE DATOS /////////////////////
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

// Función calculo encoder Rueda Derecha
void right_wheel_pulse() {   
  int val = digitalRead(C2R);
 
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

// Función calculo encoder Rueda Izquierda
void left_wheel_pulse() {
  int val_L = digitalRead(C2L);
 
  if(val_L == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    nL++;
  }
  else {
    nL--;
  }
}

void clockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, LOW);  
  digitalWrite(pin2, HIGH);      
  analogWrite(analogPin,pwm);
}

void anticlockwise(int pin1, int pin2,int analogPin, int pwm)
{
  digitalWrite(pin1, HIGH);  
  digitalWrite(pin2, LOW);      
  analogWrite(analogPin,pwm);
}
