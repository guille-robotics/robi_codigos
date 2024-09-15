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

///// VALORES VELOCIDAD ANGULAR ////
double wDerecho=0;
double wIzquierdo=0;
double constValue=27.54;// (1000*2*pi)/R ---> R = 228 Resolucion encoder cuadruple
/// DATOS PARA LECTURA DE ENCODER////
volatile long  nDerecho = 0;
volatile long  nIzquierdo = 0;

bool Direction_derecha=false;
bool Direction_izquierda=false;

//// CONTROL DEL TIEMPO /////
unsigned long lastTime = 0;  // Tiempo anterior
unsigned long sampleTime = 100;  // Tiempo de muestreo

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
  
  Serial.println("Numero de conteos");

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
     
     cvDerecho = data[0];
     cvIzquierdo = data[1];

     inputString = "";
     stringComplete = false;
  }

  //Serial.print("Cuentas: ");Serial.println(n);
  if (millis() - lastTime >= sampleTime){  
      calculo_vel_angular_derecha();
      calculo_vel_angular_izquierda();
      lastTime = millis();
      if (cvDerecho > 0) anticlockwise(in1,in2,ena,cvDerecho); else clockwise(in1,in2,ena,abs(cvDerecho));
      if (cvIzquierdo > 0) anticlockwise(in3,in4,enb,cvIzquierdo); else clockwise(in3,in4,enb,abs(cvIzquierdo));
      Serial.print("Velocidad Angular Derecha: ");
      Serial.print(wDerecho);
      Serial.print("||||||||||||");
      Serial.print("Velocidad Angular Izquierda: ");
      Serial.println(wIzquierdo);
   }
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
// Encoder Izquierdo
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
//// FUNCIONES CALCULO DE LAS VELOCIDADES ANGULARES /////
// Velocidad angular derecha
void calculo_vel_angular_derecha(){
  wDerecho = constValue*nDerecho/(millis()-lastTime);
  nDerecho=0;
}
// Velocidad angular izquierda
void calculo_vel_angular_izquierda(){
  wIzquierdo = constValue*nIzquierdo/(millis()-lastTime);
  nIzquierdo=0;
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