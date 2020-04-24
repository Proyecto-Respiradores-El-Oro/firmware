#include <Stepper.h>
#include <TimerOne.h>
#include "AsyncStepperLib.h"


//Variables para control de motor asíncrono*************************************************
const int motorPin1 = 8;  
const int motorPin2 = 9;  
const int motorPin3 = 10; 
const int motorPin4 = 11; 
const int numSteps = 8;
const int stepsLookup[8] = { B1000, B1100, B0100, B0110, B0010, B0011, B0001, B1001 };
int stepCounter = 0; 

//Funciones para el control del motor******************************************************
void clockwise()
{
  stepCounter++;
  if (stepCounter >= numSteps) stepCounter = 0;
  setOutput(stepCounter);
}

void anticlockwise()
{
  stepCounter--;
  if (stepCounter < 0) stepCounter = numSteps - 1;
  setOutput(stepCounter);
}

void setOutput(int step)
{
  digitalWrite(motorPin1, bitRead(stepsLookup[step], 0));
  digitalWrite(motorPin2, bitRead(stepsLookup[step], 1));
  digitalWrite(motorPin3, bitRead(stepsLookup[step], 2));
  digitalWrite(motorPin4, bitRead(stepsLookup[step], 3));
}

const int stepsPerRevolution = 200;
AsyncStepper stepper1(stepsPerRevolution,
  []() {clockwise(); },
  []() {anticlockwise(); }
);

void rotateCW()
{
  stepper1.Rotate(1.8, AsyncStepper::CW, rotateCCW);
}

void rotateCCW()
{
  stepper1.Rotate(1.8, AsyncStepper::CCW, rotateCW);
}
//*************************************************************************************


//Variables Ingresadas por Dr. (Potenciómetros)
float VT = 0;       //Cantidad de aire a entregar al paciente (0 a 100%) en base a peso del paciente
int RR = 8;        //BPM o Respiraciones por minuto(Respiratory Rate). Entre 8 y 30
int IE = 1;     // Relación inhalación/exhalación (1:1 a 1:4) IE=1,2,3,4.
int Pumbral = 0;     //Límite presión para realizar cambio de VolumeControl to AssitControl


//Variables a calcular
float T = 0;          //Tiempo de inhalación/exhalación T=60/RR (seg)
float Tin = 0;        //Tiempo de inhalación Tin=T/(1+IE) (seg)
float Tex = 0;        //Tiempo de exhalación Tex=T-Tin
float Vin = 0;        //Velocidad de fase de inhalación Vin=VT/Tin (pulsos/seg)
float Vex = 0;        //Velocidad de exhalación (pulsos/seg). Velocidad de apertura de dedos del motor.

//Parámetros seteados por el usuario
float Th = 150;      //Tiempo de espera (miliseg) al terminar la inhalación (mantiene Pressure Plateau)
//float Ve = 0;         
int Pmax = 40;        //Máxima presión permitida (cmH20). Es igual a Pip (peak inspiratory pressure)
const int PlatMax = 30;  //Plateau pressure cm H2O
                          

//Otros parámetros
unsigned long t = 0;           //Cantidad de tiempo en el estado actual o contador de tiempo para comparaciones.
float Pplat = 0;      //Presión a medir al finalizar Tin, durante Th
float PEEP = 0;       //Presión  residual después de exhalar
float Presion = 0;
/*
//Función de transferencia para sensor de presión MPX5050DP
Vout=Vs*(0.018*P+0.04)+-ERRORR;   //Vs=5Vdc
P=(((Vout+-ERRORR)/Vs)-0.04)/0.018;     //Esta presión es en kPas Error=0
                                        // 1 cmH2O = 0,0981 kPa
*/

const unsigned short anguloMax = 50;        //grados que representan el 100% de cerradura de la pinza 
const float angPaso = 1.8;                  //grados que respresentan cada paso del motor (360°/200pasos) 
int pasosReq = 0;           //Pasos requeridos para cerrar la pinza. Se calcula en getVT()

boolean estado = false;       //Variable para controlar el cambio de VolumeControl(false)
                              //a AssistControl(true)

boolean detenerPrograma = false;     //Si es verdadero detiene el motor y vuelve al estado inicial                              

//Variables para control de motor en inhalación
int angulo = 0;
boolean inhal = true;
unsigned short conInhal = 0;
unsigned short conExhal = 0;


unsigned short cont4=0;

int buzzer = 7;
int assistManual = 6;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), CalcVariablesIngresadas, RISING);
  attachInterrupt(digitalPinToInterrupt(3), stopProg, RISING);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(assistManual, INPUT);
  Timer1.initialize(3000);
  Timer1.attachInterrupt(actMotor); 
}

int cont2=0;
void loop() {
  //assistControl();
  if (!detenerPrograma){
    //Leer los valores al arranque. Se puede intercambiar por Homming.
    if (cont2==0){
      CalcVariablesIngresadas();
      cont2+=1;
    }
    if (!estado){
      //Serial.println("Entramos en volume Control");
      volumeControl();
      if (PIPpress()){
        Serial.print("High Pressure. Presion pico sobrepasada: ");
        Serial.println(Presion);
        //////////¿¿¿¿¿¿Se debería realizar el cambio de estado????????/////////////
        estado=true;
      }
      if (pressUnder()){
        Serial.print("Under Pressure. Low Pressure, Disconnect??: ");
        Serial.println(Pplat);
      }
      if (drivPress()){
        Serial.print("Driving pressure. Presion negativa, ELEVATED PEAK PRES : ");
        Serial.println(Pplat);
      }
    }else if(estado){
      //Assist Control
      //Serial.println("Entramos en Assist Control o detenemos el motor al pasar los 40 cmH2O");
      assistControl();
      //digitalWrite(buzzer, HIGH);
      //delay(10);
      //digitalWrite(buzzer, LOW);
      //delay(10);
    }
  }else{
    Serial.println("PROGRAMA DETENIDO");
    delay(1000);
  }
}

//Función para ubicar pinzas en posición inicial
void Homing(){
  
}

//Función para control por volumen///////////
void volumeControl(){
  //Inhalación
  if (inhal){
    if (conInhal==0){
      stepper1.SetSpeedRpm(Vin);
      stepper1.RotateToAngle(angulo*2+5, AsyncStepper::CCW);
      conInhal = 1;
    }
    if (stepper1._stopped){
      inhal = false;
      conExhal = 0;
      Presion = getPressure();        //Presion pico
      stepper1.Stop();
      delay(Th);
      Pplat = getPressure();          //Presion plateau
    }
  }else{
    if (conExhal==0){
      stepper1.SetSpeedRpm(Vex);
      stepper1.Rotate(angulo*2+5, AsyncStepper::CW);
      conExhal = 1;
    }
    if (stepper1._stopped){
      inhal = true;
      conInhal = 0;
      PEEP = getPressure();           //Presion PEEP
    }
  }
}

unsigned short conInhal2 = 0;
unsigned short conExhal2 = 0;
unsigned short cont3=0;
unsigned long t2 = 0;
const int Texhold = 50;                 //ms
//Función para el control asistido /////////////////////////////////////////////
void assistControl(){
  //Inhalación
  //Serial.println(inhal);
  if (inhal){
    if (conInhal2==0){
      stepper1.SetSpeedRpm(Vin);
      stepper1.RotateToAngle(angulo*2+5, AsyncStepper::CCW);
      conInhal2 = 1;
      t=millis();
      //Serial.println(Vin);
      //Serial.println(angulo);
    }
    t2 = millis()-t;
    if (stepper1._stopped && (t2>Tin*1000)){
      inhal = false;
      conExhal2 = 0;
      Presion = getPressure();        //Presion pico
      stepper1.Stop();
      delay(Th);
      Pplat = getPressure();          //Presion plateau
    }
  }else{
    if (conExhal2==0){
      stepper1.SetSpeedRpm(Vex);
      stepper1.Rotate(angulo*2+5, AsyncStepper::CW);
      conExhal2 = 1;
      t=millis();
    }
    t2=millis()-t;
    float Te=Tex-0.05;
    if (t2>=Te*1000){                //50ms antes del tiempo calculado
      if (cont3=0){
        PEEP = getPressure();           //Presion PEEP 1
        cont2=1;
      }
      if (stepper1._stopped){
        delay(Texhold);
        t2=millis()-t;
        if (t2 >= (Tex-Texhold/1000-Te)*1000){   //Falta hacer el cálculo de la presión (PEEP-TS)
          PEEP = getPressure();                  //TS no se encuentra en la descripción del MIT
          inhal = true;
          conInhal2 = 0;
        }
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////
// Variables de Inicio

//Calculo variables
void CalcVariablesIngresadas(){
  RR = getBPM();
  T=60/RR;  
  IE = getIE();
  Tin=T/(1+IE);
  Tex = T-Tin;
//Cálculo de velocidades en rpm para función Stepper
  Vin = getVT()/(Tin-0.335);
  Vex = getVT()/(Tex-0.335-0.15);
}

//Interrupción para detener programa
void stopProg(){
  detenerPrograma = true;
  stepper1.Stop();
}

/////Obteniendo datos de potenciómetros
int getBPM(){
  float lect = analogRead(0);
  float volt = lect*5/1023;     //Basado en un lector analogo de 10 bits y alimentado por 5V
  int bpm = round(volt*22/5)+8;      //Entre 8 y 30 BPM
  return bpm;
}

int getIE(){
  float lect = analogRead(1);
  int ie = 1;
  if (lect<=255){
    ie = 1;
  }else if (lect>255 && lect<=512){
    ie = 2;
  }else if (lect>512 && lect<=768){
    ie = 3;
  }else{
    ie = 4;
  }
  return ie;
}

int getUmbral(){
  int lect = analogRead(2);
  //Máximo recomendado 40 cmH20
  Pumbral = round(lect*40/1023);     //Basado en un lector analogo de 10 bits
  return Pumbral;
} 

int getVT(){
  float lect = analogRead(3);
  float volt = lect*5/1023;
  int porcentaje = round((volt*100)/5);           //Basado en un lector analogo de 10 bits
  angulo = porcentaje*anguloMax/100;          //Calculo del angulo requerido en base a porcentaje ingresado por Dr.
  pasosReq = round(angulo/angPaso);           //Pasos requeridos para cerrar la pinza al % calculado
  int pasosReqFac = pasosReq*3/10;         //3/10 es un factor para transformar a rpm (necesario para funcion stepper de arduino)
  return pasosReqFac;
}

float getPressure(){
  float lect = analogRead(4);
  float vout = lect*5/1024;
  //P=(((Vout+-ERRORR)/Vs)-0.04)/0.018;             //Esta presión es en kPas Error=0. FT de MPX5050DP    
  //float pr = ((vout/5)+0.095)/0.009-1.5;          //1.5 es el eeror de presión configurado por proteus. FT MPX4115
  float pr =((vout/5)-0.04)/0.00369-3.45;              //FT de MPX4250. 3.45 es el eeror de presión configurado por proteus
  pr = pr/0.0981;                                   //Presión en cm H2O
  return pr;
}
/////////////////////////////////////////////////////////////////////////////////////////////

//Funciones de generación de alarmas//////////////////////////////////

//Exceeded PIP Pressure 
boolean pressPip = false;
boolean PIPpress(){
  if (Presion>Pmax){
    pressPip = true;
  }
  return pressPip;
}

//Under Pressure. Revisar si la presión comparada es la correcta. Pplateau
boolean underPress = false;
boolean pressUnder(){
  if (Pplat<Pumbral){
    underPress = true;
  }
  return underPress;
}

//Exceeded Driving Pressure
boolean dPress = false;
boolean drivPress(){
  if (Pplat<PEEP){
    dPress = true;
  }
  return dPress;
}

//Tidal Pressure not Detected





//Over Current Fault. No hay sensor de corriente.

//Tidal Volume Not Delivered. No se puede medir. No hay encoder.




///////////////////////////Actualizar motor////////
boolean au = false;
void actMotor(){
  stepper1.Update();
}
