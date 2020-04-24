#include <Stepper.h>
//#include <TimerOne.h>


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

const unsigned short anguloMax = 69;        //grados que representan el 100% de cerradura de la pinza 
const float angPaso = 1.8;                  //grados que respresentan cada paso del motor (360°/200pasos) 
int pasosReq = 0;           //Pasos requeridos para cerrar la pinza. Se calcula en getVT()

boolean estado = false;       //Variable para controlar el cambio de VolumeControl(false)
                              //a AssistControl(true)

boolean detenerPrograma = false;     //Si es verdadero detiene el motor y vuelve al estado inicial                              


//const int stepsPerRevolution = 11;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(200, 8, 9, 10, 11);

int buzzer = 7;

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), CalcVariablesIngresadas, RISING);
  pinMode(buzzer, OUTPUT);
}

int cont2=0;
void loop() {
  if (!detenerPrograma){
    //Leer los valores al arranque. Se puede intercambiar por Homming.
    if (cont2==0){
      CalcVariablesIngresadas();
      cont2+=1;
    }
    if (!estado){
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
    }else{
      //Assist Control
      Serial.println("Entramos en Assist Control o detenemos el motor al pasar los 40 cmH2O");
      digitalWrite(buzzer, HIGH);
      delay(500);
      digitalWrite(buzzer, LOW);
      delay(500);
    }
  }else{
    Serial.println("PROGRAMA DETENIDO");
  }
}

//Función para ubicar pinzas en posición inicial
void Homing(){
  
}
unsigned long temp = 0;
//Función para control por volumen
void volumeControl(){
  //Inhalación
  myStepper.setSpeed(Vin);
  myStepper.step(pasosReq);
  Presion = getPressure();        //Presion pico
  delay(150);                     //Tiempo de espera para Presión plateau
  Pplat = getPressure();          //Presion plateau
  ///Exhalación
  myStepper.setSpeed(Vex);
  myStepper.step(-pasosReq);
  PEEP = getPressure();           //Presion PEEP
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
  Vex = getVT()/(Tex-0.335);
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
  int angulo = porcentaje*anguloMax/100;          //Calculo del angulo requerido en base a porcentaje ingresado por Dr.
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
