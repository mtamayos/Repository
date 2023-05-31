#include <DS1307.h>
#include <avr/wdt.h>
#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <ServoCds55.h>
#include <math.h>
//#define Ain_0 A0
#define Cin A1              //Pin analogo para lectura de corriente
#define Vin A2              //Pin analogo para lectura de voltaje
const int multi1 = 2;       //Pin de control del multiplexor
const int multi2 = 3;       //Pin de control del multiplexor
const int multi3 = 4;       //Pin de control del multiplexor
const int inhabilitar = 5;  //Pin para inhabilitar el multiplexor
const int gate = 6;         //Pin de señal comun para activar MOSFETs
const int buzzer = 7;      //Pin del buzzer
const int LED = 13;         //Pin del LED



const int Red = 12;
const int Green = 11;
const int Blue = 8;



int horaint = 0;
int i = 0;                  //Iniciar contador para ciclos
float vCin = 0.0;           //Iniciar variable para medir corriente
float vVin = 0.0;           //Iniciar variable para medir voltaje
float calibracion = 0.0;    //Iniciar variable de calibracion del sensor de corriente
float calibracion2 = 0.0;   //Iniciar variable de calibracion del sensor de corriente (variable)
float calibraciontemp = 0.0;//Iniciar variable de calibracion del sensor de corriente (temporal)
float corr = 0.0;           //Iniciar varible que guarda valor de corriente
float volt = 0.0;           //Iniciar varible que guarda valor de voltaje
float muestras = 300.0;
int angulo =0;


int sensorPin = 5;
int sensorPin2 = 0;
int sensorPin3 = 3;
int sensorPin4 = 4;
float rAux = 9960.0;
float vcc = 5.0;
float beta = 2955.0;
float temp0 = 298.15;
float r0 = 10380.0;
//Variables usadas en el cálculo Termistores
float vm = 0.0;
float rntc = 0.0;
float temperaturaK = 0.0;
String Tcorr = "";          //Iniciar varible que guarda valor de corriente como string
String Tvolt = "";          //Iniciar varible que guarda valor de voltaje como string

float intervalo = 10000;     //Intervalo entre medida de cada modulo(milisegundos)
float intervalo1 = 30000;
float tiempo1 = 0;
float tiempo = 0;
float t_cambio = 50;         //Tiempo de espera para  apertura o cierre de reles(milisegundos)
int mod[] = {1, 2, 3, 4}; //Definir que modulos van a ser evaluados - ej. mod[] = {1, 2, 4}; evaluaria unicamente los modulos 1, 2 y 4
//int mod[] = {1, 2}; //Definir que modulos van a ser evaluados - ej. mod[] = {1, 2, 4}; evaluaria unicamente los modulos 1, 2 y 4

bool mod1 = false;
bool mod2 = false;
bool mod3 = false;
bool mod4 = false;
bool check_file = false;


////////////////////////////////////////


String valores;
File myFile;
DS1307 rtc;

ServoCds55 myservo1;
ServoCds55 myservo2;
ServoCds55 myservo3;
int id1=1;
int id2=2;
int id3=3;


char t[30];
int pinCS = 53;
//milis para retardo
unsigned long Present_time = millis();
unsigned long Last_time = 0;

/////
///////////////////////Funciones electronica de Juan Pablo 
//Programa encargado de la lectura del sensor de corriente
//Programa encargado de la lectura del sensor de corriente
float medir_corriente() {
  vCin = 0;
  delay(t_cambio);
  
  for (i=0; i < muestras; i++) {
    vCin = vCin + (analogRead(Cin) * 5.0 / 1023.0) - calibracion2;
  }
  vCin = vCin / muestras;
  vCin = (vCin / 2.75) / 0.185;
  return vCin;
}

//Programa encargado de la lectura del voltaje
float medir_voltaje() {
  vVin = 0;
  delay(t_cambio);
  
  for (i=0; i < muestras; i++) {
    vVin = vVin + (analogRead(Vin) * 5.0 / 1023.0);
  }
  vVin = vVin / muestras;
  vVin = vVin / 4.05;
  return vVin;
}




void RGB(int Val_R, int Val_G, int Val_B)
{
  analogWrite(Red, Val_R);
  analogWrite(Green, Val_G);
  analogWrite(Blue, Val_B);
}
////////////////////////////////////

void bip(int intensidad)
{
  analogWrite(buzzer, intensidad);
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(buzzer, LOW);
  digitalWrite(LED, LOW);
  delay(100);
}

void setup() {
  
  pinMode(Red, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode(Blue, OUTPUT);
  
  

  Wire.begin();
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);


  /////////////////////// ELECTRONICA JUAN PABLO //////////////////
  wdt_disable();
  Serial.begin(9600);
  //Iniciar puertos de salida
  pinMode(inhabilitar, OUTPUT);
  digitalWrite(inhabilitar, HIGH); //Deshabilitar multiplexor
  pinMode(multi1, OUTPUT);
  pinMode(multi2, OUTPUT);
  pinMode(multi3, OUTPUT);
  pinMode(gate, OUTPUT);
  digitalWrite(gate, HIGH);       //Activar señal comun del multiplexor para encendido de los mosfets
  pinMode(buzzer, OUTPUT);
  pinMode(LED, OUTPUT);

  //Iniciar puertos de entrada
  pinMode(Cin, INPUT);
  pinMode(Vin, INPUT);
 // pinMode(Ain_0, INPUT);
  pinMode(5, INPUT);

  bip(255);
  bip(255);
  bip(255);

  //Habilitar Watchdog a 8 segundos
  wdt_enable(WDTO_8S);
//Preparar para calibracion del sensor de corriente
  //IMPORTANTE: Para realizar una correcta calibracion del sensor es necesario que el panel se encuentre
  // completamente tapado, logrando una corriente de salida de 0.00 amperios.
  //Sonar buzzer dos veces para hacer saber que se inicio la calibracion
  bip(255);
  bip(255);
  digitalWrite(inhabilitar, HIGH); //Deshabilitar multiplexor
  delay(80);
  //Calibrar sensor de corriente, se toman 200 medidas del sensor y se promedian para mayor precision.
  for (i=0; i < 200; i++) {
    calibracion = calibracion + (analogRead(Cin) * 5.0 / 1023.0);
  }
  calibracion = calibracion / 200.0;
  calibracion2 = calibracion;
  Serial.print("Calibracion 2 setup ");
  Serial.println(calibracion2);
  //Serial.println(" ");
//Se identifican los modulos a evaluar
  
  for (i=0; i < (sizeof(mod) / 2) ; i++) {

    if (mod[i] == 1) {
      mod1 = true;
    }
    else if (mod[i] == 2) {
      mod2 = true;
    }
    else if (mod[i] == 3) {
      mod3 = true;
    }
    else if (mod[i] == 4) {
      mod4 = true;
    }
    else {
    }
  }
  ///////////////////////////////////
  
  if (SD.begin()) //////////////////inicializacion modulo SD
  {
    Serial.println("SD card is ready to use.");
    RGB(0,255,0);
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
Serial.println("Init RTC...");


/////// Datos para crear archivo CSV ///////////
  if(!SD.exists("datalog.csv"))
  {
      myFile = SD.open("datalog.csv", FILE_WRITE);
      if (myFile) {
        Serial.println("Archivo nuevo, Escribiendo encabezado(fila 1)");
        //myFile.println("Data,Voc_cell1_Fix,Isc_cell1_Fix,Voc_cell2_3P,Isc_cell2_3P");
        myFile.println("Data,Voc_cell1_CT,Isc_cell1_CT,Voc_cell2_2P,Isc_cell2_2P,Voc_cell3,Isc_cell3,Temp1.1,Temp2.2,Voc_cell4,Isc_cell4,Temp1.2,Temp2.2");
       // myFile.println("Data,Voc_cell1_CT,Isc_cell1_CT,Voc_cell2_2P,Isc_cell2_2P,Voc_cell3_4P,Isc_cell3_4P,Voc_cell4_Fix,Isc_cell4_Fix");
        myFile.close();
      } else {

        Serial.println("Error creando el archivo datalog.csv");
      }
  }


  //only set the date+time one time
  //rtc.set(0, 32, 7, 18, 8, 2021); //08:00:00 24.12.2014 //sec, min, hour, day, month, year
  //stop/pause RTC
  // rtc.stop();
  //start RTC
 rtc.start();  

  /////////////////inicializar los servos////////////777

  myservo1.begin ();
  myservo2.begin ();
  myservo3.begin ();
  myservo1.setVelocity(30);
  delay(200);
  myservo2.setVelocity(30);
  delay(200);
  myservo3.setVelocity(30);
  delay(200);
  myservo1.write(id1, 150); //ID:1  Pos:300  velocity:150
  delay(1000);
  myservo2.write(id2, 150);
  delay(1000);
   myservo3.write(id3, 150);
  delay(1000);

  
  /////////////////////////

  // put your setup code here, to run once:
}




void loop() {



   ////////////////electronica juan pablo/////////////
  wdt_reset();
  ////////////////////////////////////Obtener valores RTC/////////////////////////
  uint8_t sec, min, hour, day, month;
  uint16_t year;
  //Obtener valores del RTC
  //////////////////////Configuracion del reloj /////////////////////////


  //Antes de iniciar el nuevo ciclo, calibrar la medida de la corriente para ignorar ruidos
for(i=0; i < 200; i++) {
    calibraciontemp = calibraciontemp + (analogRead(Cin) * 5.0 / 1023.0);
}
   calibraciontemp = calibraciontemp / 200;
        Serial.print("Calibracion temp loop ");
      Serial.println(calibraciontemp);
      //Serial.println(" ");
  wdt_reset();
  if ((calibraciontemp <= (calibracion - 0.010)) || (calibraciontemp >= (calibracion + 0.010))) {
      calibracion2 = calibraciontemp;
       // Serial.print("Calibracion  loop ");
      //Serial.println(calibracion);
      Serial.print("Calibracion + 0.15 ");
      Serial.println((calibracion + 0.015));
      Serial.println("Calibracion 2 ");
      Serial.println(calibracion2);
      //Serial.println(" ");

  }
  else {
  }
  calibraciontemp = 0.0;
  tiempo = millis();


///////////////////////////////
 rtc.get(&sec, &min, &hour, &day, &month, &year);
  sprintf(t, "%02d:%02d:%02d %02d/%02d/%02d",  (hour), (min), (sec), (day), (month), (year));
 // Serial.print(' ');
  Serial.print(t);
float read_time_sd = float(hour) + (float(min)/100.0);

Serial.println(" ");
Serial.print((hour));
Serial.print(" ");
Serial.print((min));
Serial.println(" ");
//tracking(int(hour));
//RGB(255,0,0);
//delay(200);
//tracking1(int(hour));
//RGB(0,0,255);
//delay(200);
//Serial.println(angulo);


if (read_time_sd >= 6.30 && read_time_sd <= 20.30)   //Horas entre las cuales se tomaran mediciones 
{
 myFile = SD.open("datalog.csv", FILE_WRITE);
 check_file = false;
  if (myFile) {    
    RGB(124,252,0);
    check_file = true;
    myFile.print(t);
    myFile.print(","); 
    //////////////////////todo la linea de las celdas ///////////////////

    bip(255);
    digitalWrite(inhabilitar, LOW);
    delay(50);
    if (mod1 == true) {
      //Leer medidas para circuito abierto
      //Encender pin 2, apagar pin 3 (Los pines 2 y 3 son los que controlan el multiplexor)
      digitalWrite(multi1, LOW);
      digitalWrite(multi2, LOW);
      digitalWrite(multi3, LOW);
      Tvolt = String(medir_voltaje(), 3);
      myFile.print(Tvolt);  
      myFile.print(","); 
      Serial.println("Tvolt1 " + Tvolt);
      
      
      //Leer medidas para cortocircuito
      //Encender pin 2, apagar pin 3 (Los pines 2 y 3 son los que controlan el multiplexor)
      digitalWrite(multi1, HIGH);
      digitalWrite(multi2, HIGH);
      digitalWrite(multi3, HIGH);
      Tcorr = String(medir_corriente(), 3);
      myFile.print(Tcorr);  
      myFile.print(","); 
      Serial.println("Tcorr1 " + Tcorr);
      Serial.println(" ");
      Tvolt = "";
      Tcorr = "";
  
      RGB(210,105,30);
      delay(100);
      
      //tracking(int(hour));
      //myFile.print(angulo);  
     // myFile.print(","); 
     // RGB(255,0,0);
      //delay(200);
      
    }
    
    //  Medicion modulo 2
    if (mod2 == true)  {

      digitalWrite(multi1, LOW);
      digitalWrite(multi2, LOW);
      digitalWrite(multi3, HIGH);
      Tvolt = String(medir_voltaje(), 3);
      myFile.print(Tvolt);  
      myFile.print(","); 
      Serial.println("Tvolt2 " + Tvolt);
      

      digitalWrite(multi1, HIGH);
      digitalWrite(multi2, HIGH);
      digitalWrite(multi3, LOW);
      Tcorr = String(medir_corriente(), 3);
      myFile.print(Tcorr);  
      myFile.print(","); 
      Serial.println("Tcorr2 " + Tcorr);
      Serial.println(" ");
      Tvolt = "";
      Tcorr = "";
      
      RGB(255,255,0);
      delay(100);
      
     // tracking1(int(hour));
      //myFile.print(angulo);  
     // myFile.print(","); 
      //RGB(0,0,255);
     // delay(200);
     //Bloque de cálculo Termistor 
 // vm=(vcc / 1023.0)*( analogRead(5));                //Calcular tensión en la entrada
  //rntc = rAux / ((vcc/vm)-1.0);                       //Calcular la resistencia de la NTC
  //temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  //Serial.println(temperaturaK );
 // myFile.print(temperaturaK -273.15);  
   //   myFile.print(","); 

    }

    //Medicion modulo 3
    
    if (mod3 == true) {

      digitalWrite(multi1, LOW);
      digitalWrite(multi2, HIGH);
      digitalWrite(multi3, LOW);
      Tvolt = String(medir_voltaje(), 3);
      myFile.print(Tvolt);  
      myFile.print(","); 
      Serial.println("Tvolt3 " + Tvolt);
      

      digitalWrite(multi1, HIGH);
      digitalWrite(multi2, LOW);
      digitalWrite(multi3, HIGH);
      Tcorr = String(medir_corriente(), 3);
      myFile.print(Tcorr);  
      myFile.print(","); 
      Serial.println("Tcorr3 " + Tcorr);
      Serial.println(", ");
      Tvolt = "";
      Tcorr = "";
     
      RGB(127,255,212);
      delay(100);
       //Bloque de cálculo Termistor 1
  vm=(vcc / 1023.0)*( analogRead(5));                //Calcular tensión en la entrada
  rntc = rAux / ((vcc/vm)-1.0);                       //Calcular la resistencia de la NTC
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  Serial.println(analogRead(5) );
  myFile.print(temperaturaK -273.15);  
      myFile.print(","); 
      //Bloque de cálculo Termistor 2
  vm=(vcc / 1023.0)*( analogRead(0));                //Calcular tensión en la entrada
  rntc = rAux / ((vcc/vm)-1.0);                       //Calcular la resistencia de la NTC
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  Serial.println(analogRead(0) );
  Serial.println("0");
  myFile.print(temperaturaK -273.15);  
      myFile.print(","); 


      
    }

    //Medicion modulo 4
    
    if (mod4 == true) {

      digitalWrite(multi1, LOW);
      digitalWrite(multi2, HIGH);
      digitalWrite(multi3, HIGH);
      Tvolt = String(medir_voltaje(), 3);
      myFile.print(Tvolt);  
      myFile.print(","); 
      Serial.println("Tvolt4 " + Tvolt);
      

      digitalWrite(multi1, HIGH);
      digitalWrite(multi2, LOW);
      digitalWrite(multi3, LOW);
      Tcorr = String(medir_corriente(), 3);
      myFile.print(Tcorr);  
      myFile.print(",");
      Serial.println("Tcorr4 " + Tcorr);
      Serial.println(" ");
      Tvolt = "";
      Tcorr = "";    
       //Bloque de cálculo Termistor 
  vm=(vcc / 1023.0)*( analogRead(3));                //Calcular tensión en la entrada
  rntc = rAux / ((vcc/vm)-1.0);                       //Calcular la resistencia de la NTC
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  Serial.println(analogRead(3) );
  myFile.print(temperaturaK -273.15);  
      myFile.print(","); 
       //Bloque de cálculo Termistor 
  vm=(vcc / 1023.0)*( analogRead(4));                //Calcular tensión en la entrada
  rntc = rAux / ((vcc/vm)-1.0);                       //Calcular la resistencia de la NTC
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  Serial.println(analogRead(4) );
  myFile.print(temperaturaK -273.15);  
      

    }
  myFile.println(" ");
  myFile.close(); // close the file
  digitalWrite(inhabilitar, HIGH);

  wdt_reset();
  bip(255);
  delay(100);
  bip(255);



  }
  // if the file didn't open, print an error:
  else {
          Serial.println("error opening test.txt");
        
  } ////////////////////////////////
/////////////////////////////////
}///////Horas entre las cuales se tomaran medidas 
else {
  RGB(255,165,0);
  myservo1.write(id1,102);
  delay(100);
  myservo2.write(id2,86);
  //myservo2.write(id2,113);
  delay(100);
  myservo3.write(id3, 69);
  delay(100);
  
  
}  
  //////////////////////////
  while ((millis() - tiempo) < intervalo) {
    wdt_reset();
    // Mientras se espera por el proximo ciclo de medicion, se verifica que no exista ningun corto que pueda dañar las celdas, para esto se
    // verifica que el sensor de corriente este completamente centrado, dando un margen de 20mA (0.015V en la salida del amplificador del sensor
    // de corriente), margen suficiente para evitar falsos "cortos" causados por ruidos electricos de los dispositivos.
    corr = analogRead(Cin) * 5.0 / 1023.0;
    i = 0;
    if ((corr >= (calibracion2 + 0.010)) || (corr <= (calibracion2 - 0.010))) {
      wdt_reset();
      i += 1;
      if (i > 10) {
        bip(255);
        bip(255);
        bip(255);
        delay(500);
      }
      if (i > 20) {
        delay(10000);  //call reset
      }
    }
    else {
      i = 0;
      delay(10);
    }
    //if (millis() > 1800000) {
     // delay(10000);
   // }
  }
 //////////////////////////////////////////s
 
 
}
