#include <math.h>

//Valores fijos del circuito
int sensorPin = A1;
float rAux = 9960.0;
float vcc = 5.0;
float beta = 2955.0;
float temp0 = 298.15;
float r0 = 10380.0;

//Variables usadas en el cálculo
float vm = 0.0;
float rntc = 0.0;
float temperaturaK = 0.0;


void setup() {

  Serial.begin(9600); 
  
}

void loop() {

  //Bloque de cálculo
  vm=(vcc / 1024)*( analogRead(sensorPin));                //Calcular tensión en la entrada
  rntc = rAux / ((vcc/vm)-1);                       //Calcular la resistencia de la NTC
  temperaturaK = beta/(log(rntc/r0)+(beta/temp0));  //Calcular la temperatura en Kelvin

  //Restar 273 para pasar a grados celsus
  Serial.println(temperaturaK -273.15 );
  
  delay(1000); 
}
