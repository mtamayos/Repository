int s;
int pul = 37;
int pd;
int t;

void setup() {

  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(pul, INPUT);
}

void loop() {

  Serial.println("Selecciona el sensor destino (1, 2, 3 o 4): ");

  while (Serial.available() == 0) {} // No hace nada mientras no haya un dato en serial

  if (Serial.available() > 0) { // Verifica que hayan escrito algo en el monitor serial

    pd = Serial.read(); // Lee el monitor serial
    Serial.println(pd); //
    
    if (pd == '1') { // El usuario selecciona el sensor 1 como destino
      Serial3.print('1'); // Manda el dato al receptor
    }

    if (pd == '2') { // El usuario selecciona el sensor 2 como destino
      Serial3.print('2'); // Manda el dato al receptor
    }

    if (pd == '3') { // El usuario selecciona el sensor 3 como destino
      Serial3.print('3'); // Manda el dato al receptor
    }

    if (pd == '4') { // El usuario selecciona el sensor 4 como destino
      Serial3.print('4'); // Manda el dato al receptor
    }
  }

  Serial.available() == 0;
  Serial.print("Del 1 al 9, ¿Cuántos segundos deseas que permanezca el objeto en el destino que seleccionaste?");

  while (Serial.available() == 0) {} // No hace nada mientras no haya un dato en serial

  if (Serial.available() > 0) { // Verifica que hayan escrito algo en el monitor serial
    t = Serial.read();
    Serial.println(t);
    Serial3.print(t);
  }

  if (digitalRead(pul) == HIGH) // Si activa el pulsador del pin 37, debe pausarse
  {
    Serial3.print('p'); // Manda el dato al receptor como 'p'
    analogWrite(A0, 0);
    analogWrite(A1, 0);
    analogWrite(A2, 255);
  }
    else {
      
      s = Serial3.read();

      if (s == 'd') {
        analogWrite(A0, 255);
        analogWrite(A1, 0);
        analogWrite(A2, 0);
      }
      else if (s = 'i') {
        analogWrite(A0, 0);
        analogWrite(A1, 255);
        analogWrite(A2, 0);
      }
    }
  }
