int p;
int pa;
int pd;
int t;

// Pines de los sensores
int s1 = 30;
int s2 = 31;
int s3 = 32;
int s4 = 33;

// Pines del sentido de giro del motor
int gd = 22;
int gi = 23;

void setup() {

  // Configuración de sensores como entrada:
  pinMode(s1, INPUT);
  pinMode(s2, INPUT);
  pinMode(s3, INPUT);
  pinMode(s4, INPUT);

  // Configuración de sentido de giro del motor como salida
  pinMode(gd, OUTPUT);
  pinMode(gi, OUTPUT);

  Serial3.begin(9600);
  Serial.begin(9600);

}

void loop() {

  // Lectura del sensor donde está ubicado
  if (digitalRead(s1) == HIGH) {
    pa = 1;
  }
  if (digitalRead(s2) == HIGH) {
    pa = 2;
  }
  if (digitalRead(s3) == HIGH) {
    pa = 3;
  }
  if (digitalRead(s4) == HIGH) {
    pa = 4;
  }

  while (Serial3.available() == 0) {} // No hace nada mientras no haya un dato en Serial3

  if (Serial3.available() > 0) { // Verifica que hayan escrito algo en el monitor serial
    {
      pd = Serial3.read(); // Lee el sensor escogido en el emisor y lo almacena en pd

      // Establecimiento del sensor destino deseado según elección del usuario

      if (pd == '1') {
        pd = 1;
      }
      if (pd == '2') {
        pd = 2;
      }
      if (pd == '3') {
        pd = 3;
      }
      if (pd == '4') {
        pd = 4;
      }
      
      for (int i = pa; i > digitalRead(pd) == HIGH; i++){ // Compara la pd hasta que sea igual a pa
        digitalWrite(gd, HIGH);
        digitalWrite(gi, LOW);
        Serial3.write('d');
      }
      
      digitalWrite(gi, LOW);
      digitalWrite(gd, LOW);
      
      t = Serial3.read(); // Lee el valor en el serial 3 y lo almacena en t
      t = Serial3.parseInt(); // El caracter almacenado lo devuelve como un entero
      delay(t);

      for (int i = pa; i < pd; digitalRead(pd) == HIGH){ // Compara la pd hasta que sea igual a pa
        digitalWrite(gi, HIGH);
        digitalWrite(gd, LOW);
        Serial3.write('i');
      }
      
      digitalWrite(gi, LOW);
      digitalWrite(gd, LOW);
      
      t = Serial3.read(); // Lee el valor en el serial 3 y lo almacena en t
      t = Serial3.parseInt(); // El caracter almacenado lo devuelve como un entero
      delay(t);
    }
    
    if (Serial3.available() > 0) { // Verifica que hayan escrito algo en el monitor serial
      {
        p = Serial3.read(); // Lee el valor y lo almacena en p
        Serial.print(p);
        if (p == 'p'){ // Detiene el motor
          digitalRead(gd) == LOW;
          digitalRead(gi) == LOW;
        }
      }
    }
  }
}
