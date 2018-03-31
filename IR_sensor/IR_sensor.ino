/*
  Software serial MSE 2202 IR tester

  The circuit:

  RX is digital pin 7 (connect to TX of other device)
  TX is digital pin 11 (connect to RX of other device)

*/
#include<SoftwareSerial.h>
SoftwareSerial mySerial(7, 7);

const int irsensor = 13;


void setup() {
  Serial.begin(9600);

  pinMode(irsensor, OUTPUT);

  while (!Serial) {
    ;
  }
  Serial.println("MSE 2202 IR tester");
  mySerial.begin(2400);
  //mySerial.println("Hello, world?");
}

void loop() {
  if (mySerial.available())
  {
    Serial.write(mySerial.read());
  }


  if (digitalRead(1))
  {
    if (mySerial.read() == 'A' || mySerial.read() == 'E')
    {
      digitalWrite(irsensor , HIGH);
    }
    else {
      digitalWrite(irsensor , LOW);
    }
  }
  if (!digitalRead(1))
  {
    if (mySerial.read() == 'I' || mySerial.read() == 'O')
    {
      //Arm and slide codes go here
      digitalWrite(irsensor , HIGH);
    }
     else {
      digitalWrite(irsensor , LOW);
    }
  }
}


