/*
  Software serial MSE 2202 IR tester

  The circuit:

  RX is digital pin 7 (connect to TX of other device)
  TX is digital pin 11 (connect to RX of other device)

*/
#include<SoftwareSerial.h>
SoftwareSerial mySerial(7, 7);
//SoftwareSerial mySerial2(6, 6);
//SoftwareSerial mySerial3(5, 5);

const int irsensor = 13;

int hallSensorPin = A1;
int state;

void setup() {
  Serial.begin(9600);

  pinMode(irsensor, OUTPUT);

  pinMode(hallSensorPin, INPUT);

  pinMode(8, OUTPUT);


  while (!Serial) {
    ;
  }
  mySerial.begin(2400);

}

void loop() {

  state = analogRead(hallSensorPin);
  Serial.println(state);

  if (state > 400) {
    //IR sensor code
    Serial.println("Found the cube");
    digitalWrite(8, HIGH);

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
        digitalWrite(irsensor , HIGH);
      }
      else {
        digitalWrite(irsensor , LOW);
      }
    }
    else {
      //ultrasonic code to find the cube
      Serial.println("keep looking");
      digitalWrite(8, LOW);
    }
  }
}


