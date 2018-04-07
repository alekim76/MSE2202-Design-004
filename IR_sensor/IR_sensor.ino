//BOARD 1
#include<SoftwareSerial.h>
SoftwareSerial mySerial(7, 7); // set up ir sensor to detect signals from pyramid


const int irsensor = 13; // pin 13 is shared by both boards, this allows communication without the TX and RX ports (which may have been faulty)

int hallSensorPin = A1;
int state; // variable for hall sensosor values

void setup() {
  Serial.begin(9600); // begin at standard baud rate

  pinMode(irsensor, OUTPUT); // ir sensor value is ouputted to board 2

  pinMode(hallSensorPin, INPUT); // hall effect sensor is used to detect when cube has been detected

  pinMode(8, OUTPUT); // connects hall sensor from board 1 to board 2 directll from pin to pin

  pinMode(9, OUTPUT); // led to viusally verify when hall effect sensor is detecting the cube


  while (!Serial) {
    ;
  }
  mySerial.begin(2400); // begin the ir sensor serial at the standard data rate

}

void loop() {

  state = analogRead(hallSensorPin); //read the value of the hall effect sensor

  if (state < 600) { // from testing. this value correspond to when the cube has been retrieved
    
    digitalWrite(9, HIGH); // when found set the led to HIGH
    digitalWrite(8, HIGH); // set the hall effect pin on Board 2 to HIGH, signalling board 2 that the cube has been found

    if (mySerial.available()) // detect if the pyramid data is availiable to be read
    {
      Serial.write(mySerial.read()); // reads what the pyramid is sending to the ir sensor
    }
    else{
      digitalWrite(irsensor,LOW); // if no data available, keep the ir sensor LOW
    }

    if (digitalRead(2)) // if the switch is in the position to read A and E signals from pyramid
    {
      if (mySerial.read() == 'A' || mySerial.read() == 'E') // if the ir sensor reads an A or an E signal
      {
        digitalWrite(irsensor , HIGH); // set the ir sensor pin 13 high, signalling other board that pyramid has been found
      }
    }
    if (!digitalRead(2))  // if the switch is in the position to read I and O signals from pyramid
    {
      if (mySerial.read() == 'I' || mySerial.read() == 'O') // if the ir sensor reads an I or an O signal
      {
        digitalWrite(irsensor , HIGH); // set the ir sensor pin 13 high, signalling other board that pyramid has been found
      }
    }
  }
  else {
    digitalWrite(8, LOW); // if the cube has not been found, set the led and the board 2 pin to LOW
    digitalWrite(9, LOW); //
  }
}



