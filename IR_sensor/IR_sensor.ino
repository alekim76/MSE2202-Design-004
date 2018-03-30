/*
  Software serial MSE 2202 IR tester

 The circuit:

* RX is digital pin 7 (connect to TX of other device)
* TX is digital pin 11 (connect to RX of other device)

*/
#include<SoftwareSerial.h>
SoftwareSerial mySerial(7,11);

void setup(){
 Serial.begin(9600);

 while(!Serial){
  ;
 }
 Serial.println("MSE 2202 IR tester");
 mySerial.begin(2400);
 //mySerial.println("Hello, world?");
}

void loop(){
  if(mySerial.available())
  {
    Serial.write(mySerial.read());
  }
 
}


