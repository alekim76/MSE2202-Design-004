/*
  Software serial MSE 2202 IR tester

 The circuit:

* RX is digital pin 7 (connect to TX of other device)
* TX is digital pin 11 (connect to RX of other device)

*/
#include<SoftwareSerial.h>
SoftwareSerial mySerial(7,7);

void setup(){
 Serial.begin(9600);

 while(!Serial){
  ;
 }
 Serial.println("MSE 2202 IR tester");
 mySerial.begin(2400);
 //mySerial.println("Hello, world?");
}

void loop() {
  // put your main code here, to run repeatedly:
if(digitalRead(1))
  {
  if(mySerial.read()=='A'||mySerial.read()=='E')
  {
    //Arm and slide codes go here
    Serial.println("Hello");
  }
  }
  if(!digitalRead(1))
  {
    if(mySerial.read()=='I'||mySerial.read()=='O')
  {
    //Arm and slide codes go here
    Serial.println("Hello");
  }
  }
  

}
