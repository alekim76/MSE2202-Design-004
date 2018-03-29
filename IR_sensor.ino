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

void loop(){
  if(mySerial.available())
  {
    Serial.write(mySerial.read());
  }
  if(digitalRead(3)
  {
  if(mySerial.read()=='A')
  {
    //Arm and slide codes go here
    Serial.println("Hello");
  }
  if(mySerial.read()=='E')
  {
    //Arm and slide codes go here
    Serial.println("Hello 2");
  }
  }
  if(!digitalRead(3))
  {
    if(mySerial.read()=='I')
  {
    //Arm and slide codes go here
    Serial.println("Hello");
  }
  if(mySerial.read()=='O')
  {
    //Arm and slide codes go here
    Serial.println("Hello 2");
  }
  }
  
 
 
}


