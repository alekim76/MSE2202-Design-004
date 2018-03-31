
int hallSensorPin=A1;
int state;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  pinMode(hallSensorPin,INPUT);
}

void loop() {
  
  // put your main code here, to run repeatedly:
  state=analogRead(hallSensorPin);
  Serial.println(state);

  if(state>400){
    //IR sensor code
    Serial.println("Found the cube");
  }
  else{
    //ultrasonic code to find the cube
    Serial.println("keep looking");
  }
}
