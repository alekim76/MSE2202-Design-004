#include <Servo.h> // library for servos
#include <EEPROM.h> // library for data storage
#include <I2CEncoder.h> // library for encoders
#include <uSTimer2.h> // library for delay functions
#include <CharliePlexM.h> // library for charliplex leds
#include <Wire.h> // library for I2C communication

Servo servo_RightMotor; // set servos for both drive motors, the swing mechanism, the lift mechanism, and the tipping mechanism
Servo servo_LeftMotor;
Servo servo_SwingServo;
Servo servo_LiftServo;
Servo servo_TipServo;

I2CEncoder encoder_RightMotor; // set encoders for both drive motors
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging outputs

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC1
//#define DEBUG_ULTRASONIC2
//#define DEBUG_ULTRASONIC3
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true; //set motors enabled

//port pin constants

//set ultrasonic sensors
const int ci_Ultrasonic_Ping_1 = A2; //input plug for front and center ultrasonic
const int ci_Ultrasonic_Data_1 = A3; //output plug
const int ci_Ultrasonic_Ping_2 = A0; //input plug for front left side ultrasonic
const int ci_Ultrasonic_Data_2 = A1; //output plug
const int ci_Ultrasonic_Ping_3 = 3;  //input plug for back left side ultrasonic
const int ci_Ultrasonic_Data_3 = 4;  //output plug

const int irsensor = 13; // set ir sensor to pin 13. This pin is shared by both boards allowing communication without the TX and RX ports (which may have been faulty)

const int ci_Charlieplex_LED3 = 6; // only 2 leds are required for charliplexing
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7; // sets mode button to be pushed to execute different cases
const int ci_Right_Motor = 9;
const int ci_Left_Motor = 8;
const int ci_Swing_servo = 10;
const int ci_Lift_servo = 11;
const int ci_Tip_servo = 2;

const int ci_Motor_Enable_Switch = 12;

const int ci_Hall_Sensor = 5; // sets hall sensor, which is diretcly connected to board 1 for easy communication

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;

//constants for encoders and motor calibration
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Lift_Servo_up = 0;     // sets positions for the lift mechanism
const int ci_Lift_Servo_down = 160;
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time_1; // variables for ultrasonic sensors
unsigned long ul_Echo_Time_2;
unsigned long ul_Echo_Time_3;
unsigned int ui_Motors_Speed = 1750;  // Default run speed
unsigned int ui_Left_Motor_Speed;
unsigned int ui_Right_Motor_Speed;
long l_Left_Motor_Position;
long l_Right_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;
unsigned long ui_Left_Motor_Offset;
unsigned long ui_Right_Motor_Offset;

unsigned int ui_Cal_Count;
unsigned int ui_Cal_Cycle;

unsigned long previousMillis = 0; //sets values used for delay functions (without using delay())
unsigned long currentMillis = 0;

unsigned int pfound = 0; // value for detemining if pyrmaid is found

unsigned int turn = 0; // value to hold last turn direction

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[4] = { // cases to be triggere by number of button presses
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;


void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600); // begin at standard baud rate


  CharliePlexM::setBtn(ci_Charlieplex_LED3, ci_Charlieplex_LED4,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  pinMode(irsensor, INPUT); // irsensor is reciving value from board 1

  pinMode(ci_Hall_Sensor, INPUT); // hall sensor is recieving value form board 1

  // set up ultrasonic sensors
  pinMode(ci_Ultrasonic_Ping_1, OUTPUT);
  pinMode(ci_Ultrasonic_Data_1, INPUT);
  pinMode(ci_Ultrasonic_Ping_2, OUTPUT);
  pinMode(ci_Ultrasonic_Data_2, INPUT);
  pinMode(ci_Ultrasonic_Ping_3, OUTPUT);
  pinMode(ci_Ultrasonic_Data_3, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up pyramid mechansims
  pinMode(ci_Swing_servo, OUTPUT);
  servo_SwingServo.attach(ci_Swing_servo);
  pinMode(ci_Lift_servo, OUTPUT);
  servo_LiftServo.attach(ci_Lift_servo);
  pinMode(ci_Tip_servo, OUTPUT);
  servo_TipServo.attach(ci_Tip_servo);

  // set up motor enable switch
  pinMode(ci_Motor_Enable_Switch, INPUT);

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
  if ((millis() - ul_3_Second_timer) > 3000) // sets 3 second delay after a case is set by button pressing
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (CharliePlexM::ui_Btn)
  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
      ui_Cal_Cycle = 0;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button 2 times to enter. Calibrate motor speeds to drive straight.
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        servo_LiftServo.write(ci_Lift_Servo_up);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        break;
      }

    case 1:
      {
        if (bt_3_S_Time_Up) //Robot Run after 3 seconds
        {

#ifdef DEBUG_ENCODERS
          l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
          l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

          Serial.print("Encoders L: ");
          Serial.print(l_Left_Motor_Position);
          Serial.print(", R: ");
          Serial.println(l_Right_Motor_Position);
#endif

          // set motor speeds
          ui_Left_Motor_Speed = constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1650, 1800); // constrains motor speeds based on motor calibration values
          ui_Right_Motor_Speed = constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1650, 1800);

          servo_LiftServo.write(120); // sets lift mechism to appropriate position so front center ultrasonic can detect walls
          delay(1000);

          while (digitalRead(ci_Hall_Sensor) == LOW) { // while the cube has not been found

            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed); // drive foreward
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

            Ping_1(); //get distances from all three ultrasonic sensors
            Ping_2();
            Ping_3();


            if ((ul_Echo_Time_1 / 58) >= 1 && (ul_Echo_Time_1 / 58) <= 3) { // values (tested for) determine how close the robot gets to the wall before turning
              // if the robot approches the wall, turn right (all values tested for perfect right turn)
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + 100);
              servo_RightMotor.writeMicroseconds(1200);
              delay(500);
              servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
              servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
              delay(1000);
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed + 130);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              delay(90);
            }
            else if ((ul_Echo_Time_2 / 58) >= 2) { //if front side ultrasonic is too far away from wall, makes a small adjustment left
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds((ui_Right_Motor_Speed) + 100); //values tested for
              delay(50);
            }
            else if ((ul_Echo_Time_3 / 58) >= 2) { //if back side ultrasonic is too far away from wall, makes a small adjustment right
              servo_LeftMotor.writeMicroseconds((ui_Left_Motor_Speed) + 100);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed); // values tested for
              delay(50);
            }
          }

          while (pfound == 0) { //none of the ir sensors see a pyramid
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 80); // drive foreward slowly
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 80);

            Ping_1(); // get distance from front ultrasonic

            if (digitalRead(irsensor) == HIGH) { // if the pyramid has been found, exit the loop
              pfound = 1;
            }
            else if ((ul_Echo_Time_1 / 58) >= 2 && (ul_Echo_Time_1 / 58) <= 4 && turn == 0) { //values tested for proper turn distance
              // if the robot approches the wall and it turned left last, turn right around
              scanturn(turn); // turns around based on last turn direction
              turn = 1;
            }
            else if ((ul_Echo_Time_1 / 58) >= 2 && (ul_Echo_Time_1 / 58) <= 4 && turn == 1) { //values tested for proper turn distance
              // if the robot approches the wall and it turned right last, turn left around
              scanturn(turn);  // turns around based on last turn direction
              turn = 0;
            }
          }

          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); // once the pyramid has been found by one of the ir sensors, stop the robot
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
          delay(1000);
          servo_LiftServo.write(ci_Lift_Servo_up); // lift front mechanism out of the way so middle ir sensor can detect pyramid
          delay(2000);

          look(); // this function has the robto scan by rotating until it sees the pyramid in its middle ir sensor

          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop); // stops after pyramid has been centered
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);

          servo_LiftServo.write(170); // lowers lift mechanism
          delay(2000);
          Ping_1(); //gets ditance to pyramid using front ultrasonic sensor
          int temp = (ul_Echo_Time_1 / 58) * 50; //uses distance to pyramid to calculate drive time
          delay(500);
          servo_LiftServo.write(ci_Lift_Servo_up); // moves mechanism out of the way
          delay(2000);
          servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed); // drives foreward a calculated amount of time so pyramid is in correct position
          servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
          delay(temp);



          servo_LiftServo.write(ci_Lift_Servo_down); // sets lift mechanism behind pyramid
          delay(2000);
          servo_TipServo.writeMicroseconds(2000); // tips the pyramid against lift mechanism
          delay(2000);
          servo_TipServo.writeMicroseconds(1500);
          delay(2000);
          servo_SwingServo.writeMicroseconds(1700); // swing mechanism places cube underneath pyramid
          delay(500);
          servo_SwingServo.writeMicroseconds(1500);
          delay(500);
          servo_TipServo.writeMicroseconds(1000); // tip mechanism retracts, lowering pyramid back to level position
          delay(1500);
          servo_TipServo.writeMicroseconds(1500);
          delay(2000);
          servo_LiftServo.write(ci_Lift_Servo_up); // lift mechanism retracts out of way of pyramid
          delay(3000);
          servo_SwingServo.writeMicroseconds(1200); // swing mechanism retracts out from under pyramid, leaving the cube inside
          delay(500);
          servo_LeftMotor.writeMicroseconds(1300); // robot backs away from pyramid and cube
          servo_RightMotor.writeMicroseconds(1300);
          delay(5000);
          servo_LeftMotor.detach(); // stops all the motors and servo mechanisms
          servo_RightMotor.detach();
          servo_SwingServo.detach();
          servo_LiftServo.detach();
          servo_TipServo.detach();
          break; // ends robot function

#ifdef DEBUG_MOTORS
          Serial.print("Motors enabled: ");
          Serial.print(bt_Motors_Enabled);
          Serial.print(", Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(", Left = ");
          Serial.print(ui_Left_Motor_Speed);
          Serial.print(", Right = ");
          Serial.println(ui_Right_Motor_Speed);
#endif
          ui_Mode_Indicator_Index = 1;
        }
        break;
      }

    case 2:    //Calibrate motor straightness after 3 seconds.
      {
        if (bt_3_S_Time_Up)
        {
          if (!bt_Cal_Initialized) // calibrates motors by running them for 5 seconds, determining the appropriate offsets
          {
            bt_Cal_Initialized = true;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            ul_Calibration_Time = millis();
            servo_LeftMotor.writeMicroseconds(ui_Motors_Speed);
            servo_RightMotor.writeMicroseconds(ui_Motors_Speed);
          }
          else if ((millis() - ul_Calibration_Time) > ci_Motor_Calibration_Time)
          {
            servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
            servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
            l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
            l_Right_Motor_Position = encoder_RightMotor.getRawPosition();
            if (l_Left_Motor_Position > l_Right_Motor_Position)
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = 0;
              ui_Left_Motor_Offset = (l_Left_Motor_Position - l_Right_Motor_Position) / 4;
            }
            else
            {
              // May have to update this if different calibration time is used
              ui_Right_Motor_Offset = (l_Right_Motor_Position - l_Left_Motor_Position) / 4;
              ui_Left_Motor_Offset = 0;
            }

#ifdef DEBUG_MOTOR_CALIBRATION
            Serial.print("Motor Offsets: Left = ");
            Serial.print(ui_Left_Motor_Offset);
            Serial.print(", Right = ");
            Serial.println(ui_Right_Motor_Offset);
#endif
            EEPROM.write(ci_Right_Motor_Offset_Address_L, lowByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Right_Motor_Offset_Address_H, highByte(ui_Right_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_L, lowByte(ui_Left_Motor_Offset));
            EEPROM.write(ci_Left_Motor_Offset_Address_H, highByte(ui_Left_Motor_Offset));

            ui_Robot_State_Index = 0;    // go back to Mode 0
          }
#ifdef DEBUG_MOTOR_CALIBRATION
          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
          ui_Mode_Indicator_Index = 2;
        }
        break;
      }
  }

  if ((millis() - ul_Display_Time) > ci_Display_Time) // allwos mode to be chosen and displayed using the charlieplex leds
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Mode_Indicator[ui_Mode_Indicator_Index], DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;
    CharliePlexM::Write(ci_Heartbeat_LED, bt_Heartbeat);
    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led
  CharliePlexM::Write(ci_Indicator_LED, !(ui_Mode_Indicator[ui_Mode_Indicator_Index] &
                                          (iArray[iArrayIndex])));
  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

// measure distance to target using ultrasonic sensors
void Ping_1() // ping front center ultrasonic
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(ci_Ultrasonic_Ping_1, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_1, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW
  ul_Echo_Time_1 = pulseIn(ci_Ultrasonic_Data_1, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC1
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_1, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_1 / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_1 / 58); //divide time by 58 to get distance in cm
#endif
}
void Ping_2() // ping front left side ultrasonic
{
  digitalWrite(ci_Ultrasonic_Ping_2, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_2, LOW);
  ul_Echo_Time_2 = pulseIn(ci_Ultrasonic_Data_2, HIGH, 10000);

#ifdef DEBUG_ULTRASONIC2
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_2, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_2 / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_2 / 58); //divide time by 58 to get distance in cm
#endif
}
void Ping_3() // ping back left side ultrasonic
{
  digitalWrite(ci_Ultrasonic_Ping_3, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(ci_Ultrasonic_Ping_3, LOW);
  ul_Echo_Time_3 = pulseIn(ci_Ultrasonic_Data_3, HIGH, 10000);

#ifdef DEBUG_ULTRASONIC3
  Serial.print("Time (microseconds): ");
  Serial.print(ul_Echo_Time_3, DEC);
  Serial.print(", Inches: ");
  Serial.print(ul_Echo_Time_3 / 148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(ul_Echo_Time_3 / 58); //divide time by 58 to get distance in cm
#endif
}

void look()  // this function has the robto scan by rotating until it sees the pyramid in its middle ir sensor
{
  while (digitalRead(irsensor) == LOW) {
    servo_LeftMotor.writeMicroseconds(1650);
    servo_RightMotor.writeMicroseconds(1350);
  }
  servo_LeftMotor.writeMicroseconds(1500);
  servo_RightMotor.writeMicroseconds(1700);
  delay(80);
  servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
  servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
}
void scanturn(int turn) { // this function has the robot turn around based on the last turn direction
  currentMillis = millis(); // millis allows a function to be executed for a set time without the delay() function
  previousMillis = currentMillis;
  do {
    if (digitalRead(irsensor) == HIGH || pfound == 1) { // if the pyramid is found, break from the function
      pfound = 1;
      break;
    }
    currentMillis = millis();
    if (turn == 0) {
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 100);
      servo_RightMotor.writeMicroseconds(1500 - (ui_Right_Motor_Speed - 100 - 1500));
    }
    else if (turn == 1) {
      servo_LeftMotor.writeMicroseconds(1500 - (ui_Left_Motor_Speed - 100 - 1500));
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 100);
    }
  } while (currentMillis - previousMillis < 1800);
  previousMillis = currentMillis;
  do {
    if (digitalRead(irsensor) == HIGH || pfound == 1) { // if the pyramid is found, break from the function
      pfound = 1;
      break;
    }
    currentMillis = millis();
    servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 100);
    servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 100);
  } while (currentMillis - previousMillis < 600);
  previousMillis = currentMillis;
  do {
    if (digitalRead(irsensor) == HIGH || pfound == 1) { // if the pyramid is found, break from the function
      pfound = 1;
      break;
    }
    currentMillis = millis();
    if (turn == 0) {
      servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed - 100);
      servo_RightMotor.writeMicroseconds(1500 - (ui_Right_Motor_Speed - 100 - 1500));
    }
    else if (turn == 1) {
      servo_LeftMotor.writeMicroseconds(1500 - (ui_Left_Motor_Speed - 100 - 1500));
      servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed - 100 );
    }
  } while (currentMillis - previousMillis < 1800);
}



