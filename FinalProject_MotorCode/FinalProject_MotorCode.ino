#include <Servo.h>
#include <EEPROM.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_SwingServo;
Servo servo_LiftServo;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC1
//#define DEBUG_ULTRASONIC2
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Ultrasonic_Ping_1 = 2;   //input plug
const int ci_Ultrasonic_Data_1 = 3;   //output plug

const int ci_Ultrasonic_Ping_2 = A0;  //input plug////////////////////////////
const int ci_Ultrasonic_Data_2 = A1;   //output plug///////////////////////////

const int irsensor = 13;

const int ci_Charlieplex_LED3 = 6;
const int ci_Charlieplex_LED4 = 7;
const int ci_Mode_Button = 7;
const int ci_Right_Motor; ///////////////////////port 8 is bad
const int ci_Left_Motor = 9;
const int ci_Swing_servo = 10;
const int ci_Lift_servo = 11;
const int ci_Motor_Enable_Switch = 12;
const int ci_Light_Sensor = A3;
//const int ci_I2C_SDA = A4;         // I2C data = white
//const int ci_I2C_SCL = A5;         // I2C clock = yellow

const int ci_Hall_Sensor; ////////////////////////////////////////////////////////////////////!!!!!!!!!!!!!!!!!!!!!!!!

// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 4;

//constants
const int ci_Left_Motor_Offset_Address_L = 12;
const int ci_Left_Motor_Offset_Address_H = 13;
const int ci_Right_Motor_Offset_Address_L = 14;
const int ci_Right_Motor_Offset_Address_H = 15;

const int ci_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Right_Motor_Stop = 1500;
const int ci_Swing_Servo_back = 180;         //
const int ci_Swing_Servo_foreward = 50;       //  "
const int ci_Lift_Servo_up = 70;     //  "
const int ci_Lift_Servo_down = 80;      //  "
const int ci_Display_Time = 500;
const int ci_Motor_Calibration_Cycles = 3;
const int ci_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time_1;
unsigned long ul_Echo_Time_2;
unsigned int ui_Motors_Speed = 1700;        // Default run speed
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

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[4] = {
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
  Serial.begin(9600);


  CharliePlexM::setBtn(ci_Charlieplex_LED3, ci_Charlieplex_LED4,
                       ci_Charlieplex_LED3, ci_Charlieplex_LED4, ci_Mode_Button);

  pinMode(irsensor, INPUT);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping_1, OUTPUT);
  pinMode(ci_Ultrasonic_Data_1, INPUT);

  // set up ultrasonic
  pinMode(ci_Ultrasonic_Ping_2, OUTPUT);
  pinMode(ci_Ultrasonic_Data_2, INPUT);

  // set up drive motors
  pinMode(ci_Right_Motor, OUTPUT);
  servo_RightMotor.attach(ci_Right_Motor);
  pinMode(ci_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(ci_Left_Motor);

  // set up arm motors
  pinMode(ci_Swing_servo, OUTPUT);
  servo_SwingServo.attach(ci_Swing_servo);
  pinMode(ci_Lift_servo, OUTPUT);
  servo_LiftServo.attach(ci_Lift_servo);

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
  if ((millis() - ul_3_Second_timer) > 3000)
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
        //Ping();
        servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        servo_SwingServo.write(ci_Swing_Servo_back);
        servo_LiftServo.write(ci_Lift_Servo_up);
        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        ui_Mode_Indicator_Index = 0;
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
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
          ui_Left_Motor_Speed = 1600; //constrain(ui_Motors_Speed + ui_Left_Motor_Offset, 1550, 1600);
          ui_Right_Motor_Speed = 1900; //constrain(ui_Motors_Speed + ui_Right_Motor_Offset, 1800, 1900);



          /*while (1) { ///////////////////////////////////// SET UP FOR HIGH LOW

            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

            Ping_1();
            Ping_2();

            if ((ul_Echo_Time_1 / 58) >= 6 && (ul_Echo_Time_1 / 58) <= 9) { /////////////////////////TEST FOR VALUE
              // if the robot approches the wall, turn right
              servo_LeftMotor.writeMicroseconds(1800);
              servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
              delay(1000); //////////////////////////////////////////////////////////TEST FOR VALUE
            }
            else if ((ul_Echo_Time_2 / 58) >= 5) {
              //if too far away from wall, makes a small adjustment /////////////////////////////// TEST FOR VALUE
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds((ui_Right_Motor_Speed) + 300);
              delay(70);
              
            }
            else if ((ul_Echo_Time_2 / 58) <= 3) {
              //if too far away from wall, makes a small adjustment /////////////////////////////// TEST FOR VALUE
              servo_LeftMotor.writeMicroseconds((ui_Left_Motor_Speed) + 300);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              delay(70);
              
            }
          }

          int turn = 1;

          while (1) { ///////////////////////////////////////////////////////////none of the sensors see a pyramid
            servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
            servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);

            if ((ul_Echo_Time_1 / 58) >= 6 && (ul_Echo_Time_1 / 58) <= 9 && turn == 1) { /////////////////////////TEST FOR VALUE
              // if the robot approches the wall, turn around
              turn = 0;
              servo_LeftMotor.writeMicroseconds(ui_Left_Motor_Speed);
              servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
              delay(2000); //////////////////////////////////////////////////////////TEST FOR VALUE - MAYBE NOT USE DELAY!!!!!!!!!!!!!!!!!!!!
            }
            else if ((ul_Echo_Time_1 / 58) >= 6 && (ul_Echo_Time_1 / 58) <= 9 && turn == 0) { /////////////////////////TEST FOR VALUE
              // if the robot approches the wall, turn around
              turn = 1;
              servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
              servo_RightMotor.writeMicroseconds(ui_Right_Motor_Speed);
              delay(2000); //////////////////////////////////////////////////////////TEST FOR VALUE
            }
          }

          servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
          servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);


        }*/

        if (digitalRead(irsensor) == HIGH){
          Serial.println("Success");
        }





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
          if (!bt_Cal_Initialized)
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

  if ((millis() - ul_Display_Time) > ci_Display_Time)
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

// measure distance to target using ultrasonic sensor
void Ping_1()
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
void Ping_2()
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

