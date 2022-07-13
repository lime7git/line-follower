#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "TRSensors.h"
#include <Wire.h>

#define MOTOR_LEFT_PWM    6           //Motor Left PWM (ENA)
#define MOTOR_LEFT_IN2    A0          //Motor Left forward (IN2).
#define MOTOR_LEFT_IN1    A1          //Motor Left backward (IN1)

#define MOTOR_RIGHT_PWM   5           //MOtor Right PWM (ENB)
#define MOTOR_RIGHT_IN1   A2          //Motor Right forward (IN3)
#define MOTOR_RIGHT_IN2   A3          //Motor Right backward (IN4)

#define OLED_RESET 9   
#define OLED_SA0   8

#define NUMBER_OF_SENSORS 5

#define TIME_INTERVAL_MS  1

// PCF8547 gpio expander
void PCF8574Write(byte data);
byte PCF8574Read();
bool ButtonRead();

bool button;

// robot velocity
float forward;
float direction;

float maximumSpeed;

void RobotUpdateSpeed(float forward, float direction);

// pid
float kp;
float ki;
float kd;

float error;
float previousError;
float totalError;
float antiWindupLimit;
float out;

void PidCalculate(void);

// ir sensors
TRSensors sensors = TRSensors();
unsigned int sensorsValues[NUMBER_OF_SENSORS];
unsigned int linePosition;

// check for speed tags
bool tagDetected1;
bool tagDetected2;
bool speedChanged;

void TagSpeedChange(void);

// OLED display
Adafruit_SSD1306 oled(OLED_RESET, OLED_SA0);
void OledInit(void);
void OledCountSeconds(unsigned int seconds);
void OledDebug(void);
void OledPrintSpeed(void);

void setup() 
{
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  pinMode(MOTOR_LEFT_PWM,   OUTPUT);                     
  pinMode(MOTOR_LEFT_IN2,   OUTPUT);      
  pinMode(MOTOR_LEFT_IN1,   OUTPUT);
  pinMode(MOTOR_RIGHT_PWM,  OUTPUT);       
  pinMode(MOTOR_RIGHT_IN1,  OUTPUT);     
  pinMode(MOTOR_RIGHT_IN2,  OUTPUT);  

  forward   = 0;
  direction = 0;

  // PD
  kp = 0.02;
  ki = 0.0;
  kd = 0.001;

  error           = 0.0;
  previousError   = 0.0;
  totalError      = 0.0;
  antiWindupLimit = 0.0;
  antiWindupLimit = 7500;

  maximumSpeed = 30;

  tagDetected1 = false;
  tagDetected2 = false;
  speedChanged = true;

  OledInit();
  
  button = false;
  while(!button)
  {
    button = ButtonRead();
    delay(25);
  }

  OledCountSeconds(3);

  RobotUpdateSpeed(0, 30);
    for (int i = 0; i < 250; i++)  
    {
      sensors.calibrate();
      delay(15);
    }
  RobotUpdateSpeed(0, 0);

  button = false;
  while(!button)
  {
    button = ButtonRead();
    
    linePosition = sensors.readLine(sensorsValues)/200;
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setCursor(0,0);
    oled.println("Calibration completed");
    oled.setCursor(25,25);
    oled.println(linePosition*200);
    oled.setCursor(0,55);
    for (int i = 0; i < 21; i++)
    {
      oled.print('_');
    }
    oled.setCursor(linePosition*6,55);
    oled.print("**");
    oled.display();
  }
  
  oled.clearDisplay();
  oled.display();

  OledCountSeconds(3);
  OledPrintSpeed();
}

void loop() 
{
  linePosition = sensors.readLine(sensorsValues);

  TagSpeedChange();
  
  previousError = error;
  error = (float)linePosition - 2000.0;
  totalError += error;

  if(totalError > antiWindupLimit)
  {
    totalError = antiWindupLimit;
  }
  else if(totalError < -antiWindupLimit)
  {
    totalError = -antiWindupLimit;
  }
  
  out = (kp * error) + (ki * totalError / (TIME_INTERVAL_MS / 1000.0)) + (kd * (error - previousError) / (TIME_INTERVAL_MS / 1000.0));
  

  if(out > maximumSpeed) 
  {
    out = maximumSpeed;
  }
  else if(out < -maximumSpeed)
  {
    out = -maximumSpeed;
  }

  forward = maximumSpeed;
  direction = out;
  
  RobotUpdateSpeed(forward, direction);

 // OledDebug();
    OledPrintSpeed();
  
  delay(TIME_INTERVAL_MS);
}

void RobotUpdateSpeed(float forward, float direction)
{
  float motorLeftPWM  = forward + direction;
  float motorRightPWM = forward - direction;

  if(motorLeftPWM > 0.0)
  {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }

  if(motorRightPWM > 0.0)
  {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }

  if(motorLeftPWM > 255.0) motorLeftPWM = 255.0;
  else if(motorLeftPWM < -255.0) motorLeftPWM = -255.0;

  if(motorRightPWM > 255.0) motorLeftPWM = 255.0;
  else if(motorRightPWM < -255.0) motorLeftPWM = -255.0;

  analogWrite(MOTOR_LEFT_PWM,  (int)abs(motorLeftPWM));
  analogWrite(MOTOR_RIGHT_PWM, (int)abs(motorRightPWM));
}

void PCF8574Write(byte data)
{
  Wire.beginTransmission(0x20);
  Wire.write(data);
  Wire.endTransmission(); 
}

byte PCF8574Read()
{
  int data = -1;
  Wire.requestFrom(0x20, 1);
  if(Wire.available()) {
    data = Wire.read();
  }
  return data;
}

bool ButtonRead()
{
  byte value = 0;
  bool pressed = false;

  PCF8574Write(0x1F | PCF8574Read());
  value = PCF8574Read() | 0xE0;

  if(value == 0xEF) pressed = true;

  return pressed;
}

void OledInit(void)
{
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(1);
  oled.setCursor(25,0);
  oled.print("press center");
  oled.setCursor(25,8);
  oled.println("to perform");
  oled.setCursor(25,16);
  oled.println("calibration");
  oled.display();
}

void OledCountSeconds(unsigned int seconds)
{
  for(int i = seconds; i > 0; i--)
  {
    oled.clearDisplay();
    oled.setTextSize(3);
    oled.setCursor(50,25);
    oled.println(i);
    oled.display();
    delay(1000);
  }
  oled.clearDisplay();
  oled.display();
}

void TagSpeedChange(void)
{
  bool robotOnTag = false;

  if (sensorsValues[0] > 900 && sensorsValues[1] > 900 && sensorsValues[2] > 900 
     && sensorsValues[3] > 900 && sensorsValues[4] > 900)    // 5 sensor wide tag
  {
    robotOnTag = true;
    tagDetected2 = true;
  }
  else if(sensorsValues[1] > 900 && sensorsValues[2] > 900 && sensorsValues[3] > 900) // 3 sensor wide tag
  {
    robotOnTag = true;
    tagDetected1 = true;
  }

  if(!robotOnTag)
  {
    if(tagDetected1)
    {
      if(maximumSpeed == 30) maximumSpeed = 60;
        else maximumSpeed = 30;

      speedChanged = true;
      tagDetected1 = false;
    }

    if(tagDetected2)
    {
       maximumSpeed = 80;

      speedChanged = true;
      tagDetected2 = false;
    }
  }
}

void OledDebug(void)
{
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(10,0);
  oled.print("line pos = ");
  oled.print(linePosition);
  oled.setCursor(10,8);
  oled.print("out = ");
  oled.print(out);
  oled.setCursor(10,16);
  oled.print("fwd / dir = ");
  oled.print(forward);
  oled.print("/");
  oled.print(direction);
  oled.setCursor(10,24);
  oled.print("speed = ");
  oled.print(maximumSpeed);
  oled.display();
}
void OledPrintSpeed(void)
{
  if(speedChanged)
  {
    speedChanged = false;
    
    oled.clearDisplay();
    oled.setTextSize(1);
    int i = 0;
    if(maximumSpeed == 30) i = 6;
    else if(maximumSpeed == 60) i = 12;
    else if(maximumSpeed == 80) i = 18;
    oled.setCursor(10,0);
    for(i; i > 1; i--) oled.print((char)219);
    oled.setCursor(30,25);
    oled.setTextSize(3);
    oled.print((int)maximumSpeed);
    oled.display();
  }
}
