#include <Wire.h> // Include the I2C library (required)
#include "QGPMaker_IICSensorbar.h" 
#include "QGPMaker_MotorShield.h"
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield(); 
QGPMaker_DCMotor *Motor1 = AFMS.getMotor(1);
QGPMaker_DCMotor *Motor2 = AFMS.getMotor(2);
QGPMaker_DCMotor *Motor3 = AFMS.getMotor(3);
QGPMaker_DCMotor *Motor4 = AFMS.getMotor(4);
// IICSensorBar I2C address :
const byte IIC_ADDRESS = 0x3F;  
SensorBar io; 

// Pin definition:
int sensor0,sensor1,sensor2,sensor3,sensor4,preerror;
double error=0,P,I,D,previous_error=0,Kp=10,Kd=0;
double Ki=0.5,PID_value=0;
void read_sensor_values();
void calc_pid();
void motor_write(int left_speed,int right_speed);
void motor_control();
void setup() 
{

  Serial.begin(9600);
  // Call io.begin(<address>) to initialize. If it
  // successfully communicates, it'll return 1.
  if (!io.begin(IIC_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }

  // use io.pinMode(<pin>, <mode>) to set input pins as either 
  // INPUT or INPUT_PULLUP
  io.pinMode(4, INPUT);
  io.pinMode(3, INPUT);
  io.pinMode(2, INPUT);
  io.pinMode(1, INPUT);
  io.pinMode(0, INPUT);
  AFMS.begin(); 
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);
  Motor1->setSpeed(150);
  Motor2->setSpeed(150);
  Motor3->setSpeed(150);
  Motor4->setSpeed(150);
  delay(10);
}

void loop() 
{
    sensor0 = io.digitalRead(4);
    sensor1 = io.digitalRead(3);
    sensor2 = io.digitalRead(2);
    sensor3 = io.digitalRead(1);
    sensor4 = io.digitalRead(0);
    Serial.print("sensor0:");
    Serial.println(sensor0);
    Serial.print("sensor1:");
    Serial.println(sensor1);
    Serial.print("sensor2:");
    Serial.println(sensor2);
    Serial.print("sensor3:");
    Serial.println(sensor3);
    Serial.print("sensor4:");
    Serial.println(sensor4);
    read_sensor_values();
    calcu();

    









}
void read_sensor_values(){  
  int num=0;
  error=0;
  if(sensor4==0)  
    {
      error+=2;
      ++num;
    }
  if(sensor3==0)  
    {
      error+=1;
      ++num;
    }
  if(sensor2==0)  
    {
      ++num;
    }
  if(sensor1==0)  
    {
      error=error-1;
      ++num;
    }
   if(sensor0==0) 
    {
      error=error-2;
      ++num;
    }
    if(num!=0)
    {error=error/num;
    preerror=error;}
    else{
      error=preerror;
    }
    Serial.print("error:");
    Serial.println(error);
}
void calcu()
{ int P=115;
double left_speed,right_speed;
  if(error<0)  
  {
    right_speed=85;
    left_speed=-error*P;
  }
  if(error>0)  
  {
    left_speed=85;
    right_speed=error*P;
  }
  if(error==0){
    left_speed=right_speed=150;//straight
  }
  motor_write(left_speed,right_speed);
}
void motor_write(int left_speed,int right_speed){
  
  Serial.print("left speed:");
  Serial.println(left_speed);
  Serial.print("right speed:");
  Serial.println(right_speed);
  if (error<0)
  {
   Motor1->setSpeed(right_speed);
  Motor2->setSpeed(left_speed);
  Motor3->setSpeed(right_speed);
  Motor4->setSpeed(left_speed);
   Motor1->run(BACKWARD);
   Motor2->run(FORWARD);
   Motor3->run(BACKWARD);
   Motor4->run(FORWARD);
  }
  if (error>0)
  {
   Motor1->setSpeed(right_speed);
  Motor2->setSpeed(left_speed);
  Motor3->setSpeed(right_speed);
  Motor4->setSpeed(left_speed);
   Motor2->run(BACKWARD);
   Motor1->run(FORWARD);
   Motor4->run(BACKWARD);
   Motor3->run(FORWARD);
  }
  if (error==0)
  {
   Motor1->setSpeed(right_speed);
  Motor2->setSpeed(left_speed);
  Motor3->setSpeed(right_speed);
  Motor4->setSpeed(left_speed);
   Motor1->run(FORWARD);
   Motor2->run(FORWARD);
   Motor3->run(FORWARD);
   Motor4->run(FORWARD);
  }

}
