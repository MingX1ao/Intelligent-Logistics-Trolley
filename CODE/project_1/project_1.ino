#include <Wire.h>
#include "QGPMaker_MotorShield.h"

// Create the motor shield object with the default I2C address
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield(); 
QGPMaker_DCMotor *Motor1 = AFMS.getMotor(1);
QGPMaker_DCMotor *Motor2 = AFMS.getMotor(2);
QGPMaker_DCMotor *Motor3 = AFMS.getMotor(3);
QGPMaker_DCMotor *Motor4 = AFMS.getMotor(4);
void setup() {
  // put your setup code here, to run once:
  AFMS.begin(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t i;
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);//change from the line
  
    Motor1->setSpeed(255);
    Motor2->setSpeed(250);
    Motor3->setSpeed(255);
    Motor4->setSpeed(250);  
    delay(10);
  
}
