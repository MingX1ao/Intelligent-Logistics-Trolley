#include<Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_IICSensorbar.h"


const byte IIC_ADDRESS = 0x3F;  
SensorBar io;

QGPMaker_MotorShield AFMS=QGPMaker_MotorShield();
QGPMaker_Servo *base=AFMS.getServo(0);
QGPMaker_Servo *rArm=AFMS.getServo(1);
QGPMaker_Servo *fArm=AFMS.getServo(2);
QGPMaker_Servo *claw=AFMS.getServo(3);

QGPMaker_DCMotor *Motor1 = AFMS.getMotor(1);
QGPMaker_DCMotor *Motor2 = AFMS.getMotor(2);
QGPMaker_DCMotor *Motor3 = AFMS.getMotor(3);
QGPMaker_DCMotor *Motor4 = AFMS.getMotor(4);
int basePos=90;
int fArmPos=90;
int rArmPos=90;
int clawPos=90;
int sensor0,sensor1,sensor2,sensor3,sensor4,preerror;
double error=0;
void read_sensor_values();
void motor_write(int left_speed,int right_speed);
const int baseMax;
const int fArmMin;
const int fArmMax;
const int rArmMin;
const int rArmMax;
const int clawMin;
const int clawMax;


void Servostatus_begin(); //机械爪抬起状态
void Servostatus_beginout();
void Servostatus_downclaw(); //机械爪后退后下爪状态
void Servostatus_tightclaw(); //机械爪收爪状态
void Servostatus_looseclaw(); //机械爪释放松爪子状态

const int DelayTime=25;

void setup()
{
   Serial.begin(9600);
   AFMS.begin(50);

  
  if (!io.begin(IIC_ADDRESS))
  {
    Serial.println("Failed to communicate.");
    while (1) ;
  }
  io.pinMode(4, INPUT);
  io.pinMode(3, INPUT);
  io.pinMode(2, INPUT);
  io.pinMode(1, INPUT);
  io.pinMode(0, INPUT);
  Servostatus_begin();
  Motor1->run(FORWARD);
  Motor2->run(FORWARD);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);
  Motor1->setSpeed(49);
  Motor2->setSpeed(50);
  Motor3->setSpeed(51);
  Motor4->setSpeed(50);
  delay(10);
  Serial.println("start");


}



void reportStatus(int basePos,int fArmPos,int rArmPos,int clawPos){  //舵机状态信息
  Serial.println("");
  Serial.println("");
  Serial.println("+ Robot-Arm Status Report +");
  Serial.print("Claw Position: "); Serial.println(clawPos);
  Serial.print("Base Position: "); Serial.println(basePos);
  Serial.print("Rear  Arm Position:"); Serial.println(rArmPos);
  Serial.print("Front Arm Position:"); Serial.println(fArmPos);
  Serial.println("++++++++++++++++++++++++++");
  Serial.println("");
}


void Servostatus_begin()
{
  int basePos_begin=90;
  int rArmPos_begin=90;
  int fArmPos_begin=90;
  int clawPos_begin=90;

  int fromPos_base=base->readDegrees();
  int fromPos_fArm=fArm->readDegrees();
  int fromPos_rArm=rArm->readDegrees();
  int fromPos_claw=claw->readDegrees();

  
                  base->writeServo(basePos_begin);
                  delay(DelayTime);                
            
  
                  rArm->writeServo(90);
                  delay(DelayTime);                
                    
                  fArm->writeServo(90);
                  delay(DelayTime);                
                        
    
                  claw->writeServo(50);
                  delay(DelayTime);                

         delay(2000);

}



void Servostatus_downclaw()
{
  
int basePos_begin=130;
  int rArmPos_begin=30;
  int fArmPos_begin=180;
  int clawPos_begin=90;

  int fromPos_base=base->readDegrees();
  int fromPos_fArm=fArm->readDegrees();
  int fromPos_rArm=rArm->readDegrees();
  int fromPos_claw=claw->readDegrees();

  
  if(fromPos_rArm<rArmPos_begin)
            {
              for(int i=fromPos_rArm;i<=rArmPos_begin;++i)
              {
                 rArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_rArm;i>=rArmPos_begin;--i)
              {
                  rArm->writeServo(i);
                  delay(DelayTime);                
              }            
            } 
  if(fromPos_fArm<fArmPos_begin)
            {
              for(int i=fromPos_fArm;i<=fArmPos_begin;++i)
              {
                 fArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_fArm;i>=fArmPos_begin;--i)
              {
                  fArm->writeServo(i);
                  delay(DelayTime);                
              }            
            }
    if(fromPos_claw<clawPos_begin)
            {
              for(int i=fromPos_claw;i<=clawPos_begin;++i)
              {
                 claw->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_claw;i>=clawPos_begin;--i)
              {
                  claw->writeServo(i);
                  delay(DelayTime);                
              }            
            }
                if(fromPos_base<basePos_begin)
            {
              for(int i=fromPos_base;i<=basePos_begin;++i)
              {
                 base->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_base;i>=basePos_begin;--i)
              {
                  base->writeServo(i);
                  delay(DelayTime);                
              }            
            }  

}


void Servostatus_downclaw2()
{
  
int basePos_begin=130;
  int rArmPos_begin=30;
  int fArmPos_begin=180;
  int clawPos_begin=90;

  int fromPos_base=base->readDegrees();
  int fromPos_fArm=fArm->readDegrees();
  int fromPos_rArm=rArm->readDegrees();
  int fromPos_claw=claw->readDegrees();

  
  if(fromPos_rArm<rArmPos_begin)
            {
              for(int i=fromPos_rArm;i<=rArmPos_begin;++i)
              {
                 rArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_rArm;i>=rArmPos_begin;--i)
              {
                  rArm->writeServo(i);
                  delay(DelayTime);                
              }            
            } 
  if(fromPos_fArm<fArmPos_begin)
            {
              for(int i=fromPos_fArm;i<=fArmPos_begin;++i)
              {
                 fArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_fArm;i>=fArmPos_begin;--i)
              {
                  fArm->writeServo(i);
                  delay(DelayTime);                
              }            
            }
                if(fromPos_base<basePos_begin)
            {
              for(int i=fromPos_base;i<=basePos_begin;++i)
              {
                 base->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_base;i>=basePos_begin;--i)
              {
                  base->writeServo(i);
                  delay(DelayTime);                
              }            
            }  
       
       if(fromPos_claw<clawPos_begin)
            {
              for(int i=fromPos_claw;i<=clawPos_begin;++i)
              {
                 claw->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_claw;i>=clawPos_begin;--i)
              {
                  claw->writeServo(i);
                  delay(DelayTime);                
              }            
            }

}


void Servostatus_tightclaw()
{
  for(int i=90;i>=45;i--)
  {
    claw->writeServo(i);
    delay(20);
  }
}


void Servostatus_beginout()
{
  int basePos_begin=90;
  int rArmPos_begin=90;
  int fArmPos_begin=90;

  int fromPos_base=base->readDegrees();
  int fromPos_fArm=fArm->readDegrees();
  int fromPos_rArm=rArm->readDegrees();
  int fromPos_claw=claw->readDegrees();

  if(fromPos_base<basePos_begin)
            {
              for(int i=fromPos_base;i<=basePos_begin;++i)
              {
                 base->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_base;i>=basePos_begin;--i)
              {
                  base->writeServo(i);
                  delay(DelayTime);                
              }            
            }
  if(fromPos_rArm<rArmPos_begin)
            {
              for(int i=fromPos_rArm;i<=rArmPos_begin;++i)
              {
                 rArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_rArm;i>=rArmPos_begin;--i)
              {
                  rArm->writeServo(i);
                  delay(DelayTime);                
              }            
            } 
  if(fromPos_fArm<fArmPos_begin)
            {
              for(int i=fromPos_fArm;i<=fArmPos_begin;++i)
              {
                 fArm->writeServo(i);
                  delay(DelayTime);                
              }              
            }
            else{
               for(int i=fromPos_fArm;i>=fArmPos_begin;--i)
              {
                  fArm->writeServo(i);
                  delay(DelayTime);                
              }            
            }
  
}






bool flag1=false;


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
   if(sensor0==0)  //s4压黑线要左转 右快左慢
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
{ int P=80;
double left_speed,right_speed;
  if(error<0)  
  {
    right_speed=50;
    left_speed=-error*P;
  }
  if(error>0)  
  {
    left_speed=50;
    right_speed=error*P;
  }
  if(error==0){
    left_speed=right_speed=50;//straight
  }
  motor_write(left_speed,right_speed);
}
void motor_write(int left_speed,int right_speed){
  Motor1->setSpeed(left_speed);
  Motor2->setSpeed(right_speed);
  Motor4->setSpeed(right_speed);
  Motor3->setSpeed(left_speed);
  Serial.print("left speed:");
  Serial.println(left_speed);
  Serial.print("right speed:");
  Serial.println(right_speed);
}

  int cnt=0;
  int cnt2=0;
void loop() 
{
    //左移
    sensor4 = io.digitalRead(4);
     sensor3 = io.digitalRead(3);
     sensor2 = io.digitalRead(2);
     sensor1 = io.digitalRead(1);
     sensor0 = io.digitalRead(0);   
    while(!(sensor4==1&&sensor3==0)) 
   {
     sensor4 = io.digitalRead(4);
     sensor3 = io.digitalRead(3);
     sensor2 = io.digitalRead(2);
     sensor1 = io.digitalRead(1);
     sensor0 = io.digitalRead(0);     
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
   }
  
     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(1000);
     


     //前进
     Motor1->setSpeed(53);
     Motor2->setSpeed(47);
     Motor3->setSpeed(53);
     Motor4->setSpeed(47);
  
    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
  
  
    sensor0 = io.digitalRead(0);
    sensor1 = io.digitalRead(1);
    sensor2 = io.digitalRead(2);
    sensor3 = io.digitalRead(3);
    sensor4 = io.digitalRead(4);
    

     

  while(cnt<4)
  {   
    
     Motor1->setSpeed(52);
     Motor2->setSpeed(48);
     Motor3->setSpeed(52);
     Motor4->setSpeed(48);
    Motor1->run(FORWARD);
    Motor2->run(FORWARD);
    Motor3->run(FORWARD);
    Motor4->run(FORWARD);
    cnt++;
    

  }
     
     Motor1->setSpeed(52);
     Motor2->setSpeed(49);
     Motor3->setSpeed(53);
     Motor4->setSpeed(49);
     delay(3000);

     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     

     //夹起来
     delay(2000);
     Servostatus_downclaw();
    
    
     Motor1->run(FORWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(FORWARD);
     Motor1->setSpeed(50);
     Motor2->setSpeed(45);
     Motor3->setSpeed(50);
     Motor4->setSpeed(45);
     delay(880);
  
    
     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     
     delay(1000);
     Servostatus_tightclaw();
     delay(1000);
     Servostatus_beginout();
     delay(2000);
     



     //左移
     Motor1->setSpeed(48);
     Motor2->setSpeed(48);
     Motor3->setSpeed(53);
     Motor4->setSpeed(50);
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
     
     
    for(int i=0;i<4;i++)
    {
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
     delay(865);
    }
    
    

     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(2000);
//放
    Servostatus_downclaw2();


     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     
     delay(2000);
     Servostatus_beginout();
     delay(2000);
     
  //右移

     Motor1->setSpeed(48);
     Motor2->setSpeed(48);
     Motor3->setSpeed(52);
     Motor4->setSpeed(47.75);
    for(int i=0;i<4;i++)
    {
     Motor1->run(FORWARD);
     Motor2->run(BACKWARD);
     Motor3->run(BACKWARD);
     Motor4->run(FORWARD);
     delay(635);
    }
     Motor1->run(FORWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(FORWARD);
     delay(50);

     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(2000);  
     

     
//夹第二个
    Servostatus_downclaw();
    delay(2000);
    Servostatus_tightclaw();
    delay(2000);

    Servostatus_beginout();
     
     delay(2000);
   //左移  
     Motor1->setSpeed(47);
     Motor2->setSpeed(49);
     Motor3->setSpeed(53);
     Motor4->setSpeed(49);
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
     
     
    for(int i=0;i<4;i++)
    {
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
     delay(950);
    }
    
    

     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(2000);
    Servostatus_downclaw2();

     delay(2000);

//后退
     Motor1->setSpeed(49);
     Motor2->setSpeed(50);
     Motor3->setSpeed(49);
     Motor4->setSpeed(50);

    while(cnt2<4)
  {  
     cnt2++;
    
     Motor1->run(BACKWARD);
     Motor2->run(BACKWARD);
     Motor3->run(BACKWARD);
     Motor4->run(BACKWARD);
     
    delay(900);


   }
     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(1000);
     
     Motor1->setSpeed(51);
     Motor2->setSpeed(49);
     Motor3->setSpeed(51);
     Motor4->setSpeed(49);
     Motor1->run(BACKWARD);
     Motor2->run(FORWARD);
     Motor3->run(FORWARD);
     Motor4->run(BACKWARD);
     delay(800);

     Motor1->run(BRAKE);
     Motor2->run(BRAKE);
     Motor3->run(BRAKE);
     Motor4->run(BRAKE);
     delay(2000);

     Servostatus_begin();
     delay(10000);

}