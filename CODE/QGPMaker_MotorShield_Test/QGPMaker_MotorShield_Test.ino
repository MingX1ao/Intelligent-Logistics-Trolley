#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"

// Create the motor shield object with the default I2C address
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();

// 创建电机对象. 
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(2);
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(3);
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);
//创建编码器对象
QGPMaker_Encoder Encoder1(1); //创建1号编码器（对应于M1电机）
QGPMaker_Encoder Encoder2(2); //创建2号编码器（对应于M2电机）
QGPMaker_Encoder Encoder3(3); //创建3号编码器（对应于M3电机）
QGPMaker_Encoder Encoder4(4); //创建4号编码器（对应于M4电机）
//创建舵机对象
QGPMaker_Servo *Servo0 = AFMS.getServo(0); //获取0号舵机
QGPMaker_Servo *Servo1 = AFMS.getServo(1); //获取1号舵机
QGPMaker_Servo *Servo2 = AFMS.getServo(2); //获取2号舵机
QGPMaker_Servo *Servo3 = AFMS.getServo(3); //获取3号舵机
//创建舵机角度变量
int theta0 = 90;
int theta1 = 90;
int theta2 = 0;
int theta3 = 0;
//创建PID相关参数
double kp = 2.2,ki = 0.68,kd = 2.1;
double targetVelocity1 = 0,targetVelocity2 = 0,targetVelocity3 = 0,targetVelocity4 = 0;
int pwmStart = 10; //初始PWM脉宽值，小于这个值电机不动
int pwmRestrict = 245; //对PWM脉宽值限幅,pwmStart + pwmRestrict = 255
int interTime = 5; //PID调整间隔，单位为ms
long t0 =0;//记录系统时间
double pidOutput1,pidOutput2,pidOutput3,pidOutput4;//PID计算得到的新的PWM值

int calPI1(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPID1(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPI2(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPID2(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPI3(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPID3(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPI4(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
int calPID4(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
//  Serial.print(bias);
//  Serial.print(",");
//  Serial.print(last_bias);
//  Serial.print(",");
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
//  Serial.print(PWM);
//  Serial.print(",");
  return PWM;
}
void setup(){
  Serial.begin(9600);
  AFMS.begin(50); // create with the default frequency 50Hz
  Encoder1.write(0);  //编码器设置为0
  Encoder2.write(0);  //编码器设置为0
  Encoder3.write(0);  //编码器设置为0
  Encoder4.write(0);  //编码器设置为0
  theta0 = 0;
  theta1 = 90;
  theta2 = 90;
  theta3 = 90;
  Servo0->writeServo(theta0);
  Servo1->writeServo(theta1);
  Servo2->writeServo(theta2);
  Servo3->writeServo(theta3);
  //设定目标速度
  targetVelocity1 = 10;
  targetVelocity2 = 10;
  targetVelocity3 = -10;
  targetVelocity4 = -10;
  
  DCMotor_1->setSpeed(pwmStart);
  DCMotor_2->setSpeed(pwmStart);
  DCMotor_3->setSpeed(pwmStart);
  DCMotor_4->setSpeed(pwmStart);
  DCMotor_1->run(BACKWARD);
  DCMotor_2->run(BACKWARD);
  DCMotor_3->run(BACKWARD);
  DCMotor_4->run(BACKWARD);
  t0 = millis();
}

void loop(){
//  while(1){
//    Serial.println(Encoder4.getRPM());
//    delay(100);
//  }
  if(millis() - t0 >= interTime){
    int temp1 = Encoder1.getRPM();
    int temp2 = Encoder2.getRPM();
    int temp3 = Encoder3.getRPM();
    int temp4 = Encoder4.getRPM();
    pidOutput1 = calPID1(temp1,targetVelocity1);
    pidOutput2 = calPID2(temp2,targetVelocity2);
    pidOutput3 = calPID3(temp3,targetVelocity3);
    pidOutput4 = calPID4(temp4,targetVelocity4);
    if(pidOutput1 > 0){
      DCMotor_1->setSpeed(pidOutput1+pwmStart);
      DCMotor_1->run(BACKWARD);
    }
    else if(pidOutput1 == 0){
      DCMotor_1->setSpeed(pidOutput1+pwmStart);
      DCMotor_1->run(RELEASE);
    }
    else{
      DCMotor_1->setSpeed(-pidOutput1+pwmStart);
      DCMotor_1->run(FORWARD);
    }
    
    if(pidOutput2 > 0){
      DCMotor_2->setSpeed(pidOutput2+pwmStart);
      DCMotor_2->run(BACKWARD);
    }
    else if(pidOutput2 == 0){
      DCMotor_2->setSpeed(pidOutput2+pwmStart);
      DCMotor_2->run(RELEASE);
    }
    else{
      DCMotor_2->setSpeed(-pidOutput2+pwmStart);
      DCMotor_2->run(FORWARD);
    }

    if(pidOutput3 > 0){
      DCMotor_3->setSpeed(pidOutput3+pwmStart);
      DCMotor_3->run(FORWARD);
    }
    else if(pidOutput3 == 0){
      DCMotor_3->setSpeed(pidOutput3+pwmStart);
      DCMotor_3->run(RELEASE);
    }
    else{
      DCMotor_3->setSpeed(-pidOutput3+pwmStart);
      DCMotor_3->run(BACKWARD);
    }

    if(pidOutput4 > 0){
      DCMotor_4->setSpeed(pidOutput4+pwmStart);
      DCMotor_4->run(FORWARD);
    }
    else if(pidOutput4 == 0){
      DCMotor_4->setSpeed(pidOutput4+pwmStart);
      DCMotor_4->run(RELEASE);
    }
    else{
      DCMotor_4->setSpeed(-pidOutput4+pwmStart);
      DCMotor_4->run(BACKWARD);
    }
    Serial.println(temp1);
    Serial.println(temp2);
    Serial.println(temp3);
    Serial.println(temp4);
    t0 = millis();
  }
  
  while(theta1>90){
   theta1 = theta1 -1;
    Servo1->writeServo(theta1);
    delay(100);
  }
  while(theta0 >10){
    theta0 = theta0 - 1;
    Servo0->writeServo(theta0);
    delay(100);
  }
  while(theta0 < 90){
    theta0 = theta0 + 1;
    Servo0->writeServo(theta0);
    delay(100);
      }
}
