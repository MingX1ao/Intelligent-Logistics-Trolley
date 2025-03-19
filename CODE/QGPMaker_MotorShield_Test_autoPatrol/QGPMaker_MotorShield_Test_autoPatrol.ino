#include <Wire.h>
#include "QGPMaker_MotorShield.h"
#include "QGPMaker_Encoder.h"
#include <QGPMaker_IICSensorbar.h>

// Create the motor shield object with the default I2C address
QGPMaker_MotorShield AFMS = QGPMaker_MotorShield();
SensorBar mySensorBar; //创建巡线传感器对象
int IR_S[5] = {0,1,2,3,4};
int sensor_current[5] = {1,1,1,1,1};
int sensor_previous[5] = {1,1,1,1,1};
int state = 0;//记录智能车执行任务过程中的各种状态

// 创建电机对象. 
QGPMaker_DCMotor *DCMotor_1 = AFMS.getMotor(1);//左前
QGPMaker_DCMotor *DCMotor_2 = AFMS.getMotor(3);//左后
QGPMaker_DCMotor *DCMotor_3 = AFMS.getMotor(2);//右前
QGPMaker_DCMotor *DCMotor_4 = AFMS.getMotor(4);//右后
//创建编码器对象
QGPMaker_Encoder Encoder1(1); //创建1号编码器（对应于M1电机）
QGPMaker_Encoder Encoder2(3); //创建2号编码器（对应于M2电机）
QGPMaker_Encoder Encoder3(2); //创建3号编码器（对应于M3电机）
QGPMaker_Encoder Encoder4(4); //创建4号编码器（对应于M4电机）
//创建舵机对象
QGPMaker_Servo *Servo0 = AFMS.getServo(0); //获取0号舵机
QGPMaker_Servo *Servo1 = AFMS.getServo(1); //获取1号舵机
QGPMaker_Servo *Servo2 = AFMS.getServo(2); //获取2号舵机
QGPMaker_Servo *Servo3 = AFMS.getServo(3); //获取3号舵机
//创建舵机脉宽变量
double pulse0 = 1.5;
double pulse1 = 1.5;
double pulse2 = 1.5;
double pulse3 = 1.5;
//创建PID相关参数
double kp = 2.2,ki = 0.68,kd = 2.1;//电机速度调整的PID参数
double targetVelocity1 = 0,targetVelocity2 = 0,targetVelocity3 = 0,targetVelocity4 = 0;
double initTargetVelocity1 = 50,initTargetVelocity2 = 50,initTargetVelocity3 = -50,initTargetVelocity4 = -50;
double irKp = 20,irKi = 0.5,irKd = 0;//弯道巡线的PID参数
double irP = 0,irI = 0,irD = 0;//巡线的PID各项值
double irBias = 0,irBias_previous = 0,irPIDValue = 0;
int pwmStart = 10; //初始PWM脉宽值，小于这个值电机不动
int pwmRestrict = 245; //对PWM脉宽值限幅,pwmStart + pwmRestrict = 255
int interTime_pid = 5; //PID调整间隔，单位为ms
long t0 =0;//记录系统时间
long t_patrol = 0;//开始巡线的时刻
double pidOutput1,pidOutput2,pidOutput3,pidOutput4;//PID计算得到的新的PWM值
int startPatrol = 0;
int endPatrol = 0;
//声明自建函数
void GoForward(int v);
void GoBackward(int v);
void GoLeft(int v);
void GoRight(int v);
void TurnLeft(int v);
void TurnRight(int v);
void BreakDown();//刹车
void Stop();//自然停车

int calPI1(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
  return PWM;
}
int calPID1(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
  return PWM;
}
int calPI2(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
  return PWM;
}
int calPID2(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
  return PWM;
}
int calPI3(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
  return PWM;
}
int calPID3(int velocity, int target){
  static double bias,PWM,last_bias,last_bias2;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias + kd*(bias - 2*last_bias + last_bias2); //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias2 = last_bias;
  last_bias = bias;
  return PWM;
}
int calPI4(int velocity, int target){
  static double bias,PWM,last_bias;
  bias = target - velocity;
  PWM += kp*(bias - last_bias) + ki*bias; //增量式PID计算
  if(PWM > pwmRestrict){
    PWM = pwmRestrict;
  }
  if(PWM < -pwmRestrict){
    PWM = -pwmRestrict;
  }
  last_bias = bias;
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
  //链接循线传感器
  if (!mySensorBar.begin(0x3F)){
    while (1); // And loop forever.
  }
  delay(1000);
  //初始化巡线传感器状态
  for(int i = 0; i < 5; i++){
    sensor_current[i] = mySensorBar.digitalRead(IR_S[i]);
    sensor_previous[i] = sensor_current[i];
  }
  Serial.begin(9600);
  AFMS.begin(50); // create with the default frequency 50Hz
  Encoder1.write(0);  //编码器设置为0
  Encoder2.write(0);  //编码器设置为0
  Encoder3.write(0);  //编码器设置为0
  Encoder4.write(0);  //编码器设置为0
  pulse0 = 1.0;//1.0向后，2.0向前
  pulse1 = 1.5;//1.0向前，2.0向后
  pulse2 = 1.0;//1.0向前，2.0向后
  pulse3 = 1.1;//张开，1.5关闭
  Servo0->setServoPulse(pulse0);
  Servo1->setServoPulse(pulse1);
  Servo2->setServoPulse(pulse2);
  Servo3->setServoPulse(pulse3);
  //设定初始目标速度,前进
  targetVelocity1 = initTargetVelocity1;
  targetVelocity2 = initTargetVelocity2;
  targetVelocity3 = -initTargetVelocity3;
  targetVelocity4 = -initTargetVelocity4;
  
  DCMotor_1->setSpeed(pwmStart);
  DCMotor_2->setSpeed(pwmStart);
  DCMotor_3->setSpeed(pwmStart);
  DCMotor_4->setSpeed(pwmStart);
  DCMotor_1->run(RELEASE);
  DCMotor_2->run(RELEASE);
  DCMotor_3->run(RELEASE);
  DCMotor_4->run(RELEASE);
  t0 = millis();
}

void loop(){
  if(millis() - t0 >= interTime_pid){ //PID调整
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
    t0 = millis();
  }
  for(int i = 0; i < 5; i++){
    sensor_current[i] = mySensorBar.digitalRead(IR_S[i]);
  }
  if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 1) && (sensor_current[3] == 1) && (sensor_current[4] == 0)){
    irBias = 2;//          1 1 1 1 0
  }
  else if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 1) && (sensor_current[3] == 0) && (sensor_current[4] == 0)){
    irBias = 2;//          1 1 1 0 0
  }
  else if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 1) && (sensor_current[3] == 0) && (sensor_current[4] == 1)){
    irBias = 1;//          1 1 1 0 1
  } 
  else if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 0) && (sensor_current[3] == 0) && (sensor_current[4] == 1)){
    irBias = 1;//          1 1 0 0 1
  }
  else if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 0) && (sensor_current[3] == 1) && (sensor_current[4] == 1)){
    irBias = 0;//          1 1 0 1 1
  } 
  else if((sensor_current[0] == 1) && (sensor_current[1] == 0) && (sensor_current[2] == 0) && (sensor_current[3] == 1) && (sensor_current[4] == 1)){
    irBias = -1;//         1 0 0 1 1
  } 
  else if((sensor_current[0] == 1) && (sensor_current[1] == 0) && (sensor_current[2] == 1) && (sensor_current[3] == 1) && (sensor_current[4] == 1)){
    irBias = -1;//         1 0 1 1 1
  }
  else if((sensor_current[0] == 0) && (sensor_current[1] == 0) && (sensor_current[2] == 1) && (sensor_current[3] == 1) && (sensor_current[4] == 1)){
    irBias = -2;//         0 0 1 1 1
  }
  else if((sensor_current[0] == 0) && (sensor_current[1] == 1) && (sensor_current[2] == 1) && (sensor_current[3] == 1) && (sensor_current[4] == 1)){
    irBias = -2;//         0 1 1 1 1
  } 
  else if((sensor_current[0] == 1) && (sensor_current[1] == 1) && (sensor_current[2] == 1) && (sensor_current[3] == 1) && (sensor_current[4] == 1)) {
    if (irBias == -2) {//  1 1 1 1 1
      irBias = -3;
    }
    if(irBias == 2){
      irBias = 3;
    }
    if(endPatrol == 1){
      targetVelocity1 = 0;
      targetVelocity2 = 0;
      targetVelocity3 = 0;
      targetVelocity4 = 0;
      DCMotor_1->run(BRAKE);
      DCMotor_2->run(BRAKE);
      DCMotor_3->run(BRAKE);
      DCMotor_4->run(BRAKE);
      t0 =millis();//停止PID
    }
  }
  else if(startPatrol == 0 && (sensor_current[0] == 0) && (sensor_current[1] == 0) && (sensor_current[2] == 0) && (sensor_current[3] == 0) && (sensor_current[4] == 0)){
    startPatrol = 1;//开始巡线
    t_patrol = millis();
  }
  else if(startPatrol == 1 && (sensor_current[0] == 0) && (sensor_current[1] == 0) && (sensor_current[2] == 0) && (sensor_current[3] == 0) && (sensor_current[4] == 0)) {
    if(millis() - t_patrol >=3000){
      targetVelocity1 = 0;
      targetVelocity2 = 0;
      targetVelocity3 = 0;
      targetVelocity4 = 0;
      DCMotor_1->run(BRAKE);
      DCMotor_2->run(BRAKE);
      DCMotor_3->run(BRAKE);
      DCMotor_4->run(BRAKE);
      endPatrol = 1;//巡线结束
      t0 =millis();//停止PID
      t_patrol = millis();
    }
  }
  else{
    
  }
  if(endPatrol == 0){
    irP = irBias;
    irI += irBias;
    irD = irBias - irBias_previous;
    irPIDValue = irKp * irP + irKi * irBias;
    irBias_previous = irBias;
    targetVelocity1 = initTargetVelocity1 - irPIDValue;
    targetVelocity2 = initTargetVelocity2 - irPIDValue;
    targetVelocity3 = initTargetVelocity3 - irPIDValue;
    targetVelocity4 = initTargetVelocity4 - irPIDValue;

    if(targetVelocity1 <= 10 || targetVelocity2 <= 10){
      targetVelocity1 = 10;
      targetVelocity2 = 10;
    }
    if(targetVelocity1 >= 100 || targetVelocity2 >= 100){
      targetVelocity1 = 100;
      targetVelocity2 = 100;
    }
    if(targetVelocity3 >= -10 || targetVelocity4 >= -10){
      targetVelocity3 = -10;
      targetVelocity4 = -10;
    }
    if(targetVelocity3 <= -100 || targetVelocity4 <= -100){
      targetVelocity3 = -100;
      targetVelocity4 = -100;
    }
  }
//  Serial.print(mySensorBar.digitalRead(IR_S[0]));
//  Serial.print(",");
//  Serial.print(mySensorBar.digitalRead(IR_S[1]));
//  Serial.print(",");
//  Serial.print(mySensorBar.digitalRead(IR_S[2]));
//  Serial.print(",");
//  Serial.print(mySensorBar.digitalRead(IR_S[3]));
//  Serial.print(",");
//  Serial.println(mySensorBar.digitalRead(IR_S[4]));
//  Serial.println(irPIDValue);
//  Serial.print(targetVelocity1);
//  Serial.print(",");
//  Serial.print(targetVelocity2);
//  Serial.print(",");
//  Serial.print(targetVelocity3);
//  Serial.print(",");
//  Serial.println(targetVelocity4);
}
