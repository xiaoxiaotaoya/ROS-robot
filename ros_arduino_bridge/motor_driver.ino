// 加载I2C和电机驱动库
#include "commands.h";
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver drive = Adafruit_PWMServoDriver(0x41);

// 定义最大PWM
#define MAX_PWM  4095

boolean directionLeft = false;
boolean directionRight = false;
  
// 初始化电机控制
 void initMotorController() {
  
     drive.begin();
     drive.setPWMFreq(50);
     
     drive.setPWM(9, 0, 0);
     drive.setPWM(8, 0, 0);
     drive.setPWM(10, 0, 0);
     drive.setPWM(11, 0, 0);
     
     drive.setPWM(12, 0, 0);
     drive.setPWM(13, 0, 0);
     drive.setPWM(15, 0, 0);
     drive.setPWM(14, 0, 0);  
  }

// 确定方向
 boolean direction(int i){
   if(i == LEFT){
      return directionLeft;
   }
   else{
      return directionRight;
   }
}

// PWM控制 
 void setMotorSpeed(int i, int spd) {
    if(spd > MAX_PWM){
      spd = MAX_PWM;
    }
    else if(spd < -MAX_PWM){
      spd = -1 * MAX_PWM;
    }
    
    if (i == LEFT){
        if(spd>=0){
            directionLeft = FORWARDS;
             drive.setPWM(9, 0, spd);
             drive.setPWM(8, 0, 0);
             drive.setPWM(10, 0, 0);
             drive.setPWM(11, 0, spd);
        }else if(spd < 0){
            directionLeft = BACKWARDS;
            spd = -1 * spd; 
             drive.setPWM(9, 0, 0);
             drive.setPWM(8, 0, spd);
             drive.setPWM(10, 0, spd);
             drive.setPWM(11, 0, 0);
        }
    }
    else {
        if(spd>=0){
            directionRight = FORWARDS;
             drive.setPWM(12, 0, spd);
             drive.setPWM(13, 0, 0);
             drive.setPWM(15, 0, 0);
             drive.setPWM(14, 0, spd);	
        }else if(spd<0){
            directionRight = BACKWARDS;
            spd = -1 * spd;
             drive.setPWM(12, 0, 0);
             drive.setPWM(13, 0, spd);
             drive.setPWM(15, 0, spd);
             drive.setPWM(14, 0, 0);
        }
    }
  }
  
//左右两轮电机控制
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
