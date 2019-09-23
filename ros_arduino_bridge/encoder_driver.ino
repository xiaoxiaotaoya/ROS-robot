#include "motor_driver.h"
#include "encoder_driver.h"
#include "commands.h";

int left_enc_pos = 0L;
int right_enc_pos = 0L;

// 初始化
void initEncoders(){
  pinMode(ENC_LEFT_PINA, INPUT);
  pinMode(ENC_LEFT_PINB, INPUT);
  attachInterrupt(5, encoderLeftISR,FALLING);  
    
  pinMode(ENC_RIGHT_PINA, INPUT);
  pinMode(ENC_RIGHT_PINB, INPUT);
  attachInterrupt(4, encoderRightISR,FALLING);

}

//每次产生中断，根据左轮方向计数器加1或减1
void encoderLeftISR(){
    if(digitalRead(ENC_LEFT_PINB)==HIGH){
      left_enc_pos++;
    }
    else{
      left_enc_pos--;
    }
}

//每次产生中断，根据右轮方向计数器加1或减1
void encoderRightISR(){
    if(digitalRead(ENC_RIGHT_PINB)==HIGH){
      right_enc_pos--;
    }
    else{
      right_enc_pos++;
    }
}

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    if (i == LEFT) return 0-left_enc_pos;
    else return right_enc_pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT){
      left_enc_pos=0L;
      return;
    } else { 
      right_enc_pos=0L;
      return;
    }
  }

void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
  }
