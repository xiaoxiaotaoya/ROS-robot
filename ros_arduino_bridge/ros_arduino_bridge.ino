// 加载库
#include "Arduino.h"
#include "commands.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"
#include "mpu6050.h"

// 串口波特率
#define BAUDRATE     57600 

// PID轮询速率 30次每秒
#define PID_RATE     30     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;

// 电机命令执行时间2s
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

// 定义电机PWM控制范围
#define MAX_PWM        4095

/* Variable initialization */
int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[32];
char argv2[32];
long arg1;
long arg2;

// 重置命令
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

// 执行命令
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[8];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch (cmd) {
    case GET_BAUDRATE: 
      Serial.println(BAUDRATE);
      break;
    case READ_PIDIN:
      Serial.print(readPidIn(LEFT));
      Serial.print(" ");
      Serial.println(readPidIn(RIGHT));
      break;
    case READ_PIDOUT:
      Serial.print(readPidOut(LEFT));
      Serial.print(" ");
      Serial.println(readPidOut(RIGHT));
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      Serial.println("OK");
      break; 
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case READ_MPU6050:
      send_imudata();
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    case UPDATE_PID:
      while ((str = strtok_r(p, ":", &p)) != '\0') {
        pid_args[i] = atoi(str);
        i++;
      }
      left_Kp = pid_args[0];
      left_Kd = pid_args[1];
      left_Ki = pid_args[2];
      left_Ko = pid_args[3];

      right_Kp = pid_args[4];
      right_Kd = pid_args[5];
      right_Ki = pid_args[6];
      right_Ko = pid_args[7];
      Serial.println("OK");
      break;
    case ANALOG_READ:
      Serial.println(analogRead(arg1));
      break;
    case DIGITAL_READ:
      Serial.println(digitalRead(arg1));
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      Serial.println("OK");
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      Serial.println("OK");
      break;
    default:
      Serial.println("Invalid Command");
      break;
  }
}

// 初始化
void setup() {
  Serial.begin(BAUDRATE);
  initEncoders();
  initMotorController();
  initmpu6050();
  resetPID();
}

// 循环执行
void loop() {

  while (Serial.available() > 0) {
    chr = Serial.read();
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}
