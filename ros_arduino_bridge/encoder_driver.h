/* *************************************************************
   Arduino mega 外部中断：
   中断号 int0  ----- 引脚号 2 
   中断号 int1  ----- 引脚号 3
   中断号 int2  ----- 引脚号 21
   中断号 int3  ----- 引脚号 20
   中断号 int4  ----- 引脚号 19
   中断号 int5  ----- 引脚号 18   
   ************************************************************ */

// 左轮
#define ENC_LEFT_PINA 18
#define ENC_LEFT_PINB 4

// 右轮
#define ENC_RIGHT_PINA 19
#define ENC_RIGHT_PINB 5

   
void initEncoders();
long readEncoder(int i);
void resetEncoders();
