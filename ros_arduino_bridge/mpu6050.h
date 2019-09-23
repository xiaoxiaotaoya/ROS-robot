/* =========================================================================
  引脚接线
    5V ---------- 5V
    GND --------- GND
    SDA --------- SDA
    SCL --------- SCL
    INT --------- pin 2
 * ========================================================================= */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void initmpu6050() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setDMPEnabled(true);

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXAccelOffset(-1169);
    mpu.setYAccelOffset(744);
    mpu.setZAccelOffset(1620);
    mpu.setXGyroOffset(48);
    mpu.setYGyroOffset(47);
    mpu.setZGyroOffset(-8);

    pinMode(INTERRUPT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); 
}


    // packet structure for InvenSense teapot demo
   uint8_t teapotPacket[28] = { '$', 0x03, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
    
  void send_imudata(){
    mpuInterrupt = false;   
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount(); 
    
     if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display quaternion values in InvenSense Teapot demo format:
        teapotPacket[2] = fifoBuffer[0];
        teapotPacket[3] = fifoBuffer[1];
        teapotPacket[4] = fifoBuffer[4];
        teapotPacket[5] = fifoBuffer[5];
        teapotPacket[6] = fifoBuffer[8];
        teapotPacket[7] = fifoBuffer[9];
        teapotPacket[8] = fifoBuffer[12];
        teapotPacket[9] = fifoBuffer[13];
        // gyro values
        teapotPacket[10] = fifoBuffer[16];
        teapotPacket[11] = fifoBuffer[17];
        teapotPacket[12] = fifoBuffer[20];
        teapotPacket[13] = fifoBuffer[21];
        teapotPacket[14] = fifoBuffer[24];
        teapotPacket[15] = fifoBuffer[25];
        // accelerometer values
        teapotPacket[16] = fifoBuffer[28];
        teapotPacket[17] = fifoBuffer[29];
        teapotPacket[18] = fifoBuffer[32];
        teapotPacket[19] = fifoBuffer[33];
        teapotPacket[20] = fifoBuffer[36];
        teapotPacket[21] = fifoBuffer[37];
        //temperature
        int16_t temperature = mpu.getTemperature();
        teapotPacket[22] = temperature >> 8;
        teapotPacket[23] = temperature & 0xFF;
        Serial.write(teapotPacket, 28);
        teapotPacket[25]++; // packetCount, loops at 0xFF on purpose
  }
 }
