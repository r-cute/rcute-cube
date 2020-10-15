#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "helper_3dmath.h"
#include "filter_buffer.h"
/*
void setAccelerometerRange(mpu6050_accel_range_t new_range) {
  Adafruit_MPU6050::setAccelerometerRange(new_range);
  if (new_range == MPU6050_RANGE_16_G)
    accScale = 1.0/2048;
  else if (new_range == MPU6050_RANGE_8_G)
    accScale = 1.0/4096;
  else if (new_range == MPU6050_RANGE_4_G)
    accScale = 1.0/8192;
  else if (new_range == MPU6050_RANGE_2_G)
    accScale = 1.0/16384;
  accScale *= SENSORS_GRAVITY_STANDARD;
}

void setGyroRange(mpu6050_gyro_range_t new_range) {
  Adafruit_MPU6050::setGyroRange(new_range);
  if (new_range == MPU6050_RANGE_250_DEG)
    gyroScale = 1.0/131;
  else if (new_range == MPU6050_RANGE_500_DEG)
    gyroScale = 1.0/65.5;
  else if (new_range == MPU6050_RANGE_1000_DEG)
    gyroScale = 1.0/32.8;
  else if (new_range == MPU6050_RANGE_2000_DEG)
    gyroScale = 1.0/16.4;
}
*/
#define INTERRUPT_PIN 15
#define POWER_PIN 16

typedef void (*MPUCallback)(void);

typedef enum{
  STATIC,
  MOMENTARY_STATIC,
  MOVING,
  MOTION_DETECTED
}MPUState_t;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {mpuInterrupt = true;}
bool shakeFilter(float t){return t> 60.0f;}

class MPUData {
  public:
  Quaternion ori;
  VectorFloat acc;
  VectorFloat gyro;
  MPUState_t momentaryState;
  long time;
  void setMomentaryState(MPUData* lastData){
    if(lastData==NULL){momentaryState=MOVING; return;}
    momentaryState=(acc.sub(lastData->acc).getMagnitude2()> 1.0f|| gyro.compLessThan(5.0f))?MOMENTARY_STATIC:MOVING;
  }
  void copy(MPUData* c){
    ori.copy(c->ori);
    acc.copy(c->acc);
    gyro.copy(c->gyro);
    momentaryState = c->momentaryState;
    time = c->time;
  }
};

class MyMPU6050: public MPU6050 {
  
protected:
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector

FilterBuffer accBuf;
JsonArray* event_array;

public:  
MPUCallback cbEvent = NULL;
MPUCallback cbUpdate = NULL;
long momentaryStaticStart = 0;
MPUData mpudata[3], *currData=mpudata, *lastData=NULL, *lastStaticData=NULL;
MPUState_t state = MOVING;

void calibrate(int discardNumber=50, int calibrateNumber=100, int delayTime=5) {
  
}

uint8_t setup(JsonArray* ea){
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, 0);
  delay(500);
  digitalWrite(POWER_PIN, 1);
  Wire.begin();
  Wire.setClock(400000);
  event_array = ea;  
  initialize();
  if(!testConnection()){
    Serial.println(F("[mpu] connection failed"));
    return 3;
  }
  Serial.println(F("[mpu] connection successful"));
  uint8_t devStatus = dmpInitialize();
  if(devStatus){
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.printf("[mpu] DMP Initialization failed (code %d)", devStatus);
    return devStatus;
  }
  setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  Serial.println(F("[mpu] DMP ready! Waiting for first interrupt..."));
  dmpReady = true;
  packetSize = dmpGetFIFOPacketSize();
  accBuf.filter = shakeFilter;
  setSleepEnabled(true);
  return 0;  
}

void enable(bool e){
  setDMPEnabled(e);
  setSleepEnabled(e);
}

void loop(){
  if (!dmpReady) return;
  if (!mpuInterrupt && fifoCount < packetSize) return;
  mpuInterrupt = false;
  mpuIntStatus = getIntStatus();
  fifoCount = getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }  else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = getFIFOCount();
    getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    
    dmpGetQuaternion(&(currData->ori), fifoBuffer);
    dmpGetAccel(&(currData->acc), fifoBuffer);
    dmpGetGyro(&(currData->gyro), fifoBuffer);
    currData->time = millis();
    currData->setMomentaryState(lastData);
//    dmpGetGravity(&gravity, &q);
//    dmpGetLinearAccel(&aaReal, &aa, &gravity);
//    dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    if(lastData){
      if(lastData->momentaryState == MOMENTARY_STATIC){
        if(currData->momentaryState == MOMENTARY_STATIC){ // m_static -> m_static
          if(currData->time - momentaryStaticStart > 500){
            if(lastStaticData && state == MOVING){ // no shake detected yet
              //...
            }            
            state = STATIC;
          }
        }else { //currData->momentaryState == MOVING: m_static -> m_moving
          if(state == STATIC){
            if(lastStaticData==NULL)lastStaticData=mpudata+2;
            lastStaticData->copy(lastData);            
            state = MOVING;
            accBuf.reset();
            accBuf.feed(currData->acc.getMagnitude2());
          }
        }
      }else{ //lastData->momentaryState == MOVING
        if(currData->momentaryState == MOMENTARY_STATIC){ // m_moving -> m_static
          momentaryStaticStart = currData->time;
        }else{ // m_moving -> m_moving
          if(state == MOVING) {
            accBuf.feed(currData->acc.getMagnitude2());
            if(accBuf.count > 5) { // shake
              state = MOTION_DETECTED;              
            }
          }
        }
      }
    }

    // swap currData and lastData
    if(lastData==NULL) lastData= mpudata+1;
    MPUData* temp = lastData;
    lastData = currData;
    currData = temp;
  }
}

uint8_t dmpGetAccel(VectorFloat *q, const uint8_t* packet) {
    // 8g
    static int16_t qI[3];
    uint8_t status = MPU6050::dmpGetAccel(qI, packet);
    if (status == 0) {
        q -> x = (float)qI[0] * 0.002392578125f;
        q -> y = (float)qI[1] * 0.002392578125f;
        q -> z = (float)qI[2] * 0.002392578125f;
        return 0;
    }
    return status;
}

uint8_t dmpGetGyro(VectorFloat *q, const uint8_t* packet) {
    // 250 deg
    static int16_t qI[3];
    uint8_t status = MPU6050::dmpGetGyro(qI, packet);
    if (status == 0) {
        q -> x = (float)qI[0] * 0.00763358779f;
        q -> y = (float)qI[1] * 0.00763358779f;
        q -> z = (float)qI[2] * 0.00763358779f;
        return 0;
    }
    return status;
}

};
