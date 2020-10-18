#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "filter_buffer.h"
#define INTERRUPT_PIN 15
#define POWER_PIN 16

typedef void (*MPUCallback)(void);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}
bool shakeFilter(float t){return t> 60.0f;}

typedef enum{
  STATIC,
  MOMENTARY_STATIC,
  MOVING,
  MOTION_DETECTED
}MPUState_t;
float la=0;
class MPUData {
  public:
  Quaternion ori;
  VectorFloat acc;
  VectorInt16 gyro;
  MPUState_t momentaryState;
  long time;
  void setMomentaryState(MPUData* lastData){
    if(lastData==NULL){momentaryState=MOVING; return;}
    momentaryState=(acc.sub(lastData->acc).getMagnitude2()< 1.0f /* && abs(acc.getMagnitude2()-(9.8*9.8))<1.0f */ && gyro.compLessThan(10))?MOMENTARY_STATIC:MOVING;
  }
  void copy(MPUData* c){
    ori.copy(c->ori);
    acc.copy(c->acc);
    gyro.copy(c->gyro);
    momentaryState = c->momentaryState;
    time = c->time;
  }
};

class MyMPU6050: public MPU6050{
public:
bool sleeping = true;
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

MPUCallback cbEvent = NULL;
MPUCallback cbUpdate = NULL;
long momentaryStaticStart = 0;
MPUData mpudata[3], *currData=mpudata, *lastData=NULL, *lastStaticData=NULL;
MPUState_t state = MOVING;

void calibrate(int discardNumber=50, int calibrateNumber=100, int delayTime=5) {
  
}

void sleep(bool e){
  setDMPEnabled(!e);
  setSleepEnabled(e);
  sleeping = e;
}

uint8_t setup(JsonArray* ea)
{
  pinMode(POWER_PIN, OUTPUT);
  digitalWrite(POWER_PIN, 0);
  delay(500);
  digitalWrite(POWER_PIN, 1);
  delay(500);
  event_array = ea;
  Wire.begin();
  Wire.setClock(300000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.println(F("[mpu] Initializing I2C devices..."));
  initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  if(!testConnection()){
    Serial.println(F("[mpu] MPU6050 connection failed"));    
    return 3;
  }
  delay(200);
  Serial.println(F("[mpu] MPU6050 connection successful"));
  Serial.println(F("[mpu] Initializing DMP..."));
  devStatus = dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
//  setXGyroOffset(220);
//  setYGyroOffset(76);
//  setZGyroOffset(-85);
//  setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.printf("[mpu] DMP Initialization failed (code %d)\n", devStatus);
    return devStatus;
  }
  Serial.println(F("[mpu] Enabling DMP..."));
  setDMPEnabled(true);
  Serial.println(F("[mpu] Enabling interrupt detection (Arduino external interrupt 0)..."));
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = getIntStatus();
  Serial.println(F("[mpu] DMP ready! Waiting for first interrupt..."));
  dmpReady = true;
  packetSize = dmpGetFIFOPacketSize();
  accBuf.filter = shakeFilter;
  sleep(false);
  return 0;
}

int temcou=0;
void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = getIntStatus();

  // get current FIFO count
  fifoCount = getFIFOCount();
  temcou++;
  if(temcou>1000){temcou=0;Serial.print("x");}
//  Serial.println(F("[mpu] get data!"));
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    resetFIFO();
    Serial.println(F("[mpu] FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = getFIFOCount();

    // read a packet from FIFO
    getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    dmpGetQuaternion(&(currData->ori), fifoBuffer);
    dmpGetAccel(&(currData->acc), fifoBuffer);
    dmpGetGyro(&(currData->gyro), fifoBuffer);
    currData->time = millis();
    currData->setMomentaryState(lastData);
    if(lastData){
      if(lastData->momentaryState == MOMENTARY_STATIC){
        if(currData->momentaryState == MOMENTARY_STATIC){ // m_static -> m_static
          if(currData->time - momentaryStaticStart > 250){
            if(lastStaticData && state == MOVING){ // no shake detected yet
              float ang = lastStaticData->acc.angleDeg(currData->acc);
              Serial.printf("ang %f\n", ang);
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
  }else{
    Serial.println(mpuIntStatus);
    setDMPEnabled(false);
    setDMPEnabled(true);
  }

  
}

uint8_t dmpGetAccel(VectorFloat *q, const uint8_t* packet) {
    // 2g
    static int16_t qI[3];
    uint8_t status = MPU6050::dmpGetAccel(qI, packet);
    if (status == 0) {
        q -> x = (float)qI[0] * 0.00119628906f;
        q -> y = (float)qI[1] * 0.00119628906f;
        q -> z = (float)qI[2] * 0.00119628906f;
        return 0;
    }
    return status;
}

uint8_t dmpGetGyro(VectorInt16 *q, const uint8_t* packet) {
    // 250 deg
    static int16_t qI[3];
    uint8_t status = MPU6050::dmpGetGyro(qI, packet);
    if(status == 0){
      q->x = qI[0];
      q->y = qI[1];
      q->z = qI[2];      
    }
    return status;
}
};
