#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "acc_buffer.h"
#define INTERRUPT_PIN 15
#define POWER_PIN 16
// connect AD0 to GND

typedef void (*MPUCallback)(void);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

typedef enum{
  STATIC,
  MOMENTARY_STATIC,
  MOVING,
  MOTION_DETECTED
}MPUState_t;

class MPUData {
  public:
  Quaternion ori; // orientation
  VectorFloat acc;
  VectorInt16 gyro;
  MPUState_t momentaryState;
  long time;
  void setMomentaryState(MPUData* lastData){
    if(lastData==NULL){momentaryState=MOVING; return;}
    momentaryState=(acc.sub(lastData->acc).getMagnitude2()< 0.01f /* && fabs(acc.getMagnitude2()-(9.8*9.8))<1.0f */ && gyro.compLessThan(6))?MOMENTARY_STATIC:MOVING;
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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

AccelBuffer accBuf;
JsonArray* event_array;

MPUCallback cbEvent = NULL;
MPUCallback cbUpdate = NULL;
long momentaryStaticStart = 0;
MPUData mpudata[3], *currData=mpudata, *lastData=NULL, *lastStaticData=NULL;
MPUState_t state = MOVING;

void saveOffsets(int offset[]){
  StaticJsonDocument<200> doc;
  for(uint8_t i=0;i<3;i++){
    doc[i]=offset[i];  
  }
  File f=SPIFFS.open("/offset.json", "w");
  serializeJson(doc, f);
  f.close();
  Serial.print("[mpu] save offsets serialized: ");
  serializeJson(doc, Serial);  
  Serial.println();
}

uint8_t readOffsets(int offset[]){
  File f=SPIFFS.open("/offset.json", "r");
  if(!f) return 1;
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, f);  
  f.close();
  if (error) {
    Serial.print(F("[mpu] reading offsets deserializeJson() failed: "));
    Serial.println(error.c_str());
    return 2;
  }
  for(uint8_t i=0;i<3;i++){
    offset[i]=doc[i];
  }
  Serial.printf("[mpu] read offsets: %d, %d, %d\n", offset[0], offset[1], offset[2]);
  return 0;
}

void setOffsets(int TheOffsets[]){
  setXGyroOffset (TheOffsets [0]);
  setYGyroOffset (TheOffsets [1]);
  setZGyroOffset (TheOffsets [2]);
//  setXAccelOffset(TheOffsets [0]);
//  setYAccelOffset(TheOffsets [1]);
//  setZAccelOffset(TheOffsets [2]);
  
}
  
void calibrate(uint16_t discardNumber=5000, uint16_t calibrateNumber=500, int delayTime=5) {
  int32_t sum[3]={0,0,0};
  int16_t reading[6];
  for(uint16_t loop=0;loop<discardNumber;loop++){
    getMotion6(&reading[0],&reading[1],&reading[2],&reading[3],&reading[4],&reading[5]);
    delay(delayTime);
  }
  for(uint16_t loop=0;loop<calibrateNumber;loop++){
    getMotion6(&reading[0],&reading[1],&reading[2],&reading[3],&reading[4],&reading[5]);
    delay(delayTime);
    for(uint8_t i=0;i<3;i++){
      sum[i]+=reading[i+3];
    }
  }
  for(uint8_t i=0;i<3;i++){
    sum[i]/=calibrateNumber;
  }
  saveOffsets(sum);
  setOffsets(sum);
}

void sleep(bool e){
  setSleepEnabled(e);
  setDMPEnabled(!e);
  sleeping = e;
}

uint8_t setup(JsonArray* ea){
  pinMode(POWER_PIN, OUTPUT);  
  digitalWrite(POWER_PIN,0);
  delay(500);
  digitalWrite(POWER_PIN,1);
  delay(500);
  event_array = ea;
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.println(F("[mpu] Initializing I2C devices..."));
  initialize();
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  if(!testConnection()){
    Serial.println(F("[mpu] MPU6050 connection failed"));    
    return 3;
  }
  Serial.println(F("[mpu] MPU6050 connection successful"));
  Serial.println(F("[mpu] Initializing DMP..."));
  devStatus = dmpInitialize();

//  int offsets[3];
//  if(readOffsets(offsets)==0){
//    setOffsets(offsets);
//  }
  
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
  sleep(true);
  return 0;
}

void send(const char* a, const char* b){
  Serial.print(a);  Serial.println(b); // debug print
  (*event_array)[0]=a;
  (*event_array)[1]=b;
  if(cbEvent) cbEvent();
}

void loop(){
  // if programming failed, don't try to do anything
  if (sleeping || !dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = getIntStatus();

  // get current FIFO count
  fifoCount = getFIFOCount();

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
    dmpGetAccel(&aa, &(currData->acc), fifoBuffer);
    dmpGetGyro(&(currData->gyro), fifoBuffer);
    currData->time = millis();
    currData->setMomentaryState(lastData);
    
    if(state==MOVING && lastStaticData)
      accBuf.feed(currData->acc, lastStaticData->acc, (currData->time - lastData->time)/1000.0f);
      
    if(lastData){
      if(lastData->momentaryState == MOMENTARY_STATIC){
        if(currData->momentaryState == MOMENTARY_STATIC){// m_static -> m_static
          if(state==MOVING){
            float duration = currData->time - momentaryStaticStart;
            if(duration > 300){
              if(lastStaticData && currData->time - lastStaticData->time < 2000){
                float ang = lastStaticData->acc.angleDeg(currData->acc);
  //              Serial.printf("ang %f\n", ang); // debug print
                if(fabs(ang-90)<15) send("flipped", "90");
                else if(fabs(ang-180)<15) send("flipped", "180");
                else if(fabs(ang)<15) {// no flip, check horitontal rotation
                  Rotation rot(lastStaticData->ori.getRotationTo(currData->ori));
  //                Serial.printf("rot, %f, %f\n", rot.angleDeg, rot.axis.angleDeg(currData->acc)); // debug print
                  if(rot.angleDeg > 35) {
                    send("rotated", fabs(rot.axis.angleDeg(currData->acc))<10 ? "CCW":"CW");
                  } else if(rot.angleDeg < 15){ // no rotation, check horizontal move(push)
//                    Serial.printf("push: ang:%f, dist.mag:%f\n", accBuf.dist.angleDeg(currData->acc),accBuf.dist.getMagnitude()); // debug print
//                    Serial.printf("dist(x,y,z): %f, %f, %f\n",accBuf.dist.x, accBuf.dist.y, accBuf.dist.z);// debug print
//                    Serial.printf("macc(x,y,z): %f, %f, %f, ang:%f\n",accBuf.macc.x, accBuf.macc.y, accBuf.macc.z, accBuf.macc.angleDeg(currData->acc));// debug print
                    ang = accBuf.macc.angleDeg(currData->acc); 
                    if((fabs(ang)<25 || fabs(ang-180)<25) && accBuf.macc.getMagnitude2()>3.0f){ // tap
                      send("tapped", NULL);
                    }else if(fabs(accBuf.dist.angleDeg(currData->acc)-90)<25 && accBuf.dist.getMagnitude2()> 10.0f) { // push 2cm
                      char mc[3];
                      accBuf.dist.getMainComp(mc);
                      send("pushed", mc);
                    }
                  }
                }
              }
              state = STATIC;
//              Serial.println("state: static"); // debug print
            }
          }
        }else { //currData->momentaryState == MOVING: m_static -> m_moving
          if(state == STATIC){
            if(lastStaticData==NULL)lastStaticData=mpudata+2;
            lastStaticData->copy(lastData);            
            state = MOVING;
//            Serial.println("state: moving"); // debug print
            accBuf.reset();
          }
        }
      }else{ //lastData->momentaryState == MOVING        
        if(currData->momentaryState == MOMENTARY_STATIC){ // m_moving -> m_static
          momentaryStaticStart = currData->time;          
        }else{ // m_moving -> m_moving          
          if(state == MOVING) {            
//            Serial.printf("count %d, %f\n", accBuf.filterCount, currData->acc.getMagnitude()); // debug print
            if(accBuf.fallCount>2) {
              send("fall", NULL);
              state = MOTION_DETECTED;
            }
            /*if(accBuf.filterCount > 20) { // shaked hard
              send("shaked", "hard");
              state = MOTION_DETECTED;              
            }*/else if(accBuf.shakeCount > 10) { // shaked not so hard
//              send("shaked", "median");
              send("shaked", NULL);
              state = MOTION_DETECTED;
            }
          } else if(state== MOTION_DETECTED){
            if(accBuf.shakeCount ==0 && accBuf.fallCount==0)
              state = MOVING;
          }
        }
      }
    }

    if(cbUpdate) cbUpdate();
    
    // swap currData and lastData
    if(lastData==NULL) lastData= mpudata+1;
    MPUData* temp = lastData;
    lastData = currData;
    currData = temp;

    
  }

  
 
}

uint8_t dmpGetAccel(VectorInt16 *a, VectorFloat *q, const uint8_t* packet) {
    // 2g? or 8g?
    uint8_t status = MPU6050::dmpGetAccel(a, packet);
    if (status == 0) {
        q -> x = (float)a->x * 0.00119628906f;
        q -> y = (float)a->y * 0.00119628906f;
        q -> z = (float)a->z * 0.00119628906f;
    }
    return status;
}

uint8_t dmpGetGyro(VectorInt16 *q, const uint8_t* packet) {
  //2000 full range
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
