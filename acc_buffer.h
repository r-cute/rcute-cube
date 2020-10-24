#define BUF_LEN 25
#define SHAKE2 256.0F // 16 m/s^2
#define FALL2 4.0F

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

class AccelBuffer{
protected:
  float buf[BUF_LEN];
  float *bufEnd = buf+BUF_LEN;
  float *head=buf, *tail=buf; 
  VectorFloat gravity;
public:
  uint16_t shakeCount, fallCount;
  VectorFloat vel, dist, // cm/s^2
              macc; // m/s^2
  float maxMagnitude2;
  void reset(MPUData* last) {
    tail = head;
    fallCount = shakeCount = maxMagnitude2 = 0;
    dist = vel = macc = VectorFloat(0,0,0);
//    Quaternion q = last->ori.getConjugate();
//    gravity_1 = last->acc.getRotated(&q);
    gravity.copy(last->acc);
  }
  void feed(VectorFloat& acc, Quaternion& q, float sec){
    *tail = acc.getMagnitude2();
    if(*tail > SHAKE2) shakeCount++;
    else if(*tail < FALL2) fallCount++;
    tail++; if(tail == bufEnd) tail = buf;
    if(tail == head){
      if(*head > SHAKE2) shakeCount--;
      else if(*head < FALL2) fallCount--;
      head++; if(head == bufEnd) head = buf;
    }    
//    VectorFloat gravity = gravity_1.getRotated(&q);
    VectorFloat lacc = acc.sub(gravity);
    vel.iadd(lacc, sec*100);// cm/s
    dist.iadd(vel, sec); // cm
    float laccMag2 = lacc.getMagnitude2();
    if(laccMag2 > maxMagnitude2) {maxMagnitude2 = laccMag2; macc.copy(lacc);}
  }
};
