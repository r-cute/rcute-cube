#define BUF_LEN 25
#define SHAKE2 256.0F // 16 m/s^2
#define FALL2 4.0F

class AccelBuffer{
protected:
  float buf[BUF_LEN];
  float *bufEnd = buf+BUF_LEN;
  float *head=buf, *tail=buf; 
   
public:
  uint16_t shakeCount, fallCount;
  VectorFloat vel, dist, // cm/s^2
              macc; // m/s^2
  float maxMagnitude2;
  void reset() {
    tail = head;
    fallCount = shakeCount = maxMagnitude2 = 0;
    dist = vel = macc = VectorFloat(0,0,0);
  }
  void feed(VectorFloat& acc, VectorFloat& gravity, float sec){
    *tail = acc.getMagnitude2();
    if(*tail > SHAKE2) shakeCount++;
    else if(*tail < FALL2) fallCount++;
    tail++; if(tail == bufEnd) tail = buf;
    if(tail == head){
      if(*head > SHAKE2) shakeCount--;
      else if(*head < FALL2) fallCount--;
      head++; if(head == bufEnd) head = buf;
    }    
    VectorFloat lacc = acc.sub(gravity);
    vel.iadd(lacc, sec*100);// cm/s
    dist.iadd(vel, sec); // cm
    float laccMag2 = lacc.getMagnitude2();
    if(laccMag2 > maxMagnitude2) {maxMagnitude2 = laccMag2; macc.copy(lacc);}
  }
};
