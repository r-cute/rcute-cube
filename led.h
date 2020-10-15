#define RED 15
#define GREEN 12
#define BLUE 13

class RGB {
  public:
  uint8_t r, g, b;
  bool l;
  void rgb(uint8_t rr, uint8_t gg, uint8_t bb) {
    analogWrite(RED, rr);
    analogWrite(GREEN, gg);
    analogWrite(BLUE, bb);
    r =rr; g=gg; b=bb;    
  }
  
  void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
    led(false);
    rgb(0, 0, 0);
  }
  
  void led(int v) {
    digitalWrite(LED_BUILTIN, !v);
    l = !v;
  }
  
  void blink(uint8_t n, long d, long doff=-1) {
    if(doff==-1)doff=d;
    while (n==-1 || n--) {
      led(true);
      delay(d);
      led(false);
      delay(doff);
    }
  }
  
  void blink_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t n, long d, long doff=-1) {
    if(doff==-1)doff=d;
    while (n==-1 || n--) {    
      rgb(r, g, b);
      delay(d);  
      rgb(0, 0, 0);
      delay(doff);  
    }
  }
};
