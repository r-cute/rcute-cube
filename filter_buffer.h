#define T float
#define BUF_LEN 22

typedef bool (*filter_cb)(T);

class FilterBuffer{
protected:
  T buf[BUF_LEN];
  T *bufEnd = buf+BUF_LEN;
  T *head=buf, *tail=buf; 
   
public:
  uint8_t count;
//  FilterBuffer(filter_cb cb){filter=cb;}
  filter_cb filter=NULL;
  int filterCount;
  void reset() {
    tail=head;
    filterCount =count =0;
  }
  void feed(T a){
    if(filter(a)) filterCount++;
    *tail = a;
    tail++; if(tail==bufEnd) tail=buf;
    if(tail==head){
      if(filter(*head)) filterCount--;
      head++;if(head==bufEnd) head=buf;
    }
  }
};
