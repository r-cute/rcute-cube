#include <ESP8266httpUpdate.h>

class UpgradeHelper{
public:

t_httpUpdate_return sketch_upgrade_ret=HTTP_UPDATE_NO_UPDATES, spiffs_upgrade_ret=HTTP_UPDATE_NO_UPDATES;

t_httpUpdate_return try_upgrade(String url, String type, int retry) {
  ESPhttpUpdate.rebootOnUpdate(false);
  ESPhttpUpdate.followRedirects(true);
  ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);    
  BearSSL::WiFiClientSecure sslClient;
  sslClient.setInsecure();
  WiFiClient client;
  t_httpUpdate_return ret;
  while(retry-- ){
    Serial.printf("[update] %s(%d): %s" , type.c_str(), 3-retry, url.c_str());
    if(type == "spiffs")
      spiffs_upgrade_ret = ret = ESPhttpUpdate.updateSpiffs(url.startsWith("https")?sslClient:client, url);
    else
      sketch_upgrade_ret = ret = ESPhttpUpdate.update(url.startsWith("https")?sslClient:client, url);
    switch(ret) {
    case HTTP_UPDATE_FAILED:
      append_log(type, "HTTP_UPDATE_FAILED Error (" +String(ESPhttpUpdate.getLastError())+ "): "+ ESPhttpUpdate.getLastErrorString());
      break;
    case HTTP_UPDATE_NO_UPDATES:
      append_log(type, "HTTP_UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      append_log(type, "HTTP_UPDATE_OK");
      return ret;
    }
  }
  return ret;
}

String brief_upgrade_log() {
  File f=SPIFFS.open("/upgrade_log.txt", "r");
  String line;
  bool sk=false, sp=false;
  while(f.available()){
    line = f.readStringUntil('\n');
    if(!sk && line.startsWith("sketch, HTTP_UPDATE_OK"))sk=true;
    if(!sp && line.startsWith("spiffs, HTTP_UPDATE_OK"))sp=true;
  }
  f.close();
  if(sk&& sp) return "both";
  else if(sk&& !sp) return "sketch";
  else if(!sk && sp) return "spiffs";
  return "";
}

void append_log(String type, String mes){
  File f=SPIFFS.open("/upgrade_log.txt", "w+");
  Serial.printf("%s, %s\n", type.c_str(), mes.c_str());
  f.printf("%s, %s\n", type.c_str(), mes.c_str());
  f.close();
}

void clear_log(){
  File f=SPIFFS.open("/upgrade_log.txt", "w");
  f.print("");
  f.close();
}
};
