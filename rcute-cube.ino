#include <FS.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "msg_type.h"
#include "my_mpu6050.h"
#include "wifi.h"
#include "led.h"
#include "upgrade_helper.h"
#include "version.h"

ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
#define BUF_SIZE 200
StaticJsonDocument<BUF_SIZE> recv_doc;
StaticJsonDocument<BUF_SIZE> send_doc;
StaticJsonDocument<BUF_SIZE> data_doc;
StaticJsonDocument<BUF_SIZE> event_doc;
JsonArray data_trunck_array, event_trunck_array;
uint8_t buf[BUF_SIZE];
UpgradeHelper upgrade;
RGB rgb;
WIFI wifi;
MyMPU6050 mpu;
uint8_t mpuStarted;
long mpu_event_msgid, mpu_data_msgid, mpu_data_start;
uint8_t client_id; 

void serialize_send(StaticJsonDocument<BUF_SIZE>& doc){
  serializeMsgPack(doc, buf, BUF_SIZE);
  webSocket.sendBIN(client_id, buf, measureMsgPack(doc));
}

void send_response(char* err, char* result) {
  send_doc[2] = err;
  send_doc[3] = result;
  serialize_send(send_doc);
}
 
void send_redirect(const String& txt, const String& url="/", const String& sec="8") {
  server.send(200, "text/html", "<html><head><meta charset='utf-8'/><meta http-equiv='refresh' content='"+sec+";url="+url+"'/></head><body>"+txt+"</body></html>");
}

void send_body(const String& txt) {
  server.send(200, "text/html", "<html><head><meta charset='utf-8'/></head><body>"+txt+"</body></html>");
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  DeserializationError err;
  JsonArray params;
  const char* method;
  long msgid;
  int msgtype;
  switch(type) {
    case WStype_BIN:
//      Serial.printf("[ws] recv bin [%u]: ", num); // debug print
      err = deserializeMsgPack(recv_doc, payload, length);
      if(err) {
        Serial.print("deserializeMsgPack() failed: ");
        Serial.println(err.c_str());
      } else {
        msgtype = recv_doc[0];
        msgid = recv_doc[1];
        switch(msgtype) { // mtype
          case MTYPE_REQUEST:
          case MTYPE_NOTIFY:
            send_doc.clear();
            send_doc[0] = MTYPE_RESPONSE;
            send_doc[1] = msgid;
            method = recv_doc[2];
            params = recv_doc[3].as<JsonArray>();
//            Serial.printf("[rpc] %d, %ld, %s, (%d)[...]\n", msgtype, msgid, method, params.size()); // debug print
            
            if(strcmp(method, "rgb")==0) {
              rgb.rgb(params[0], params[1], params[2]);
              send_response(NULL, NULL);
            }else if(strcmp(method, "led")==0) {
              rgb.led(params[0]);
              send_response(NULL, NULL);
            }/*else if(strcmp(method, "mpu_cali")==0) {
              mpu.calibrate();
              send_response(NULL, NULL);
            }else if(strcmp(method, "mpu_sleep")==0) {
              mpu.enable(params[0]);
              send_response(NULL, NULL);
            }else if(strcmp(method, "mpu_acc_range")==0) {
              mpu.setAccelerometerRange(params[0]);
              send_response(NULL, NULL);
            }else if(strcmp(method, "mpu_gyro_range")==0) {
              mpu.setGyroRange(params[0]);
              send_response(NULL, NULL);
            }else if(strcmp(method, "mpu_filter_band")==0) {
              mpu.setFilterBandwidth(params[0]);
              send_response(NULL, NULL);
            }*/else if(strcmp(method, "mpu_static")==0) {
              send_doc[2] = (char*)NULL;
              send_doc[3] = mpu.state==STATIC;
              serialize_send(send_doc);
            }else if(strcmp(method, "mpu_raw")==0) {
              data_doc[1] = mpu_data_msgid = msgid;
              mpu_data_start = millis();
              mpu.cbUpdate = mpuUpdate;
            }else if(strcmp(method, "mpu_event")==0) {
              event_doc[1] = mpu_event_msgid = msgid;
              mpu.cbEvent = mpuEvent;
            }else if(strcmp(method, "mpu_acc")==0) {
              send_doc[2] = (char*)NULL;
              JsonArray ja = send_doc.createNestedArray();
              ja[0]=-mpu.currData->acc.x;ja[1]=-mpu.currData->acc.y;ja[2]=-mpu.currData->acc.z;
              serialize_send(send_doc);
            }else {
              send_response("Unknown method", NULL);
            }
            break;
            
          case MTYPE_REQUEST_CANCEL:
//            Serial.printf("[rpc] %d, %ld\n", msgtype, msgid); // debug print
            if(msgid== mpu_data_msgid) mpu.cbUpdate = NULL;
            else if(msgid== mpu_event_msgid) mpu.cbEvent = NULL;
            break;
        }
      }
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[ws] disconn (%u)\n", num);
      mpu.cbUpdate = NULL;
      mpu.cbEvent = NULL;
      mpu.sleep(true);
      break;
    case WStype_CONNECTED:
      Serial.printf("[ws] conn (%u): %s\n", num, ip_str(webSocket.remoteIP(num)).c_str());
      if(webSocket.connectedClients()>1){
        webSocket.sendTXT(num, "-1");
        webSocket.disconnect(num);
      }else{
        webSocket.sendTXT(num, "0");
        client_id = num;
        mpuStarted = mpu.setup(&event_trunck_array);
        if(mpuStarted) {
          Serial.printf("MPU error %d\n", mpuStarted);
          rgb.blink_rgb(0, 0, 100, mpuStarted, 250, 750);
        }
//        mpu.sleep(false);      
      }
      break;
    case WStype_TEXT:
      Serial.printf("[ws] recv text (%u): %s\n", num, payload);      
      break;
  }
}

void send_file(const String& file, const String& type="text/html") {
  Serial.printf("[server] req: %s\n", file.c_str());
  File f = SPIFFS.open(file, "r");
  server.streamFile(f, type);
  f.close();
}

typedef void (*TemplateHandler)(String& s);

void send_template(const TemplateHandler& fn, const String& file, const String& type="text/html") {
  Serial.printf("[server] tmp: %s\n", file.c_str());
  String s = read_file(file);
  fn(s);
  server.send(200, type, s);  
}

void handle_index_template(String& html) {
  html.replace("{ssid}", wifi.ssid);
  html.replace("{hostname}", wifi.hostname);
  html.replace("{mac}", wifi.mac);
  html.replace("{local_ip}", wifi.local_ip);
  html.replace("{sn}", wifi.sn);
  html.replace("{wifi}", wifi.mode);
  html.replace("{version}", VERSION);
  html.replace("{upgrade_log}", upgrade.brief_upgrade_log());
}

void handle_save_wifi() {
  Serial.println("[server] visit: /save_wifi");
  String ssid = server.arg("ssid"), pw = server.arg("pw");
  String ret;
  if(pw.length()<8) {
    ret = String("错误： 密码少于八位");
  } else if(ssid.length()==0) {
    ret = String("错误： SSID为空");
  } else {
    wifi.save(ssid, pw);
    ret = String("wifi 设置已保存，重启后生效<br><form action='reboot'><input type='submit' value='重启'/></form>");    
    rgb.blink_rgb(0, 100, 0, 1, 250);
  }
  send_body(ret); 
}

void handle_save_cali() {
  String offsetStr = server.arg("offset");
  int offset[6];
  int ind, from=0;
  for(uint8_t i=0;i<6;i++){
    int ind = offsetStr.indexOf(',', from);
    offset[i] = (ind==-1?offsetStr.substring(from):offsetStr.substring(from, ind)).toInt();
    from = ind+1;
  }
  mpu.saveOffsets(offset);
  server.send(200, "text/plain", "已保存");
}

void handle_upgrade() {
  String sketch_url = server.arg("sketch");
  String spiffs_url = server.arg("spiffs");
  int retry= server.arg("retry").toInt();
  if(!retry) retry=3;
  Serial.printf("[server] visit /upgrade\n[upgrade] sketch=%s, spiffs=%s, retry=%d", sketch_url.c_str(), spiffs_url.c_str(), retry);  
  send_redirect("正在更新，可能需要一两分钟...", String("upgrade_ret?sketch=")+(sketch_url.length()>0?"1":"0")+"&spiffs="+(spiffs_url.length()>0?"1":"0"), "30");
  if(sketch_url.length())
	  if(HTTP_UPDATE_OK != upgrade.try_upgrade(sketch_url, "sketch", retry)) return;
  if(spiffs_url.length()){
    int offset[3];
    bool readoffset = mpu.readOffsets(offset);
	  if(HTTP_UPDATE_OK == upgrade.try_upgrade(spiffs_url, "spiffs", retry) && readoffset) {wifi.save(wifi.ssid, wifi.pw); mpu.saveOffsets(offset);}
  }
}

void handle_upgrade_ret() {
  if((server.arg("sketch")=="1" && upgrade.sketch_upgrade_ret!=HTTP_UPDATE_OK) || (server.arg("spiffs")=="1" && upgrade.spiffs_upgrade_ret!=HTTP_UPDATE_OK))
    send_body("更新未完成，<a href='/'>返回");
  else
	  send_body("更新成功，重启后生效，<a href='/'>返回</a><br><br><form action='reboot'><input type='submit' value='重启'/></form>");
}

void mpuEvent() {
  serializeMsgPack(event_doc, buf, BUF_SIZE);
  webSocket.sendBIN(client_id, buf, measureMsgPack(event_doc));
}

void mpuUpdate() {
  for(uint8_t i=0;i<6;i++)
    data_trunck_array[i]=mpu.raw[i];
  data_trunck_array[6]=(mpu.rawUpdateTime-mpu_data_start)/1000.0f;
  serializeMsgPack(data_doc, buf, BUF_SIZE);
  webSocket.sendBIN(client_id, buf, measureMsgPack(data_doc));
}

void setup() {
  //return msgpack 
  data_doc[0] = event_doc[0] =MTYPE_RESPONSE_STREAM_CHUNCK;
  data_doc[1] = event_doc[1] =0l;
  data_trunck_array = data_doc.createNestedArray();
  event_trunck_array = event_doc.createNestedArray();
  rgb.setup();
  rgb.led(true);
  Serial.begin(115200);  
  Serial.println(); 
  if(!SPIFFS.begin()) {
    Serial.println("FPIFFS error");
    rgb.blink_rgb(100, 0, 0, 2, 250, 750);
  }
  rgb.blink_rgb(0, 100, 0, wifi.setup(), 250, 750);
  server.on("/", [](){send_template(handle_index_template, "/index.tmpl");});
  server.on("/save_wifi", handle_save_wifi);
  server.on("/test", [](){send_file("/test.html");});
  server.on("/wsmprpc.client.js", [](){send_file("/wsmprpc.client.js", "text/javascript");});
  server.on("/msgpack.min.js", [](){send_file("/msgpack.min.js", "text/javascript");});
  server.on("/canvasjs.min.js", [](){send_file("/canvasjs.min.js", "text/javascript");});
  server.on("/reboot", [](){send_redirect("正在重启...");delay(1000);ESP.restart();});
  server.on("/upgrade", handle_upgrade);
  server.on("/upgrade_ret", handle_upgrade_ret);
  server.on("/upgrade_log", [](){send_file("/upgrade_log.txt", "text/plain");});
  server.on("/clear_upgrade_log", [](){upgrade.clear_log();});
  server.on("/format_fs", [](){send_body("Format SPIFFS..." + String(SPIFFS.format()? "completed": "error"));});
  server.on("/cali", [](){send_file("/cali.html");});
  server.on("/save_cali", handle_save_cali);
  server.on("/about", [](){send_template(handle_index_template, "/about.tmpl", "application/json");});
  server.begin();
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
//  mpuStarted = mpu.setup(&event_trunck_array);
//  if(mpuStarted) {
//    Serial.printf("MPU error %d\n", mpuStarted);
//    rgb.blink_rgb(0, 0, 100, mpuStarted, 250, 750);
//  }
  // there should be no delay after mpu starts and immediately goto loop()
  rgb.led(false);
}
void loop() {
  mpu.loop();
  server.handleClient();
  webSocket.loop();
  wifi.loop();  
}
