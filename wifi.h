inline String read_file(String file) {
  File f = SPIFFS.open(file, "r");
  if(!f) Serial.println("file not found: "+ file);
  String s = f.readString();
  f.close();
  return s;
}

String ip_str(IPAddress ip) {
  return String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
}

class WIFI {
  public:
  String sn, hostname, mac, local_ip, ssid, mode, pw;
  
  uint8_t setup() {
    /*
    String s = read_file("/wifi.txt");
    int sep = s.indexOf('\n');
    ssid = s.substring(0, sep);
    ssid.trim();
    pw = s.substring(sep+1);
    pw.trim();*/

    char30 s, p;
    EEPROM.get(EEPROM_SSID, s);
    EEPROM.get(EEPROM_PW, p);
    ssid = String(s.str);
    pw = String(p.str);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pw);
    Serial.print("[wifi] STA mode");
    int count = 5;
    while(count -- >0) {
      if(WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
      } else {      
        break;
      }
    }
    
    mac = WiFi.macAddress();
    sn = mac.substring(12);
    sn.replace(":", "");
    sn.toLowerCase();
    hostname = "rcute-cube-" + sn;
    
    if(WiFi.status() == WL_CONNECTED) {
      local_ip = ip_str(WiFi.localIP());
      mode = "STA";
      Serial.println("Connected: " + local_ip);
    } else {
      Serial.println("Disabled");          
      WiFi.mode(WIFI_AP);            
      WiFi.softAP(hostname, sn+sn);
      mode = "AP";
      local_ip = ip_str(WiFi.softAPIP());
      Serial.println("[wifi] AP mode enabled: " + local_ip);
    }
    
    Serial.println("[mDNS] " + String(MDNS.begin(hostname)? hostname+".local": "error"));
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("ws", "tcp", 81);
    return mode=="STA"?0:1;
  }

  void save(String _ssid, String _pw) {
    /*
     File file = SPIFFS.open("/wifi.txt", "w");
     file.println(_ssid + "\n" + _pw);
     file.close();*/
     char30 s, p;
     memcpy(s.str, _ssid.c_str(), _ssid.length()+1);
     memcpy(p.str, _pw.c_str(), _pw.length()+1);
     EEPROM.put(EEPROM_SSID, s);
     EEPROM.put(EEPROM_PW, p);
     EEPROM.commit();
     Serial.println("[wifi] saved: " + _ssid + "/" + _pw);
     ssid =_ssid;
     pw =_pw;
  }  

  void loop() {    
    MDNS.update();
  }
};
