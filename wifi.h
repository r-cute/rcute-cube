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
    String s = read_file("/wifi.txt");
    int sep = s.indexOf('\n');
    ssid = s.substring(0, sep);
    ssid.trim();
    pw = s.substring(sep+1);
    pw.trim();
    
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
     File file = SPIFFS.open("/wifi.txt", "w");
     file.println(_ssid + "\n" + _pw);
     file.close();
     Serial.println("[wifi] saved: " + _ssid + "/" + _pw);
     ssid =_ssid;
  }

  void loop() {    
    MDNS.update();
  }
};
