#include <WiFi.h>


const char* ssid = "YinzCam-5G";
const char* password = "glenplaid84patiobuggy";

void WIFISetup(const char* deviceName){
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(deviceName);
  WiFi.begin(ssid, password);

  //pinMode(2, OUTPUT);
  
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
}
