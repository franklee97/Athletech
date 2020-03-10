#include <PubSubClient.h>
// LED Pin
const int ledPin = 4;
const char* mqtt_server = "10.242.1.54";
WiFiClient espClient;
PubSubClient client(espClient);

void MQTTSetup(const char* deviceName){
  long lastMsg = 0;
  char msg[50];
  int value = 0;
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  }

  void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if (String(topic) == "esp32/output") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
      digitalWrite(ledPin, HIGH);
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void MQTTLoop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
 void MQTTPublish(char* dest, char* reading)
 {
    //char humString[8];
    //dtostrf(reading, 1, 2, humString);
    client.publish(dest, reading);
 }
 
