
const char* deviceName = "EMG_sensor";

// Input pins
#define sensorPin 4


//// Analog reading
int reading = 0;

void setup() {
   // declaration of pin modes
    pinMode(sensorPin, INPUT);    

  
  // begin sending over serial port
    Serial.begin(9600);
  WIFISetup(deviceName);
  MQTTSetup(deviceName);
}

void loop() { 
  
   MQTTLoop();
    reading= analogRead(sensorPin);

  // print out value over the serial port
    Serial.println(reading);
  
    // Convert the value to a char array
  MQTTPublish(reading);
  // wait for a bit to not overload the port
  delay(10000);

}
