/*
 * This is a simple hello world script for ESP32
 * It prints "Hello World" every second
 */

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hello World");
  delay(1000);
}
