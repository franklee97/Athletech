int sensorPin = 13; // select the input pin for the potentiometer
int sensorValue = 0; // variable to store the value coming from the sensor
 
void setup () 
{
  pinMode (sensorPin, OUTPUT);
}
 
void loop () 
{
  
  digitalWrite (sensorPin, HIGH);
  
}
