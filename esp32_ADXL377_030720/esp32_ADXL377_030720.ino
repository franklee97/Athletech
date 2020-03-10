int scale = 200;
boolean micro_is_5V = false; 

void setup()
{
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
}

void loop()
{
  // Get raw accelerometer data for each axis
  int rawX = analogRead(15);
  int rawY = analogRead(13);
  int rawZ = analogRead(4);

  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  if (micro_is_5V) // microcontroller runs off 5V
  {
    scaledX = mapf(rawX, 0, 2703, -scale, scale); // 3.3/5 * 4095 =~ 675
  }
  else // microcontroller runs off 3.3V
  {
    scaledX = mapf(rawX, 0, 4095, -scale, scale);
    scaledY = mapf(rawY, 0, 4095, -scale, scale);
    scaledZ = mapf(rawZ, 0, 4095, -scale, scale);
  }
  /*
  // Print out raw X,Y,Z accelerometer readings
  Serial.print("X: "); Serial.println(rawX);
  // Print out scaled X,Y,Z accelerometer readings
  Serial.print("X: "); Serial.print(scaledX); Serial.println(" g");
  
  // Print out raw X,Y,Z accelerometer readings
  Serial.print("Y: "); Serial.println(rawY);
  // Print out scaled X,Y,Z accelerometer readings
  Serial.print("Y: "); Serial.print(scaledY); Serial.println(" g");
  
  // Print out raw X,Y,Z accelerometer readings
  Serial.print("Z: "); Serial.println(rawZ);
  // Print out scaled X,Y,Z accelerometer readings
  Serial.print("Z: "); Serial.print(scaledZ); Serial.println(" g");
  Serial.println();
  */

  Serial.print(scaledX); Serial.print(",");
  Serial.print(scaledY); Serial.print(",");
  Serial.print(scaledZ); 

  Serial.println();
  Serial.println();
  
  //delay(2000);

}

double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
