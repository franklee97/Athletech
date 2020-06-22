#include "SparkFun_LIS331.h"
#include <Wire.h>

LIS331 xl;

void setup() 
{
  
  // put your setup code here, to run once:
  pinMode(9,INPUT);       // Interrupt pin input
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  Wire.begin();
  LIS331::fs_range range = LIS331::LOW_RANGE;
  xl.setFullScale(range);
  xl.setI2CAddr(0x19);    // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
  
  Serial.begin(115200);
}
char out[128];
void loop() 
{
  static long loopTimer = 0;
  int16_t x, y, z;
  

  xl.readAxes(x, y, z);  // The readAxes() function transfers the
                         //  current axis readings into the three
                         //  parameter variables passed to it.
  sprintf(out, "%f, %f, %f", xl.convertToG(100,x), xl.convertToG(100,y), xl.convertToG(100,z));
  //Serial.println(xl.convertToG(400,x));
  Serial.println(out);

  delay(100);

}
