#include <SPI.h> // SPI library included for SparkFunLSM9DS1
#include <Wire.h> // I2C library included for SparkFunLSM9DS1
#include <SparkFunLSM9DS1.h> // SparkFun LSM9DS1 library
#include "SparkFun_LIS331.h"

// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

// Initialize LIS331 module
LIS331 xl;



void setup() {

  Serial.begin(9600);
  Wire.begin();
  imu.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C

  LIS331::fs_range range = LIS331::LOW_RANGE;
  
  xl.setFullScale(range);
  xl.setI2CAddr(0x19);    // This MUST be called BEFORE .begin() so 
                          //  .begin() can communicate with the chip
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.
                          
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1)
      ;
  }
  imu.setAccelScale(16);
}
char out[128];
void loop() {
  int16_t x, y, z;
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    Serial.println("LSM9DS1's value is : ");
    imu.readAccel();
  }
  printAccel();
  xl.readAxes(x, y, z);  // The readAxes() function transfers the
                         //  current axis readings into the three
                         //  parameter variables passed to it.
  Serial.println("H3LIS's value is : ");
  sprintf(out, "%f, %f, %f", xl.convertToG(100,x), xl.convertToG(100,y), xl.convertToG(100,z));
  //Serial.println(xl.convertToG(400,x));
  Serial.println(out);
  Serial.println();
  delay(100);
}

void printAccel()
{
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 6);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 6);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 6);
  Serial.println(" g");


}
