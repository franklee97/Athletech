#include <Wire.h> // I2C library
#include <SparkFunLSM9DS1.h>  // SparkFun LSM9DS1 library
#include "SparkFun_LIS331.h"  // SparkFun LIS331 library

const char* deviceName = "Accel_Sensor";  // Name for MQTT

LSM9DS1 imu_1;    // LSM9DS1 object
LIS331 imu_2;     // LIS331 object

void setup() {
  Serial.begin(9600);   // Begin serial
  Wire.begin();         // Begin I2C
  imu_1.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C for LSM9DS1

  LIS331::fs_range range = LIS331::LOW_RANGE;   
  imu_2.setFullScale(range);      // Sets LIS331 to LOW_RANGE(+-100g)

  // Sets I2C address and begins
  imu_2.setI2CAddr(0x19); 
  imu_2.begin(LIS331::USE_I2C); 
                          
  if (!imu_1.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Looping to infinity.");
    while (1)
      ;
  }

  // Sets IMU1's scale to +-16g
  imu_1.setAccelScale(16);

  // Begin WIFI
  WIFISetup(deviceName);
  MQTTSetup(deviceName);
}

// Character array for output
char out_1[128];
char out_2[128];

void loop() { 
  
  MQTTLoop();   // MQTT stuff

  // Begin IMU1
  if ( imu_1.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    //Serial.println("LSM9DS1's value is : ");
    imu_1.readAccel();
  }
  sprintf(out_1, "1: %f, %f, %f", (imu_1.calcAccel(imu_1.ax), 6), (imu_1.calcAccel(imu_1.ay), 6), (imu_1.calcAccel(imu_1.az), 6));
  Serial.println(out_1);
  // End IMU1

  // Begin IMU2
  int16_t x, y, z;
  imu_2.readAxes(x, y, z);  // The readAxes() function transfers the
                         //  current axis readings into the three
                         //  parameter variables passed to it.
  sprintf(out_2, "2: %f, %f, %f", imu_2.convertToG(100,x), imu_2.convertToG(100,y), imu_2.convertToG(100,z));
  Serial.println(out_2);
  // End IMU2
  
  Serial.println();

  // Publish MQTT
  MQTTPublish(out_1);
  MQTTPublish(out_2);
  
  delay(100);

}
