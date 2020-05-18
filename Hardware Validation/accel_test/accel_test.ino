/*
 * This is a test that checks both accelerometers
 */

#include <Wire.h> // I2C library
#include <SparkFunLSM9DS1.h>  // SparkFun LSM9DS1 library

LSM9DS1 imu_1;    // Accel #1
LSM9DS1 imu_2;    // Accel #2

void setup() {
  Serial.begin(9600); // Begin serial
  Wire.begin();       // Begin I2C


  // Init IMU1

  if (!imu_1.begin(0x6B, 0x1E, Wire))
  {
    Serial.println("LSM9DS1 #1 initialization failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("LSM9DS1 #1 initialization successful!");
  }
  // Sets IMU1's scale to +-16g
  imu_1.setAccelScale(16);

  // Init IMU2


  if (!(imu_2.begin(0x6A, 0x1C, Wire)))
  {
    Serial.println("LSM9DS1 #2 initialization failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("LSM9DS1 #2 initialization successful!");
  }
  // Sets IMU1's scale to +-16g
  imu_2.setAccelScale(16);
}


char out_1[64];
char out_2[64];

void loop() {
  // Read IMU1
  if (imu_1.accelAvailable())
  {
    imu_1.readAccel();
  }
  double x, y, z;
  x = imu_1.calcAccel(imu_1.ax);
  y = imu_1.calcAccel(imu_1.ay);
  z = imu_1.calcAccel(imu_1.az);
  sprintf(out_1, "accel_1: %f, %f, %f    ", x, y, z);
  Serial.print(out_1);

  // Read IMU2
  if (imu_2.accelAvailable())
  {
    imu_2.readAccel();
  }
  x = imu_2.calcAccel(imu_2.ax);
  y = imu_2.calcAccel(imu_2.ay);
  z = imu_2.calcAccel(imu_2.az);
  sprintf(out_2, "accel_2: %f, %f, %f\n", x, y, z);
  Serial.print(out_2);

  delay(100);
}
