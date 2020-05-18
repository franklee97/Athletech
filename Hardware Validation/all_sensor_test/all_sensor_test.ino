/*
 * This is a test that checks both accelerometers and al 6 analog inputs
 */

#include <Wire.h> // I2C library
#include <SparkFunLSM9DS1.h>  // SparkFun LSM9DS1 library

#define analog_1 36
#define analog_2 39
#define analog_3 34
#define analog_4 35
#define analog_5 32
#define analog_6 33

LSM9DS1 imu_1;    // Accel #1
LSM9DS1 imu_2;    // Accel #2

void setup() {
  Serial.begin(9600); // Begin serial
  Wire.begin();       // Begin I2C


  pinMode(analog_1, INPUT);
  pinMode(analog_2, INPUT);
  pinMode(analog_3, INPUT);
  pinMode(analog_4, INPUT);
  pinMode(analog_5, INPUT);
  pinMode(analog_6, INPUT);
  

  // Init IMU1
  imu_1.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C for LSM9DS1      
  imu_1.settings.device.agAddress = 0x6B; // Set ag address to 0x6B
  imu_1.settings.device.mAddress = 0x1E; // Set mag address to 0x1E
  if (!imu_1.begin())
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
  imu_2.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C for LSM9DS1      
  imu_2.settings.device.agAddress = 0x6A; // Set ag address to 0x6B
  imu_2.settings.device.mAddress = 0x1C; // Set mag address to 0x1E
  if (!imu_2.begin())
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
int val_1, val_2, val_3, val_4, val_5, val_6;
char out[256];

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

  
  // Read analog
  val_1 = analogRead(analog_1);
  val_2 = analogRead(analog_2);
  val_3 = analogRead(analog_3);
  val_4 = analogRead(analog_4);
  val_5 = analogRead(analog_5);
  val_6 = analogRead(analog_6);

  sprintf(out, "%d, %d, %d, %d, %d, %d\n", val_1, val_2, val_3, val_4, val_5, val_6);
  Serial.print(out);
  delay(200);

  delay(100);
}
