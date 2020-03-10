#include <Wire.h> // I2C library
#include <SparkFunLSM9DS1.h>  // SparkFun LSM9DS1 library
#include <MPU9255.h>          // MPU9255 library

const char* deviceName = "Accel_Sensor";  // Name for MQTT

LSM9DS1 imu_1;    // LSM9DS1 object
MPU9255 imu_2;    // MPU9255 object

void setup() {
  Serial.begin(9600);   // Begin serial
  Wire.begin();         // Begin I2C

  // Begin IMU1 init
  imu_1.settings.device.commInterface = IMU_MODE_I2C; // Set mode to I2C for LSM9DS1      
  imu_1.settings.device.agAddress = 0x6B; // Set ag address to 0x6B
  imu_1.settings.device.mAddress = 0x1E; // Set mag address to 0x1E
  if (!imu_1.begin())
  {
    Serial.println("LSM9DS1 initialization failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("LSM9DS1 initialization successful!");
  }
  // Sets IMU1's scale to +-16g
  imu_1.setAccelScale(16);
  // End IMU1 init

  // Begin IMU2 init
  if (imu_2.init())
  {
    Serial.println("MPU9265 initialization failed");
    while (1)
      ;
  }
  else
  {
    Serial.println("MPU9265 initialization sucessful!");
  }
  // Setting range
  imu_2.set_acc_scale(scale_16g);
  // End IMU2 init

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
  double x, y, z;
  x = imu_1.calcAccel(imu_1.ax);
  y = imu_1.calcAccel(imu_1.ay);
  z = imu_1.calcAccel(imu_1.az);
  sprintf(out_1, "1: %f, %f, %f", x, y, z);
  Serial.println(out_1);


  // Begin IMU2
  imu_2.read_acc();
  x = scaleCalc(imu_2.ax)/9.8;
  y = scaleCalc(imu_2.ay)/9.8;
  z = scaleCalc(imu_2.az)/9.8;
  sprintf(out_2, "2: %f, %f, %f", x, y, z);
  Serial.println(out_2);
  // End IMU2

  Serial.println();

  // Publish MQTT
  MQTTPublish(out_1);
  MQTTPublish(out_2);
  
  delay(100);

}

double scaleCalc(int input)
{
  double output;

  // 
  output = 32767 / 16;
  output = input / output;
  output = output * 9.8;
  return output;
}
