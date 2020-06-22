#include <Wire.h> // I2C library
#include <SparkFunLSM9DS1.h>  // SparkFun LSM9DS1 library
#include <MPU9255.h>          // MPU9255 library

const char* deviceName = "Both_Sensor";  // Name for MQTT

LSM9DS1 imu_1;    // LSM9DS1 object
MPU9255 imu_2;    // MPU9255 object

#define sensorPin 36    // EMG sensor pin
// Load sensor pins
#define loadPin_1 39
#define loadPin_2 34
#define loadPin_3 32



void setup() {
  Serial.begin(9600);   // Begin serial
  Wire.begin();         // Begin I2C

  pinMode(sensorPin, INPUT);  // declaration of pin mode
  pinMode(loadPin_1, INPUT);
  pinMode(loadPin_2, INPUT);
  pinMode(loadPin_3, INPUT);

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
/*
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
  */
}

// Character array for output
char out_emg[64];
char out_load[64];
char out_1[64];
char out_2[64];
char out[256];
char out_accel[128];

int reading = 0;        // Analog reading
int load_1 = 0;
int load_2 = 0;
int load_3 = 0;


void loop() { 
  /*
  MQTTLoop();   // MQTT stuff

  // Begin EMG
  reading = analogRead(sensorPin);
  sprintf(out_emg, "emg: %d\n", reading);
  //Serial.print(out_emg);
  // End EMG

  // Begin Load
  load_1 = analogRead(loadPin_1);
  load_2 = analogRead(loadPin_2);
  load_3 = analogRead(loadPin_3);
  sprintf(out_load, "load: %d, %d, %d\n", load_1, load_2, load_3);
  // End Load
 */
  // Begin IMU1
  if ( imu_1.accelAvailable() )
  {
    imu_1.readAccel();
  }
  double x, y, z;
  x = imu_1.calcAccel(imu_1.ax);
  y = imu_1.calcAccel(imu_1.ay);
  z = imu_1.calcAccel(imu_1.az);
  sprintf(out_1, "accel_1: %f, %f, %f\n", x, y, z);
  Serial.print(out_1);
  // End IMU1

/*
  // Begin IMU2
  imu_2.read_acc();
  x = scaleCalc(imu_2.ax)/9.8;
  y = scaleCalc(imu_2.ay)/9.8;
  z = scaleCalc(imu_2.az)/9.8;
  sprintf(out_2, "accel_2: %f, %f, %f\n", x, y, z);
  //Serial.print(out_2);
  // End IMU2

  */
/*
  // Publish MQTT
  
  sprintf(out, "%s%s%s%s", out_load, out_emg, out_1, out_2);
  //Serial.println(out);
  //Serial.println();
  //MQTTPublish(out);

  sprintf(out_accel, "%s%s", out_1, out_2);
  MQTTPublish("sportsLab/load/test", out_load);
  MQTTPublish("sportsLab/emg/test", out_emg);
  MQTTPublish("sportsLab/accel/test", out_accel);
  
  */
  delay(1);   

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
