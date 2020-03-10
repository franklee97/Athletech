#include <SD.h>   // include SD library
#include <MPU9255.h>// include MPU9255 library
#include <SPI.h>

MPU9255 mpu;
File myFile;

void setup() {
  Serial.begin(115200);// initialize Serial port
  pinMode(10, OUTPUT);  // Setting Chip Select pin as output
  if(mpu.init())
  {
    Serial.println("Accel initialization failed");
  }
  else
  {
    Serial.println("Accel initialization successful!");
  }
  int SD_error = SD.begin(10);
  if (!SD_error)
  {
    Serial.println("SD card initialization failed!");
    Serial.println(SD_error, DEC);
    
  }
  else
  {
    Serial.println("SD card initialization done");
  }
  


  

  
  mpu.set_acc_scale(scale_16g);

}

double scaleCalc(int input)
{
  double output;

  output = 32767 / 16;
  output = input / output;
  output = output * 9.8;
  return output;
}

void loop() {
  //take readings
  mpu.read_acc();
  
  ////process and print acceleration data////

  double x, y, z;
  
  //X axis
  Serial.print("AX: ");
  x = scaleCalc(mpu.ax);
  Serial.print(x);
  //Y axis
  Serial.print("  AY: ");
  y = scaleCalc(mpu.ay);
  Serial.print(y);
  //Z axis
  Serial.print("  AZ: ");
  z = scaleCalc(mpu.az);
  Serial.print(z);
  
  myFile = SD.open("test.csv", FILE_WRITE);

  if (myFile)
  {
    myFile.print(x);
    myFile.print(",");
    myFile.print(y);
    myFile.print(",");
    myFile.print(z);
    myFile.println();
    myFile.close();
  }
  else
  {
    Serial.println("error opening test.txt");
  }
  Serial.println();
  delay(20);
}
