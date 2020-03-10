#include <SD.h>   // include SD library
#include <MPU9255.h>// include MPU9255 library
#include <SPI.h>

MPU9255 mpu;
File myFile;
int num_files = 0;
String file_name;

void setup() {
  Serial.begin(115200);// initialize Serial port
  
  // initialize accelerometer
  if(mpu.init())
  {
    Serial.println("Accel initialization failed");
  }
  else
  {
    Serial.println("Accel initialization successful!");
  }
  // Set range
  mpu.set_acc_scale(scale_16g);

  // initializa SD card
  pinMode(10, OUTPUT);  // Setting Chip Select pin as output(SD card use SPI
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

  // count the number of file in current folder
  File root;
  root = SD.open("/");
  numOfFiles(root);

  // Creat file with unique id
  file_name = "sample" + String(num_files) + ".csv";

  Serial.println("");
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

void numOfFiles(File dir) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    num_files++;
    Serial.print(entry.name());
    Serial.println();
    entry.close();
  }
  Serial.println("Total number of files is: ");
  Serial.print(num_files);
}

void loop() {
  //take readings
  mpu.read_acc();

  // Read the input on analog pin 0
  //int fsr_val = analogRead(A0);
  
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


  //Serial.print("  Pressure: ");
  //Serial.print(fsr_val);

  myFile = SD.open(file_name, FILE_WRITE);
  if (myFile)
  {
    myFile.print(x);
    myFile.print(",");
    myFile.print(y);
    myFile.print(",");
    myFile.print(z);
    //myFile.print(",");
    //myFile.print(fsr_val);
    myFile.println();
    myFile.close();
  }
  else
  {
    Serial.println("error opening file!");
  }
  Serial.println();
  delay(20); // pause 20 ms -> T~=20ms -> f=1/T=50Hz
}
