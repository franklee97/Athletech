
#include <MPU9255.h>// include MPU9255 library

MPU9255 mpu;


void setup() {
  Serial.begin(115200);// initialize Serial port

  if(mpu.init())
  {
    Serial.println("initialization failed");
  }
  else
  {
    Serial.println("initialization successful!");
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
  //X axis
  Serial.print("AX: ");
  Serial.print(scaleCalc(mpu.ax));

  

  //Y axis
  Serial.print("  AY: ");
  Serial.print(scaleCalc(mpu.ay));

  //Z axis
  Serial.print("  AZ: ");
  Serial.print(scaleCalc(mpu.az));


  Serial.println();
  delay(100);
}
