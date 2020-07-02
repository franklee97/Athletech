/*
 * This is a test to check all analog inputs
 */

#define analog_1 36
#define analog_2 39
#define analog_3 34
#define analog_4 35
#define analog_5 32
#define analog_6 33

void setup() {
  Serial.begin(115200);

  pinMode(analog_1, INPUT);
  pinMode(analog_2, INPUT);
  pinMode(analog_3, INPUT);
  pinMode(analog_4, INPUT);
  pinMode(analog_5, INPUT);
  pinMode(analog_6, INPUT);
}

int val_1, val_2, val_3, val_4, val_5, val_6;
char out[256];
void loop() {
  val_1 = analogRead(analog_1);
//  val_2 = analogRead(analog_2);
//  val_3 = analogRead(analog_3);
//  val_4 = analogRead(analog_4);
//  val_5 = analogRead(analog_5);
//  val_6 = analogRead(analog_6);

//  sprintf(out, "%d, %d, %d, %d, %d, %d\n", val_1, val_2, val_3, val_4, val_5, val_6);
  Serial.println(val_1);
  delay(1);
}
