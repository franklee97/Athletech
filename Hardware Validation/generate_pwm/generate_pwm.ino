
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  ledcAttachPin(16, 1);
  ledcSetup(1, 12000, 8);
  pinMode(4, INPUT);
  
}
int pwm = 0;
void loop() {
  // put your main code here, to run repeatedly:
  
  ledcWrite(1, pwm);
  
  if (pwm == 255)
  {
    pwm = 0;
  }
  pwm++;

  //Serial.println(analogRead(4));
  Serial.println(pwm);
  delay(200);
}
