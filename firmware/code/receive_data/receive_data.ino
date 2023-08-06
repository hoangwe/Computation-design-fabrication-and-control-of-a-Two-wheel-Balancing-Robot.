char data[42];
String text;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  while (!Serial)
  {
    ;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial2.available() > 0)
  {
    text = Serial2.read();
    text = String(text);



  }
  Serial.print(text);
  delay(100);


}
