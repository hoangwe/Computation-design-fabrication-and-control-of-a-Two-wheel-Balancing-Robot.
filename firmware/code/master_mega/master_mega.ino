/*
  Software serial multple serial test

  Receives from the hardware serial, sends to software serial.
  Receives from software serial, sends to hardware serial.

  The circuit:
   RX is digital pin 10 (connect to TX of other device)
   TX is digital pin 11 (connect to RX of other device)

  Note:
  Not all pins on the Mega and Mega 2560 support change interrupts,
  so only the following can be used for RX:
  10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

  Not all pins on the Leonardo and Micro support change interrupts,
  so only the following can be used for RX:
  8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
*/
#include<TimerOne.h>
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete
float t_count = 0;
bool flag_rec = false;

int bdn = 0;
//int idataRx;
void setup() {
  // initialize serial:
  Serial.begin(115200);
  Serial1.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(74);//33
  Timer1.initialize(15000);//10us
  Timer1.attachInterrupt(Timer1_ISR);

}

void loop() {
  // print the string when a newline arrives:
  //  if (Serial.available()) {
  //    Serial.write(Serial.read());
  //  }

  if (Serial.available() > 0)
  {
    int idataRx = Serial.read();
    Serial1.write(idataRx);
  }
  if (Serial1.available() > 0)
  {
    char inChar = Serial1.read();
    inputString += inChar;

    if (inChar == '\n') {
      stringComplete = true;
    }

  }
  if (t_count >= 10)
  {
    t_count=0;
    if (stringComplete == true) {
      Serial.print(inputString);
      inputString = "";
      stringComplete = false;
      // clear the string:

    }
  }

}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

//void serialEvent() {
//  while (Serial.available()) {
//    int idataRx = Serial.read();
//    Serial2.write(idataRx);
//  }
//}

//void serialEvent2() {
//  while (Serial2.available()) {
//    // get the new byte:
//    char inChar = Serial2.read();
//    inputString += inChar;
//    // if the incoming character is a newline, set a flag so the main loop can
//    // do something about it:
//    if (inChar == '\n') {
//      stringComplete = true;
//    }
//  }
//}
void Timer1_ISR()
{
  t_count ++;
}
