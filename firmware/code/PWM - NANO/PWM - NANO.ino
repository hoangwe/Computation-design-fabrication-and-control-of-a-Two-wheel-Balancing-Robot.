#include <avr/interrupt.h>
#include <Arduino.h>
#include <TimerOne.h>
#include <WString.h>

#define PP 11
#define PG 10
#define NP 9
#define NG 8


String RXdataFull = "";
char inCome = "";
int8_t indexofS, indexofA;
String RXdataNumber = "";
int16_t pulseGUI = 0;
int16_t pulseEE = 0, pulseSV = 0;
float angleEE = 0, angleSV = 0;
int16_t angleEE_zoom = 0, angleSV_zoom = 0;
bool flag_Timer1 = false;
void setup() {
  Serial.begin(115200);
  pinMode(PP, OUTPUT);
  pinMode(PG, OUTPUT);
  pinMode(NP, OUTPUT);
  pinMode(NG, OUTPUT);

  digitalWrite(PP, 1);
  digitalWrite(NP, 1);
  digitalWrite(PG, 1);
  digitalWrite(NG, 1);

  //=====================Interrupts Joint===========
  pinMode(19, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  //=====================Interrupts AC Servo===========
  pinMode(18, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  //===================================================
  TCCR2B = TCCR2B & B11111000 | B00000001;

  Timer1.initialize(50000);
  Timer1.attachInterrupt(Timer1_ISR);

  attachInterrupt(digitalPinToInterrupt(18), Read_encoder_AC, FALLING);
  attachInterrupt(digitalPinToInterrupt(19), Read_encoder_Joint, FALLING);
}




void loop() {
  if (flag_Timer1 == true) {

    flag_Timer1 = false;
    bool flag_UART = false;

    if (Serial.available() > 0) {
      RXdataFull = Serial.readStringUntil('a');  //+xxxxa -> RXdataFull = +xxxx
      int END = Serial.read();
      pulseGUI = RXdataFull.toInt();
      flag_UART = true;
    }

    if (flag_UART == true) {
      flag_UART = false;
      PWM(pulseGUI, 100);
      angleSV = ((((float)pulseSV)*360*100)/400);  
      angleEE = ((((float)pulseEE)*360*100)/400); 
      Serial.println((String)pulseSV + "a" + (String)pulseEE + "b");
      // Serial.println((String)angleSV + "a" + (String)angleEE + "b");

    }
  }
}
void Timer1_ISR() {
  flag_Timer1 = true;
}

void PWM(int16_t pulseIN, uint16_t us) {
  if (pulseIN < 0) {
    for (uint16_t i = 0; i < abs(pulseIN); i++) {
      digitalWrite(NP, 0);
      delayMicroseconds(us);
      digitalWrite(NP, 1);
      delayMicroseconds(us);
    }
  }
  if (pulseIN > 0) {
    for (uint16_t i = 0; i < abs(pulseIN); i++) {
      digitalWrite(PP, 0);
      delayMicroseconds(us);
      digitalWrite(PP, 1);
      delayMicroseconds(us);
    }
  }
}
void Read_encoder_AC() {
  if (digitalRead(6) == LOW)
    pulseSV++;
  else
    pulseSV--;
}
void Read_encoder_Joint() {
  if (digitalRead(4) == LOW)
    pulseEE++;
  else
    pulseEE--;
}
