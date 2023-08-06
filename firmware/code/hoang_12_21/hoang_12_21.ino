#include <Wire.h>
#include <Kalman.h>  //https://github.com/TKJElectronics/KalmanFilter
#define ToDeg 180 / PI
#define ToRad PI / 180

Kalman kalman;  //Kalman filter define: kalman.getAngle(pitch, gyrorate, dt);

int inChar;
uint32_t timerloop, timerold;

//Motor control Pin//

int leftpwm = 9;   //Control PWM left motor
int leftdir = 37;   //Control direction left motor
int leftdir2 = 35;  //Control direction left motor

int righpwm = 10;    //Control PWM right motor
int righdir = 31;   //Control direction right motor
int righdir2 = 33;  //Control direction right motor

volatile long leftencoder = 0;  //Read left encoder value
volatile long righencoder = 0;  //Read right encoder value
// INITIALIZE ENCODER PIN
int leftencoder_a = 2;  //Read state encoder channel LOW or HIGH
int leftencoder_b = 5;
int righencoder_a = 3;
int righencoder_b = 6;

//MPU6050 Data//
double mpudata;  //Save psi angle (Y axis)
double accX, accZ;
float Gyro;
int t;
uint32_t timer;  //Timer for kalman filter psi angle;
uint8_t i2cData[14];
String Data = "";
//LQR data//
long PWML, PWMR;                   //PWM output for H-Brigde
float K1, K2, K3, K4, K5, K6, ki;  //The factor of K maxtrix
bool falldown;                     //Run = true; Stop = false;

float theta = 0, psi = 0, phi = 0;
float thetadot, psidot, phidot;
float thetaold = 0, psiold = 0, phiold = 0, theta_p = 0;


float addtheta,addphi;
float leftvolt;  //output volt left motor in LQR
float righvolt;  //output volt right motor in LQR
char data[20];
/////////////////////////////////////////////////
///////////    SERIAL BEGIN   ///////////////////
/////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  //  K1 = 10.1;//9.78;//
  //  K2 = 32.2;//18.8;//
  //  K3 = 2132.36;//502.58;
  //  K4 = 232.8;//93.8;
  //  K5 = 8;//10.78;//11;
  //  K6 = 42;//32.3;//26;
  //  ki = 0;
  K1 = 7.1; //9.78;//
  K2 = 36.2; //18.8;//
  K3 = 702.36; //502.58;
  K4 = 188.8; //93.8;
  K5 = 16.1; //10.78;//11;
  K6 = 25.3; //32.3;//26;
  ki = 0;
  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  //Set PWM frequency 23429.4Hz f_PWM = 12e6/(510*N) = 23429.4Hz
  // Phase Correct PWM
  // Fast PWM
  //  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2B = TCCR2B & B11111000 | B00000001;  //Pin 9 & Pin 10
  //  Timer1.initialize(10000);//10us
  //  Timer1.attachInterrupt(NGAT);
  //Output pin control motor left and right//
  pinMode(leftdir, OUTPUT);
  pinMode(righdir, OUTPUT);
  pinMode(leftdir2, OUTPUT);
  pinMode(righdir2, OUTPUT);

  //Input pin read encorder//
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT_PULLUP);

  //interrupt encoder//
  attachInterrupt(0, left_isr, RISING);
  attachInterrupt(1, righ_isr, RISING);

  //Data MPU6050//
  Wire.begin();

  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Set I2C frequency to 400kHz
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false))
    ;
  while (i2cWrite(0x6B, 0x01, true))
    ;
  while (i2cRead(0x75, i2cData, 1))
    ;
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1)
      ;
  }

  delay(100);

  while (i2cRead(0x3B, i2cData, 6))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  kalman.setAngle(pitch);
  kalman.setQangle(0.0000085);
  kalman.setQbias(0.000005);
  kalman.setRmeasure(0.0009);
  timer = micros();
}

//////////////////////////////////
//       MAIN PROGRAMMING       //
//////////////////////////////////
void loop() {

  readmpu();
  if ((micros() - timerloop) > 6000) {                   //Set time loop update and control motor
    theta = gettheta(leftencoder, righencoder) * ToRad;  //Read theta value and convert to Rad
    psi = (mpudata + 4.6) * ToRad;                         //Read psi value and convert to Rad
    phi = getphi(leftencoder, righencoder) * ToRad;      //Read phi value and convert to Rad
    //Update time compare with timeloop
    double dt = (float)((micros() - timerloop)) / 1000000.0;
    timerloop = micros();
    //Update input angle value
    thetadot = (theta - thetaold) / dt;
    theta_p = (theta + thetaold) * ki * dt;
    psidot = (psi - psiold) / dt;
    phidot = (phi - phiold) / dt;
    //Update old angle value
    thetaold = theta;
    psiold = psi;
    phiold = phi;

    getlqr(theta-addtheta, thetadot, psi, psidot, phi-addphi, phidot, theta_p);
    motorcontrol(PWML, PWMR, (mpudata + 4.6), falldown);
    //Send data to serial
    //    Serial.print(psi * ToDeg);
    //    Serial.print(" ");
    //    Serial.print( phi* ToDeg);
    //    Serial.print(" ");
    //    Serial.println(theta * ToDeg);

//    Data = (String)(psi * ToDeg) +  ',' + (String)(theta * ToDeg)  +  ',' + (String)(phi * ToDeg) +  ',' + (String)(addtheta * ToDeg)+  ',' + (String)(addphi * ToDeg) ;

    Data = (String)(theta * ToDeg) +  ',' +(String)(phi * ToDeg);
    Serial.println(Data);

  }
}
//void serialEvent2() {
//  String test1 = "";
//  if (Serial2.available() > 0) {
//    test1 = Serial2.readStringUntil(',');  // read theta1
//    if (test1 != " ") {
//      K1 = (float)(test1.toFloat());
//    }
//
//    test1 = Serial2.readStringUntil(',');  // read theta2
//    if (test1 != " ") {
//      K2 = (float)(test1.toFloat());
//    }
//    test1 = Serial.readStringUntil(',');  // read theta1
//    if (test1 != " ") {
//      K3 = (float)(test1.toFloat());
//    }
//
//    test1 = Serial2.readStringUntil(',');  // read theta2
//    if (test1 != " ") {
//      K4 = (float)(test1.toFloat());
//    }
//    test1 = Serial2.readStringUntil(',');  // read theta1
//    if (test1 != " ") {
//      K5 = (float)(test1.toFloat());
//    }
//
//    test1 = Serial2.readStringUntil(',');  // read theta2
//    if (test1 != " ") {
//      K6 = (float)(test1.toFloat());
//    }
//  }
//}
//void serialEvent2() {
//  if (Serial2.available() > 0) {
//    char idataRx = Serial2.read();
//    if (idataRx == 85)
//    {
//      Serial.println(idataRx); addtheta += 5;
//    }
//    if (idataRx == 68)
//    {
//      Serial.println(idataRx); addtheta -= 5;
//    }
//    if (idataRx == 82)
//    {
//      Serial.println(idataRx); addphi += 5;
//    }
//    if (idataRx == 76)
//    {
//      Serial.println(idataRx); addphi += 5;
//    }
//    Serial.println(String(addphi) + ',' + String(addtheta));
//  }
//}
//left motor encoder interrupt//
void left_isr() {
  if (digitalRead(leftencoder_b) == HIGH) leftencoder++;
  else leftencoder--;
}

//right motor encoder interrupt//
void righ_isr() {
  if (digitalRead(righencoder_b) == HIGH) righencoder++;
  else righencoder--;
}

//Read psi//
void readmpu() {
  while (i2cRead(0x3B, i2cData, 14))
    ;
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);

  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double Gyrorate = Gyro / 131.0;
  mpudata = kalman.getAngle(pitch, Gyrorate, dt) - 1.23;

  if (abs(mpudata) > 30) {
    falldown = true;
  } else {
    falldown = false;
  }
  //
}

//Read theta angle function
float gettheta(long lencoder, long rencoder) {
  float angle = 0.5 * (360 / (11 * 30)) * (lencoder - rencoder);
  return angle;
}

//Read phi angle function//
float getphi(long lencoder, long rencoder) {
  float angle = (3.4 / 22.5) * (360 / (11 * 30)) * (- rencoder - lencoder );
  return angle;
}

//LQR function
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_, float theta_p) {
  righvolt = (K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ - K5 * phi_ - K6 * phidot_ + theta_p);
  leftvolt = (K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ + K5 * phi_ + K6 * phidot_ + theta_p);
  int cap_xung = 180;
  PWML = map(leftvolt, -K3 * PI / 15, K3 * PI / 15, -cap_xung, cap_xung);  //Limit 20 deg.
  PWMR = map(righvolt, -K3 * PI / 15, K3 * PI / 15, -cap_xung, cap_xung);

  PWML = constrain(PWML, -cap_xung, cap_xung);  // SATURATION
  PWMR = constrain(PWMR, -cap_xung, cap_xung);
}

//Motor control function
void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  if (abs(angle) > 30)  //angle psi > 30 motor will stop
  {
    stopandreset();
  } else {
    if (leftvolt > 0) {
      leftmotor(abs(lpwm), 1);  //Forward
    } else if (leftvolt < 0) {
      leftmotor(abs(lpwm), 0);  //Back
    } else {
      stopandreset();
    }
    //
    if (righvolt > 0) {
      righmotor(abs(rpwm), 1);
    } else if (righvolt < 0) {
      righmotor(abs(rpwm), 0);
    } else {
      stopandreset();
    }
  }
}

//Stop motor and reset data
void stopandreset()  //The data angle and encoder will be reset back to zero.
{
  analogWrite(leftpwm, 0);
  digitalWrite(leftdir, LOW);
  digitalWrite(leftdir2, LOW);
  analogWrite(righpwm, 0);
  digitalWrite(righdir, LOW);
  digitalWrite(righdir2, LOW);
  //Reset default place
  leftencoder = 0;
  righencoder = 0;
}
//Control left motor
void leftmotor(uint8_t lpwm, int direct) {
  analogWrite(leftpwm, lpwm);
  if (direct == 1) {  //angle > 0
    digitalWrite(leftdir, LOW);
    digitalWrite(leftdir2, HIGH);
  } else {
    digitalWrite(leftdir, HIGH);
    digitalWrite(leftdir2, LOW);
  }
}
//Control right motor
void righmotor(uint8_t rpwm, int direct) {
  analogWrite(righpwm, rpwm);
  if (direct == 1) {  //angle > 0
    digitalWrite(righdir, LOW);
    digitalWrite(righdir2, HIGH);
  } else {
    digitalWrite(righdir, HIGH);
    digitalWrite(righdir2, LOW);
  }
}
