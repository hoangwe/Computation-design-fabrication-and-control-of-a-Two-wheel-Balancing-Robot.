#include <Wire.h>
#include <Kalman.h>
#include<TimerOne.h>
#define ToDeg 180/PI
#define ToRad PI/180
Kalman kalman;
static float xRx_, yRx_;
int mode = 0;
float xRx = 0, yRx = 0, tRx = 0, rRx = 0;
uint32_t timerloop, timerold;
long long bdn;
double bdn_c;
float x;
//Motor control Pin//
int leftpwm = 9; // using pwm2 timer2 with pin 9 & 10 is output.
int leftdir = 37;
int leftdir2 = 35;

int rightpwm = 10;
int rightdir = 31;
int rightdir2 = 33;

volatile float leftencoder = 0;
volatile float rightencoder = 0;

// INITIALIZE ENCODER PIN
#define leftencoder_a  2
#define leftencoder_b  5
#define rightencoder_a  3
#define rightencoder_b  6

//MPU6050 Data//
double mpudata; //Save psi angle (Y axis)
double accX, accZ;
float Gyro;

uint32_t timer;
uint8_t i2cData[14];

//LQR data//
int16_t PWML, PWMR; //PWM output for H-Brigde
float K1, K2, K3, K4, K5, K6;

bool falldown; //Run = true; Stop = false;
//   Angle
float theta, psi, phi;
// DERIVATIVE
float thetadot, psidot, phidot;
// UPDATE ANGLE
float thetaold = 0, psiold = 0, phiold = 0;
// U CONTROL
float leftvolt;
float rightvolt;
int n;
double pitch;
float addTheta, addPhi;
int idataRx;

//////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  /*
    //Set factor of K matrix
    K = [k1 k2 k3 k4  k5  k6;
         k1 k2 k3 k4 -k5 -k6]
  */
  // Initialize Parameters LQR Controller.
  n = 10;
  K1 = 9.78;//9.78;//
  K2 = 18.8;//18.8;//
  K3 = 502.58;//502.58;
  K4 = 93.8;//93.8;
  K5 = 8;//10.78;//11;
  K6 = 32.3;//32.3;//26;


  pinMode(leftpwm, OUTPUT);
  pinMode(rightpwm, OUTPUT);
  x = 4.6;
  //Set PWM frequency 31kHz f_PWM = 16e6/(510*N) = 32kHz
  // Phase Correct PWM and Non-Invese Mode
  TCCR2B = TCCR2B & B11111000 | B00000001; //Pin 9 & Pin 10
  //Output pin control motor left and right//
  pinMode(leftdir, OUTPUT);
  pinMode(rightdir, OUTPUT);
  pinMode(leftdir2, OUTPUT);
  pinMode(rightdir2, OUTPUT);
  //Input pin read encorder//
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT_PULLUP);
  pinMode(rightencoder_a, INPUT_PULLUP);
  pinMode(rightencoder_b, INPUT_PULLUP);
  // Timer1
  Timer1.initialize(1000); //T=0.01s
  Timer1.attachInterrupt(LAYMAU);
  //interrupt encoder//
  attachInterrupt(digitalPinToInterrupt(leftencoder_a), left_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(rightencoder_a), right_isr, RISING);
  //Data MPU6050//
  Wire.begin();
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100);

  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

  kalman.setQangle(0.0000085);
  kalman.setQbias(0.000005);
  kalman.setRmeasure(0.0009);

  kalman.setAngle(pitch);
  timer = micros();

  addTheta = 0;
  addPhi = 0;
}
void loop() {
  if (bdn >= n) {
    readmpu();
    //    if ((micros() - timerloop) > 6000) {
    //      double dt = (float)((micros() - timerloop)) / 1000000.0;
    //      timerloop = micros();
    bdn = 0;
    double dt = 0.001 * n;
    theta = gettheta(leftencoder, rightencoder) * ToRad;
    psi = (mpudata + x) * ToRad;
    phi = getphi(leftencoder, rightencoder) * ToRad;
    //Update time compare with timeloop
    //Update input angle value
    // CALCULATE LQR
    thetadot = (theta - thetaold) / dt;
    phidot = (phi - phiold) / dt;
    //Update old angle value
    thetaold = theta;
    phiold = phi;
    if (mode == 1) {
      addTheta = DHN_The(circle_x(rRx, xRx, tRx), circle_y(rRx, yRx, tRx));
      addPhi = DHN_Phi(circle_x(rRx, xRx, tRx), circle_y(rRx, yRx, tRx));
      if (bdn_c > tRx * 1000) {
        xRx_ = circle_x(rRx, xRx, tRx);
        yRx_ = circle_y(rRx, yRx, tRx);
      }
    } else if (mode == 2) {
      addTheta = DHN_The(line_x(xRx_, xRx, tRx), line_y(yRx_, yRx, tRx));
      addPhi = DHN_Phi(line_x(xRx_, xRx, tRx), line_y(yRx_, yRx, tRx));
      if (bdn_c > tRx * 1000) {
        xRx_ = line_x(rRx, xRx, tRx);
        yRx_ = line_y(rRx, yRx, tRx);
        mode = 0;
      }
    }
    if (abs(psi) > 30) falldown = true;
    else falldown = false;
    getlqr(theta - addTheta, thetadot, psi, psidot, phi - addPhi, phidot);
    motorcontrol(PWML, PWMR, mpudata + x, falldown);
    float px = DHT_x(theta, phi);
    float py = DHT_y(theta, phi);
    String Data = "";
    //    Data = (String)(psi * ToDeg) + ',' + (String)(theta * ToDeg) + ',' + (String)(phi * ToDeg) + ',' + (String)(addTheta * ToDeg) + ',' + (String)(addPhi * ToDeg);
    //    Data = Data + ',' + (String)px + ',' + (String)py + ',' + (String)xRx + ',' + (String)yRx + ',' + (String)(bdn_c/1000);
    Data = (String)(psi * ToDeg);
    Serial.println(Data);
    //    Serial1.println(Data);
  }
}
//left motor encoder interrupt//
void left_isr() {
  if (digitalRead(leftencoder_b) == HIGH) leftencoder++;
  else leftencoder--;
}
//right motor encoder interrupt//
void right_isr() {
  if (digitalRead(rightencoder_b) == HIGH) rightencoder--;
  else rightencoder++;
}
//Read psi//
void readmpu() {
  double dt = double(micros() - timer) / 1000000.0;
  timer = micros();
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]); //GyroY
  double pitch = (atan2(-accX, accZ)) * RAD_TO_DEG; // Eq. 29
  double Gyrorate = Gyro / 131.0; // Convert to deg/s
  psidot = Gyrorate * ToRad;
  mpudata = kalman.getAngle(pitch, Gyrorate, dt) - 1.23;
  if (abs(mpudata) > 30) falldown = true;
  else falldown = false;
}
//Read theta angle function
float gettheta(long lencoder, long rencoder) {
  //  double dt = 0.001 * n;
  float angle = 0.5 * (360 / (11 * 30)) * (lencoder - rencoder);
  return angle;
}
//Read phi angle function//
float getphi(long lencoder, long rencoder) {
  //  double dt = 0.001 * n;
  float angle = (3.4 / 22.5) * (360 / (11 * 30)) * (-lencoder - rencoder);
  return angle;
}
//LQR function u = -K*x;
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_) {
  rightvolt = (K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ - K5 * phi_ - K6 * phidot_);
  leftvolt = -(K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ + K5 * phi_ + K6 * phidot_);
  float limit = (K3 * PI / 15) ;
  PWML = map(leftvolt, -limit, limit, -180, 180); // MAPPING FROM U CONTROL SIGNAL TO PWM VALUE
  PWMR = map(rightvolt, -limit, limit, -180, 180); //

  PWML = constrain(PWML, -240, 240); // SATURATION
  PWMR = constrain(PWMR, -240, 240);
}

//Motor control function
void motorcontrol(long lpwm, long rpwm, float angle, bool stopstate) {
  if (stopstate == true) stopandreset();
  else {
    if (abs(angle) > 30) stopandreset();
    else
    {
      if (leftvolt > 0) leftmotor(abs(lpwm), 1); //Forward
      else if (leftvolt < 0) leftmotor(abs(lpwm), 0); //Back
      else stopandreset();
      if (rightvolt > 0) rightmotor(abs(rpwm), 1);
      else if (rightvolt < 0) rightmotor(abs(rpwm), 0);
      else stopandreset();
    }
  }
}
//Stop motor and reset data
void stopandreset()
{
  analogWrite(leftpwm, 0);
  digitalWrite(leftdir, LOW);
  digitalWrite(leftdir2, LOW);
  analogWrite(rightpwm, 0);
  digitalWrite(rightdir, LOW);
  digitalWrite(rightdir2, LOW);

  leftencoder = 0;
  rightencoder = 0;
  //Reset default place
  addTheta = 0;
  addPhi = 0;
}
//Control left motor
void leftmotor(uint8_t lpwm, int direct) {
  analogWrite(leftpwm, lpwm);
  if (direct == 1) {
    digitalWrite(leftdir, LOW);
    digitalWrite(leftdir2, HIGH);
  }
  else {
    digitalWrite(leftdir, HIGH);
    digitalWrite(leftdir2, LOW);
  }
}
//Control right motor
void rightmotor(uint8_t rpwm, int direct) {
  analogWrite(rightpwm, rpwm);
  if (direct == 1) {
    digitalWrite(rightdir, LOW);
    digitalWrite(rightdir2, HIGH);
  }
  else {
    digitalWrite(rightdir, HIGH);
    digitalWrite(rightdir2, LOW);
  }
}
void serialEvent1() {
  while (Serial1.available()) {
    idataRx = Serial1.read();
    if (idataRx == 70) addTheta += 5 * PI / 18;
    else if (idataRx == 66) addTheta -= 5 * PI / 18;
    else if (idataRx == 82) addPhi += PI / 3;
    else if (idataRx == 76) addPhi -= PI / 3;
    else if (idataRx == 72) {
      if (mode == 0) {
        addTheta = 0;
        addPhi = 0;
      } else if (mode == 1) {
        xRx = 0; yRx = 0;
        bdn_c = 0;
        mode = 2;
      } else if (mode == 2) {
        xRx = 0; yRx = 0;
        bdn_c = 0;
      }
    }
    else if (idataRx == 67) {
      xRx = Serial1.parseFloat();
      yRx = Serial1.parseFloat();
      tRx = Serial1.parseFloat();
      bdn_c = 0;
      mode = 2;
    }
    else if (idataRx == 65) {
      xRx = Serial1.parseFloat();
      yRx = Serial1.parseFloat();
      rRx = Serial1.parseFloat();
      tRx = Serial1.parseFloat();
      mode = 1;
      bdn_c = 0;
    }
  }
  idataRx = 0;
}
float circle_x(float rt, float xt, float t) {
  if (bdn_c <= t * 1000) {
    return xt + rt * cos(2 * PI * (bdn_c / 1000) / t);
  } else {
    bdn_c = t * 1000 + 1;
    return xt + rt * cos(2 * PI);
  }
}

float circle_y(float rt, float yt, float t) {
  if (bdn_c <= t * 1000) {
    return yt + rt * sin(2 * PI * (bdn_c / 1000) / t);
  } else {
    bdn_c = t * 1000 + 1;
    return yt + rt * sin(2 * PI);
  }
}
float line_x(float xt, float _xt, float tf) {
  float t = bdn_c / 1000;
  if (bdn_c <= tf * 1000)
    return ((2 * xt) / pow(tf, 3) - (2 * _xt) / pow(tf, 3)) * pow(t, 3) + ((3 * _xt) / pow(tf, 2) - (3 * xt) / pow(tf, 2)) * pow(t, 2) + xt;
  else {
    bdn_c = tf * 1000 + 1;
    return _xt;
  }
}
float line_y(float yt, float _yt, float tf) {
  float t = bdn_c / 1000;
  if (bdn_c <= tf * 1000)
    return ((2 * yt) / pow(tf, 3) - (2 * _yt) / pow(tf, 3)) * pow(t, 3) + ((3 * _yt) / pow(tf, 2) - (3 * yt) / pow(tf, 2)) * pow(t, 2) + yt;
  else {
    bdn_c = tf * 1000 + 1;
    return _yt;
  }
}
float DHT_x(float thei, float phii) {
  float R = 0.0325;
  return R * thei * cos(phii);
}
float DHT_y(float thei, float phii) {
  float R = 0.0325;
  return R * thei * sin(phii);
}
float DHN_The (float xt, float yt) {
  float R = 0.0325;
  float ttt = sqrt((pow(xt, 2) + pow(yt, 2)) / (R * R));
  return ttt;
}
float DHN_Phi (float xt, float yt) {
  float R = 0.0325;
  float the_i = DHN_The(xt, yt);
  float ppp = atan2(yt / (R * the_i), xt / (R * the_i));
  return ppp;
}
void LAYMAU() {
  bdn++;
  bdn_c++;
  Timer1.initialize(1000); // set timer1
}
float derivative(float a)
{
  static float P_a;
  uint32_t preTime = 0;
  float E_dt = 0;
  E_dt = (a - P_a) / (float(micros() - preTime) / 1000000);
  P_a = a;
  preTime = micros();
  return E_dt;
}
void DHN()
