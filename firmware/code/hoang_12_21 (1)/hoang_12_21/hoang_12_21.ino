#include <Wire.h>
#include <Kalman.h>  //https://github.com/TKJElectronics/KalmanFilter
#include<TimerOne.h>
#include<TimerThree.h>
#define ToDeg 180 / PI
#define ToRad PI / 180

Kalman kalman;  //Kalman filter define: kalman.getAngle(pitch, gyrorate, dt);

int inChar;
uint32_t timerloop, timerold;

//Motor control Pin//

int leftpwm = 9;   //Control PWM left motor
int leftdir = 28;   //Control direction left motor//24
int leftdir2 = 30;  //Control direction left motor//26

int righpwm = 10;    //Control PWM right motor
int righdir = 26;   //Control direction right motor//28
int righdir2 = 24;  //Control direction right motor//30

volatile long leftencoder = 0;  //Read left encoder value
volatile long righencoder = 0;  //Read right encoder value
// INITIALIZE ENCODER PIN
int leftencoder_a = 3;  //Read state encoder channel LOW or HIGH
int leftencoder_b = 6;
int righencoder_a = 2;
int righencoder_b = 5;

//MPU6050 Data//
double mpudata;  //Save psi angle (Y axis)
double accX, accZ, accY;
float Gyro;
int t;
uint32_t timer;  //Timer for kalman filter psi angle;
uint8_t i2cData[14];
String Data = "";
//Sensor voltage
float voltage = 0, cal_per = 0;
//LQR data//
long PWML, PWMR;                   //PWM output for H-Brigde
float K1, K2, K3, K4, K5, K6, ki;  //The factor of K maxtrix
bool falldown;                     //Run = true; Stop = false;

float theta = 0, psi = 0, phi = 0;
float thetadot, psidot, phidot;
float thetaold = 0, psiold = 0, phiold = 0, theta_p = 0;
float addtheta = 0, addphi = 0;

float leftvolt;  //output volt left motor in LQR
float righvolt;  //output volt right motor in LQR
// Serial
bool flag_Timer1 = false;
bool flag_Timer3 = false;
bool flag_receive = false;
int flag_send = 0;
bool flag_read_sensor = false;
bool flag_cal = false;

// Trajectory Planning  variable
float  t_count = 0, tf = 0, tr = 0, addtheta_ = 0, addphi_ = 0, t_tp = 0, xt = 0, yt = 0, xt_ = 0, yt_ = 0, xtam_ = 0, ytam_ = 0, Rad = 0;
bool mode = false;
int mode_t = 0;
/////////////////////////////////////////////////
///////////    SERIAL BEGIN   ///////////////////
/////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  K1 = 10.5; //9.78;//
  K2 = 9.5; //18.8;//
  K3 = 340; //502.58;
  K4 = 40; //93.8;
  K5 = 9; //10.78;//11;
  K6 = 10.5; //32.3;//26; lúc 10h35_15_12

  ki = 0;
  pinMode(leftpwm, OUTPUT);
  pinMode(righpwm, OUTPUT);
  //  pinMode(7, OUTPUT);
  TCCR2B = TCCR2B & B11111000 | B00000001;  //Pin 9 & Pin 10
  Timer1.initialize(10000);//10us
  Timer1.attachInterrupt(Timer1_ISR);
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
  attachInterrupt(1, left_isr, RISING);
  attachInterrupt(0, righ_isr, RISING);
  // Sensor Voltage/
  pinMode(A0, INPUT);

  //Data MPU6050//
  Wire.begin();

  TWBR = ((F_CPU / 400000UL) - 16) / 2;  // Set I2C frequency to 400kHz
  i2cData[0] = 7;
  i2cData[1] = 0x00;
  i2cData[2] = 0x00;
  i2cData[3] = 0x00;
  while (i2cWrite(0x19, i2cData, 4, false));
  while (i2cWrite(0x6B, 0x01, true));
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) {
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100);
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[0] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
#ifdef RESTRICT_PITCH
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalman.setQangle(0.001);
  kalman.setQbias( 0.003);
  kalman.setRmeasure(0.03);
  kalman.setAngle(pitch);
  timer = micros();
}

//////////////////////////////////
//       MAIN PROGRAMMING       //
//////////////////////////////////
void loop() {
  if (flag_Timer1 == true) {
    flag_Timer1 = false;
    // read data
    readdata();
    readmpu();// 1.98ms
    //1.14ms
    theta = gettheta(leftencoder, righencoder) * ToRad;  //Read theta value and convert to Rad
    psi = (mpudata + 4.6) * ToRad;                         //Read psi value and convert to Rad
    phi = getphi(-leftencoder, righencoder) * ToRad;      //Read phi value and convert to Rad
    //    if (t_count == 100)
    //    {
    //      t_count = 0;
    //      voltage = getvoltage();// Read voltage value
    //      cal_per = (voltage / 12.6) * 100;
    //    }
    //Update time compare with timeloop
    update_var();
    if (mode_t == 1)
    {
      if (t_tp < tf * 100)
      {
        xt_ = qhqd_line(xtam_, xt, tf);
        yt_ = qhqd_line(ytam_, yt, tf);
        addtheta_ = DHN_the(xt_, yt_);
        addphi_   = DHN_phi(xt_, yt_);
      }
      else
      {
        t_tp = tf * 100 + 1;
        xtam_ = xt_;
        ytam_ = yt_;
      }
    }
    else if (mode_t == 2)
    {
      if (t_tp <= (tf / 2) * 100)
      {
        addtheta_ = addtheta;
      }
      else
      {
        addtheta_ = 0;
      }
      if (t_tp == tf * 100)
      {
        t_tp = 0;
      }
    }
    // quy hoach theo xung pulse
    //
    getlqr(theta - addtheta_, thetadot, psi, psidot, phi - addphi_, phidot, theta_p);
    motorcontrol(PWML, PWMR, (mpudata + 4.6), falldown);
    //Send data to serial
//    Data = (String)(psi * ToDeg) +  ',' + (String)(theta * ToDeg) + ',' + (String)(phi * ToDeg) + ',' + (String)(addtheta_ * ToDeg) + ',' + (String)(addphi_ * ToDeg);
//    Data = Data + ',' + (String)((theta - addtheta_) * ToDeg) + ',' + (String)((phi - addphi_) * ToDeg);
    Data = (String)(psi * ToDeg)+','+(String)((leftencoder) * ToDeg)+','+(String)(phi * ToDeg);
    Serial.println(Data);
  }
}

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
  double dt = double(micros() - timer) / 1000000.0;
  timer = micros();
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  Gyro = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  double Gyrorate = Gyro / 131.0;
  psidot = Gyrorate * ToRad;
  mpudata = kalman.getAngle(pitch, Gyrorate, dt) - 1.23;

  if (abs(mpudata) > 30) {
    falldown = true;
  } else {
    falldown = false;
  }
}
//

//Read theta angle function
float gettheta(long lencoder, long rencoder) {

  float angle = 0.5 * (360 / (11 * 30)) * (lencoder - rencoder);
  return angle;
}

//Read phi angle function//
float getphi(long lencoder, long rencoder) {
  float angle = (4 / 24.2) * (360 / (11 * 30)) * (- rencoder - lencoder );
  return angle;
}
// Read voltage function
//float getvoltage()
//{
//  float raw_input = analogRead(A0);
//  float voltage = raw_input * 5 / (1024);
//  float R1 = 30000, R2 = 7500;
//  return  voltage / (R2 / (R1 + R2));
//}
//LQR function
void getlqr(float theta_, float thetadot_, float psi_, float psidot_, float phi_, float phidot_, float theta_p) {
  righvolt = (K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ - K5 * phi_ - K6 * phidot_ + theta_p);
  leftvolt = (K1 * theta_ + K2 * thetadot_ + K3 * psi_ + K4 * psidot_ + K5 * phi_ + K6 * phidot_ + theta_p);
  int cap_xung = 210;
  PWML = map(leftvolt, (-K3 * PI * 5 / 180), (K3 * PI * 5 / 180), -cap_xung, cap_xung); //Limit 20 deg.
  PWMR = map(righvolt, (-K3 * PI * 5 / 180), (K3 * PI * 5 / 180), -cap_xung, cap_xung);
  // lan 1 160

  PWML = constrain(PWML, -cap_xung, cap_xung); // SATURATION
  PWMR = constrain(PWMR, -cap_xung, cap_xung);
  //
  //  PWML =Sat(PWML, -180,  180);
  //  PWMR =Sat(PWMR, -180,  180);
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
  addtheta = 0;
  addphi = 0;
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

// Trajectory Planning
float qhqd_line(float xt, float xtf, float tf)
{
  float   a0 = xt;
  float   a1 = 0;
  float   a2 = 3 * (xtf - xt) / (tf * tf);
  float   a3 = -2 * (xtf - xt) / (tf * tf * tf);
  float t = t_tp / 100;

  if (t_tp < tf * 100)
  {
    return (a0 + a1 * t + a2 * ( t * t) + a3 * ( t * t * t));
  }
  else
  {
    t_tp = tf * 100 + 1;
    return xtf;
  }
}
float DHN_the(float x_t, float y_t)
{
  float R = 3.4;
  return  sqrt((x_t * x_t + y_t * y_t) / (R * R));

}
float DHN_phi(float x_t, float y_t)
{
  float R = 3.4;
  float the_ = DHN_the( x_t, y_t);
  return  atan2(y_t / (R * the_), x_t / (R * the_));
}

void Timer1_ISR() {
  flag_Timer1 = true;
  flag_send ++;
  t_count ++;
  t_tp ++;
}
// read data from Master
void readdata()
{
  if (Serial1.available() > 0)
  {
    int idataRx = Serial1.read();
    if (idataRx == 85) {
      addtheta_ += 2 * PI;
      mode_t = 0;
    }
    else if (idataRx == 68) {
      addtheta_ -= 2 * PI;
      mode_t = 0;
    }
    else if (idataRx == 82) {
      addphi_ += PI / 4;
      mode_t = 0;
    }
    else if (idataRx == 76) {
      addphi_ -= PI / 4;
      mode_t = 0;
    }
    else if (idataRx == 72)
    {
      addtheta_ = 0;
      addphi_  = 0;
      mode_t = 0;
    }
    else if (idataRx == 77)
    {
      addphi_  = (10 * PI) / 4;
    }
    else if (idataRx == 78)
    {
      addphi_  = -(10 * PI) / 4;
    }
    else if (idataRx == 84)
    {
      //      addtheta_ = Serial1.parseFloat() * ToRad;
      //      addphi_ = Serial1.parseFloat() * ToRad;
      xt = Serial1.parseFloat();// read time final
      yt = Serial1.parseFloat();// read time final
      tf = Serial1.parseFloat();// read time final
      mode = true;
      mode_t = 1;
      t_tp = 0;
    }
    else if (idataRx == 67)
    {
      //      addtheta_ = Serial1.parseFloat() * ToRad;
      //      addphi_ = Serial1.parseFloat() * ToRad;
      addtheta = Serial1.parseFloat() * PI; // read xt final
      tr = Serial1.parseFloat();// read yt final
      tf = Serial1.parseFloat();// read time final
      mode = true;
      mode_t = 2;
      t_tp = 0;
    }

    idataRx = 0;
  }
}
// update variables of Slave
void update_var()
{
  double dt = (float)((micros() - timerloop)) / 1000000.0;
  timerloop = micros();
  //Update input angle value
  thetadot = (theta - thetaold) / dt;
  theta_p = (theta + thetaold) * ki * dt;
  phidot = (phi - phiold) / dt;
  //Update old angle value
  thetaold = theta;
  psiold = psi;
  phiold = phi;
}

float Sat(float u, float uplimit, float downlimit)
{
  if (u > uplimit)
  {
    return uplimit;
  }
  else if (u < downlimit)
  {
    return  downlimit;
  }
  else if (u >= downlimit && u <= uplimit)
  {
    return u;
  }
}
