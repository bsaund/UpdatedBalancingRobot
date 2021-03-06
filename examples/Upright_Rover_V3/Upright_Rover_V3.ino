// SainSmart Instabots Upright Rover rev. 3.0
// Updatas at http://www.sainsmart.com

#include "blink.h"
#include <Wire.h>
#include <SPI.h>
/* #include <Mirf.h> */
/* #include <nRF24L01.h> */
/* #include <MirfHardwareSpiDriver.h> */
#include <I2Cdev.h>
#include <MPU6050.h>


MPU6050 imu;

#define Gyro_offset 0  //The offset of the gyro
#define Gyro_gain 131
#define Angle_offset 1.3  // The offset of the accelerator
#define RMotor_offset 40  // The offset of the Motor
#define LMotor_offset 40  // The offset of the Motor
#define pi 3.14159
#define TICKS_TO_ANG 1151

double wheelRadius = 0.033;
double wheelGap = .17;

double maxVel = 0.5;


float kp, ki, kd;
double thetaBody;
double lSpeed, rSpeed; /* Angular Velocities of l and r wheels */
double measVel; /* m/s */
double measDist; /* m */
double measAngular; /* NOTE: ANGULAR IN DEGREES  */

float cmdAngular=0, cmdVel=0;
double goalPosition = 0;


unsigned long lastTime;

long rEncoder = 0, rEncoderPrev = 0, lEncoder = 0, lEncoderPrev = 0;


int blinkFreq = 150;
int numBlinks = 2;

int TN1 = 23;
int TN2 = 22;
int ENA = 5;
int TN3 = 24;
int TN4 = 25;
int ENB = 4;

enum Mode {
  Balancing,
  DirectControl,
  StationKeeping,
  Off
};
Mode mode;

enum Orientation {
  Upright,
  Fallen  
};
Orientation orientation;
  


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  TCCR3A = _BV(COM3A1) | _BV(WGM31) | _BV(WGM30); // TIMER_3 @1K Hz, fast pwm
  TCCR3B = _BV(CS31);
  TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); // TIMER_0 @1K Hz, fast pwm
  TCCR0B = _BV(CS01) | _BV(CS00);

  /* If the robot was turned on with the angle over 45(-45) degrees,the wheels
   will not spin until the robot is in right position. */
  imu.initialize();

  lastTime = millis();

  filterIMU(10);
  pinMode(TN1, OUTPUT);
  pinMode(TN2, OUTPUT);
  pinMode(TN3, OUTPUT);
  pinMode(TN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(18, INPUT);
  pinMode(2, INPUT);
  pinMode(41, OUTPUT);

  attachInterrupt(4, State_A, FALLING);
  attachInterrupt(1, State_B, FALLING);

  // 24L01 initialization
  /* Mirf.cePin = 53; */
  /* Mirf.csnPin = 48; */
  /* Mirf.spi = &MirfHardwareSpi; */
  /* Mirf.init(); */
  /* Mirf.setRADDR((byte *)"serv1"); */
  /* Mirf.payload = 24; */
  /* Mirf.config(); */
  //digitalWrite(43, HIGH);
  digitalWrite(41, LOW);
  Blink.init(43);
  kp = 22.000;
  ki = 0;
  kd = 1.60;

  /* mode = Mode::StationKeeping; */
  mode = Mode::Balancing;

}

void loop()
{

  int dt_ms = millis() - lastTime;
  if (dt_ms < 10)
    return;
  lastTime = millis();  
  
  
  Recieve();



  Blink.blinkFor(blinkFreq, numBlinks, 3);

  double dt = (double)dt_ms/1000;
  filterIMU(dt);
  filterSpeed(dt);
  writeSerialData();

  balancingPID(dt);
  motorControl(dt);
}

void Recieve()
{
  if(!Serial.available())
    return;
  char serialData[20];
  double value;
  Serial.readBytesUntil(';', serialData, 19);
  value = atof(&serialData[2]);
  bool isValid = true;
  char cmdType = serialData[1];

  /* Serial.print("debug: "); */
  /* Serial.println(serialData); */

  switch(serialData[0]){ /* Command Category */
  case 'v': /* VelocityCommands */
    switch(cmdType){ /* Command Type */
    case 'x': /* move in the x direction */
      cmdVel = value;       break;
    case 'w': /* v turn about veritcal axis */
      cmdAngular = value;   break;
    default:
      isValid = false;
    }
    break;

  case 'm':  /* Set Mode  */
    switch(cmdType){
    case 'd':
      mode = Mode::DirectControl;  break;
    case 'b':
      mode = Mode::Balancing;      break;
    case 's':
      mode = Mode::StationKeeping;      break;
    case 'o':
      mode = Mode::Off;      break;
    default:
      isValid = false;
    }
    break;
    
  default:
    isValid = false;
  }

  if(!isValid){
    Serial.print("debug: incorrect  end: ");
    Serial.println(serialData);
  }
    




}

/* Filter the IMU to estimate theta using a */
/* Complimentary Filter */
void filterIMU(double dt)
{
  double tau = 0.5;

  // Raw datas
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  /* double measuredAngle = (atan2(ay, az) * 180 / pi + Angle_offset); */
  double measuredAngle = (atan2(ay, -ax) * 180 / pi + Angle_offset);
  measAngular = (double)gz / Gyro_gain + Gyro_offset;

  double a = tau/(tau+dt);

  thetaBody = a*(thetaBody + measAngular*dt) + (1-a)*measuredAngle;
  

  orientation = Orientation::Upright;
  if (abs(thetaBody) > 45){
    orientation = Orientation::Fallen;
    goalPosition = getDisplacement();
    if(mode != Mode::DirectControl)
      cmdVel = 0;
  }
}

/* Low pass filter the speed as read from the encoders */
void filterSpeed(double dt){
  double tau = 0.01;
  double a = tau/(tau+dt);


  lSpeed = a*lSpeed + (1-a)*(lEncoder - lEncoderPrev)/TICKS_TO_ANG/dt;
  rSpeed = a*rSpeed + (1-a)*(rEncoder - rEncoderPrev)/TICKS_TO_ANG/dt;
  measVel = (lSpeed + rSpeed)*wheelRadius/2;
  lEncoderPrev = lEncoder;
  rEncoderPrev = rEncoder;

  /* Serial.print("debug: vel:"); */
  /* Serial.println(measVel); */



}
  

void writeSerialData()
{
  Serial.print(thetaBody);
  Serial.print(" ");
  Serial.print((double)lEncoder/TICKS_TO_ANG);
  Serial.print(" ");
  Serial.print((double)rEncoder/TICKS_TO_ANG);
  Serial.print(" ");
  Serial.print(lSpeed);
  Serial.print(" ");
  Serial.print(rSpeed);
  Serial.println(" ");
}

void balancingPID(double dt)
{
/* TODO - I COMMENTED OUT LINES THAT BREAK THIS */
/*   REVISIT */
  if(orientation == Orientation::Fallen)
    return;
  if(mode != Mode::Balancing &&
     mode != Mode::StationKeeping)
    return;


  double thetaTarget = 0;

  /* if(mode == Mode::StationKeeping){ */
  /*   thetaTarget = (getDisplacement()-goalPosition)*10; */
  /* } */

  /* 0 Velocity Setpoint */
  /* thetaTarget += measVel*2; */
  
  /* cmdVel = -(thetaBody-thetaTarget)/20; */
  
  double cmdAccel = 0;
  cmdAccel += 1.0*(getDisplacement() - goalPosition);
  cmdAccel += 18*measVel;
  cmdAccel += -65*(thetaBody-thetaTarget) *pi/180;
  cmdAccel += -21*measAngular * pi/180;



  /* Serial.print("debug: thetaBody:"); */
  /* Serial.println(thetaBody); */


  cmdVel = cmdVel + cmdAccel * dt;

  cmdVel = max(min(cmdVel, maxVel), -maxVel);
  
  // Calculating the output values using the gesture values and the PID values.
  /* error = thetaBody; */
  /* errSum += error; */
  /* dErr = error - lastErr; */
  /* /\* Output = kp * error + ki * errSum + kd * omega; *\/ */
  /* Output = kp * error + ki * errSum; */
  /* lastErr = error; */
  /* noInterrupts(); */
  
  /* Speed = (rEncoder + lEncoder) / 2; */
  /* Distance += Speed + Run_Speed; */
  /* Distance = constrain(Distance, -8000, 8000); */
  /* Output += Speed * 2.4 + Distance * 0.025; */
  /* Sum_Right_Temp = rEncoder; */
  /* Sum_Left_Temp = rEncoder; */
  /* /\* rEncoder = 0; *\/ */
  /* /\* lEncoder = 0; *\/ */

  /* ROutput = Output + Turn_Speed; */
  /* LOutput = Output - Turn_Speed; */
  /* interrupts(); */
}

void motorControl(double dt) {
  if(mode == Mode::DirectControl){
    controlWorldToMotor(cmdVel, cmdAngular);
    return;
  }

  if(orientation == Orientation::Fallen ||
     mode == Mode::Off) {
    digitalWrite(TN1, HIGH);
    digitalWrite(TN2, HIGH);
    digitalWrite(TN3, HIGH);
    digitalWrite(TN4, HIGH);
    return;
  }

  controlWorldToMotor(cmdVel, cmdAngular);
}

void controlWorldToMotor(double linear, double angular){
  double v = linear/wheelRadius;
  double w = angular*wheelGap/wheelRadius;
  motorControlPid(v + w, v - w); 
}


void motorControlPid(float lCmdSpeed, float rCmdSpeed){

  
  /* float lPwm = lCmdSpeed * 20 + (lCmdSpeed - lSpeed) * 60; */
  /* float rPwm = rCmdSpeed * 20 + (rCmdSpeed - rSpeed) * 60; */
  float lPwm = lCmdSpeed * 20;
  float rPwm = rCmdSpeed * 20;

  /* Serial.print("debug: r_err: "); */
  /* Serial.print(rCmd - rSpeed); */
  /* Serial.print("\t l_err: "); */
  /* Serial.println(lCmd - lSpeed); */
    
  PWMControl(lPwm, rPwm);  
}
  
void PWMControl(float lOutput, float rOutput)
{
  digitalWrite(TN1, lOutput <= 0);
  digitalWrite(TN2, lOutput > 0);
  digitalWrite(TN3, rOutput <= 0);
  digitalWrite(TN4, rOutput > 0);

  OCR3A = min(1023, (abs(lOutput) + LMotor_offset) * 4); // Timer/Counter3 is a general purpose 16-bit Timer/Counter module
  OCR0B = min(255, abs(rOutput) + RMotor_offset); // Timer/Counter0 is a general purpose 8-bit Timer/Counter module
}

/* Returns the average of the displacement of the two wheels in meters */
double getDisplacement(){
  return (lEncoder + rEncoder)*wheelRadius/TICKS_TO_ANG/2;
}


void State_A()
{
  if (!digitalRead(18))  {
    lEncoder++;
  }  else  {
    lEncoder--;
  }
}

void State_B()
{
  if (digitalRead(2))  {
    rEncoder++;
  }  else  {
    rEncoder--;
  }
}

