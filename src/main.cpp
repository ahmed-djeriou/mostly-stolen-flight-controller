#include <Arduino.h>

#include <Wire.h>
#define REMOTEXY_MODE__ESP32CORE_BLE


#include <motor_TB.h>
c_motor_TB motor1,motor2,motor3,motor4;
#include <BLEDevice.h>
// RemoteXY connection settings 
#define REMOTEXY_BLUETOOTH_NAME "RemoteXY"


#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 38 bytes
  { 255,4,0,0,0,31,0,19,0,0,0,0,31,1,106,200,1,1,2,0,
  5,23,10,60,60,32,2,6,31,5,24,128,60,60,32,2,26,31 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  int8_t jx1; // from -100 to 100
  int8_t jy1; // from -100 to 100
  int8_t jx2; // from -100 to 100
  int8_t jy2; // from -100 to 100

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)


// IMU calibration values - UPDATE THESE WITH YOUR CALIBRATED VALUES
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch = -0.04;   // UPDATE WITH YOUR VALUES
float RateCalibrationRoll = 0.02;     // UPDATE WITH YOUR VALUES
float RateCalibrationYaw = 0.02;      // UPDATE WITH YOUR VALUES
float AccXCalibration = 0.01;         // UPDATE WITH YOUR VALUES
float AccYCalibration = -0.01;        // UPDATE WITH YOUR VALUES
float AccZCalibration = -0.00;        // UPDATE WITH YOUR VALUES

// PID tuning parameters
int ESCfreq = 500;
float PAngleRoll = 2; 
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5; 
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007; 
float DAnglePitch = DAngleRoll;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;
float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;
float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

// Timing control
uint32_t LoopTimer;
float t = 0.004;  // time cycle (250Hz)




/*const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;*/

// RC receiver variables
volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;
volatile int ReceiverValue[6];


// PID variables
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filter variables
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

// Complementary filter variables
float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

// Motor output variables
volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// Kalman function for control position and shit idk read the wiki
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4); // here 4 is the variance of IMU i.e 4 deg/s
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // std deviation of error is 3 deg
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}
//what rear IMU signals
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
  
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}
//PID
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + (I * (Error + PrevError) * (t / 2));
  
  if (Iterm > 400) {
    Iterm = 400;
  } else if (Iterm < -400) {
    Iterm = -400;
  }
  
  float Dterm = D * ((Error - PrevError) / t);
  float PIDOutput = Pterm + Iterm + Dterm;
  
  if (PIDOutput > 400) {
    PIDOutput = 400;
  } else if (PIDOutput < -400) {
    PIDOutput = -400;
  }
  
  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}
void setup(){
  RemoteXY_Init ();
  
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  
  // MPU6050 initialization
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // PWM initialization for ESCs
  

  delay(1000);
  /// settting motors and shit type shii
  motor1.setPins(21,20,10,1);
  motor2.setPins(21,20,9,1);
  motor3.setPins(4,5,2,0);
  motor4.setPins(4,5,3,0);





  // Initialize loop timer
  LoopTimer = micros();
}
void loop(void) {
  RemoteXY_Handler ();
  ReceiverValue[0] = map(RemoteXY.jy2 /*roll*/, -100, 100, 1000, 2000);    // Roll
  ReceiverValue[1] = map(RemoteXY.jx2/*pitch*/, -100, 100, 1000, 2000);   // Pitch
  ReceiverValue[2] = map(RemoteXY.jx1/*throttle*/, 0, 100, 1000, 2000);   // Throttle
  ReceiverValue[3] = map(RemoteXY.jy1/*yaw*/, -100, 100, 1000, 2000);     // Yaw
  
  // Read IMU data
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Apply calibration
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;
  
  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Calculate angles from accelerometer
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  // Apply complementary filter
  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;
  
  // Clamp angles to ±20 degrees
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);

  // Convert RC inputs to desired angles/rates
  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // Angle PID for Roll
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Angle PID for Pitch
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Rate PID for Roll
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Rate PID for Pitch
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Rate PID for Yaw
  ErrorRateYaw = DesiredRateYaw - RateYaw;
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // Throttle limit
  if (InputThrottle > 1800) {
    InputThrottle = 1800;
  }

  // Motor mixing (quadcopter X configuration)
  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); // front right - counter clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); // rear right - clockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); // rear left - counter clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); // front left - clockwise

  // Motor output limits
  if (MotorInput1 > 2000) MotorInput1 = 1999;
  if (MotorInput2 > 2000) MotorInput2 = 1999;
  if (MotorInput3 > 2000) MotorInput3 = 1999;
  if (MotorInput4 > 2000) MotorInput4 = 1999;

  // Motor idle speed
  if (MotorInput1 < ThrottleIdle) MotorInput1 = ThrottleIdle;
  if (MotorInput2 < ThrottleIdle) MotorInput2 = ThrottleIdle;
  if (MotorInput3 < ThrottleIdle) MotorInput3 = ThrottleIdle;
  if (MotorInput4 < ThrottleIdle) MotorInput4 = ThrottleIdle;

  // Disarm motors if throttle is too low
  if (ReceiverValue[2] < 1030) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;

    // Reset PID integrals
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
  }

  MotorInput1=map(MotorInput1,1000,2000,0,255);
  MotorInput2=map(MotorInput2,1000,2000,0,255);
  MotorInput3=map(MotorInput3,1000,2000,0,255);
  MotorInput4=map(MotorInput4,1000,2000,0,255);
  motor1.motor_speed(MotorInput1);
  motor2.motor_speed(MotorInput2);
  motor3.motor_speed(MotorInput3);
  motor4.motor_speed(MotorInput4);
  /* motors order so i don't fuck it up 

  Motor 1: Front Right → Counter-Clockwise (CCW)

Motor 2: Rear Right → Clockwise (CW)

Motor 3: Rear Left → Counter-Clockwise (CCW)

Motor 4: Front Left → Clockwise (CW)
*/
  
  //mot1.writeMicroseconds(MotorInput1);
  //mot2.writeMicroseconds(MotorInput2);
  //mot3.writeMicroseconds(MotorInput3);
  //mot4.writeMicroseconds(MotorInput4); yeah not happenning son

  // Maintain loop timing
  while (micros() - LoopTimer < (t * 1000000)) {
    // Wait for next cycle
  }
  LoopTimer = micros();
}

