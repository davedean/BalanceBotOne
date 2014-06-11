// https://anthropomorphist.wordpress.com/
// Self Balancing Bot
// Based on code from http://bajdi.com and TKJ Electronics
 
#include <Wire.h>    // built in?
#include "Kalman.h"  // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2Cdev.h"  //
#include "MPU6050.h" //


// DEFINES: #####
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define   GUARD_GAIN  10.0 // limits Ki   
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t)) // ??
  
// USER PARAMS: #####
// PID
const float Kp = 13; // 13 seems stableish
const float Ki = 0;
const float Kd = 0;
float pTerm, iTerm, dTerm, integrated_error, last_error, error;
const float K =  1.9*1.12; // ?? I don't know how this was decided?
float setPoint = 182;
float variance = .1;

// Motor controller pins
const int AIN1 = 3;  // (pwm) pin 3 connected to pin AIN1
const int AIN2 = 9;  // (pwm) pin 9 connected to pin AIN2
const int BIN1 = 10; // (pwm) pin 10 connected to pin BIN1 
const int BIN2 = 11;  // (pwm) pin 11 connected to pin BIN2


// Declarations: ###########

Kalman kalmanX; // Create the Kalman instance

const uint8_t MPU6050 = 0x68; // If AD0 is logic low on the PCB the address is 0x68, otherwise set this to 0x69
 
/* IMU Data */
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

double accXangle;// Angle calculate using the accelerometer
double gyroXangle;// Angle calculate using the gyro
double kalAngleX;// Calculate the angle using a Kalman filter
double compAngleX; // Calculated angle using a complementary filter


uint32_t timer;      // unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data
float CurrentAngle;
 
int speed;
 


 
void setup() { 
  pinMode(AIN1, OUTPUT); // set pins to output
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  Serial.begin(57600);
  Wire.begin();
  
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(MPU6050, 0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(MPU6050, 0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(MPU6050, 0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
 
  delay(100); // Wait for sensor to stabilize
 
  /* Set kalman and gyro starting angle */
  updateMPU6050();
  updateRoll();
 
  kalmanX.setAngle(accXangle); // Set starting angle
  gyroXangle = accXangle;      // Set gyro to same angle as accelerometer
   
  timer = micros();            // Initialie the timer
}
 
void loop() {
  
  updateMPU6050();
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  updateRoll();
  
  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s 131 is a magic number to me for now.
  
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((accXangle < -90 && kalAngleX > 90) || (accXangle > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(accXangle);
    compAngleX = accXangle;
    kalAngleX = accXangle;
    gyroXangle = accXangle;
  } else
    kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt); // Calculate the angle using a Kalman filter
    
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  
  /* Estimate angles using gyro only */
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate from the Kalman filter

  /* Estimate angles using complimentary filter */
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * accXangle; // Calculate the angle using a Complimentary filter

  // Reset the gyro angles when they has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  
  
  // Tie it all together! 
  
  // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
  CurrentAngle = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000);  
   
  if ((CurrentAngle <= (setPoint+variance)) && (CurrentAngle >= (setPoint-variance)))
  {
    stop();
  } 
  else 
  {
    Pid();
    Motors();
  }
  
  // End Loop
}
 
void Motors(){
  
  if (speed > 0)
  {
    //forward
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
  }
  else
  {
    // backward
    speed = abs(speed);
    analogWrite(AIN1, 0);
    analogWrite(AIN2, speed);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
  }
}
 
void stop()
{
  analogWrite(AIN1, 0);
  analogWrite(AIN2, 0);
  analogWrite(BIN1, 0);
  analogWrite(BIN2, 0);
}
 
void Pid(){
  error = setPoint - CurrentAngle;  // setPoint = level
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -GUARD_GAIN, GUARD_GAIN);
  dTerm = Kd * (error - last_error);
  last_error = error;
  
  // This is WAY too simplistic 
  // Adding the terms and multiplying them by K
  speed = constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}
 

void updateMPU6050() {
  
  while (i2cRead(MPU6050, 0x3B, i2cData, 14)); // Get accelerometer and gyroscope values
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = -((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = -(i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = -(i2cData[12] << 8) | i2cData[13];
}




void updateRoll() {
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  accXangle = atan2(accY, accZ) * RAD_TO_DEG;
#else // Eq. 28 and 29
  accXangle = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif
}
