// https://anthropomorphist.wordpress.com/
// Self Balancing Bot
// Based on code from http://bajdi.com and TKJ Electronics
 
#include <Wire.h>    // built in?
#include "Kalman.h"  // Source: https://github.com/TKJElectronics/KalmanFilter
#include "I2Cdev.h"  //
#include "MPU6050.h" //


// DEFINES: #####
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define   GUARD_GAIN  50.0 // limits Ki to this value
#define runEvery(t) for (static typeof(t) _lasttime;(typeof(t))((typeof(t))millis() - _lasttime) > (t);_lasttime += (t)) // ??
  
// USER PARAMS: #####
// PID
const float K =  1; // multiplier
const float Kp = 25; 
const float Ki = 0;
const float Kd = 0;
float setPoint = -5;
float deadZone = 1;  // shut off motors if this close in degrees to setpoint
float minSpeed = 10;
float maxAngle = 40; // shut off motors if leaning more than this 

float pTerm, iTerm, dTerm, integrated_error, last_error, error = 0;

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
float CurrentAngle;

uint32_t timer;      // unsigned long timer;
uint8_t i2cData[14]; // Buffer for I2C data

int speed;
int pid;
 


 
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
   
  timer = micros();            // Initialize the timer
}
 
void loop() {
  
  runEvery(5)
  {
  
    updateMPU6050();
  
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    updateRoll();
  
    double gyroXrate = gyroX / 131.0; // Convert to deg/s 131 is a magic number to me for now.

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    //if ((accXangle < -90 && kalAngleX > 90) || (accXangle > 90 && kalAngleX < -90)) {
    //  kalmanX.setAngle(accXangle);
    //  compAngleX = accXangle;
    //  kalAngleX = accXangle;
    //  gyroXangle = accXangle;
    //} else
      kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, dt); // Calculate the angle using a Kalman filter
  
    /* Estimate angles using gyro only */
    //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter

    /* Estimate angles using complimentary filter */
    //compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * accXangle; // Calculate the angle using a Complimentary filter

    // Reset the gyro angles when they has drifted too much
    //if (gyroXangle < -180 || gyroXangle > 180)
    //  gyroXangle = kalAngleX;
 
    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    //CurrentAngle = compAngleX;
    CurrentAngle = kalAngleX;
    
    Serial.print("\t");
    Serial.print(accXangle);
    Serial.print("\t");
    Serial.print(gyroXrate);
    Serial.print("\t");
    Serial.print(dt);
    
    Serial.print("\t");
    Serial.print(CurrentAngle);

  
    
    if ((CurrentAngle <=-maxAngle) || (CurrentAngle>=maxAngle))
    {
      //break;
      stop();
      while(1);
    }
    else
    {
      if (deadZone>=abs(CurrentAngle-setPoint))
      {
        stop();
        Serial.print("\taction: stop");
      } 
      else 
      {
        Pid();
        Motors();
      }
    } // overtilt protection
     Serial.print("\n");
  }
  // End Loop
}
 
void Motors(){
    
  speed = K*pid;
   
  // if speed is less than minSpeed, make it minSpeed
  if ((speed>0) && (speed<minSpeed)) speed = minSpeed;
  if ((speed<0) && (speed>-minSpeed)) speed = -minSpeed;
   
  speed = constrain(speed, -255, 255);
  Serial.print("\tspeed:");
  Serial.print(speed); 

  
  if (speed > 0)
  {
    //forward
    analogWrite(AIN1, 0);
    analogWrite(AIN2, speed);
    analogWrite(BIN1, 0);
    analogWrite(BIN2, speed);
   }
  else
  {
    // backward
    speed = abs(speed);
    analogWrite(AIN1, speed);
    analogWrite(AIN2, 0);
    analogWrite(BIN1, speed);
    analogWrite(BIN2, 0);
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
  pid = pTerm + iTerm + dTerm;  
  Serial.print("\t");
  Serial.print(pid);
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
