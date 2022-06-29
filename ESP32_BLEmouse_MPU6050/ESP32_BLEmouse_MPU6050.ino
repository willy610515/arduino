#include "I2Cdev.h"
#include <Wire.h>
#include <MPU6050.h>
#include <BleMouse.h>

#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

BleMouse bleMouse;
MPU6050 imu;
int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double tempAngleX, tempAngleY;

double vx, vy;
int buttonL = 0; // IO0 button
int buttonR = 4;
int buttonLstate = HIGH; 
int buttonRstate = HIGH; 

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

void setup() {
  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
  pinMode(buttonL, INPUT_PULLUP);
  pinMode(buttonR, INPUT_PULLUP);
  
  Serial.print("MPU6050 initializing");
  imu.initialize();
  while (!imu.testConnection()) { Serial.print("."); }
  delay(100); // Wait for sensor to stabilize
  Serial.println();  
  Serial.println("BLE Mouse starting !");
  bleMouse.begin();
  imu.getAcceleration(&accX, &accY, &accZ);

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}

void loop() {
  imu.getAcceleration(&accX, &accY, &accZ);
  imu.getRotation(&gyroX, &gyroY, &gyroZ);
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
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
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.95 * (compAngleX + gyroXrate * dt) + 0.05 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.95 * (compAngleY + gyroYrate * dt) + 0.05 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  if(abs(kalAngleX-tempAngleX)>1) vx =kalAngleX;
  else vx=0;
  if(abs(kalAngleY-tempAngleY)>1) vy =kalAngleY;
  else vy=0;
  // using Kalman filter

  //if(abs(compAngleX-tempAngleX)>1) vx =compAngleX;
  //else vx=0;
  //if(abs(compAngleY-tempAngleY)>1) vy =compAngleY;
  //else vy=0;
  // using complementary filter
  
    /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");
  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");
  Serial.print("\t");
#endif

#if 1
  Serial.print(roll); Serial.print("\t");
  Serial.print(gyroXangle); Serial.print("\t");
  Serial.print(compAngleX); Serial.print("\t");
  Serial.print(kalAngleX); Serial.print("\t");

  Serial.print("\t");

  Serial.print(pitch); Serial.print("\t");
  Serial.print(gyroYangle); Serial.print("\t");
  Serial.print(compAngleY); Serial.print("\t");
  Serial.print(kalAngleY); Serial.print("\t");
#endif

  
  Serial.print("\t");
  Serial.print("X = ");    Serial.print(vx);
  Serial.print(", Y = ");  Serial.println(vy);
  Serial.print("\r\n");
  delay(2);
  
  bleMouse.move(vy, vx);

  tempAngleX=kalAngleX;
  tempAngleY=kalAngleY;
  // using Kalman filter

  //tempAngleX=compAngleX;
  //tempAngleY=compAngleY;
  // using complementary filter
    
  buttonLstate = digitalRead(buttonL);
  buttonRstate = digitalRead(buttonR);  
  
  if (buttonLstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_LEFT);
    delay(100);
  } 
  else if(buttonRstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_RIGHT);
    delay(100);
  }
  delay(100);
}
