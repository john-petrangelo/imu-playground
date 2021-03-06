// The SFE_LSM9DS1 library requires both Wire and SPI be included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#include "common.h"

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
LSM9DS1 imu;

////////////////////////////
// Sketch Output Settings //
////////////////////////////
unsigned long const PRINT_SPEED = 25; // ms between prints
unsigned long const  UPDATE_SPEED = 5; // ms between updates
static unsigned long lastPrint = 0;
static unsigned long lastUpdate = 0;

static int accelCount = 0;
static int gyroCount = 0;
static int magCount = 0;

void setup()
{
  Serial.begin(115200);

  initSensors();
}

void loop_cal()
{
  readSensors();
  Vector const accel = getAccelTicks();
  Vector const mag = getMagTicks();
  
  // Print the sensor data
  Serial.print("Raw:");
  Serial.print((int)accel.x);
  Serial.print(',');
  Serial.print((int)accel.y);
  Serial.print(',');
  Serial.print((int)accel.z);
  Serial.print(',');
  Serial.print(0 /*gyro.raw.x*/);
  Serial.print(',');
  Serial.print(0 /*gyro.raw.y*/);
  Serial.print(',');
  Serial.print(0 /*gyro.raw.z*/);
  Serial.print(',');
  Serial.print((int)mag.x);
  Serial.print(',');
  Serial.print((int)mag.y);
  Serial.print(',');
  Serial.print((int)mag.z);
  Serial.println();

  delay(50);
}

void loop()
{
  unsigned long const now = millis();
  readSensors();
  Attitude attitude;

  if (now < 1000) {
    // For the first second use mag and accel.
    if ((lastPrint + PRINT_SPEED) < now) {
      Vector const accel = getAccel();
      Vector const mag = getMag();
      attitude = init_attitude_with_accel_mag(accel, mag);

      lastPrint = now;
      resetCounts();

      Serial.print(rad2deg(attitude.heading));
      Serial.print(",");
      Serial.print(rad2deg(attitude.pitch));
      Serial.print(",");
      Serial.print(rad2deg(attitude.roll));
      Serial.print("\n");
    }
  } else {
     // After one second use the gyro.
     Vector const gyro = Gyro::get();

     if ((lastUpdate + UPDATE_SPEED) < now) {
       attitude = update_attitude_with_gyro(gyro);
       lastUpdate = now;
     }

    if ((lastPrint + PRINT_SPEED) < now) {
       lastPrint = now;
//       printUpdate();
       resetCounts();

      Serial.print(rad2deg(attitude.heading));
      Serial.print(",");
      Serial.print(rad2deg(attitude.pitch));
      Serial.print(",");
      Serial.print(rad2deg(attitude.roll));
      Serial.print("\n");
    }
  }
}

void printUpdate()
{
//    update_gyro_attitude();
//    plot_gyro_attitude();
//
//    Serial.println();  
}

void calc_attitude_with_accel_mag()
{
  // Read the accelerometer and 
  Vector const accel = getAccel();
  Vector const mag = getMag();

  Attitude attitude = get_attitude_from_accel_mag(accel, mag);
  Vector const &euler = attitude.euler;
  Vector const &ihat = attitude.ihat;
  Vector const &jhat = attitude.jhat;
  Vector const &khat = attitude.khat;

  Serial.print(" e:");
  euler.print();

  float const headingRad = atan2(ihat.y, jhat.y);
  float const pitchRad = asin(khat.y);
  float const rollRad = atan2(khat.x, khat.z);

  float const heading = normalizeDeg(rad2deg(headingRad));
  float const pitch = normalizeDeg(rad2deg(pitchRad));
  float const roll = normalizeDeg(rad2deg(rollRad));

  print_attitude(heading, pitch, roll);

  // Make the quaternions to reverse the roll, undo the pitch, and turn the heading North.
  Quaternion q_roll(rollRad/2, jhat);
  Quaternion q_pitch(-pitchRad/2, ihat);
  Quaternion q_heading(headingRad/2, khat);

  // Combine the individual rotations into a single quaternion.
  Quaternion q_rot = q_roll.multiply(q_pitch).multiply(q_heading);

  // q_euler is the euler vector expressed as a quaternion.
  Quaternion q_euler(euler);

  // rotation = q x euler x q*
  Quaternion q_out = q_rot.multiply(q_euler).multiply(q_rot.conjugate());

  Serial.print(" v_out:");
  q_out.vector().print();

  Vector v_gyro = Gyro::get();
  Serial.print(" gyro:");
  v_gyro.print();

  Serial.println();
}

void printCounts()
{
  Serial.print("#accel=");
  Serial.print(accelCount);
  Serial.print(" #gyro=");
  Serial.print(gyroCount);
  Serial.print(" #mag=");
  Serial.print(magCount);
  Serial.print(" ");
}

void resetCounts()
{
  accelCount = 0;
  gyroCount = 0;
  magCount = 0;
}
