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
unsigned long const PRINT_SPEED = 250; // ms between prints
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

void loop()
{
  unsigned long const now = millis();
  readSensors();

  if (now < 1000) {
    // For the first second use mag and accel.
    if ((lastPrint + PRINT_SPEED) < now)
    {
        vector const accel = getAccel();
        vector const mag = getMag();

      init_attitude_with_accel_mag(accel, mag);
      printUpdate();
      lastPrint = now;
      resetCounts();
    }
  } else {
     // After one second use the gyro.
     if ((lastUpdate + UPDATE_SPEED) < now)
     {
       update_attitude_with_gyro();
       lastUpdate = now;
     }

    if ((lastPrint + PRINT_SPEED) < now)
    {
       lastPrint = now;
//       printUpdate();
//       resetCounts();
      calc_attitude_with_accel_mag();
    }
  }
}

void printUpdate()
{
//    update_gyro_attitude();
    plot_gyro_attitude();

    Serial.println();  
}

void calc_attitude_with_accel_mag()
{
  // Read the accelerometer and 
  vector accel = getAccel();
  vector mag = getMag();

  attitude_t attitude = get_attitude_from_accel_mag(accel, mag);
  vector const &euler = attitude.euler;
  vector const &ihat = attitude.ihat;
  vector const &jhat = attitude.jhat;
  vector const &khat = attitude.khat;

  Serial.print(" e:");
  v_print(euler);

  float const headingRad = atan2(ihat.y, jhat.y);
  float const pitchRad = asin(khat.y);
  float const rollRad = atan2(khat.x, khat.z);

  float heading = normalizeDeg(rad2deg(headingRad));
  float pitch = normalizeDeg(rad2deg(pitchRad));
  float roll = normalizeDeg(rad2deg(rollRad));

  print_attitude(heading, pitch, roll);

  // Make the quaternions to reverse the roll, undo the pitch, and turn the heading North.
  quaternion q_roll = q_make(rollRad/2, jhat);
  quaternion q_pitch = q_make(-pitchRad/2, ihat);
  quaternion q_heading = q_make(headingRad/2, khat);

  // Combine the individual rotations into a single quaternion.
  quaternion q_rot = q_multiply(q_multiply(q_roll, q_pitch), q_heading);

  // q_euler is the euler vector expressed as a quaternion.
  quaternion q_euler = q_make(euler);

  // rotation = q x euler x q*
  quaternion q_out = q_multiply(q_multiply(q_rot, q_euler), q_conjugate(q_rot));

  Serial.print(" v_out:");
  v_print(q_vector(q_out));

  vector v_gyro = getGyro();
  Serial.print(" gyro:");
  v_print(v_gyro);

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
