#ifndef ARDUINO
#include <iostream>
#include <iomanip>
#include <math.h>
#include "common.h"
#endif

//static float heading = 0.0;
//static float pitch = 0.0;
//static float roll = 0.0;
static long lastUpdateTime = 0;

static Quaternion q_current_euler;

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf

void init_attitude_with_accel_mag(Vector const &accel, Vector const &mag)
{
#ifdef ARDUINO
  Attitude attitude = get_attitude_from_accel_mag(accel, mag);
  q_current_euler = Quaternion(attitude.euler);
  lastUpdateTime = millis();
#endif
}

Attitude get_attitude_from_accel_mag(Vector const &accel, Vector const &mag)
{
  Attitude attitude;
  
  // NOTE
  // For now we assume that the accelerometer is measuring g (i.e. we're not moving).
  // Later we'll need to address not using accelerometer for g when we're moving.

  // std::cout << "accel: ";
  // accel.print();
  // std::cout << "mag: ";
  // mag.print();
  
  // khat represents the direction of "up" expressed relative to the sensor.
  attitude.khat = accel.normalize();
  // std::cout << "khat: ";
  // attitude.khat.print();

  // ihat represents the direction of "east" expressed relative to the sensor.
  attitude.jhat = attitude.khat.crossproduct(mag).normalize();
  // std::cout << "ihat: ";
  // attitude.ihat.print();

  // jhat represents the direction of "north" expressed relative to the sensor.
  attitude.ihat = attitude.jhat.crossproduct(attitude.khat).normalize();
  // std::cout << "jhat: ";
  // attitude.jhat.print();

  // The euler vector is the orientation of the sensor expressed in global coordinates.
  // The euler vector is made up of the projection (i.e. dot product)
  // of the local vector over each of the global unit vectors (i, j, k).
  //   vector localVector = {0.0, 1.0, 0.0};
  //   float euler_x = v_dotproduct(localVector, ihat);
  //   float euler_y = v_dotproduct(localVector, jhat);
  //   float euler_z = v_dotproduct(localVector, khat);

  attitude.euler = Vector(
    attitude.ihat.x,
    attitude.jhat.x,
    attitude.khat.x
  ).normalize();

  attitude.q = Quaternion(attitude.ihat, attitude.jhat, attitude.khat);

  // Axis-angle
  attitude.aaVector = Vector();
  attitude.aaAngle = 0.0;

  attitude.heading = attitude.q.yaw();
  attitude.pitch = attitude.q.pitch();
  attitude.roll = attitude.q.roll();
  Serial.print(rad2deg(attitude.heading));
  Serial.print(",");
  Serial.print(rad2deg(attitude.pitch));
  Serial.print(",");
  Serial.print(rad2deg(attitude.roll));
  return attitude;
}

void update_attitude_with_gyro()
{
#ifdef ARDUINO
  long now = millis();
#else
  long now = 0;
#endif
  
#if 0
  Vector const gyro = getGyro();

  // Calculate delta time in seconds.
  float dt = (now - lastUpdateTime) * 0.001;

  float delta_roll = deg2rad(gyro.y * dt);
  float delta_pitch = deg2rad(gyro.x * dt);
  float delta_heading = deg2rad(gyro.z * dt);

  // Create quaternion representing rotation on all three axes.
  Quaternion q_roll(delta_roll / 2, Vector{0.0, 1.0, 0.0});
  Quaternion q_pitch(delta_pitch / 2, Vector{1.0, 0.0, 0.0});
  Quaternion q_heading(delta_heading / 2, Vector{0.0, 0.0, 1.0});

  // Combine the individual rotations into a single quaternion.
  Quaternion q_rot = q_roll.multiply(q_pitch).multiply(q_heading);

  // q_euler is the euler vector expressed as a quaternion.
  Quaternion q_euler(q_current_euler.vector());

  // rotation = q x euler x q*
  q_current_euler = q_rot.multiply(q_euler).multiply(q_rot.conjugate()).normalize();
#endif // 0

  lastUpdateTime = now;
}

void plot_gyro_attitude()
{
//  Serial.print(heading);
//  Serial.print(" ");
//  Serial.print(pitch);
//  Serial.print(" ");
//  Serial.print(roll);
}
