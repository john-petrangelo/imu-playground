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
static Attitude attitude;

Attitude init_attitude_with_accel_mag(Vector const &accel, Vector const &mag)
{
#ifdef ARDUINO
  Attitude attitude = get_attitude_from_accel_mag(accel, mag);
  q_current_euler = Quaternion(attitude.euler);
  lastUpdateTime = millis();

  return attitude;
#endif
}

Attitude get_attitude_from_accel_mag(Vector const &accel, Vector const &mag)
{
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
  q_current_euler = attitude.q;

  attitude.heading = attitude.q.yaw();
  attitude.pitch = attitude.q.pitch();
  attitude.roll = attitude.q.roll();

  return attitude;
}

Attitude update_attitude_with_gyro()
{
#ifdef ARDUINO
  long now = millis();
#else
  long now = 0;
#endif

  Vector const v_gyro = Gyro::get();

  // Calculate delta time in seconds.
  float dt = (now - lastUpdateTime) * 0.001;

//  Serial.print("dt=");
//  Serial.print(dt, 3);
//  Serial.print(" gyro=");
//  v_gyro.print();

  Quaternion q_gyro = Quaternion(dt * deg2rad(v_gyro.magnitude())/2, v_gyro.normalize());
//  Serial.print("q=");
//  q_gyro.print(3);

//  Serial.print("before=");
//  attitude.q.print(3);
  attitude.q = attitude.q.multiply(q_gyro);
//  Serial.print("after=");
//  attitude.q.print(3);

  attitude.heading = attitude.q.yaw();
  attitude.pitch = attitude.q.pitch();
  attitude.roll = attitude.q.roll();

  lastUpdateTime = now;

//  Serial.println();

  return attitude;
}

void plot_gyro_attitude()
{
//  Serial.print(heading);
//  Serial.print(" ");
//  Serial.print(pitch);
//  Serial.print(" ");
//  Serial.print(roll);
}
