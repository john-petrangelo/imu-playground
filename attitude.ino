#ifndef ARDUINO
#include <iostream>
#include <iomanip>
#include "common.h"
#endif

static long lastUpdateTime = NOW_MS;

static Attitude attitude;

Attitude init_attitude_with_accel_mag(Vector const &accel, Vector const &mag)
{
  Attitude attitude = get_attitude_from_accel_mag(accel, mag);

  lastUpdateTime = NOW_MS;

  return attitude;
}

Attitude get_attitude_from_accel_mag(Vector const &accel, Vector const &mag)
{
  // khat represents the direction of "up" expressed relative to the sensor.
  attitude.khat = accel.normalize();

  // ihat represents the direction of "east" expressed relative to the sensor.
  attitude.jhat = attitude.khat.crossproduct(mag).normalize();

  // jhat represents the direction of "north" expressed relative to the sensor.
  attitude.ihat = attitude.jhat.crossproduct(attitude.khat).normalize();

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

  attitude.heading = attitude.q.yaw();
  attitude.pitch = attitude.q.pitch();
  attitude.roll = attitude.q.roll();

  return attitude;
}

Attitude update_attitude_with_gyro(Vector const &gyro)
{
  long now = NOW_MS;

  // Calculate delta time in seconds.
  float const dt = (now - lastUpdateTime) * 0.001;

  Quaternion const q_gyro = Quaternion(dt * deg2rad(gyro.magnitude())/2, gyro.normalize());

  attitude.q = attitude.q.multiply(q_gyro);

  attitude.heading = attitude.q.yaw();
  attitude.pitch = attitude.q.pitch();
  attitude.roll = attitude.q.roll();

  lastUpdateTime = now;

  return attitude;
}
