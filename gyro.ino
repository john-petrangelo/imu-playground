#ifndef ARDUINO
#include "common.h"
#endif

Vector Gyro::gyroRaw = Vector();

void Gyro::read()
{
  gyroRaw = Vector(
    imu.calcGyro(imu.gy),
    -imu.calcGyro(imu.gx),
    -imu.calcGyro(imu.gz)
  );
}

Vector Gyro::get()
{
  return gyroRaw;
}
