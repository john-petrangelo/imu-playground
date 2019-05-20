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

void Gyro::print()
{
  Serial.print("G: ");
  Serial.print(gyroRaw.x, 2);
  Serial.print(", ");
  Serial.print(gyroRaw.y, 2);
  Serial.print(", ");
  Serial.print(gyroRaw.z, 2);
  Serial.print(" deg/s");
}
