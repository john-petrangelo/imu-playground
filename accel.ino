static Vector accelRaw;

void readAccel()
{
  accelRaw = Vector(
    -imu.calcAccel(imu.ay),
    imu.calcAccel(imu.ax),
    imu.calcAccel(imu.az)
  );
}

Vector getAccelTicks()
{
  return Vector(-imu.ay, imu.ax, imu.az);
}

Vector getAccel()
{
  return accelRaw;
}
