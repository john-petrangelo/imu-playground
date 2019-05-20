static Vector gyroRaw = Vector();

void readGyro()
{
  gyroRaw = Vector(
    imu.calcGyro(imu.gy),
    imu.calcGyro(imu.gz),
    -imu.calcGyro(imu.gz)
  );
}

Vector getGyro()
{
  return gyroRaw;
}

void printGyro()
{
  Serial.print("G: ");
  Serial.print(gyroRaw.x, 2);
  Serial.print(", ");
  Serial.print(gyroRaw.y, 2);
  Serial.print(", ");
  Serial.print(gyroRaw.z, 2);
  Serial.print(" deg/s");
}
