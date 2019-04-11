static vector_t gyroRaw = vector_t();

void readGyro()
{
  gyroRaw = vector_t(
    imu.calcGyro(imu.gx),
    imu.calcGyro(imu.gy),
    imu.calcGyro(imu.gz)
  );
}

vector_t getGyro()
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
