static vector_t gyroRaw = {0};

void readGyro()
{
  gyroRaw.x = imu.calcGyro(imu.gx);
  gyroRaw.y = imu.calcGyro(imu.gy);
  gyroRaw.z = imu.calcGyro(imu.gz);
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
