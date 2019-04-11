static vector_t accelRaw;

void readAccel()
{
  accelRaw = vector_t(
    imu.calcAccel(imu.ax),
    -imu.calcAccel(imu.ay),
    -imu.calcAccel(imu.az)
  );
}

vector_t getAccel()
{
  return accelRaw;
}

void plotAccel()
{  
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(accelRaw.x, 2);
  Serial.print(" ");
  Serial.print(accelRaw.y, 2);
  Serial.print(" ");
  Serial.print(accelRaw.z, 2);
  Serial.print(" ");
  Serial.print(accelRaw.magnitude(), 2);
  Serial.print(" ");
}
