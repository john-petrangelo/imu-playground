static vector accelRaw = {0};

void readAccel()
{
  accelRaw.x =  imu.calcAccel(imu.ax);
  accelRaw.y = -imu.calcAccel(imu.ay);
  accelRaw.z = -imu.calcAccel(imu.az);
}

vector getAccel()
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
  Serial.print(v_magnitude(accelRaw), 2);
  Serial.print(" ");
}
