/////////////////////////////////////////////////////////////////////
// Magnetometer base ranges. These ranges must be mapped into -1..1
// in order for the atan2 function to calculate heading properly.
/////////////////////////////////////////////////////////////////////
vector MAG_MIN = {-0.18, 0.03,-0.87};
vector MAG_MAX = { 0.30, 0.44, 0.17};

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 14.22 // Declination (degrees) in Shrewsbury, MA.

vector readMag()
{
  vector adjMag;
  adjMag.x = fmap(imu.calcMag(imu.mx), MAG_MIN.x, MAG_MAX.x, -1.0, 1.0);
  adjMag.y = fmap(imu.calcMag(imu.my), MAG_MIN.y, MAG_MAX.y, -1.0, 1.0);
  adjMag.z = fmap(imu.calcMag(imu.mz), MAG_MIN.z, MAG_MAX.z, -1.0, 1.0);  

  return adjMag;
}

void printMag()
{  
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(millis());
  Serial.print(",");
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(",");
  Serial.print(imu.calcMag(imu.mz), 2);
}

void float getMagHeading()
{
  float heading;
  if (mag.y == 0) {
    heading = (mag.x < 0) ? PI : 0;
  } else {
    heading = atan2(-mag.x, mag.y);
  }

  heading -= DECLINATION * PI / 180;

  if (heading < 0) {
    heading += 2 * PI;
  }
  
//  if (heading > PI) heading -= (2 * PI);
//  else if (heading < -PI) heading += (2 * PI);
//  else if (heading < 0) heading += 2 * PI;

  return heading;
}
