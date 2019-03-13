/////////////////////////////////////////////////////////////////////
// Magnetometer base ranges. These ranges must be mapped into -1..1
// in order for the atan2 function to calculate heading properly.
/////////////////////////////////////////////////////////////////////
// Mmin(-0.01,-0.37,-0.80), Mmax(0.53,0.31,-0.56)
// Mmin(0.02,-0.31,-0.84) Mmax(0.51,0.28,-0.65)
// Mmin(-0.06,-0.21,-0.87) Mmax(0.46,0.35,-0.75)

// With full rotations, use the following:
// MIN(-0.42, -0.38, -0.87) MAX(0.70, 0.72, 0.36)
static const vector MAG_MIN = {-0.42,-0.38,-0.87};
static const vector MAG_MAX = {0.70,0.72,0.36};

//static const vector MAG_MIN = {-0.06,-0.21,-0.87};
//static const vector MAG_MAX = {0.46,0.35,-0.75};

static vector magRaw = {0};
static vector magAdj = {0};

static vector magMinSeen = { 999.0,  999.0,  999.0};
static vector magMaxSeen = {-999.0, -999.0, -999.0};

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 14.22 // Declination (degrees) in Shrewsbury, MA.

void readMag()
{
  magRaw.x = imu.calcMag(imu.mx);
  magRaw.y = imu.calcMag(imu.my);
  magRaw.z = imu.calcMag(imu.mz);
  
  magAdj.x = fmap(magRaw.x, MAG_MIN.x, MAG_MAX.x, -1.0, 1.0);
  magAdj.y = fmap(magRaw.y, MAG_MIN.y, MAG_MAX.y, -1.0, 1.0);
  magAdj.z = fmap(magRaw.z, MAG_MIN.z, MAG_MAX.z, -1.0, 1.0);  

  magMinSeen.x = min(magMinSeen.x, magRaw.x);
  magMinSeen.y = min(magMinSeen.y, magRaw.y);
  magMinSeen.z = min(magMinSeen.z, magRaw.z);

  magMaxSeen.x = max(magMaxSeen.x, magRaw.x);
  magMaxSeen.y = max(magMaxSeen.y, magRaw.y);
  magMaxSeen.z = max(magMaxSeen.z, magRaw.z);
}

vector getMag()
{
  return magAdj;
}

float getMagHeading()
{
  float heading;
  if (magAdj.y == 0.0) {
    heading = (magAdj.x < 0.0) ? 180.0 : 0.0;
  } else {
    heading = rad2deg(atan2(-magAdj.x, magAdj.y));
  }

  heading = normalizeDeg(heading - DECLINATION);

  return heading;
}

void plotMagRaw()
{  
  Serial.print(magRaw.x, 2);
  Serial.print(" ");
  Serial.print(magRaw.y, 2);
  Serial.print(" ");
  Serial.print(magRaw.z, 2);
}

void plotMagAdj()
{  
  Serial.print(magAdj.x, 2);
  Serial.print(" ");
  Serial.print(magAdj.y, 2);
  Serial.print(" ");
  Serial.print(magAdj.z, 2);
}

void printMagMin()
{  
  Serial.print("Mmin(");
  Serial.print(magMinSeen.x, 2);
  Serial.print(",");
  Serial.print(magMinSeen.y, 2);
  Serial.print(",");
  Serial.print(magMinSeen.z, 2);
  Serial.print(") ");
}

void printMagMax()
{  
  Serial.print("Mmax(");
  Serial.print(magMaxSeen.x, 2);
  Serial.print(",");
  Serial.print(magMaxSeen.y, 2);
  Serial.print(",");
  Serial.print(magMaxSeen.z, 2);
  Serial.print(") ");
}

void plotMagMinMax()
{  
  Serial.print(magMinSeen.x, 2);
  Serial.print(" ");
  Serial.print(magMinSeen.y, 2);
  Serial.print(" ");
  Serial.print(magMinSeen.z, 2);
  Serial.print(" ");
  Serial.print(magMaxSeen.x, 2);
  Serial.print(" ");
  Serial.print(magMaxSeen.y, 2);
  Serial.print(" ");
  Serial.print(magMaxSeen.z, 2);
}
