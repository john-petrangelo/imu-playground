/////////////////////////////////////////////////////////////////////
// Magnetometer base ranges. These ranges must be mapped into -1..1
// in order for the atan2 function to calculate heading properly.
/////////////////////////////////////////////////////////////////////

// With full rotations, use the following:
// MIN(-0.42, -0.38, -0.87) MAX(0.70, 0.72, 0.36)
// min(-0.38, -0.27, -0.71) max(0.54,0.69,0.31)
static const Vector MAG_MIN = {-0.38, -0.27, -0.71};
static const Vector MAG_MAX = {0.54,0.69,0.31};

//static const vector MAG_MIN = {-0.06,-0.21,-0.87};
//static const vector MAG_MAX = {0.46,0.35,-0.75};

static Vector magRaw;
static Vector magAdj;

static Vector magMinSeen = Vector( 999.0,  999.0,  999.0);
static Vector magMaxSeen = Vector(-999.0, -999.0, -999.0);

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading. Calculate 
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION 14.22 // Declination (degrees) in Shrewsbury, MA.

void readMag()
{
  magRaw = (Vector){
    imu.calcMag(imu.my),
    imu.calcMag(imu.mx),
    -imu.calcMag(imu.mz)
  };
    
  magAdj = (Vector){
    fmap(magRaw.x, MAG_MIN.x, MAG_MAX.x, -1.0, 1.0),
    fmap(magRaw.y, MAG_MIN.y, MAG_MAX.y, -1.0, 1.0),
    fmap(magRaw.z, MAG_MIN.z, MAG_MAX.z, -1.0, 1.0)
  };

  magMinSeen = Vector{
    min(magMinSeen.x, magRaw.x),
    min(magMinSeen.y, magRaw.y),
    min(magMinSeen.z, magRaw.z)
  };

  magMaxSeen = Vector(
    max(magMaxSeen.x, magRaw.x),
    max(magMaxSeen.y, magRaw.y),
    max(magMaxSeen.z, magRaw.z)
  );
}

Vector getMagTicks()
{
  return Vector(imu.my, imu.mx, -imu.mz);
}

Vector getMag()
{
  return magAdj;
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
