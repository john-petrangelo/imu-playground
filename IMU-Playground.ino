/*****************************************************************
This Arduino sketch is a demo of the simple side of the
t library. It'll demo the following:
* How to create a LSM9DS1 object, using a constructor (global
  variables section).
* How to use the begin() function of the LSM9DS1 class.
* How to read the gyroscope, accelerometer, and magnetometer
  using the readGryo(), readAccel(), readMag() functions and 
  the gx, gy, gz, ax, ay, az, mx, my, and mz variables.
* How to calculate actual acceleration, rotation speed, 
  magnetic field strength using the calcAccel(), calcGyro() 
  and calcMag() functions.
* How to use the data from the LSM9DS1 to calculate 
  orientation and heading.
*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
LSM9DS1 imu;

struct vector {
  float x;
  float y;
  float z;
};

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_SPEED 250 // ms between prints
#define UPDATE_SPEED 5 // ms between updates
static unsigned long lastPrint = 0;
static unsigned long lastUpdate = 0;

static int accelCount = 0;
static int gyroCount = 0;
static int magCount = 0;

void setup()
{
  Serial.begin(115200);
  initSensors();
}

void loop()
{
  unsigned long const now = millis();
  readSensors();

  if (now < 1000) {
    // For the first second use mag and accel.
    if ((lastPrint + PRINT_SPEED) < now)
    {
      initAttitude();
      printUpdate();
      lastPrint = now;
      resetCounts();
    }
  } else {
     // After one second use the gyro.
     if ((lastUpdate + UPDATE_SPEED) < now)
     {
       updateGyroAttitude();
       lastUpdate = now;
     }

    if ((lastPrint + PRINT_SPEED) < now)
    {
       lastPrint = now;
//       printUpdate();
//       resetCounts();
      print_attitude();
    }
  }
}

void printUpdate()
{
//    updateGyroAttitude();
    plotGyroAttitude();

    Serial.println();  
}

void print_attitude()
{
  // Read the accelerometer and 
  vector g = getAccel();
  vector mag = getMag();

  vector khat = v_opposite(g);
  v_normalize(khat);

  vector ihat = v_crossproduct(mag, khat);
  v_normalize(ihat);
  
  vector jhat = v_crossproduct(khat, ihat);
  v_normalize(jhat);


  // The euler vector is made up of the projection (i.e. dot product)
  // of the local vector over each of the global unit vectors (i, j, k).
  //   vector localVector = {0.0, 1.0, 0.0};
  //   float euler_x = v_dotproduct(localVector, ihat);
  //   float euler_y = v_dotproduct(localVector, jhat);
  //   float euler_z = v_dotproduct(localVector, khat);
  
  vector euler = {ihat.y, jhat.y, khat.y};
  v_normalize(euler);

  float heading = normalizeDeg(rad2deg(atan2(ihat.y, jhat.y)));
  float pitch = normalizeDeg(rad2deg(asin(khat.y)));
  float roll = normalizeDeg(rad2deg(atan2(khat.x, khat.z)));



//  quaternion q = {f(roll), euler.x, euler.y, euler.z};

//  Serial.print("g");
//  v_print(g);
//  Serial.print(" m");
//  v_print(mag);
//  Serial.print(" i");
//  v_print(ihat);
//  Serial.print(" j");
//  v_print(jhat);
//  Serial.print(" k");
//  v_print(khat);
//  Serial.print(" e");
//  v_print(euler);
//  Serial.print(" H:");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
  Serial.println();
}

void printCounts()
{
  Serial.print("#accel=");
  Serial.print(accelCount);
  Serial.print(" #gyro=");
  Serial.print(gyroCount);
  Serial.print(" #mag=");
  Serial.print(magCount);
  Serial.print(" ");
}

void resetCounts()
{
  accelCount = 0;
  gyroCount = 0;
  magCount = 0;
}
