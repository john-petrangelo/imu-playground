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

///////////////
// I2C Setup //
///////////////
#define LSM9DS1_M	0x1E // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG	0x6B // Would be 0x6A if SDO_AG is LOW

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

  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin())
  {
    Serial.println("Failed to communicate with LSM9DS1 IMU.");
    while (1)
      ;
  }
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
       updateAttitude();
       lastUpdate = now;
     }

    if ((lastPrint + PRINT_SPEED) < now)
    {
       lastPrint = now;
//       printUpdate();
//       resetCounts();
      new_test();
    }
  }
}

void printUpdate()
{
//    printGyro();
//    Serial.print(" ");
//    plotAccel();
//    Serial.print(" ");
//    printMagRaw();
//    printMagAdj();
//    printMagMin();
//    printMagMax();
//    Serial.print(" ");

//    updateAttitude();
    plotAttitude();

//    plotMagMinMax();
//    printMagAdj();
//    printMagMin();
//    printMagMax();
//    printCounts();

    Serial.println();  
}

void readSensors()
{
    // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
    gyroCount++;
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
    accelCount++;
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
    magCount++;
  }

  readAccel();
  readGyro();
  readMag();
}

void new_test()
{
  vector g = v_opposite(getAccel());
  vector mag = getMag();

  vector khat = v_opposite(g);
  vector ihat = v_crossproduct(mag, khat);
  vector jhat = v_crossproduct(khat, ihat);

  v_normalize(ihat);
  v_normalize(jhat);
  v_normalize(khat);

  float heading = normalizeDeg(rad2deg(atan2(jhat.x, jhat.y)));

  vector localVector = {0.0, 1.0, 0.0};
  float i = v_dotproduct(localVector, ihat);
  float j = v_dotproduct(localVector, jhat);
  float k = v_dotproduct(localVector, khat);
  heading = normalizeDeg(rad2deg(atan2(i, j)));
  vector euler = {i, j, k};

  v_print(ihat);
  Serial.print(" ");
  v_print(jhat);
  Serial.print(" ");
  v_print(khat);
  Serial.print(" ");
  v_print(euler);
  Serial.print(" H:");
  Serial.print(heading);
  Serial.println();
}

// sample: (0.43,-0.79,0.44) (0.90,0.38,-0.20) (-0.01,0.49,0.87)



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
