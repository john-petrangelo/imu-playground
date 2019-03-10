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
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250 // ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

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
  unsigned long now = millis();
  
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
  
  if ((lastPrint + PRINT_SPEED) < millis())
  {
//    printGyro();
//    Serial.print(" ");
    readAccel();
//    plotAccel();
//    Serial.print(" ");
//    printMagRaw();
//    printMagAdj();
//    printMagMin();
//    printMagMax();
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
//    Serial.print(" ");
    readMag();

    plotAttitude(imu.ax, imu.ay, imu.az);
//    printMagAdj();
//    printMagMin();
//    printMagMax();
    printCounts();
    resetCounts();

    Serial.println();
    
    lastPrint = millis(); // Update lastPrint time
  }
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

void printGyro()
{
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.print(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.print(imu.gz);
#endif
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf?fpsp=1
// Heading calculations taken from this app note:
// http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
void printAttitude(float ax, float ay, float az, vector mag)
{
//JHP  float roll = atan2(ay, az);
  float roll = atan2(ax, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading = getMagHeading();

  heading = rad2deg(heading);
  pitch = rad2deg(pitch);
  roll = rad2deg(roll);

  Serial.print("Heading:");
  Serial.print(heading);

//  Serial.print("Heading, Pitch, Roll: ");
//  Serial.print(heading, 2);
//  Serial.print(", ");
//  Serial.print(pitch, 2);
//  Serial.print(", ");
//  Serial.print(roll, 2);
}

void plotAttitude(float ax, float ay, float az)
{
  float roll = atan2(ax, az);
  float pitch = atan2(ay, az);
  
  float heading = getMagHeading();

  heading = rad2deg(heading);
  pitch = rad2deg(pitch);
  roll = rad2deg(roll);

  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);
}
