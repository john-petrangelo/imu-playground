// The SFE_LSM9DS1 library requires both Wire and SPI be included BEFORE including the 9DS1 library.
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

struct quaternion {
  float w;
  float x;
  float y;
  float z;
};

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_SPEED 1000 // ms between prints
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

  // khat represents the direction of "up" expressed relative to the sensor.
  vector khat = v_opposite(g);
  v_normalize(khat);

  // ihat represents the direction of "east" expressed relative to the sensor.
  vector ihat = v_crossproduct(mag, khat);
  v_normalize(ihat);

  // jhat represents the direction of "north" expressed relative to the sensor.
  vector jhat = v_crossproduct(khat, ihat);
  v_normalize(jhat);

  Serial.print(" [ijk]hat: ");
  v_print(ihat);
  v_print(jhat);
  v_print(khat);

  // The euler vector is the orientation of the sensor expressed in global coordinates.
  // The euler vector is made up of the projection (i.e. dot product)
  // of the local vector over each of the global unit vectors (i, j, k).
  //   vector localVector = {0.0, 1.0, 0.0};
  //   float euler_x = v_dotproduct(localVector, ihat);
  //   float euler_y = v_dotproduct(localVector, jhat);
  //   float euler_z = v_dotproduct(localVector, khat);
  
  vector euler = {ihat.y, jhat.y, khat.y};
  v_normalize(euler);

//  Serial.print(" e");
//  v_print(euler);

  float headingRad = atan2(ihat.y, jhat.y);
  float pitchRad = asin(khat.y);
  float rollRad = atan2(khat.x, khat.z);

  float heading = normalizeDeg(rad2deg(headingRad));
  float pitch = normalizeDeg(rad2deg(pitchRad));
  float roll = normalizeDeg(rad2deg(rollRad));

  Serial.print(" HPR:");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(roll);

  // Make the quaternions to reverse the roll, undo the pitch, and turn the heading North.
  quaternion q_roll = q_make(rollRad/2, jhat);
  quaternion q_pitch = q_make(-pitchRad/2, ihat);
  quaternion q_heading = q_make(headingRad/2, khat);

//  Serial.print(" q_heading: ");
//  q_print(q_heading);

  // Combine the individual rotations into a single quaternion.
  quaternion q_rot = q_multiply(q_multiply(q_roll, q_pitch), q_heading);

//  Serial.print(" q_rot: ");
//  q_print(q_rot);

  // q_euler is the euler vector expressed as a quaternion.
  quaternion q_euler = q_make(euler);

//  Serial.print(" q_euler: ");
//  q_print(q_euler);

  // rotation = q x euler x q*
  quaternion q_left = q_multiply(q_rot, q_euler);
  quaternion q_out = q_multiply(q_left, q_conjugate(q_rot));
//  Serial.print(" q_left: ");
//  q_print(q_left);
//  Serial.print(" q_out: ");
//  q_print(q_out);

  Serial.print(" v_out: ");
  v_print(q_vector(q_out));
//  Serial.print(v_out.x);
//  Serial.print(" ");
//  Serial.print(v_out.y);
//  Serial.print(" ");
//  Serial.print(v_out.z);
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
